// Tabasco link: message bodies, link state and the on-change publisher. See tb_msgs.h.
#include "tb_msgs.h"

#include <limits.h>
#include <string.h>

// ---------------------------------------------------------------- link

void tb_link_init(tb_link_t* l, const uint8_t key[TB_KEY_LEN], const uint8_t mac[6], uint8_t node, uint16_t boot) {
  memset(l, 0, sizeof(*l));
  memcpy(l->key, key, TB_KEY_LEN);
  memcpy(l->mac, mac, 6);
  l->node = node;
  l->boot = boot;
}

size_t tb_link_seal(tb_link_t* l, uint8_t kind, uint8_t flags, const uint8_t* body, size_t len, uint8_t* out) {
  tb_header_t h = {kind, l->node, flags, l->boot, ++l->ctr};
  const size_t n = tb_seal(l->key, l->mac, &h, body, len, out, TB_MAX_FRAME);
  if (n) {
    l->tx++;
  }
  return n;
}

bool tb_link_open(tb_link_t* l, const uint8_t src_mac[6], const uint8_t* frame, size_t len, uint32_t now_ms,
                  tb_header_t* h, uint8_t* body, size_t* body_len) {
  if (!tb_is_frame(frame, len)) {
    return false;  // not our format at all (old JSON packets, other devices): not counted
  }
  if (!tb_open(l->key, src_mac, frame, len, h, body, body_len)) {
    l->rx_bad++;
    return false;
  }
  if (!tb_replay_check(&l->replay, src_mac, h->boot, h->ctr, now_ms)) {
    l->rx_replay++;
    return false;
  }
  l->rx_ok++;
  return true;
}

// ---------------------------------------------------------------- DATA

bool tb_data_parse(const uint8_t* body, size_t len, tb_data_cb cb, void* ctx) {
  tb_reader_t r;
  tb_r_init(&r, body, len);
  while (tb_r_more(&r)) {
    const uint8_t id = tb_r_u8(&r);
    if (id & TB_ID_NA) {
      cb(ctx, id & 0x7F, false, 0);
      continue;
    }
    const int32_t v = tb_r_svar(&r);
    if (r.error) {
      return false;
    }
    cb(ctx, id, true, v);
  }
  return !r.error;
}

void tb_pub_init(tb_pub_t* p, uint8_t node) {
  memset(p, 0, sizeof(*p));
  p->node = node;
  p->count = tb_sig_count(node);
  if (p->count > TB_PUB_MAX) {
    p->count = TB_PUB_MAX;
  }
  p->force_full = true;
}

static bool changed(const tb_pub_t* p, uint8_t i, int32_t v, bool valid) {
  if (!p->ever[i] || valid != p->sent_valid[i]) {
    return true;
  }
  if (!valid) {
    return false;
  }
  const tb_sigdef_t* d = tb_sig(p->node, i);
  const int32_t band = (d && d->kind == TB_SK_NUMBER && d->deadband > 0) ? d->deadband : 1;
  const int64_t diff = (int64_t)v - (int64_t)p->sent[i];
  return diff >= band || diff <= -band;
}

size_t tb_pub_build(tb_pub_t* p, const int32_t* values, const bool* valid, uint32_t now_ms, uint8_t* body,
                    size_t cap, uint8_t* flags) {
  const bool full = p->force_full || (now_ms - p->last_full_ms) >= TB_FULL_PERIOD_MS;
  if (!full && (now_ms - p->last_delta_ms) < TB_MIN_DELTA_MS) {
    return 0;
  }
  tb_writer_t w;
  tb_w_init(&w, body, cap);
  for (uint8_t i = 0; i < p->count; i++) {
    if (!full && !changed(p, i, values[i], valid[i])) {
      continue;
    }
    const size_t need = valid[i] ? 1 + tb_svar_size(values[i]) : 1;
    if (w.len + need > cap) {
      break;  // the rest goes in the next frame (another delta), or the next full refresh
    }
    if (valid[i]) {
      tb_w_u8(&w, i);
      tb_w_svar(&w, values[i]);
    } else {
      tb_w_u8(&w, (uint8_t)(i | TB_ID_NA));
    }
    p->sent[i] = values[i];
    p->sent_valid[i] = valid[i];
    p->ever[i] = true;
  }
  if (full) {
    p->last_full_ms = now_ms;
    p->force_full = false;
  }
  if (w.len == 0) {
    return 0;
  }
  p->last_delta_ms = now_ms;
  *flags = full ? (TB_FLAG_FULL | TB_FLAG_LAST) : 0;
  return w.len;
}

// ---------------------------------------------------------------- J1587

void tb_j1587_put(tb_writer_t* w, const tb_j1587_val_t* v) {
  tb_w_u8(w, v->mid);
  tb_w_uvar(w, v->pid);
  tb_w_u8(w, (uint8_t)((v->known ? 0x80 : 0) | (v->valid ? 0 : 0x40) | (v->decimals & 0x0F)));
  if (v->valid) {
    tb_w_svar(w, v->value);
  }
}

bool tb_j1587_get(tb_reader_t* r, tb_j1587_val_t* v) {
  v->mid = tb_r_u8(r);
  v->pid = (uint16_t)tb_r_uvar(r);
  const uint8_t info = tb_r_u8(r);
  v->known = (info & 0x80) != 0;
  v->valid = (info & 0x40) == 0;
  v->decimals = info & 0x0F;
  v->value = v->valid ? tb_r_svar(r) : 0;
  return !r->error;
}

// ---------------------------------------------------------------- DTC

void tb_dtc_put(tb_writer_t* w, const tb_dtc_t* d) {
  tb_w_u8(w, d->mid);
  tb_w_uvar(w, d->num);
  tb_w_u8(w, (uint8_t)((d->is_sid ? 1 : 0) | (d->active ? 2 : 0)));
  tb_w_u8(w, d->fmi);
  tb_w_u8(w, d->count);
}

bool tb_dtc_get(tb_reader_t* r, tb_dtc_t* d) {
  d->mid = tb_r_u8(r);
  d->num = (uint16_t)tb_r_uvar(r);
  const uint8_t f = tb_r_u8(r);
  d->is_sid = (f & 1) != 0;
  d->active = (f & 2) != 0;
  d->fmi = tb_r_u8(r);
  d->count = tb_r_u8(r);
  return !r->error;
}

// ---------------------------------------------------------------- BMS module

size_t tb_bms_module_put(uint8_t* body, size_t cap, const tb_bms_module_t* m) {
  tb_writer_t w;
  tb_w_init(&w, body, cap);
  tb_w_u8(&w, m->slot);
  tb_w_u8(&w, m->can);
  tb_w_u8(&w, m->online ? 1 : 0);
  tb_w_u16(&w, m->status_word);
  tb_w_svar(&w, m->total_mv);
  tb_w_svar(&w, m->delta_mv);
  tb_w_svar(&w, m->avg_temp_df == INT32_MIN ? INT16_MIN : m->avg_temp_df);
  const uint8_t nc = m->ncells > TB_BMS_CELLS ? TB_BMS_CELLS : m->ncells;
  tb_w_u8(&w, nc);
  for (uint8_t i = 0; i < nc; i++) {
    tb_w_svar(&w, m->cells_mv[i]);
  }
  const uint8_t nt = m->ntemps > TB_BMS_TEMPS ? TB_BMS_TEMPS : m->ntemps;
  tb_w_u8(&w, nt);
  for (uint8_t i = 0; i < nt; i++) {
    tb_w_svar(&w, m->temps_df[i]);
  }
  return w.overflow ? 0 : w.len;
}

bool tb_bms_module_get(const uint8_t* body, size_t len, tb_bms_module_t* m) {
  tb_reader_t r;
  tb_r_init(&r, body, len);
  memset(m, 0, sizeof(*m));
  m->slot = tb_r_u8(&r);
  m->can = tb_r_u8(&r);
  m->online = (tb_r_u8(&r) & 1) != 0;
  m->status_word = tb_r_u16(&r);
  m->total_mv = tb_r_svar(&r);
  m->delta_mv = tb_r_svar(&r);
  m->avg_temp_df = tb_r_svar(&r);
  if (m->avg_temp_df == INT16_MIN) {
    m->avg_temp_df = INT32_MIN;
  }
  m->ncells = tb_r_u8(&r);
  if (m->ncells > TB_BMS_CELLS) {
    return false;
  }
  for (uint8_t i = 0; i < m->ncells; i++) {
    m->cells_mv[i] = (int16_t)tb_r_svar(&r);
  }
  m->ntemps = tb_r_u8(&r);
  if (m->ntemps > TB_BMS_TEMPS) {
    return false;
  }
  for (uint8_t i = 0; i < m->ntemps; i++) {
    m->temps_df[i] = (int16_t)tb_r_svar(&r);
  }
  return !r.error && m->slot >= 1 && m->slot <= TB_BMS_MODULES;
}

// ---------------------------------------------------------------- commands

size_t tb_cmd_put(uint8_t* body, size_t cap, const tb_cmd_msg_t* c) {
  tb_writer_t w;
  tb_w_init(&w, body, cap);
  tb_w_u8(&w, c->target);
  tb_w_u16(&w, c->target_boot);
  tb_w_u16(&w, c->cmd_id);
  tb_w_u8(&w, c->cmd);
  tb_w_svar(&w, c->arg);
  return w.overflow ? 0 : w.len;
}

bool tb_cmd_get(const uint8_t* body, size_t len, tb_cmd_msg_t* c) {
  tb_reader_t r;
  tb_r_init(&r, body, len);
  c->target = tb_r_u8(&r);
  c->target_boot = tb_r_u16(&r);
  c->cmd_id = tb_r_u16(&r);
  c->cmd = tb_r_u8(&r);
  c->arg = tb_r_svar(&r);
  return !r.error;
}

size_t tb_ack_put(uint8_t* body, size_t cap, const tb_ack_msg_t* a) {
  tb_writer_t w;
  tb_w_init(&w, body, cap);
  tb_w_bytes(&w, a->to_mac, 6);
  tb_w_u16(&w, a->cmd_id);
  tb_w_u8(&w, a->result);
  return w.overflow ? 0 : w.len;
}

bool tb_ack_get(const uint8_t* body, size_t len, tb_ack_msg_t* a) {
  tb_reader_t r;
  tb_r_init(&r, body, len);
  tb_r_bytes(&r, a->to_mac, 6);
  a->cmd_id = tb_r_u16(&r);
  a->result = tb_r_u8(&r);
  return !r.error;
}

bool tb_cmd_seen(tb_cmd_dedup_t* d, const uint8_t mac[6], uint16_t cmd_id, uint8_t* result) {
  for (int i = 0; i < TB_CMD_SEEN; i++) {
    const tb_cmd_seen_t* e = &d->e[i];
    if (e->used && e->cmd_id == cmd_id && memcmp(e->mac, mac, 6) == 0) {
      *result = e->result;
      return true;
    }
  }
  return false;
}

void tb_cmd_remember(tb_cmd_dedup_t* d, const uint8_t mac[6], uint16_t cmd_id, uint8_t result) {
  tb_cmd_seen_t* e = &d->e[d->next];
  d->next = (uint8_t)((d->next + 1) % TB_CMD_SEEN);
  memcpy(e->mac, mac, 6);
  e->cmd_id = cmd_id;
  e->result = result;
  e->used = true;
}

// ---------------------------------------------------------------- subscriptions

size_t tb_sub_put(uint8_t* body, size_t cap, const tb_sub_msg_t* s) {
  tb_writer_t w;
  tb_w_init(&w, body, cap);
  tb_w_u16(&w, s->mask);
  tb_w_u16(&w, s->module_mask);
  return w.overflow ? 0 : w.len;
}

bool tb_sub_get(const uint8_t* body, size_t len, tb_sub_msg_t* s) {
  tb_reader_t r;
  tb_r_init(&r, body, len);
  s->mask = tb_r_u16(&r);
  s->module_mask = tb_r_u16(&r);
  return !r.error;
}

void tb_subs_note(tb_subs_t* t, const uint8_t mac[6], const tb_sub_msg_t* s, uint32_t now_ms) {
  tb_sub_slot_t* slot = NULL;
  for (int i = 0; i < TB_SUB_SLOTS && !slot; i++) {
    if (t->s[i].used && memcmp(t->s[i].mac, mac, 6) == 0) {
      slot = &t->s[i];
    }
  }
  for (int i = 0; i < TB_SUB_SLOTS && !slot; i++) {
    if (!t->s[i].used || (int32_t)(now_ms - t->s[i].until_ms) > 0) {
      slot = &t->s[i];
    }
  }
  if (!slot) {
    return;  // more than TB_SUB_SLOTS live subscribers: the newest waits for a slot to expire
  }
  memcpy(slot->mac, mac, 6);
  slot->sub = *s;
  slot->until_ms = now_ms + TB_SUB_LEASE_MS;
  slot->used = true;
}

tb_sub_msg_t tb_subs_active(const tb_subs_t* t, uint32_t now_ms) {
  tb_sub_msg_t out = {0, 0};
  for (int i = 0; i < TB_SUB_SLOTS; i++) {
    if (t->s[i].used && (int32_t)(t->s[i].until_ms - now_ms) > 0) {
      out.mask |= t->s[i].sub.mask;
      out.module_mask |= t->s[i].sub.module_mask;
    }
  }
  return out;
}
