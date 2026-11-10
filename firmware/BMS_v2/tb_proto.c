// Tabasco link: body encoding, AES-GCM sealing and the replay guard. See tb_proto.h.
#include "tb_proto.h"

#include <string.h>

#include "mbedtls/gcm.h"

// ---------------------------------------------------------------- writer

void tb_w_init(tb_writer_t* w, uint8_t* buf, size_t cap) {
  w->buf = buf;
  w->cap = cap;
  w->len = 0;
  w->overflow = false;
}

void tb_w_u8(tb_writer_t* w, uint8_t v) {
  if (w->len >= w->cap) {
    w->overflow = true;
    return;
  }
  w->buf[w->len++] = v;
}

void tb_w_u16(tb_writer_t* w, uint16_t v) {
  tb_w_u8(w, (uint8_t)v);
  tb_w_u8(w, (uint8_t)(v >> 8));
}

void tb_w_u32(tb_writer_t* w, uint32_t v) {
  tb_w_u16(w, (uint16_t)v);
  tb_w_u16(w, (uint16_t)(v >> 16));
}

void tb_w_uvar(tb_writer_t* w, uint32_t v) {
  while (v >= 0x80) {
    tb_w_u8(w, (uint8_t)(v | 0x80));
    v >>= 7;
  }
  tb_w_u8(w, (uint8_t)v);
}

static uint32_t zigzag(int32_t v) {
  return ((uint32_t)v << 1) ^ (uint32_t)(v >> 31);
}

void tb_w_svar(tb_writer_t* w, int32_t v) {
  tb_w_uvar(w, zigzag(v));
}

size_t tb_svar_size(int32_t v) {
  uint32_t z = zigzag(v);
  size_t n = 1;
  while (z >= 0x80) {
    z >>= 7;
    n++;
  }
  return n;
}

void tb_w_bytes(tb_writer_t* w, const void* data, size_t len) {
  if (w->len + len > w->cap) {
    w->overflow = true;
    return;
  }
  memcpy(w->buf + w->len, data, len);
  w->len += len;
}

// ---------------------------------------------------------------- reader

void tb_r_init(tb_reader_t* r, const uint8_t* buf, size_t len) {
  r->buf = buf;
  r->len = len;
  r->pos = 0;
  r->error = false;
}

uint8_t tb_r_u8(tb_reader_t* r) {
  if (r->error || r->pos >= r->len) {
    r->error = true;
    return 0;
  }
  return r->buf[r->pos++];
}

uint16_t tb_r_u16(tb_reader_t* r) {
  const uint16_t lo = tb_r_u8(r);
  return (uint16_t)(lo | ((uint16_t)tb_r_u8(r) << 8));
}

uint32_t tb_r_u32(tb_reader_t* r) {
  const uint32_t lo = tb_r_u16(r);
  return lo | ((uint32_t)tb_r_u16(r) << 16);
}

uint32_t tb_r_uvar(tb_reader_t* r) {
  uint32_t v = 0;
  for (int shift = 0; shift < 35; shift += 7) {
    const uint8_t b = tb_r_u8(r);
    if (r->error) {
      return 0;
    }
    v |= (uint32_t)(b & 0x7F) << shift;
    if (!(b & 0x80)) {
      return v;
    }
  }
  r->error = true;
  return 0;
}

int32_t tb_r_svar(tb_reader_t* r) {
  const uint32_t z = tb_r_uvar(r);
  return (int32_t)((z >> 1) ^ (~(z & 1) + 1));
}

bool tb_r_bytes(tb_reader_t* r, void* out, size_t len) {
  if (r->error || r->pos + len > r->len) {
    r->error = true;
    return false;
  }
  memcpy(out, r->buf + r->pos, len);
  r->pos += len;
  return true;
}

// ---------------------------------------------------------------- frames

static void write_header(const tb_header_t* h, uint8_t* out) {
  out[0] = TB_MAGIC0;
  out[1] = TB_MAGIC1;
  out[2] = (uint8_t)((TB_VERSION << 4) | (h->kind & 0x0F));
  out[3] = h->node;
  out[4] = h->flags;
  out[5] = (uint8_t)h->boot;
  out[6] = (uint8_t)(h->boot >> 8);
  out[7] = (uint8_t)h->ctr;
  out[8] = (uint8_t)(h->ctr >> 8);
  out[9] = (uint8_t)(h->ctr >> 16);
  out[10] = (uint8_t)(h->ctr >> 24);
}

static void make_nonce(const uint8_t mac[6], const uint8_t* hdr, uint8_t nonce[12]) {
  memcpy(nonce, mac, 6);
  memcpy(nonce + 6, hdr + 5, 6);  // boot + counter, as sent
}

bool tb_is_frame(const uint8_t* frame, size_t len) {
  return frame && len >= TB_HDR_LEN + TB_TAG_LEN && len <= TB_MAX_FRAME && frame[0] == TB_MAGIC0 &&
         frame[1] == TB_MAGIC1 && (frame[2] >> 4) == TB_VERSION;
}

size_t tb_seal(const uint8_t key[TB_KEY_LEN], const uint8_t src_mac[6], const tb_header_t* h,
               const uint8_t* body, size_t body_len, uint8_t* out, size_t out_cap) {
  const size_t total = TB_HDR_LEN + body_len + TB_TAG_LEN;
  if (body_len > TB_MAX_BODY || total > out_cap) {
    return 0;
  }
  write_header(h, out);
  uint8_t nonce[12];
  make_nonce(src_mac, out, nonce);
  // The ESP hardware GCM rejects a NULL input even when its length is 0, so an empty body needs a real pointer.
  static const uint8_t kEmpty[1];
  mbedtls_gcm_context gcm;
  mbedtls_gcm_init(&gcm);
  int rc = mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, key, TB_KEY_LEN * 8);
  if (rc == 0) {
    rc = mbedtls_gcm_crypt_and_tag(&gcm, MBEDTLS_GCM_ENCRYPT, body_len, nonce, sizeof(nonce), out, TB_HDR_LEN,
                                   body_len ? body : kEmpty, out + TB_HDR_LEN, TB_TAG_LEN,
                                   out + TB_HDR_LEN + body_len);
  }
  mbedtls_gcm_free(&gcm);
  return rc == 0 ? total : 0;
}

bool tb_open(const uint8_t key[TB_KEY_LEN], const uint8_t src_mac[6], const uint8_t* frame, size_t frame_len,
             tb_header_t* h, uint8_t* body, size_t* body_len) {
  if (!tb_is_frame(frame, frame_len)) {
    return false;
  }
  const size_t n = frame_len - TB_HDR_LEN - TB_TAG_LEN;
  uint8_t nonce[12];
  make_nonce(src_mac, frame, nonce);
  mbedtls_gcm_context gcm;
  mbedtls_gcm_init(&gcm);
  int rc = mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, key, TB_KEY_LEN * 8);
  if (rc == 0) {
    rc = mbedtls_gcm_auth_decrypt(&gcm, n, nonce, sizeof(nonce), frame, TB_HDR_LEN, frame + TB_HDR_LEN + n,
                                  TB_TAG_LEN, frame + TB_HDR_LEN, body);
  }
  mbedtls_gcm_free(&gcm);
  if (rc != 0) {
    return false;
  }
  h->kind = frame[2] & 0x0F;
  h->node = frame[3];
  h->flags = frame[4];
  h->boot = (uint16_t)(frame[5] | (frame[6] << 8));
  h->ctr = (uint32_t)frame[7] | ((uint32_t)frame[8] << 8) | ((uint32_t)frame[9] << 16) | ((uint32_t)frame[10] << 24);
  *body_len = n;
  return true;
}

// ---------------------------------------------------------------- replay guard

bool tb_replay_check(tb_replay_t* rp, const uint8_t mac[6], uint16_t boot, uint32_t ctr, uint32_t now_ms) {
  tb_replay_slot_t* free_slot = NULL;
  tb_replay_slot_t* oldest = &rp->slot[0];
  for (int i = 0; i < TB_REPLAY_SLOTS; i++) {
    tb_replay_slot_t* s = &rp->slot[i];
    if (!s->used) {
      if (!free_slot) {
        free_slot = s;
      }
      continue;
    }
    if (memcmp(s->mac, mac, 6) == 0) {
      // Boot counters are 16-bit and wrap; compare with serial-number arithmetic.
      const int16_t dboot = (int16_t)(boot - s->boot);
      const bool newer = dboot > 0 || (dboot == 0 && ctr > s->ctr);
      if (!newer) {
        return false;
      }
      s->boot = boot;
      s->ctr = ctr;
      s->last_ms = now_ms;
      return true;
    }
    if ((now_ms - s->last_ms) > (now_ms - oldest->last_ms)) {
      oldest = s;
    }
  }
  tb_replay_slot_t* s = free_slot ? free_slot : oldest;
  memcpy(s->mac, mac, 6);
  s->boot = boot;
  s->ctr = ctr;
  s->last_ms = now_ms;
  s->used = true;
  return true;
}

// ---------------------------------------------------------------- key

static int hexval(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

bool tb_key_from_hex(const char* hex, uint8_t key[TB_KEY_LEN]) {
  if (!hex || strlen(hex) != TB_KEY_LEN * 2) {
    return false;
  }
  for (int i = 0; i < TB_KEY_LEN; i++) {
    const int hi = hexval(hex[2 * i]);
    const int lo = hexval(hex[2 * i + 1]);
    if (hi < 0 || lo < 0) {
      return false;
    }
    key[i] = (uint8_t)((hi << 4) | lo);
  }
  return true;
}
