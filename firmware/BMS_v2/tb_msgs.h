// Tabasco link: message bodies, the per-node send/receive state, and the on-change publisher.
#pragma once

#include "tb_proto.h"
#include "tb_signals.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------- link state (one per node)
typedef struct {
  uint8_t key[TB_KEY_LEN];
  uint8_t mac[6];   // own MAC, as other nodes see it in ESP-NOW frames
  uint8_t node;     // own tb_node_t
  uint16_t boot;    // raised on every restart, kept in flash by the caller
  uint32_t ctr;
  tb_replay_t replay;
  uint32_t rx_ok, rx_bad, rx_replay, tx;
} tb_link_t;

void tb_link_init(tb_link_t* l, const uint8_t key[TB_KEY_LEN], const uint8_t mac[6], uint8_t node, uint16_t boot);
// Seals one frame of this node. out must hold TB_MAX_FRAME bytes. Returns the length, 0 on error.
size_t tb_link_seal(tb_link_t* l, uint8_t kind, uint8_t flags, const uint8_t* body, size_t len, uint8_t* out);
// Opens a received frame and applies the replay guard. False = not ours, forged or replayed.
bool tb_link_open(tb_link_t* l, const uint8_t src_mac[6], const uint8_t* frame, size_t len, uint32_t now_ms,
                  tb_header_t* h, uint8_t* body, size_t* body_len);

// ---------------------------------------------------------------- DATA bodies
// Body: repeated [id (bit 7 = no reading), zigzag varint value if bit 7 clear].
typedef void (*tb_data_cb)(void* ctx, uint8_t id, bool valid, int32_t value);
bool tb_data_parse(const uint8_t* body, size_t len, tb_data_cb cb, void* ctx);

// On-change publisher for a node's DATA frames.
#define TB_PUB_MAX 64
typedef struct {
  uint8_t node;
  uint8_t count;
  int32_t sent[TB_PUB_MAX];
  bool sent_valid[TB_PUB_MAX];
  bool ever[TB_PUB_MAX];
  uint32_t last_full_ms;
  uint32_t last_delta_ms;
  bool force_full;
} tb_pub_t;

void tb_pub_init(tb_pub_t* p, uint8_t node);
// Builds the next DATA body from the current values. Returns 0 when nothing is due.
// *flags gets TB_FLAG_FULL for a full refresh. values/valid hold tb_sig_count(node) entries.
size_t tb_pub_build(tb_pub_t* p, const int32_t* values, const bool* valid, uint32_t now_ms, uint8_t* body,
                    size_t cap, uint8_t* flags);

// ---------------------------------------------------------------- J1587 (TIM)
// Body: repeated [mid u8, pid uvar, info u8 (bit 7 known scaling, bit 6 no value, bits 0-3 decimals), svar value]
typedef struct {
  uint8_t mid;
  uint16_t pid;
  bool known;
  bool valid;
  uint8_t decimals;
  int32_t value;  // real x 10^decimals
} tb_j1587_val_t;
void tb_j1587_put(tb_writer_t* w, const tb_j1587_val_t* v);
bool tb_j1587_get(tb_reader_t* r, tb_j1587_val_t* v);

// ---------------------------------------------------------------- DTC (TIM)
// Body: total u8, first index u8, then repeated [mid u8, num uvar, flags u8 (bit 0 SID, bit 1 active), fmi u8, count u8]
typedef struct {
  uint8_t mid;
  uint16_t num;
  bool is_sid;
  bool active;
  uint8_t fmi;
  uint8_t count;
} tb_dtc_t;
void tb_dtc_put(tb_writer_t* w, const tb_dtc_t* d);
bool tb_dtc_get(tb_reader_t* r, tb_dtc_t* d);

// ---------------------------------------------------------------- BMS module
typedef struct {
  uint8_t slot;  // 1..16
  uint8_t can;
  bool online;
  uint16_t status_word;
  int32_t total_mv;     // -1 = none
  int32_t delta_mv;     // -1 = none
  int32_t avg_temp_df;  // deci-degF, INT32_MIN = none
  uint8_t ncells;
  int16_t cells_mv[TB_BMS_CELLS];  // -1 = none
  uint8_t ntemps;
  int16_t temps_df[TB_BMS_TEMPS];  // deci-degF, INT16_MIN = none
} tb_bms_module_t;
size_t tb_bms_module_put(uint8_t* body, size_t cap, const tb_bms_module_t* m);
bool tb_bms_module_get(const uint8_t* body, size_t len, tb_bms_module_t* m);

// ---------------------------------------------------------------- commands
typedef struct {
  uint8_t target;       // tb_node_t
  uint16_t target_boot; // the target's current boot number (from its frames); stops replays across restarts
  uint16_t cmd_id;      // per sender, wraps
  uint8_t cmd;          // tb_cmd_t
  int32_t arg;
} tb_cmd_msg_t;
size_t tb_cmd_put(uint8_t* body, size_t cap, const tb_cmd_msg_t* c);
bool tb_cmd_get(const uint8_t* body, size_t len, tb_cmd_msg_t* c);

typedef struct {
  uint8_t to_mac[6];
  uint16_t cmd_id;
  uint8_t result;  // tb_ack_t
} tb_ack_msg_t;
size_t tb_ack_put(uint8_t* body, size_t cap, const tb_ack_msg_t* a);
bool tb_ack_get(const uint8_t* body, size_t len, tb_ack_msg_t* a);

// Remembers recent commands, so a resent command is acknowledged again but applied once.
#define TB_CMD_SEEN 16
typedef struct {
  uint8_t mac[6];
  uint16_t cmd_id;
  uint8_t result;
  bool used;
} tb_cmd_seen_t;
typedef struct {
  tb_cmd_seen_t e[TB_CMD_SEEN];
  uint8_t next;
} tb_cmd_dedup_t;
// Returns true if (mac, cmd_id) was already applied; *result gets the earlier answer.
bool tb_cmd_seen(tb_cmd_dedup_t* d, const uint8_t mac[6], uint16_t cmd_id, uint8_t* result);
void tb_cmd_remember(tb_cmd_dedup_t* d, const uint8_t mac[6], uint16_t cmd_id, uint8_t result);

// ---------------------------------------------------------------- subscriptions
typedef struct {
  uint16_t mask;         // TB_SUB_*
  uint16_t module_mask;  // BMS modules wanted, bit 0 = slot 1
} tb_sub_msg_t;
size_t tb_sub_put(uint8_t* body, size_t cap, const tb_sub_msg_t* s);
bool tb_sub_get(const uint8_t* body, size_t len, tb_sub_msg_t* s);

// Source side: union of every live subscription.
#define TB_SUB_SLOTS 8
typedef struct {
  uint8_t mac[6];
  tb_sub_msg_t sub;
  uint32_t until_ms;
  bool used;
} tb_sub_slot_t;
typedef struct {
  tb_sub_slot_t s[TB_SUB_SLOTS];
} tb_subs_t;
void tb_subs_note(tb_subs_t* t, const uint8_t mac[6], const tb_sub_msg_t* s, uint32_t now_ms);
tb_sub_msg_t tb_subs_active(const tb_subs_t* t, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
