// Tabasco link: the ESP-NOW packet format shared by the TIM, BMS, house node and HMI panels.
// Master copy: Modules/Common/tabasco_link. The Arduino sketches keep a copy in their own folder
// (the Arduino IDE only builds files inside the sketch folder); copy changes there with sync_common.sh.
//
// One frame = one ESP-NOW broadcast on channel 1, at most 250 bytes:
//   header (11 bytes, sent in clear, authenticated)
//     'T' 'B' | version<<4 | kind | node type | flags | boot (u16 LE) | counter (u32 LE)
//   body (encrypted, AES-128-GCM)
//   tag (8 bytes)
// Nonce = sender MAC (6) + boot (2) + counter (4). Each sender raises its counter on every frame and
// raises boot on every restart (kept in flash), so a nonce is never reused. Receivers drop any frame whose
// (boot, counter) is not newer than the last one from that MAC, which stops replays.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TB_MAGIC0 'T'
#define TB_MAGIC1 'B'
#define TB_VERSION 1
#define TB_CHANNEL 1
#define TB_MAX_FRAME 250
#define TB_HDR_LEN 11
#define TB_TAG_LEN 8
#define TB_MAX_BODY (TB_MAX_FRAME - TB_HDR_LEN - TB_TAG_LEN)
#define TB_KEY_LEN 16

// Timing shared by every node.
#define TB_FULL_PERIOD_MS 5000      // full refresh of every value, so a waking panel catches up
#define TB_MIN_DELTA_MS 200         // a changed value is sent at most this often
#define TB_SUB_LEASE_MS 30000       // a subscription lasts this long unless renewed
#define TB_STALE_MS 12000           // a value older than this is shown as stale (two missed full refreshes)
#define TB_LOST_MS 20000            // a node not heard for this long is shown as offline
#define TB_CMD_RETRY_MS 300
#define TB_CMD_TRIES 5

typedef enum {
  TB_NODE_NONE = 0,
  TB_NODE_TIM = 1,
  TB_NODE_BMS = 2,
  TB_NODE_HMI = 3,
  TB_NODE_HOUSE = 4,
} tb_node_t;

typedef enum {
  TB_KIND_DATA = 0,        // signal values of the sender's node type (flags: TB_FLAG_FULL)
  TB_KIND_J1587 = 1,       // TIM: decoded J1587 parameters
  TB_KIND_DTC = 2,         // TIM: active J1587 trouble codes (always the full list)
  TB_KIND_BMS_MODULE = 3,  // BMS: one module's cells and temperatures
  TB_KIND_TEXT = 4,        // a status line (u8 text id, then UTF-8)
  TB_KIND_CMD = 5,         // command to a node type
  TB_KIND_ACK = 6,         // owner's answer to a command
  TB_KIND_SUB = 7,         // panel asks for on-demand data
  TB_KIND_PRESENCE = 8,    // panel heartbeat
  TB_KIND_SETTING = 9,     // one shared panel setting
  TB_KIND_SETTING_REQ = 10,// "send me every setting"
  TB_KIND_NAV = 11,        // mirrored navigation
} tb_kind_t;

#define TB_FLAG_FULL 0x01    // DATA/J1587: this frame is part of a full refresh
#define TB_FLAG_LAST 0x02    // last frame of a full refresh

// ---------------------------------------------------------------- commands
typedef enum {
  TB_CMD_BMS_POWER = 1,      // arg 0/1: CMU 12 V (on also runs naming)
  TB_CMD_BMS_CONTACTOR = 2,  // arg 0/1: EMS300 enable
  TB_CMD_BMS_BALANCE = 3,    // arg 0/1: balance all
  TB_CMD_BMS_FAN = 4,        // arg = fan (0 = all, 1, 2) << 8 | percent
  TB_CMD_BMS_RENAME = 5,     // rerun CMU naming
  TB_CMD_TIM_CLEAR_MINMAX = 20,
  TB_CMD_HOUSE_SLIDE = 40,   // arg = slide << 8 | (0 stop, 1 in, 2 out); reserved, slides are not wired yet
} tb_cmd_t;

typedef enum {
  TB_ACK_OK = 0,
  TB_ACK_BAD_ARG = 1,
  TB_ACK_UNKNOWN = 2,
  TB_ACK_REFUSED = 3,  // valid but not allowed now (interlock)
} tb_ack_t;

// ---------------------------------------------------------------- subscriptions (bit mask in a SUB frame)
#define TB_SUB_J1587 0x0001u       // TIM sends every J1587 parameter it hears
#define TB_SUB_BMS_MODULES 0x0002u // BMS sends module packets (body also carries a 16-bit module mask)

// ---------------------------------------------------------------- engine state (TIM signal TB_TIM_ENGINE)
typedef enum { TB_ENG_OFF = 0, TB_ENG_CRANKING = 1, TB_ENG_RUNNING = 2, TB_ENG_UNKNOWN = 3 } tb_engine_t;

// "No reading" marker in a DATA entry: the signal id with this bit set, no value follows.
#define TB_ID_NA 0x80u

// ---------------------------------------------------------------- body writer / reader
typedef struct {
  uint8_t* buf;
  size_t cap;
  size_t len;
  bool overflow;
} tb_writer_t;

typedef struct {
  const uint8_t* buf;
  size_t len;
  size_t pos;
  bool error;
} tb_reader_t;

void tb_w_init(tb_writer_t* w, uint8_t* buf, size_t cap);
void tb_w_u8(tb_writer_t* w, uint8_t v);
void tb_w_u16(tb_writer_t* w, uint16_t v);
void tb_w_u32(tb_writer_t* w, uint32_t v);
void tb_w_uvar(tb_writer_t* w, uint32_t v);
void tb_w_svar(tb_writer_t* w, int32_t v);  // zigzag varint: small values of either sign take 1 byte
void tb_w_bytes(tb_writer_t* w, const void* data, size_t len);
size_t tb_svar_size(int32_t v);

void tb_r_init(tb_reader_t* r, const uint8_t* buf, size_t len);
uint8_t tb_r_u8(tb_reader_t* r);
uint16_t tb_r_u16(tb_reader_t* r);
uint32_t tb_r_u32(tb_reader_t* r);
uint32_t tb_r_uvar(tb_reader_t* r);
int32_t tb_r_svar(tb_reader_t* r);
bool tb_r_bytes(tb_reader_t* r, void* out, size_t len);
static inline bool tb_r_more(const tb_reader_t* r) { return !r->error && r->pos < r->len; }

// ---------------------------------------------------------------- frames
typedef struct {
  uint8_t kind;
  uint8_t node;
  uint8_t flags;
  uint16_t boot;
  uint32_t ctr;
} tb_header_t;

// Encrypts body into out (TB_HDR_LEN + body_len + TB_TAG_LEN bytes). Returns the frame length, 0 on error.
size_t tb_seal(const uint8_t key[TB_KEY_LEN], const uint8_t src_mac[6], const tb_header_t* h,
               const uint8_t* body, size_t body_len, uint8_t* out, size_t out_cap);

// Checks and decrypts a received frame. body must hold TB_MAX_BODY bytes. Returns false for anything that
// is not a valid Tabasco frame for this key (other ESP-NOW traffic, such as the old JSON packets, included).
bool tb_open(const uint8_t key[TB_KEY_LEN], const uint8_t src_mac[6], const uint8_t* frame, size_t frame_len,
             tb_header_t* h, uint8_t* body, size_t* body_len);

// True if data looks like a Tabasco frame (magic and version), before decrypting.
bool tb_is_frame(const uint8_t* frame, size_t len);

// Replay guard: remembers the newest (boot, counter) per sender.
#define TB_REPLAY_SLOTS 24
typedef struct {
  uint8_t mac[6];
  uint16_t boot;
  uint32_t ctr;
  uint32_t last_ms;
  bool used;
} tb_replay_slot_t;

typedef struct {
  tb_replay_slot_t slot[TB_REPLAY_SLOTS];
} tb_replay_t;

// Returns true (and records it) if this (boot, counter) is newer than anything seen from mac.
// The table is in RAM, so after the receiver restarts it accepts the next frame from each sender. Commands
// are still safe then: a CMD carries the target's own boot number, and a target refuses any other.
bool tb_replay_check(tb_replay_t* rp, const uint8_t mac[6], uint16_t boot, uint32_t ctr, uint32_t now_ms);

// Parses a key written as 32 hex digits. Returns false if it is not exactly that.
bool tb_key_from_hex(const char* hex, uint8_t key[TB_KEY_LEN]);

#ifdef __cplusplus
}
#endif
