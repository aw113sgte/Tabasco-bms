// Tabasco link: every value a node broadcasts in DATA frames, with its id, scaling and deadband.
// A value travels as a whole number: real value x 10^decimals. Ids are per node type and never reused;
// add new ones at the end of a list.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "tb_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { TB_SK_NUMBER = 0, TB_SK_BOOL = 1, TB_SK_ENUM = 2 } tb_sigkind_t;

typedef struct {
  uint8_t node;       // tb_node_t
  uint8_t id;         // 0..127
  uint8_t kind;       // tb_sigkind_t
  uint8_t decimals;   // wire value = real x 10^decimals
  int32_t deadband;   // in wire units; a change smaller than this waits for the next full refresh
  const char* key;    // short stable name, used in saved gauge layouts
  const char* name;   // shown on the panels
  const char* unit;
} tb_sigdef_t;

// ---------------------------------------------------------------- TIM (node 1)
enum {
  TB_TIM_V1437 = 0,
  TB_TIM_V1139,
  TB_TIM_V1138,
  TB_TIM_V1431,
  TB_TIM_V1274,
  TB_TIM_V2010,
  TB_TIM_V1435A,
  TB_TIM_V1435B,
  TB_TIM_V2085,
  TB_TIM_SHUNT,
  TB_TIM_SENS5V,
  TB_TIM_C59_PSIA,
  TB_TIM_FUEL_VAC,
  TB_TIM_OIL_IN,
  TB_TIM_OIL_OUT,
  TB_TIM_OIL_DP,
  TB_TIM_AIR,
  TB_TIM_AIR_PSIA,
  TB_TIM_C58,
  TB_TIM_ENGINE,
  TB_TIM_RPM,
  TB_TIM_COUNT
};

// ---------------------------------------------------------------- BMS v2 (node 2)
enum {
  TB_BMS_CMUS_UART = 0,
  TB_BMS_CMUS_CAN,
  TB_BMS_MOD_V_HI,
  TB_BMS_MOD_V_AVG,
  TB_BMS_MOD_V_LO,
  TB_BMS_T_HI,
  TB_BMS_T_AVG,
  TB_BMS_T_LO,
  TB_BMS_DELTA_MAX,
  TB_BMS_SOC,
  TB_BMS_CURRENT,
  TB_BMS_BATT_V,
  TB_BMS_POWER,
  TB_BMS_CONTACTOR,
  TB_BMS_EMS_LED,
  TB_BMS_BAL_ALL,
  TB_BMS_T_BOARD,
  TB_BMS_T_AIR,
  TB_BMS_FAN1_PCT,
  TB_BMS_FAN2_PCT,
  TB_BMS_FAN1_RPM,
  TB_BMS_FAN2_RPM,
  TB_BMS_V24,
  TB_BMS_COUNT
};
#define TB_BMS_TEXT_STATUS 1  // TEXT id: the BMS status line
#define TB_BMS_MODULES 16
#define TB_BMS_CELLS 12
#define TB_BMS_TEMPS 6

// ---------------------------------------------------------------- house node (node 4), placeholders until it exists
#define TB_HOUSE_TANKS 6
#define TB_HOUSE_SLIDES 4
enum {
  TB_HOUSE_TANK1 = 0,  // tank levels in %, TB_HOUSE_TANK1 + n
  TB_HOUSE_SLIDE1_POS = TB_HOUSE_TANK1 + TB_HOUSE_TANKS,         // % out
  TB_HOUSE_SLIDE1_STATE = TB_HOUSE_SLIDE1_POS + TB_HOUSE_SLIDES,  // tb_slide_state_t
  TB_HOUSE_SLIDE1_AMPS = TB_HOUSE_SLIDE1_STATE + TB_HOUSE_SLIDES, // motor current
  TB_HOUSE_COUNT = TB_HOUSE_SLIDE1_AMPS + TB_HOUSE_SLIDES
};

typedef enum {
  TB_SLIDE_UNKNOWN = 0,
  TB_SLIDE_IN = 1,
  TB_SLIDE_OUT = 2,
  TB_SLIDE_MOVING_IN = 3,
  TB_SLIDE_MOVING_OUT = 4,
  TB_SLIDE_FAULT = 5,
} tb_slide_state_t;

// Lookup. Returns NULL for an id this build doesn't know (a newer node).
const tb_sigdef_t* tb_sig(uint8_t node, uint8_t id);
uint8_t tb_sig_count(uint8_t node);
const tb_sigdef_t* tb_sig_by_key(uint8_t node, const char* key);

// Names for J1587 parameters and MIDs (from the TIM's table; same scaling notes apply).
typedef struct {
  uint16_t pid;
  const char* name;
  const char* unit;
} tb_j1587_name_t;

const tb_j1587_name_t* tb_j1587_pid(uint16_t pid);  // NULL = unknown
const tb_j1587_name_t* tb_j1587_pid_at(size_t i);   // the known PIDs in order; NULL past the end
const char* tb_j1587_mid_name(uint8_t mid);         // never NULL
const char* tb_j1587_fmi_text(uint8_t fmi);         // never NULL

#ifdef __cplusplus
}
#endif
