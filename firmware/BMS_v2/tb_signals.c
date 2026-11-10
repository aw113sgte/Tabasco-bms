// Tabasco link: signal and J1587 name tables. See tb_signals.h.
#include "tb_signals.h"

#include <string.h>

#define DEGF "\xC2\xB0" "F"

// node, id, kind, decimals, deadband, key, name, unit
static const tb_sigdef_t kTim[TB_TIM_COUNT] = {
    {TB_NODE_TIM, TB_TIM_V1437, TB_SK_NUMBER, 2, 5, "v1437", "Truck battery 24 V", "V"},
    {TB_NODE_TIM, TB_TIM_V1139, TB_SK_NUMBER, 2, 5, "v1139", "Starter motor B+", "V"},
    {TB_NODE_TIM, TB_TIM_V1138, TB_SK_NUMBER, 2, 5, "v1138", "Starter - / shunt top", "V"},
    {TB_NODE_TIM, TB_TIM_V1431, TB_SK_NUMBER, 2, 5, "v1431", "Crank command", "V"},
    {TB_NODE_TIM, TB_TIM_V1274, TB_SK_NUMBER, 2, 5, "v1274", "Alternator B+", "V"},
    {TB_NODE_TIM, TB_TIM_V2010, TB_SK_NUMBER, 2, 5, "v2010", "Alternator field", "V"},
    {TB_NODE_TIM, TB_TIM_V1435A, TB_SK_NUMBER, 2, 5, "v1435a", "Alternator ground sense", "V"},
    {TB_NODE_TIM, TB_TIM_V1435B, TB_SK_NUMBER, 2, 5, "v1435b", "Shunt-end ground", "V"},
    {TB_NODE_TIM, TB_TIM_V2085, TB_SK_NUMBER, 2, 5, "v2085", "Diag branch C60-E", "V"},
    {TB_NODE_TIM, TB_TIM_SHUNT, TB_SK_NUMBER, 0, 5, "shunt", "Starter current (+ = discharge)", "A"},
    {TB_NODE_TIM, TB_TIM_SENS5V, TB_SK_NUMBER, 2, 5, "sens5v", "Sensor supply", "V"},
    {TB_NODE_TIM, TB_TIM_C59_PSIA, TB_SK_NUMBER, 2, 5, "c59_psia", "Fuel inlet pressure", "psia"},
    {TB_NODE_TIM, TB_TIM_FUEL_VAC, TB_SK_NUMBER, 2, 5, "fuel_vac", "Fuel inlet vacuum", "psi"},
    {TB_NODE_TIM, TB_TIM_OIL_IN, TB_SK_NUMBER, 1, 5, "oil_in", "Oil filter inlet", "psi"},
    {TB_NODE_TIM, TB_TIM_OIL_OUT, TB_SK_NUMBER, 1, 5, "oil_out", "Oil filter outlet", "psi"},
    {TB_NODE_TIM, TB_TIM_OIL_DP, TB_SK_NUMBER, 1, 5, "oil_dp", "Oil filter drop", "psi"},
    {TB_NODE_TIM, TB_TIM_AIR, TB_SK_NUMBER, 1, 2, "air", "Air cleaner restriction", "inH2O"},
    {TB_NODE_TIM, TB_TIM_AIR_PSIA, TB_SK_NUMBER, 2, 5, "air_psia", "Air cleaner pressure", "psia"},
    {TB_NODE_TIM, TB_TIM_C58, TB_SK_BOOL, 0, 1, "c58", "Fuel separator switch open", ""},
    {TB_NODE_TIM, TB_TIM_ENGINE, TB_SK_ENUM, 0, 1, "engine", "Engine", ""},
    {TB_NODE_TIM, TB_TIM_RPM, TB_SK_NUMBER, 0, 10, "rpm", "Engine speed", "rpm"},
};

static const tb_sigdef_t kBms[TB_BMS_COUNT] = {
    {TB_NODE_BMS, TB_BMS_CMUS_UART, TB_SK_NUMBER, 0, 1, "cmus_uart", "CMUs named", ""},
    {TB_NODE_BMS, TB_BMS_CMUS_CAN, TB_SK_NUMBER, 0, 1, "cmus_can", "CMUs on CAN", ""},
    {TB_NODE_BMS, TB_BMS_MOD_V_HI, TB_SK_NUMBER, 3, 20, "mod_v_hi", "Module voltage high", "V"},
    {TB_NODE_BMS, TB_BMS_MOD_V_AVG, TB_SK_NUMBER, 3, 20, "mod_v_avg", "Module voltage avg", "V"},
    {TB_NODE_BMS, TB_BMS_MOD_V_LO, TB_SK_NUMBER, 3, 20, "mod_v_lo", "Module voltage low", "V"},
    {TB_NODE_BMS, TB_BMS_T_HI, TB_SK_NUMBER, 1, 10, "t_hi", "Battery temp high", DEGF},
    {TB_NODE_BMS, TB_BMS_T_AVG, TB_SK_NUMBER, 1, 10, "t_avg", "Battery temp avg", DEGF},
    {TB_NODE_BMS, TB_BMS_T_LO, TB_SK_NUMBER, 1, 10, "t_lo", "Battery temp low", DEGF},
    {TB_NODE_BMS, TB_BMS_DELTA_MAX, TB_SK_NUMBER, 0, 5, "delta_max", "Cell spread", "mV"},
    {TB_NODE_BMS, TB_BMS_SOC, TB_SK_NUMBER, 1, 5, "soc", "State of charge", "%"},
    {TB_NODE_BMS, TB_BMS_CURRENT, TB_SK_NUMBER, 3, 200, "current", "Battery current (+ = charging)", "A"},
    {TB_NODE_BMS, TB_BMS_BATT_V, TB_SK_NUMBER, 3, 50, "batt_v", "House battery", "V"},
    {TB_NODE_BMS, TB_BMS_POWER, TB_SK_BOOL, 0, 1, "power", "CMU power", ""},
    {TB_NODE_BMS, TB_BMS_CONTACTOR, TB_SK_BOOL, 0, 1, "contactor", "Contactor", ""},
    {TB_NODE_BMS, TB_BMS_EMS_LED, TB_SK_BOOL, 0, 1, "ems_led", "EMS300 LED", ""},
    {TB_NODE_BMS, TB_BMS_BAL_ALL, TB_SK_BOOL, 0, 1, "bal_all", "Balance", ""},
    {TB_NODE_BMS, TB_BMS_T_BOARD, TB_SK_NUMBER, 1, 10, "t_board", "BMS battery probe", DEGF},
    {TB_NODE_BMS, TB_BMS_T_AIR, TB_SK_NUMBER, 1, 10, "t_air", "BMS air probe", DEGF},
    {TB_NODE_BMS, TB_BMS_FAN1_PCT, TB_SK_NUMBER, 0, 1, "fan1_pct", "Fan 1", "%"},
    {TB_NODE_BMS, TB_BMS_FAN2_PCT, TB_SK_NUMBER, 0, 1, "fan2_pct", "Fan 2", "%"},
    {TB_NODE_BMS, TB_BMS_FAN1_RPM, TB_SK_NUMBER, 0, 50, "fan1_rpm", "Fan 1 speed", "rpm"},
    {TB_NODE_BMS, TB_BMS_FAN2_RPM, TB_SK_NUMBER, 0, 50, "fan2_rpm", "Fan 2 speed", "rpm"},
    {TB_NODE_BMS, TB_BMS_V24, TB_SK_NUMBER, 3, 50, "v24", "BMS supply", "V"},
};

static const tb_sigdef_t kHouse[TB_HOUSE_COUNT] = {
    {TB_NODE_HOUSE, TB_HOUSE_TANK1 + 0, TB_SK_NUMBER, 0, 1, "tank1", "Tank 1", "%"},
    {TB_NODE_HOUSE, TB_HOUSE_TANK1 + 1, TB_SK_NUMBER, 0, 1, "tank2", "Tank 2", "%"},
    {TB_NODE_HOUSE, TB_HOUSE_TANK1 + 2, TB_SK_NUMBER, 0, 1, "tank3", "Tank 3", "%"},
    {TB_NODE_HOUSE, TB_HOUSE_TANK1 + 3, TB_SK_NUMBER, 0, 1, "tank4", "Tank 4", "%"},
    {TB_NODE_HOUSE, TB_HOUSE_TANK1 + 4, TB_SK_NUMBER, 0, 1, "tank5", "Tank 5", "%"},
    {TB_NODE_HOUSE, TB_HOUSE_TANK1 + 5, TB_SK_NUMBER, 0, 1, "tank6", "Tank 6", "%"},
    {TB_NODE_HOUSE, TB_HOUSE_SLIDE1_POS + 0, TB_SK_NUMBER, 0, 1, "slide1_pos", "Slide 1", "%"},
    {TB_NODE_HOUSE, TB_HOUSE_SLIDE1_POS + 1, TB_SK_NUMBER, 0, 1, "slide2_pos", "Slide 2", "%"},
    {TB_NODE_HOUSE, TB_HOUSE_SLIDE1_POS + 2, TB_SK_NUMBER, 0, 1, "slide3_pos", "Slide 3", "%"},
    {TB_NODE_HOUSE, TB_HOUSE_SLIDE1_POS + 3, TB_SK_NUMBER, 0, 1, "slide4_pos", "Slide 4", "%"},
    {TB_NODE_HOUSE, TB_HOUSE_SLIDE1_STATE + 0, TB_SK_ENUM, 0, 1, "slide1_state", "Slide 1 state", ""},
    {TB_NODE_HOUSE, TB_HOUSE_SLIDE1_STATE + 1, TB_SK_ENUM, 0, 1, "slide2_state", "Slide 2 state", ""},
    {TB_NODE_HOUSE, TB_HOUSE_SLIDE1_STATE + 2, TB_SK_ENUM, 0, 1, "slide3_state", "Slide 3 state", ""},
    {TB_NODE_HOUSE, TB_HOUSE_SLIDE1_STATE + 3, TB_SK_ENUM, 0, 1, "slide4_state", "Slide 4 state", ""},
    {TB_NODE_HOUSE, TB_HOUSE_SLIDE1_AMPS + 0, TB_SK_NUMBER, 1, 2, "slide1_amps", "Slide 1 motor", "A"},
    {TB_NODE_HOUSE, TB_HOUSE_SLIDE1_AMPS + 1, TB_SK_NUMBER, 1, 2, "slide2_amps", "Slide 2 motor", "A"},
    {TB_NODE_HOUSE, TB_HOUSE_SLIDE1_AMPS + 2, TB_SK_NUMBER, 1, 2, "slide3_amps", "Slide 3 motor", "A"},
    {TB_NODE_HOUSE, TB_HOUSE_SLIDE1_AMPS + 3, TB_SK_NUMBER, 1, 2, "slide4_amps", "Slide 4 motor", "A"},
};

static const tb_sigdef_t* table(uint8_t node, uint8_t* count) {
  switch (node) {
    case TB_NODE_TIM:
      *count = TB_TIM_COUNT;
      return kTim;
    case TB_NODE_BMS:
      *count = TB_BMS_COUNT;
      return kBms;
    case TB_NODE_HOUSE:
      *count = TB_HOUSE_COUNT;
      return kHouse;
    default:
      *count = 0;
      return NULL;
  }
}

const tb_sigdef_t* tb_sig(uint8_t node, uint8_t id) {
  uint8_t n = 0;
  const tb_sigdef_t* t = table(node, &n);
  return (t && id < n) ? &t[id] : NULL;
}

uint8_t tb_sig_count(uint8_t node) {
  uint8_t n = 0;
  table(node, &n);
  return n;
}

const tb_sigdef_t* tb_sig_by_key(uint8_t node, const char* key) {
  uint8_t n = 0;
  const tb_sigdef_t* t = table(node, &n);
  for (uint8_t i = 0; t && key && i < n; i++) {
    if (strcmp(t[i].key, key) == 0) {
      return &t[i];
    }
  }
  return NULL;
}

// Same list as the TIM's j1587.cpp. Scalings live on the TIM; the panels only need names and units.
static const tb_j1587_name_t kPids[] = {
    {84, "Road speed", "mph"},
    {91, "Accelerator pedal", "%"},
    {92, "Engine load", "%"},
    {96, "Fuel level", "%"},
    {98, "Engine oil level", "%"},
    {100, "Oil pressure", "psi"},
    {102, "Boost pressure", "psi"},
    {105, "Intake manifold temp", DEGF},
    {110, "Coolant temp", DEGF},
    {111, "Coolant level", "%"},
    {168, "Battery potential", "V"},
    {171, "Ambient air temp", DEGF},
    {173, "Exhaust gas temp", DEGF},
    {175, "Engine oil temp", DEGF},
    {177, "Transmission oil temp", DEGF},
    {183, "Fuel rate", "gal/h"},
    {190, "Engine speed", "rpm"},
    {245, "Total distance", "mi"},
    {247, "Engine hours", "h"},
};

const tb_j1587_name_t* tb_j1587_pid(uint16_t pid) {
  for (size_t i = 0; i < sizeof(kPids) / sizeof(kPids[0]); i++) {
    if (kPids[i].pid == pid) {
      return &kPids[i];
    }
  }
  return NULL;
}

const tb_j1587_name_t* tb_j1587_pid_at(size_t i) {
  return i < sizeof(kPids) / sizeof(kPids[0]) ? &kPids[i] : NULL;
}

const char* tb_j1587_mid_name(uint8_t mid) {
  switch (mid) {
    case 128: return "Engine";
    case 130: return "Transmission";
    case 136: return "Brakes (ABS)";
    case 140: return "Instrument cluster";
    case 142: return "Vehicle management";
    default: return "Module";
  }
}

const char* tb_j1587_fmi_text(uint8_t fmi) {
  static const char* const kFmi[16] = {
      "above normal range", "below normal range", "erratic or intermittent data", "voltage high / shorted high",
      "voltage low / shorted low", "current low / open circuit", "current high / grounded",
      "mechanical system not responding", "abnormal frequency or pulse width", "abnormal update rate",
      "abnormal rate of change", "failure mode not identifiable", "bad device or component",
      "out of calibration", "special instructions", "reserved"};
  return kFmi[fmi & 0x0F];
}
