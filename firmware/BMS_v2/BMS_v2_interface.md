# BMS controller v2: firmware notes and remote interface

Sketch: `SCIBarduino/BMS_v2/BMS_v2.ino`. Board: ESP32-S3 DevKitC-1 on the v2 PCB, Waveshare 2" ST7789 LCD.
Build: Arduino ESP32 core 3.3.x, board "ESP32S3 Dev Module", libraries M5GFX and SCIBCMUComs.

## Start-up state

- Fans 0 % (gate held high, so the fan PWM line is low). They run full speed only while the ESP is in reset or booting.
- CMU 12 V off (GPIO8 low), so the CAN transceiver is in standby.
- EMS300 contact open (GPIO9 low).

## CMU naming

Ported unchanged from `StamPLC_SCIB.ino`: CAN `0x3C3` arm frames every 120 ms for 1400 ms, one more arm frame,
60 ms gap, the 8 × `0x00` frame at 1200 8E1 on the K-line (GPIO15), arm held until the reply arrives on the naming
return (GPIO16) or 4000 ms, then 8 disarm frames, then the CMU manager starts with a 6000 ms TX hold-off.
Turning CMU power on waits 700 ms for the rail, then runs naming.

## Buttons

| Page | A | hold A | B | C |
|---|---|---|---|---|
| Main | wake | debug page | CMU power on/off | contactor (EMS300) on/off |
| Debug | next item | main page | decrease / off | increase / on / run |

A press while the screen is dark only wakes it. The screen sleeps after 5 minutes.

Debug items: CMU power, rerun naming, EMS300 enable, fan 1, fan 2 (10 % steps), both fans, balance all, backlight.
The debug page also shows both temperature probes (DS18B20 or NTC, detected automatically), fan rpm, EMS300 LED
input, 24 V input, CMU current-sense voltage, CAN, VE.Direct and ESP-NOW counters.

## ESP-NOW

Channel 1, unencrypted. Every second the board broadcasts two JSON packets (each under 250 bytes):

```json
{"type":"status","cmus_uart":4,"cmus_can":4,"v_hi_mv":27341,"v_avg_mv":27295,"v_lo_mv":27224,"t_hi_df":812,"t_avg_df":784,"t_lo_df":753,"delta_max_mv":52,"soc_dpct":726,"cur_ma":12340,"batt_mv":32400}
{"type":"board","power":1,"contactor":0,"ems_led":0,"bal_all":1,"family":"0A","t_batt_dc":253,"t_air_dc":221,"fan_pct":[0,0],"fan_rpm":[0,0],"v24_mv":32400,"status":"Naming OK: 4 CMUs"}
```

`t_batt_dc` and `t_air_dc` are the board's own probes in 0.1 °C (-1000 when absent). Other units are as in the old
StamPLC interface. Invalid values are -1.

Commands are plain ASCII sent to the board's MAC (or broadcast). Replies go back to the sender. The same commands
work on the USB serial port at 115200 (add `HELP`).

| Command | Effect |
|---|---|
| `STATUS` / `GET` | both status packets |
| `MODULE n` | `module` packet (cells) and `module_t` packet (temperatures) for slot 1 to 16 |
| `POWER ON\|OFF\|TOGGLE` | CMU 12 V; ON also runs naming |
| `CONTACTOR ON\|OFF\|TOGGLE` (or `EMS`) | EMS300 enable |
| `FAN 1\|2\|ALL pct` | fan speed 0 to 100 |
| `BAL`, `KEEP`, `FAMILY`, `TEMPOFFSET`, `NAME`, `SCREEN` | as on the StamPLC |
| `ROTATE [0-3]` | display rotation (saved); no argument flips 180° |
| `PAGE MAIN\|DEBUG` | switch screen page |
| `VE [RAW on\|off]` | USB serial only: VE.Direct counters and field ages; RAW echoes every line received |

## Tabasco link (HMI panels)

Alongside the JSON, the board speaks the encrypted Tabasco link used by the HMI panels
(`Modules/Common/tabasco_link`, copied into this sketch as `tb_*.c` / `tb_*.h`; run `Modules/Common/sync_common.sh`
after changing the shared files). AES-128-GCM with a network key shared by every node; frames carry a boot number
and counter, so old frames can't be replayed.

- DATA: every value in `tb_signals.h` (BMS list), changes at most every 200 ms and a full refresh every 5 s.
- TEXT: the status line, when it changes and every 5 s.
- BMS_MODULE: one module's cells and temperatures every 125 ms, only while a panel is showing the Battery page.
- Commands from the panels: CMU power, contactor, balance, fans, rerun naming. Each is answered with an ACK as soon as
  it's checked (before it runs, since power-on and naming take seconds); a repeated command is answered again but not
  run twice.

Unencrypted ESP-NOW commands that switch something (POWER, CONTACTOR, FAN, BAL, NAME, ...) are now refused
(`unencrypted_control_refused`); STATUS, GET and MODULE still work. While an old TabascoMaster panel still needs
to control the BMS, type `PLAINCMD ON` on USB serial (saved; `PLAINCMD OFF` to lock it again).

USB serial: `NETKEY` shows the link counters; `NETKEY <32 hex digits>` sets the network key (must match the TIM and
every panel).

ESP-NOW receive now queues up to 8 packets instead of dropping everything that arrives while one command waits.
