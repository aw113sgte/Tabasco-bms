# Deprecated: StamPLC SCIB ESP-NOW Interface

This document is retained for reference only. The active transport for `StamPLC_SCIB.ino` is now UDP over Wi-Fi.

Use:
- `SCIBarduino/StamPLC_SCIB/udp_interface.md`

Sketch path:
- `SCIBarduino/StamPLC_SCIB/StamPLC_SCIB.ino`

## Current Assumptions

1. `Relay 1` is PLC relay channel `0` and switches CMU/K-line/12 V comm power.
2. `Relay 2` is PLC relay channel `1` and is treated as logical `contactor enabled`.
3. The relay-2 wiring assumption is: energizing relay 2 enables the external contactor path, de-energized leaves the NC disable path active.
4. CMU command family default is fixed to `0x0A`.
5. Balance-all default is `ON`.
6. Keepalive default is `ON`.
7. ESP-NOW is locked to Wi-Fi channel `1`.
8. Any remote ESP32 that should receive or send commands must also be fixed to channel `1`.

## Wiring Used By The Sketch

1. `CMU CAN` uses the StampPLC dedicated CAN port through `M5StamPLC` / TWAI on pins `TX=42`, `RX=43`.
2. `CMU naming UART` uses Port A:
- `G1` as UART TX to K-line transmitter input
- `G2` as UART RX from K-line receiver output
- UART mode `1200 8E1`
3. `Victron VE.Direct` uses Port C:
- `G5` as RX from SmartShunt TX
- `G4` as TX to SmartShunt RX
- UART mode `19200 8N1`

## Physical Button Behavior

1. `A` wakes the screen.
2. `B` toggles comm power relay 1. Turning it on also runs the CMU naming sequence.
3. `C` toggles contactor relay 2.

## Broadcast Packet

Broadcast interval:
- every `1000 ms`

Destination:
- ESP-NOW broadcast `FF:FF:FF:FF:FF:FF`
- Wi-Fi channel `1`

Format:
- JSON text

Example:

```json
{"type":"status","cmus_uart":4,"cmus_can":4,"v_hi_mv":27341,"v_avg_mv":27295,"v_lo_mv":27224,"t_hi_df":812,"t_avg_df":784,"t_lo_df":753,"soc_dpct":726,"cur_ma":12340,"power":1,"contactor":0,"bal_all":0,"family":"0A","screen":1,"delta_max_mv":52,"status":"Naming OK: 4 CMUs"}
```

Field units:

1. `cmus_uart`: CMU count from UART naming reply.
2. `cmus_can`: online CMU count currently seen on CAN.
3. `v_hi_mv`, `v_avg_mv`, `v_lo_mv`: module total voltage high/average/low in `mV`.
4. `t_hi_df`, `t_avg_df`, `t_lo_df`: temperature high/average/low in `0.1 F`.
5. `soc_dpct`: state of charge in `0.1 %`.
6. `cur_ma`: current in `mA`.
7. `power`: relay-1 logical state, `1=on`.
8. `contactor`: relay-2 logical state, `1=enabled`.
9. `bal_all`: CMU balance-all command state, `1=on`.
10. `family`: active configured command family.
11. `screen`: LCD backlight state, `1=on`.
12. `delta_max_mv`: largest live module cell delta in `mV`.
13. `status`: short status string from the StampPLC.

Invalid numeric values are reported as `-1`.

## Commands

Commands are plain ASCII text sent as the ESP-NOW payload.

Supported commands:

1. `STATUS`
- returns one unicast status JSON packet.

2. `GET`
- alias for `STATUS`.

3. `MODULE <slot>`
- returns one unicast module-detail JSON packet.
- `slot` is `1..16`.

Example:

```text
MODULE 3
```

Example reply:

```json
{"type":"module","slot":3,"can":3,"online":1,"status_word":0,"total_mv":27284,"avg_cell_mv":2274,"delta_mv":18,"avg_temp_df":781,"cells_mv":[2274,2273,2274,2275,2273,2274,2274,2273,2275,2274,2273,2274],"temps_df":[780,781,779,782,783,781]}
```

Reply field units:

1. `total_mv`: module total voltage in `mV`
2. `avg_cell_mv`: average cell voltage in `mV`
3. `delta_mv`: module cell spread in `mV`
4. `avg_temp_df`: average module temperature in `0.1 F`
5. `cells_mv`: 12 cell voltages in `mV`
6. `temps_df`: 6 temperatures in `0.1 F`

4. `POWER ON`
5. `POWER OFF`
6. `POWER TOGGLE`
- toggles relay 1.
- `POWER ON` also runs UART naming and restarts CMU CAN comms.

7. `CONTACTOR ON`
8. `CONTACTOR OFF`
9. `CONTACTOR TOGGLE`
- toggles relay 2 logical state.

10. `BAL ON`
11. `BAL OFF`
12. `BAL TOGGLE`
- controls `balance-all`.

13. `KEEP ON`
14. `KEEP OFF`
15. `KEEP TOGGLE`
- controls periodic keepalive traffic when balancing is off.

16. `FAMILY AUTO`
17. `FAMILY 09`
18. `FAMILY 0A`
- changes the configured CMU command family.

19. `TEMPOFFSET <degC>`
- changes the applied CMU temperature offset before Fahrenheit conversion.

Example:

```text
TEMPOFFSET 10.0
```

20. `NAME`
- reruns the CMU UART naming sequence.
- requires comm power to already be on.

21. `SCREEN ON`
22. `SCREEN OFF`
23. `SCREEN TOGGLE`
- controls the LCD backlight state.

## ACK Replies

State-changing commands return a small ACK JSON reply.

Example:

```json
{"type":"ack","cmd":"BAL","ok":1,"msg":"on"}
```

## CMU Naming Notes

The sketch verifies the UART return rather than assuming naming worked.

Expected reply format:

1. `8 bytes` total
2. `byte 0` is `0x00`
3. `bytes 1..6` are the CMU presence bitmap
4. `byte 7` is the 8-bit sum of bytes `0..6`

Examples from captured traces:

1. `1 CMU` -> `00 01 00 00 00 00 00 01`
2. `4 CMUs` -> `00 55 00 00 00 00 00 55`
3. `6 CMUs` -> `00 55 05 00 00 00 00 5A`

The sketch counts set bits across bytes `1..6` to derive `cmus_uart`.

## Follow-Up Options

If you want the next revision to go further, the most useful options are:

1. Add per-module balance targeting over ESP-NOW instead of balance-all only.
2. Add a second screen page for cell-level local display on the StampPLC.
3. Add a paired receiver sketch that is preconfigured for ESP-NOW channel `1`.
