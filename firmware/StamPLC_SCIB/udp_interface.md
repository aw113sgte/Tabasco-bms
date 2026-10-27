# StamPLC SCIB UDP Interface

Sketch path:
- `SCIBarduino/StamPLC_SCIB/StamPLC_SCIB.ino`

This sketch now uses normal Wi-Fi plus UDP instead of ESP-NOW.

## Network Model

1. Default mode is `WIFI_STA` to the configured router.
2. If the router disappears, the node enters `RECONNECTING` and keeps retrying the saved SSID.
3. After `120 s` without reconnect, the node starts a fallback AP while continuing periodic router retries.
4. If the saved router comes back, the node automatically leaves recovery AP and rejoins the router.

Default credentials in code:
- SSID: empty
- Password: empty
- If no router is provisioned, the node can join `SCIB-Master-*` recovery AP or start its own recovery AP.

Recovery AP:
- SSID pattern: `SCIB-PLC-XXXXXX`
- Password: `scibsetup`

## UDP Transport

UDP port:
- `40120`

Broadcast interval:
- every `1000 ms`

Broadcast payload:
- JSON text status packet to UDP broadcast on the local subnet

Status example:

```json
{"type":"status","node_id":"plc-12AB34","name":"StampPLC_SCIB","ip":"192.168.1.50","net_state":"CONNECTED","cmus_uart":4,"cmus_can":4,"v_hi_mv":27341,"v_avg_mv":27295,"v_lo_mv":27224,"t_hi_df":812,"t_avg_df":784,"t_lo_df":753,"soc_dpct":726,"cur_ma":12340,"power":1,"contactor":0,"bal_all":1,"family":"0A","screen":1,"delta_max_mv":52,"status":"Naming OK: 4 CMUs"}
```

Key fields:
- `node_id`: stable node identifier
- `ip`: current node IP address
- `net_state`: `CONNECTED`, `RECONNECTING`, or `RECOVERY_AP`
- `cmus_uart`, `cmus_can`: UART-named and CAN-online counts
- `v_*_mv`: module voltage high/average/low in `mV`
- `t_*_df`: temperature high/average/low in `0.1 F`
- `soc_dpct`: state of charge in `0.1 %`
- `cur_ma`: current in `mA`
- `power`, `contactor`, `bal_all`, `screen`: boolean state flags as `0/1`
- `family`: current CMU command family
- `delta_max_mv`: largest live module delta in `mV`
- `status`: short human status line

Invalid numeric values are reported as `-1`.

## Commands

Commands are plain ASCII text sent as UDP payloads to port `40120`.

Supported commands:

1. `STATUS`
2. `GET`
- returns one direct status JSON reply

3. `MODULE <slot>`
- returns one module-detail JSON reply for `slot 1..16`

Example reply:

```json
{"type":"module","slot":3,"can":3,"online":1,"status_word":0,"total_mv":27284,"avg_cell_mv":2274,"delta_mv":18,"avg_temp_df":781,"cells_mv":[2274,2273,2274,2275,2273,2274,2274,2273,2275,2274,2273,2274],"temps_df":[780,781,779,782,783,781]}
```

4. `POWER ON|OFF|TOGGLE`
5. `CONTACTOR ON|OFF|TOGGLE`
6. `BAL ON|OFF|TOGGLE`
7. `BALANCE ON|OFF|TOGGLE`
8. `KEEP ON|OFF|TOGGLE`
9. `FAMILY AUTO|09|0A`
10. `TEMPOFFSET <degC>`
11. `NAME`
12. `SCREEN ON|OFF|TOGGLE`
13. `NETCFG <ssid><TAB><password>`

`NETCFG` stores new Wi-Fi credentials, saves them to preferences, and immediately starts reconnecting with the new router settings.

## ACK Replies

State-changing commands return a direct ACK JSON reply.

Example:

```json
{"type":"ack","node_id":"plc-12AB34","cmd":"BAL","ok":1,"msg":"on"}
```

## Wiring Used By The Sketch

1. `CMU CAN` uses the StampPLC dedicated CAN port through `M5StamPLC` / TWAI on pins `TX=42`, `RX=43`.
2. `CMU naming UART` uses Port A:
- `G1` as UART TX to K-line transmitter input
- `G2` as UART RX from K-line receiver output
- UART mode `1200 8E1`
- The CMU return path must idle high. In the working StampPLC setup, the naming return line needed a pull-up / bias resistor; without that bias the CMUs did not return a decodable naming reply even when CAN arm and TX UART were correct.
3. `Victron VE.Direct` uses Port C:
- `G4` as RX from SmartShunt TX
- `G5` as TX to SmartShunt RX
- UART mode `19200 8N1`
- For simple monitoring, no polling or keepalive is required. Victron text-mode devices automatically transmit their readings about once per second.
