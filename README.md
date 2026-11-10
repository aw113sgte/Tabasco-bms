# Tabasco BMS

Firmware for the Tabasco LTO battery system.

| Folder | Contents |
|---|---|
| `firmware/BMS_v2/` | Arduino sketch for the v2 BMS controller board (ESP32-S3 DevKitC-1, ESP-NOW) and its serial/command interface doc |
| `firmware/StamPLC_SCIB/` | M5Stack StamPLC sketch for the SCIB link, with ESP-NOW and UDP interface docs |

Each folder is an Arduino sketch folder: open the `.ino` in the Arduino IDE.

`firmware/BMS_v2/` includes a copy of the shared Tabasco link (`tb_*.c` / `tb_*.h`, master copy in `Modules/Common/tabasco_link`). The network key is not in the repo: copy `tb_netkey.example.h` to `tb_netkey.h` and put the truck's key in it before building.
