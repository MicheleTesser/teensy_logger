# Third-Party Dependencies

This project tracks external dependencies as Git submodules under `third_party/`.

## Active in build

- `third_party/FreeRTOS-Kernel`
  - RTOS kernel used by the firmware build.
- `third_party/fatfs`
  - FatFs core sources used by the firmware build (`ff.c`, `diskio.c`, `ffsystem.c`, NXP SD bridge).

## Staged for integration

- `third_party/tinyusb`
  - Added as managed dependency for future USB stack migration.
  - Current firmware still uses the MCUXpresso USB device stack in `middleware/usb`.
- `third_party/mdflib`
  - MF4/MDF4 reference library tracked as submodule for format interoperability and future migration.
  - Current firmware keeps a lightweight in-target MF4 writer optimized for embedded logging.

## Notes

- Local duplicate copies (`freertos/`, `fatfs/`) were removed to keep a single source of truth.
- Update submodules after clone:

```bash
git submodule update --init --recursive
```
