# STM32U575 USB-PD 3.1 EPR Sink

Bare-metal USB Power Delivery sink firmware for STM32U575, built for
STM32CubeIDE. This branch is the PD 3.1 EPR version: it negotiates standard
SPR contracts first, enters EPR when the source supports it, and requests a
28 V contract with timer-assisted EPR keepalive servicing.

This project intentionally does not include IAR/EWARM project files. The
supported project format is STM32CubeIDE / STM32CubeMX (`.project`,
`.cproject`, and `u.ioc`).

## Branches

The repository keeps two clear product branches:

- `pd3.0-spr-lite`: lightweight PD 3.0 SPR-only sink. This is the default
  branch for simple 5 V / 9 V / 12 V / 15 V / 20 V operation.
- `pd3.1-epr-timer-keepalive`: PD 3.1 EPR sink. This branch adds EPR entry,
  chunked EPR Source Capabilities handling, 28 V selection, and TIM16-assisted
  keepalive scheduling.

Version tags:

- `pd3.0-spr-lite-v1.0.0`
- `pd3.1-epr-timer-keepalive-v1.0.0`

## Hardware Target

- MCU: STM32U575
- Board profile: NUCLEO-U575ZI-Q
- USB-C/PD peripheral: UCPD1
- Toolchain: STM32CubeIDE GCC
- UART log port: COM1 at 115200 baud
- Timer used for PD service timing: TIM16
- HAL time base: TIM17

The CubeMX configuration in `u.ioc` enables:

- UCPD1 sink mode
- GPDMA1 channels for UCPD TX/RX
- TIM16 periodic interrupt for EPR keepalive service wakeup
- USART1 through the Nucleo VCP for minimal status output

## What This Firmware Does

The sink negotiates power in this order:

1. Attach as USB-C sink.
2. Receive SPR Source Capabilities.
3. If EPR is supported and enabled, first request the 20 V SPR contract needed
   before EPR entry.
4. Send EPR Mode Enter with enough sink PDP budget for 28 V at 5 A.
5. Receive chunked EPR Source Capabilities and request the configured EPR PDO.
6. Maintain EPR using periodic EPR KeepAlive messages.
7. Fall back to the best available SPR contract if EPR is not available or is
   rejected for the current source.

The current application profile table in `Core/Src/main.c` is:

| Profile | Voltage | Requested current | Role |
| --- | ---: | ---: | --- |
| 0 | 5 V | 0.5 A | baseline SPR |
| 1 | 9 V | 1 A | SPR fallback |
| 2 | 12 V | 1 A | SPR fallback |
| 3 | 20 V | 1 A minimum | SPR fallback / EPR entry base |
| 4 | 28 V | 3 A minimum | preferred EPR target |

When a 28 V fixed EPR PDO provides more current, the stack requests the source
PDO current. With a 28 V / 5 A source, the reported contract is:

```text
USB-PD ready: profile=4, voltage=28000 mV, current=5000 mA
```

## Runtime Output

Debug traces are disabled in the application. The firmware only prints the
high-level status lines:

```text
USB-PD ready: profile=4, voltage=28000 mV, current=5000 mA
USB-PD fallback: profile=3, voltage=20000 mV, current=5000 mA
USB-PD detached
```

The PD core still has an optional trace callback in `PD_BM_Config`, but this
application sets it to `NULL` so UART printing cannot disturb PD timing.

## EPR Stability Notes

USB-PD EPR timing is strict. The implementation avoids blocking UART output in
the PD negotiation path and keeps EPR service work short:

- GoodCRC is sent immediately for received non-GoodCRC PD messages.
- Chunk requests for EPR Source Capabilities are queued and sent from the main
  PD task, inside the PD chunk timing window.
- EPR KeepAlive is scheduled by the PD task and woken by TIM16.
- TIM16 does not transmit PD packets from the interrupt. It only wakes the
  service logic so the main loop can send keepalive safely.
- Physical detach is checked while connected, not only during negotiation.
- EPR failure suppression is tied to the current source capability set. If a
  20 V-only source fails EPR and a different 28 V source is attached next, EPR
  is retried instead of waiting for a long hard-reset cycle.

## Building With STM32CubeIDE

1. Open STM32CubeIDE.
2. Choose `File > Import > Existing Projects into Workspace`.
3. Select this repository folder.
4. Import the project `u`.
5. Build the `Debug` configuration.

From a CubeIDE toolchain shell, the same build is equivalent to:

```powershell
cd Debug
make -j8 all
```

The output ELF is generated as:

```text
Debug/u.elf
```

## Flashing

Use STM32CubeIDE with the included `u.launch` debug configuration, or flash the
generated `Debug/u.elf` with your normal ST-LINK workflow.

After flashing:

1. Open the Nucleo VCP at 115200 baud.
2. Connect a USB-C PD source.
3. Watch for either a `ready`, `fallback`, or `detached` status line.

## Source Layout

Important files:

- `Core/Src/main.c`: board init, profile table, UART status reporting, TIM16
  start, PD service loop.
- `Core/Src/pd_bm.c`: bare-metal USB-PD sink state machine with PD 3.1 EPR
  additions.
- `Core/Inc/pd_bm.h`: public PD core interface and configuration types.
- `Core/Src/stm32u5xx_it.c`: UCPD, GPDMA, TIM16, and TIM17 interrupt handlers.
- `u.ioc`: STM32CubeMX/CubeIDE project configuration.
- `STM32U575ZITXQ_FLASH.ld`: flash linker script.

Generated build artifacts under `Debug/` are not part of the source design.

## Configuration

Change requested profiles in `Core/Src/main.c`:

```c
pd_config.profiles[0] = (PD_BM_Profile){ 5000U, 500U, 1U };
pd_config.profiles[1] = (PD_BM_Profile){ 9000U, 1000U, 1U };
pd_config.profiles[2] = (PD_BM_Profile){ 12000U, 1000U, 1U };
pd_config.profiles[3] = (PD_BM_Profile){ 20000U, 1000U, 1U };
pd_config.profiles[4] = (PD_BM_Profile){ 28000U, 3000U, 1U };
```

Notes:

- Any enabled profile above 20 V makes the sink attempt EPR.
- The current field is the minimum acceptable current for that profile.
- For a matching fixed EPR PDO, the PD core reports and requests the available
  source PDO current.

## Limitations

- This is a sink-only implementation.
- The preferred EPR target is fixed 28 V. It does not implement a generic UI
  for selecting arbitrary 36 V or 48 V PDOs.
- PPS/AVS policy control is not exposed as an application feature.
- The implementation is intentionally compact and bare-metal. It is not the ST
  USBPD middleware stack.

## License

Copyright (c) 2026 Naser Attarzadeh.

SPDX-License-Identifier: Apache-2.0
