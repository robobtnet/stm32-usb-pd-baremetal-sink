# stm32-usb-pd-baremetal-sink

Minimal bare-metal USB-C Power Delivery sink for STM32 UCPD, with configurable PDO profiles and no RTOS or ST USBPD middleware.

This repository contains a tested STM32CubeIDE example for the NUCLEO-G474RE plus a small reusable PD sink core:

- `Core/Inc/pd_bm.h`
- `Core/Src/pd_bm.c`

The PD core is intentionally independent from board-specific details. It does not configure GPIO pins, clocks, NVIC, DMA channels, LEDs, COM ports, or HAL application logic. The application provides those through `PD_BM_Config`.

## Features

- Bare-metal USB-C PD sink, no FreeRTOS.
- No ST USBPD middleware dependency.
- Uses the STM32 UCPD peripheral.
- Configurable fixed PDO profiles.
- Tries optional profiles from highest index to lowest, then falls back to profile 0.
- Profile current is a minimum requirement. If the source advertises more current at the same voltage, the request uses the source current.
- RX uses an application-provided DMA callback.
- TX is handled directly through the UCPD peripheral.
- Optional DBCC/dead-battery handoff support in the application.

## Tested Hardware

The included example was tested on:

- Board: NUCLEO-G474RE
- Peripheral: `UCPD1`
- CC pins:
  - `PB6` -> `UCPD1_CC1`
  - `PB4` -> `UCPD1_CC2`
- Optional dead-battery pins in the `.ioc`:
  - `PA9` -> `UCPD1_DBCC1`
  - `PA10` -> `UCPD1_DBCC2`

The reusable PD core is not limited to this board. For another STM32 with UCPD, copy `pd_bm.h` and `pd_bm.c`, then provide the board glue shown below.

## What The Core Supports

This is a compact fixed-PDO sink policy engine. It is intended for simple projects that need to request a fixed USB-PD voltage such as 9 V, 12 V, 15 V, or 20 V.

Currently supported:

- Type-C sink attach detection through UCPD.
- `Source_Capabilities` parsing.
- Fixed PDO matching.
- `Request` message generation.
- `GoodCRC`, `Accept`, `Reject`, `Wait`, and `PS_RDY` handling.
- Fallback to lower enabled profiles.

Not implemented:

- PPS/APDO negotiation.
- EPR.
- Source mode.
- DRP mode.
- Cable discovery.
- Full ST USBPD middleware policy engine.

## Profile Selection

`profiles[0]` is the mandatory fallback and is always forced enabled by `PD_BM_Init`.

Example:

```c
pd_config.profiles[0] = (PD_BM_Profile){ 5000U, 500U, 1U };
pd_config.profiles[1] = (PD_BM_Profile){ 9000U, 1000U, 1U };
pd_config.profiles[2] = (PD_BM_Profile){ 12000U, 1000U, 1U };
pd_config.profiles[3] = (PD_BM_Profile){ 15000U, 1000U, 1U };
pd_config.profiles[4] = (PD_BM_Profile){ 20000U, 1000U, 1U };
```

The request order is:

```text
20 V -> 15 V -> 12 V -> 9 V -> 5 V fallback
```

If the profile is `9 V / 1 A` and the charger advertises `9 V / 2 A`, the source PDO matches and the request uses `2 A`.

## Runtime Status

After `PD_BM_GetState()` returns `PD_BM_STATE_READY`, the selected contract can be read with:

```c
uint8_t profile = PD_BM_GetActiveProfile();
uint16_t voltage_mv = PD_BM_GetRequestedVoltage();
uint16_t current_ma = PD_BM_GetRequestedCurrent();
```

`current_ma` is the negotiated source capability/current limit, not the real instantaneous load current. Measure the load current separately if your application needs actual consumption.

The NUCLEO-G474RE example prints the selected contract once over the board COM port:

```text
USB-PD ready: profile=2, voltage=12000 mV, current=3000 mA
```

## Porting Checklist

For a new STM32 project, you need these parts:

1. Copy the reusable core files:
   - `Core/Inc/pd_bm.h`
   - `Core/Src/pd_bm.c`
2. Configure UCPD and CC pins in CubeMX/CubeIDE.
3. Configure UCPD RX DMA.
4. Enable the UCPD interrupt.
5. Add the UCPD IRQ hook in `stm32xxxx_it.c`.
6. Add the board callbacks and `PD_BM_Config` setup in `main.c`.
7. Call `PD_BM_Task()` continuously in the main loop.
8. Disable UCPD dead-battery mode after `PD_BM_Init()` if your MCU/family supports it.

## CubeMX / CubeIDE Configuration

### 1. UCPD Peripheral

Enable one UCPD instance, for example `UCPD1`.

Recommended settings:

- UCPD mode: sink.
- Use LL driver for UCPD if CubeMX asks for HAL/LL selection. The core uses `LL_UCPD_InitTypeDef` and LL UCPD helper functions.
- Make sure the project includes the UCPD LL driver/header, for example `stm32g4xx_ll_ucpd.h`.
- Make sure the project includes DMA LL definitions if you use the callback code below, for example `stm32g4xx_ll_dma.h`.
- Do not enable ST USBPD middleware.
- Do not enable FreeRTOS only for this PD core.

Pinout:

- Connect the USB-C connector `CC1` to the MCU `UCPDx_CC1` pin.
- Connect the USB-C connector `CC2` to the MCU `UCPDx_CC2` pin.
- In CubeMX, select the CC pins as UCPD CC signals. On the tested G474 project:
  - `PB6` is `UCPD1_CC1`
  - `PB4` is `UCPD1_CC2`

Do not hardcode the tested pins for another board. Use the pins that your MCU package exposes for the selected UCPD instance.

### 2. DBCC / Dead-Battery Pins

DBCC is optional, but useful when the board is powered only from USB-C.

Enable DBCC if:

- The product starts with no other power source.
- You want the charger/source to see a sink pull-down before firmware starts.
- Your schematic actually connects the DBCC pins as required by the MCU datasheet.

On the tested G474 example:

- `PA9` is `UCPD1_DBCC1`
- `PA10` is `UCPD1_DBCC2`

The firmware disables dead-battery mode only after `PD_BM_Init()` has enabled UCPD sink handling:

```c
if (PD_BM_Init(&pd_config) == 0U)
{
  Error_Handler();
}
HAL_PWREx_DisableUCPDDeadBattery();
```

Do not disable dead-battery mode before UCPD is initialized on a board that is powered only from USB-C, because the source may briefly stop seeing Rd.

### 3. DMA

UCPD RX DMA is required by the current example glue code.

Add a DMA request for UCPD RX:

- Request: `UCPD1_RX` or `UCPD2_RX`
- Direction: peripheral to memory
- Mode: normal
- Peripheral increment: disabled
- Memory increment: enabled
- Peripheral data alignment: byte
- Memory data alignment: byte
- Priority: low or medium is fine

The tested project uses:

- `UCPD1_RX` -> `DMA1_Channel1`

TX DMA is not required by this PD core. The tested `.ioc` also has `UCPD1_TX` DMA configured, but the current `pd_bm.c` transmits directly through UCPD registers and does not use a TX DMA callback.

If you choose a different RX DMA channel, update the application callbacks in `main.c`.

### 4. NVIC / Interrupts

Enable the UCPD global interrupt:

- `UCPD1_IRQn` for `UCPD1`
- `UCPD2_IRQn` for `UCPD2`, if available on your MCU

DMA RX interrupt is not required by the PD core because message completion is detected by the UCPD `RXMSGEND` interrupt. It is harmless if CubeMX enables DMA IRQs, but the core does not depend on them.

### 5. Time Base

The application must provide a monotonic millisecond tick. In HAL projects, `HAL_GetTick()` is enough:

```c
static uint32_t PD_App_GetTick(void *user)
{
  (void)user;
  return HAL_GetTick();
}
```

For non-HAL projects, provide your own 1 ms tick function.

## Required Code In `stm32xxxx_it.c`

Include the PD core header in the interrupt file:

```c
/* USER CODE BEGIN Includes */
#include "pd_bm.h"
/* USER CODE END Includes */
```

Call `PD_BM_IRQHandler()` from the UCPD IRQ handler.

For `UCPD1`:

```c
void UCPD1_IRQHandler(void)
{
  /* USER CODE BEGIN UCPD1_IRQn 0 */
  PD_BM_IRQHandler();
  /* USER CODE END UCPD1_IRQn 0 */

  /* USER CODE BEGIN UCPD1_IRQn 1 */
  /* USER CODE END UCPD1_IRQn 1 */
}
```

For `UCPD2`, use the `UCPD2_IRQHandler()` generated for your MCU and call the same function inside it.

## Required Code In `main.c`

### 1. Include `pd_bm.h`

```c
/* USER CODE BEGIN Includes */
#include "pd_bm.h"
/* USER CODE END Includes */
```

`pd_bm.h` tries to auto-detect the available STM32 LL UCPD header with `__has_include`. For non-STM32G4 projects, you can still override the LL UCPD include before including `pd_bm.h`:

```c
#define PD_BM_STM32_LL_UCPD_HEADER "stm32xxxx_ll_ucpd.h"
#include "pd_bm.h"
```

Replace `stm32xxxx_ll_ucpd.h` with the correct header for your STM32 family. This define must be visible when compiling both `main.c` and `pd_bm.c`; if you use a compiler symbol instead of a source define, apply it project-wide.

### 2. Add RX DMA State

```c
/* USER CODE BEGIN PV */
static uint16_t pd_app_rx_dma_size;
/* USER CODE END PV */
```

### 3. Add Function Prototypes

```c
/* USER CODE BEGIN PFP */
static uint32_t PD_App_GetTick(void *user);
static void PD_App_RxDmaStart(uint8_t *buffer, uint16_t size, void *user);
static void PD_App_RxDmaStop(void *user);
static uint16_t PD_App_RxDmaCount(void *user);
/* USER CODE END PFP */
```

### 4. Add Board Glue Callbacks

This example is for `UCPD1_RX` on `DMA1_Channel1`, as used by the NUCLEO-G474RE example. Change the DMA instance, channel, clear flag, and UCPD instance for your board.

```c
/* USER CODE BEGIN 0 */
static uint32_t PD_App_GetTick(void *user)
{
  (void)user;
  return HAL_GetTick();
}

static void PD_App_RxDmaStart(uint8_t *buffer, uint16_t size, void *user)
{
  (void)user;

  pd_app_rx_dma_size = size;
  CLEAR_BIT(DMA1_Channel1->CCR, DMA_CCR_EN);
  while ((DMA1_Channel1->CCR & DMA_CCR_EN) != 0U)
  {
  }

  DMA1_Channel1->CPAR = (uint32_t)&UCPD1->RXDR;
  DMA1_Channel1->CMAR = (uint32_t)buffer;
  DMA1_Channel1->CNDTR = size;
  LL_DMA_ClearFlag_GI1(DMA1);
  SET_BIT(DMA1_Channel1->CCR, DMA_CCR_EN);
}

static void PD_App_RxDmaStop(void *user)
{
  (void)user;

  CLEAR_BIT(DMA1_Channel1->CCR, DMA_CCR_EN);
  while ((DMA1_Channel1->CCR & DMA_CCR_EN) != 0U)
  {
  }
}

static uint16_t PD_App_RxDmaCount(void *user)
{
  (void)user;
  return (uint16_t)(pd_app_rx_dma_size - DMA1_Channel1->CNDTR);
}
/* USER CODE END 0 */
```

When porting:

- Replace `UCPD1` with your selected UCPD instance.
- Replace `DMA1_Channel1` with your selected UCPD RX DMA channel.
- Replace `LL_DMA_ClearFlag_GI1(DMA1)` with the clear flag for your channel.
- Keep peripheral address as `&UCPDx->RXDR`.

### 5. Initialize The PD Core

Call this after Cube-generated peripheral initialization:

```c
MX_GPIO_Init();
MX_DMA_Init();
MX_UCPD1_Init();

/* USER CODE BEGIN 2 */
static LL_UCPD_InitTypeDef pd_ucpd_init;
static PD_BM_Config pd_config;

LL_UCPD_StructInit(&pd_ucpd_init);

pd_config.ucpd = UCPD1;
pd_config.ucpd_init = &pd_ucpd_init;
pd_config.get_tick_ms = PD_App_GetTick;
pd_config.rx_dma_start = PD_App_RxDmaStart;
pd_config.rx_dma_stop = PD_App_RxDmaStop;
pd_config.rx_dma_count = PD_App_RxDmaCount;
pd_config.user = NULL;
pd_config.get_source_cap_interval_ms = 500U;
pd_config.attach_debounce_ms = 40U;

pd_config.profiles[0] = (PD_BM_Profile){ 5000U, 500U, 1U };
pd_config.profiles[1] = (PD_BM_Profile){ 9000U, 1000U, 1U };
pd_config.profiles[2] = (PD_BM_Profile){ 12000U, 1000U, 1U };
pd_config.profiles[3] = (PD_BM_Profile){ 15000U, 1000U, 1U };
pd_config.profiles[4] = (PD_BM_Profile){ 20000U, 1000U, 1U };

if (PD_BM_Init(&pd_config) == 0U)
{
  Error_Handler();
}

HAL_PWREx_DisableUCPDDeadBattery();
/* USER CODE END 2 */
```

Notes:

- Do not call `PD_BM_Init()` inside the main loop.
- `PD_BM_Init()` does not wait for PD negotiation to finish. It only prepares UCPD and enables sink detection.
- Negotiation happens later while `PD_BM_Task()` runs in the main loop and UCPD interrupts are serviced.
- If your MCU family does not have `HAL_PWREx_DisableUCPDDeadBattery()`, use the equivalent LL function or omit it if dead-battery is not present.

### 6. Call `PD_BM_Task()` In The Main Loop

Minimal loop:

```c
while (1)
{
  PD_BM_Task();

  if (PD_BM_GetState() == PD_BM_STATE_READY)
  {
    uint8_t profile = PD_BM_GetActiveProfile();
    uint16_t voltage_mv = PD_BM_GetRequestedVoltage();
    uint16_t current_ma = PD_BM_GetRequestedCurrent();

    (void)profile;
    (void)voltage_mv;
    (void)current_ma;
  }
}
```

Optional one-time UART report:

```c
static uint8_t pd_reported_profile = 0xFFU;
static uint16_t pd_reported_voltage_mv = 0U;
static uint16_t pd_reported_current_ma = 0U;

PD_BM_Task();

if (PD_BM_GetState() == PD_BM_STATE_READY)
{
  uint8_t active_profile = PD_BM_GetActiveProfile();
  uint16_t voltage_mv = PD_BM_GetRequestedVoltage();
  uint16_t current_ma = PD_BM_GetRequestedCurrent();

  if ((active_profile != pd_reported_profile)
      || (voltage_mv != pd_reported_voltage_mv)
      || (current_ma != pd_reported_current_ma))
  {
    pd_reported_profile = active_profile;
    pd_reported_voltage_mv = voltage_mv;
    pd_reported_current_ma = current_ma;
    printf("USB-PD ready: profile=%u, voltage=%u mV, current=%u mA\r\n",
        active_profile, voltage_mv, current_ma);
  }
}
```

## Full Example Files To Check

In this repository, the important integration points are:

- `Core/Inc/pd_bm.h`: public PD core API.
- `Core/Src/pd_bm.c`: portable PD core implementation.
- `Core/Src/main.c`: board glue, DMA callbacks, profile config, main loop.
- `Core/Src/stm32g4xx_it.c`: UCPD IRQ hook.
- `pdo_bm.ioc`: CubeMX configuration for the tested NUCLEO-G474RE project.

## UCPD2 Or Different DMA Channel

If your project uses `UCPD2`:

- Set `pd_config.ucpd = UCPD2`.
- Use `&UCPD2->RXDR` in the RX DMA start callback.
- Enable `UCPD2_IRQn`.
- Call `PD_BM_IRQHandler()` from `UCPD2_IRQHandler()`.
- Configure DMA request `UCPD2_RX`.

If your RX DMA is not `DMA1_Channel1`, update:

- DMA channel enable/disable register access.
- `CPAR`, `CMAR`, and `CNDTR` channel references.
- DMA clear flag function.
- DMA request in CubeMX.

## STM32U5 / GPDMA Porting Notes

STM32U projects, such as STM32U575, can use the same `pd_bm.c` and `pd_bm.h`. The PD core does not need U-specific code. Only the application glue changes because STM32U5 uses GPDMA instead of the classic DMA channel registers used by the G474 example.

CubeMX/CubeIDE checklist for a U5 project:

- Enable the UCPD instance, for example `UCPD1`.
- Assign the real CC pins for your package. One tested U5 setup used `PA15 -> UCPD1_CC1` and `PB15 -> UCPD1_CC2`.
- Enable `GPDMA1`.
- Add a DMA request for `UCPD1_RX`, for example `GPDMA1 Channel 1` with request `LL_GPDMA1_REQUEST_UCPD1_RX`.
- Direction must be peripheral-to-memory.
- Source/peripheral increment disabled, destination/memory increment enabled.
- Source and destination data width must be byte.
- Enable `UCPD1_IRQn` and call `PD_BM_IRQHandler()` from `UCPD1_IRQHandler()`.
- TX DMA is not required by this core, even if CubeMX generated a `UCPD1_TX` DMA channel.

For STM32U5, the RX DMA callbacks look like this:

```c
/* USER CODE BEGIN PV */
static uint16_t pd_app_rx_dma_size;
/* USER CODE END PV */

/* USER CODE BEGIN 0 */
static void PD_App_RxDmaStart(uint8_t *buffer, uint16_t size, void *user)
{
  (void)user;

  pd_app_rx_dma_size = size;

  LL_DMA_DisableChannel(GPDMA1, LL_DMA_CHANNEL_1);
  LL_DMA_SetSrcAddress(GPDMA1, LL_DMA_CHANNEL_1, (uint32_t)&UCPD1->RXDR);
  LL_DMA_SetDestAddress(GPDMA1, LL_DMA_CHANNEL_1, (uint32_t)buffer);
  LL_DMA_SetBlkDataLength(GPDMA1, LL_DMA_CHANNEL_1, size);
  LL_DMA_EnableChannel(GPDMA1, LL_DMA_CHANNEL_1);
}

static void PD_App_RxDmaStop(void *user)
{
  (void)user;
  LL_DMA_DisableChannel(GPDMA1, LL_DMA_CHANNEL_1);
}

static uint16_t PD_App_RxDmaCount(void *user)
{
  (void)user;
  return (uint16_t)(pd_app_rx_dma_size -
                    LL_DMA_GetBlkDataLength(GPDMA1, LL_DMA_CHANNEL_1));
}
/* USER CODE END 0 */
```

Change `GPDMA1`, `LL_DMA_CHANNEL_1`, and `UCPD1` to match your CubeMX configuration. Use the `size` value passed into `PD_App_RxDmaStart()` as shown above; avoid hard-coding the receive buffer size in `PD_App_RxDmaCount()`.

Dead-battery handoff has the same rule on U5 as on G4: if the board may be powered only from USB-C, call `HAL_PWREx_DisableUCPDDeadBattery()` after `PD_BM_Init()` has enabled the UCPD sink path. If your board is independently powered and you intentionally want to release DBCC earlier, that is a board-level choice, not a requirement of the PD core.

## Hardware Notes

- The USB-C connector `GND` must share ground with the STM32 board.
- `CC1` and `CC2` must go to the correct UCPD CC pins.
- Do not swap `CC1` and `CC2` unless you also verify the selected MCU pin mapping.
- VBUS measurement/control is not part of this minimal core.
- The PD contract current is a limit/capability, not forced current. The load decides actual current draw.
- If your final product is powered only from USB-C, implement the power path so the MCU receives the default 5 V before PD negotiation and can survive the transition to the selected voltage.

## Troubleshooting

Only 5 V appears:

- Check that `PD_BM_Task()` is called continuously.
- Check that `PD_BM_IRQHandler()` is called from the UCPD IRQ.
- Check that UCPD global interrupt is enabled in NVIC.
- Check that UCPD RX DMA is configured and that the callback uses the correct DMA channel.
- Check that dead-battery mode is disabled after `PD_BM_Init()`.
- Check that the selected optional profiles are enabled.
- Check that the charger actually advertises the requested fixed voltage.
- Check CC voltage and wiring. One CC line should indicate attach; the other is usually open depending on cable orientation.

No messages are received:

- Verify `UCPDx_RX` DMA request in CubeMX.
- Verify `UCPDx->RXDR` is used as the DMA peripheral address.
- Verify memory increment and byte alignment are enabled.
- Verify `LL_UCPD_RxDMAEnable()` and `LL_UCPD_RxEnable()` are reached after attach. The core does this inside `PD_BM_Attach()`.

The board turns off when disabling dead-battery:

- Move the dead-battery disable call after `PD_BM_Init()`.
- Make sure UCPD CC sink mode is enabled before dead-battery is disabled.
- Make sure the board power path keeps the MCU alive from the default 5 V.

## Author

Created and tested by Naser Attarzadeh.

Repository owner: robobtnet

## License

The reusable project code is licensed under the Apache License 2.0.

Files generated by STM32CubeIDE and files under ST-provided driver folders may also be subject to STMicroelectronics license terms included in those files.
