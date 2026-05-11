/*
 * pd_bm.h
 *
 * Minimal bare-metal USB-C Power Delivery sink core for STM32 UCPD.
 *
 * Author: Naser Attarzadeh
 * Repository: https://github.com/robobtnet/stm32-usb-pd-baremetal-sink
 *
 * Copyright (c) 2026 Naser Attarzadeh
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PD_BM_H
#define PD_BM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#ifndef PD_BM_STM32_LL_UCPD_HEADER
#if defined(__has_include)
#if __has_include("stm32g4xx_ll_ucpd.h")
#define PD_BM_STM32_LL_UCPD_HEADER "stm32g4xx_ll_ucpd.h"
#elif __has_include("stm32u5xx_ll_ucpd.h")
#define PD_BM_STM32_LL_UCPD_HEADER "stm32u5xx_ll_ucpd.h"
#elif __has_include("stm32g0xx_ll_ucpd.h")
#define PD_BM_STM32_LL_UCPD_HEADER "stm32g0xx_ll_ucpd.h"
#elif __has_include("stm32l5xx_ll_ucpd.h")
#define PD_BM_STM32_LL_UCPD_HEADER "stm32l5xx_ll_ucpd.h"
#endif
#endif
#endif

#ifndef PD_BM_STM32_LL_UCPD_HEADER
#define PD_BM_STM32_LL_UCPD_HEADER "stm32g4xx_ll_ucpd.h"
#endif

#include PD_BM_STM32_LL_UCPD_HEADER

#define PD_BM_PROFILE_COUNT 5U

typedef enum
{
  PD_BM_STATE_DETACHED = 0,
  PD_BM_STATE_ATTACHED,
  PD_BM_STATE_RX_ACTIVITY,
  PD_BM_STATE_SRC_CAP_RX,
  PD_BM_STATE_REQUEST_SENT,
  PD_BM_STATE_ACCEPT_RX,
  PD_BM_STATE_PS_RDY_RX,
  PD_BM_STATE_READY,
  PD_BM_STATE_EPR_ENTER_SENT,
  PD_BM_STATE_EPR_READY,
  PD_BM_STATE_ERROR
} PD_BM_State;

typedef struct
{
  uint16_t voltage_mv;
  uint16_t current_ma;
  uint8_t enabled;
} PD_BM_Profile;

typedef struct
{
  UCPD_TypeDef *ucpd;
  const LL_UCPD_InitTypeDef *ucpd_init;

  uint32_t (*get_tick_ms)(void *user);
  void (*rx_dma_start)(uint8_t *buffer, uint16_t size, void *user);
  void (*rx_dma_stop)(void *user);
  uint16_t (*rx_dma_count)(void *user);
  void (*trace)(const char *event, uint32_t a, uint32_t b, uint32_t c, uint32_t d, void *user);
  void *user;

  PD_BM_Profile profiles[PD_BM_PROFILE_COUNT];
  uint16_t get_source_cap_interval_ms;
  uint16_t attach_debounce_ms;
} PD_BM_Config;

uint8_t PD_BM_Init(const PD_BM_Config *config);
uint8_t PD_BM_NeedsService(void);
void PD_BM_Task(void);
void PD_BM_IRQHandler(void);

PD_BM_State PD_BM_GetState(void);
uint16_t PD_BM_GetRequestedVoltage(void);
uint16_t PD_BM_GetRequestedCurrent(void);
uint8_t PD_BM_GetActiveProfile(void);
uint16_t PD_BM_GetLastSourcePDOCount(void);

#ifdef __cplusplus
}
#endif

#endif /* PD_BM_H */
