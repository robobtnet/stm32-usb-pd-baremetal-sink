/*
 * pd_bm.c
 *
 * Minimal bare-metal USB-C Power Delivery sink core for STM32 UCPD.
 *
 * Author: Naser Attarzadeh
 * Repository: https://github.com/robobtnet/stm32-usb-pd-baremetal-sink
 *
 * Copyright (c) 2026 Naser Attarzadeh
 * SPDX-License-Identifier: Apache-2.0
 *
 * This file implements a compact fixed-PDO sink policy engine. It negotiates
 * source capabilities, requests the highest enabled matching profile, and
 * falls back to profile 0 when optional profiles are unavailable or rejected.
 *
 * The implementation is intentionally independent from board-specific clocks,
 * GPIO pins, DMA instances, LEDs, and HAL application code. The application
 * supplies the UCPD instance, millisecond tick, RX DMA callbacks, and profiles
 * through PD_BM_Config.
 */

#include "pd_bm.h"

#include <stddef.h>
#include <string.h>

#define PD_MSG_GOODCRC          0x01U
#define PD_MSG_ACCEPT           0x03U
#define PD_MSG_REJECT           0x04U
#define PD_MSG_WAIT             0x0CU
#define PD_MSG_PS_RDY           0x06U
#define PD_MSG_GET_SRC_CAP      0x07U

#define PD_DATA_SRC_CAP         0x01U
#define PD_DATA_REQUEST         0x02U

#define PD_SPEC_REV_30          2U
#define PD_ROLE_SINK            0U
#define PD_DATA_ROLE_UFP        0U

#define PD_RX_BUFFER_SIZE       32U
#define PD_DEFAULT_GET_CAP_MS   500U
#define PD_DEFAULT_DEBOUNCE_MS  40U
#define PD_TX_BYTE_TIMEOUT_MS   2U
#define PD_TX_MSG_TIMEOUT_MS    10U

#define PD_HEADER_TYPE(_h_)     ((uint8_t)((_h_) & 0x1FU))
#define PD_HEADER_SPEC(_h_)     ((uint8_t)(((_h_) >> 6U) & 0x03U))
#define PD_HEADER_MSGID(_h_)    ((uint8_t)(((_h_) >> 9U) & 0x07U))
#define PD_HEADER_NDO(_h_)      ((uint8_t)(((_h_) >> 12U) & 0x07U))

#define PD_PDO_TYPE(_pdo_)      (((_pdo_) >> 30U) & 0x03U)
#define PD_PDO_FIXED_VOLT_MV(_pdo_) \
  ((uint16_t)((((uint32_t)(_pdo_) >> 10U) & 0x3FFU) * 50U))
#define PD_PDO_FIXED_CURR_10MA(_pdo_) \
  ((uint16_t)(((uint32_t)(_pdo_)) & 0x3FFU))

typedef enum
{
  PD_CC_NONE = 0,
  PD_CC_1,
  PD_CC_2
} PD_CC;

static PD_BM_Config pd_cfg;
static volatile PD_BM_State pd_state = PD_BM_STATE_DETACHED;
static volatile uint8_t pd_rx_ready;
static volatile uint8_t pd_request_pending;
static volatile uint8_t pd_tx_busy;
static volatile uint8_t pd_tx_msg_id;
static volatile uint8_t pd_rx_count;
static volatile uint8_t pd_active_cc;
static volatile uint8_t pd_active_profile = 0xFFU;
static volatile uint16_t pd_requested_voltage_mv;
static volatile uint16_t pd_requested_current_ma;
static volatile uint16_t pd_last_pdo_count;
static volatile uint32_t pd_last_get_src_cap_tick;
static volatile uint32_t pd_attach_tick;

static uint8_t pd_rx_buffer[PD_RX_BUFFER_SIZE];
static uint32_t pd_last_src_pdo[7];
static uint8_t pd_last_src_pdo_count;
static uint8_t pd_last_spec_rev = PD_SPEC_REV_30;
static uint8_t pd_selected_position;
static uint16_t pd_selected_current_10ma;

static uint32_t PD_BM_Tick(void);
static uint16_t PD_BM_ProfileCurrent10mA(const PD_BM_Profile *profile);
static uint16_t PD_BM_MakeHeader(uint8_t msg_type, uint8_t ndo, uint8_t msg_id, uint8_t spec_rev);
static uint32_t PD_BM_ReadLE32(const uint8_t *buf);
static void PD_BM_WriteLE16(uint8_t *buf, uint16_t value);
static void PD_BM_WriteLE32(uint8_t *buf, uint32_t value);
static void PD_BM_ResetProtocol(void);
static void PD_BM_EnableDetect(void);
static void PD_BM_Attach(PD_CC cc);
static void PD_BM_Detach(void);
static void PD_BM_RxDMAArm(void);
static void PD_BM_RxDMADisarm(void);
static void PD_BM_ProcessMessage(void);
static void PD_BM_SendGoodCRC(uint16_t rx_header);
static uint8_t PD_BM_SendSOP(const uint8_t *payload, uint8_t size);
static void PD_BM_SendGetSourceCap(void);
static void PD_BM_RequestProfile(uint8_t profile_index);
static uint8_t PD_BM_SelectProfileFromSourceCaps(int8_t start_index);
static uint8_t PD_BM_RequestNextLowerProfile(void);
static void PD_BM_HandleTypeCEvent(void);
static uint8_t PD_BM_IsActiveCCOpen(void);
static uint8_t PD_BM_TimedOut(uint32_t start_tick, uint16_t timeout_ms);

uint8_t PD_BM_Init(const PD_BM_Config *config)
{
  LL_UCPD_InitTypeDef default_init;

  if ((config == NULL) || (config->ucpd == NULL) || (config->get_tick_ms == NULL)
      || (config->rx_dma_start == NULL) || (config->rx_dma_stop == NULL)
      || (config->rx_dma_count == NULL))
  {
    pd_state = PD_BM_STATE_ERROR;
    return 0U;
  }

  pd_cfg = *config;
  if ((pd_cfg.profiles[0].voltage_mv == 0U) || (pd_cfg.profiles[0].current_ma == 0U))
  {
    pd_state = PD_BM_STATE_ERROR;
    return 0U;
  }
  pd_cfg.profiles[0].enabled = 1U;

  if (pd_cfg.get_source_cap_interval_ms == 0U)
  {
    pd_cfg.get_source_cap_interval_ms = PD_DEFAULT_GET_CAP_MS;
  }
  if (pd_cfg.attach_debounce_ms == 0U)
  {
    pd_cfg.attach_debounce_ms = PD_DEFAULT_DEBOUNCE_MS;
  }

  LL_UCPD_Disable(pd_cfg.ucpd);
  if (pd_cfg.ucpd_init != NULL)
  {
    (void)LL_UCPD_Init(pd_cfg.ucpd, pd_cfg.ucpd_init);
  }
  else
  {
    LL_UCPD_StructInit(&default_init);
    (void)LL_UCPD_Init(pd_cfg.ucpd, &default_init);
  }

  LL_UCPD_SetRxOrderSet(pd_cfg.ucpd, LL_UCPD_ORDERSET_SOP | LL_UCPD_ORDERSET_HARDRST);
  LL_UCPD_SetRxMode(pd_cfg.ucpd, LL_UCPD_RXMODE_NORMAL);
  LL_UCPD_SetSNKRole(pd_cfg.ucpd);
  LL_UCPD_SetccEnable(pd_cfg.ucpd, LL_UCPD_CCENABLE_NONE);
  LL_UCPD_Enable(pd_cfg.ucpd);

  PD_BM_ResetProtocol();
  PD_BM_EnableDetect();

  return 1U;
}

void PD_BM_Task(void)
{
  if (pd_state == PD_BM_STATE_DETACHED)
  {
    uint32_t cc1 = LL_UCPD_GetTypeCVstateCC1(pd_cfg.ucpd);
    uint32_t cc2 = LL_UCPD_GetTypeCVstateCC2(pd_cfg.ucpd);

    if ((cc1 != LL_UCPD_SNK_CC1_VOPEN) || (cc2 != LL_UCPD_SNK_CC2_VOPEN))
    {
      if (pd_attach_tick == 0U)
      {
        pd_attach_tick = PD_BM_Tick();
      }
      else if ((PD_BM_Tick() - pd_attach_tick) >= pd_cfg.attach_debounce_ms)
      {
        if (LL_UCPD_GetTypeCVstateCC1(pd_cfg.ucpd) != LL_UCPD_SNK_CC1_VOPEN)
        {
          PD_BM_Attach(PD_CC_1);
        }
        else if (LL_UCPD_GetTypeCVstateCC2(pd_cfg.ucpd) != LL_UCPD_SNK_CC2_VOPEN)
        {
          PD_BM_Attach(PD_CC_2);
        }
      }
    }
    else
    {
      pd_attach_tick = 0U;
    }
  }
  else if (PD_BM_IsActiveCCOpen() != 0U)
  {
    PD_BM_Detach();
    return;
  }

  if (pd_rx_ready != 0U)
  {
    __disable_irq();
    pd_rx_ready = 0U;
    __enable_irq();
    PD_BM_ProcessMessage();
  }

  if ((pd_request_pending != 0U) && (pd_tx_busy == 0U))
  {
    pd_request_pending = 0U;
    PD_BM_RequestProfile(pd_active_profile);
  }

  if (((pd_state == PD_BM_STATE_ATTACHED) || (pd_state == PD_BM_STATE_RX_ACTIVITY))
      && ((PD_BM_Tick() - pd_last_get_src_cap_tick) >= pd_cfg.get_source_cap_interval_ms)
      && (pd_tx_busy == 0U)
      && (pd_rx_ready == 0U))
  {
    pd_last_get_src_cap_tick = PD_BM_Tick();
    PD_BM_SendGetSourceCap();
  }
}

void PD_BM_IRQHandler(void)
{
  uint32_t sr = pd_cfg.ucpd->SR;

  if ((sr & UCPD_SR_TYPECEVT1) != 0U)
  {
    LL_UCPD_ClearFlag_TypeCEventCC1(pd_cfg.ucpd);
    PD_BM_HandleTypeCEvent();
  }

  if ((sr & UCPD_SR_TYPECEVT2) != 0U)
  {
    LL_UCPD_ClearFlag_TypeCEventCC2(pd_cfg.ucpd);
    PD_BM_HandleTypeCEvent();
  }

  if ((sr & UCPD_SR_RXORDDET) != 0U)
  {
    pd_rx_count = 0U;
    if (pd_state == PD_BM_STATE_ATTACHED)
    {
      pd_state = PD_BM_STATE_RX_ACTIVITY;
    }
    LL_UCPD_ClearFlag_RxOrderSet(pd_cfg.ucpd);
  }

  if ((sr & UCPD_SR_RXHRSTDET) != 0U)
  {
    LL_UCPD_ClearFlag_RxHRST(pd_cfg.ucpd);
    PD_BM_Detach();
  }

  if ((sr & UCPD_SR_RXOVR) != 0U)
  {
    LL_UCPD_ClearFlag_RxOvr(pd_cfg.ucpd);
    PD_BM_RxDMAArm();
    pd_rx_count = 0U;
  }

  if ((sr & UCPD_SR_RXMSGEND) != 0U)
  {
    LL_UCPD_ClearFlag_RxMsgEnd(pd_cfg.ucpd);
    PD_BM_RxDMADisarm();
    pd_rx_count = (uint8_t)pd_cfg.rx_dma_count(pd_cfg.user);
    pd_rx_ready = 1U;
  }

  if ((sr & UCPD_SR_TXMSGSENT) != 0U)
  {
    LL_UCPD_ClearFlag_TxMSGSENT(pd_cfg.ucpd);
    pd_tx_busy = 0U;
  }

  if ((sr & UCPD_SR_TXMSGDISC) != 0U)
  {
    LL_UCPD_ClearFlag_TxMSGDISC(pd_cfg.ucpd);
    pd_tx_busy = 0U;
  }

  if ((sr & UCPD_SR_TXMSGABT) != 0U)
  {
    LL_UCPD_ClearFlag_TxMSGABT(pd_cfg.ucpd);
    pd_tx_busy = 0U;
  }

  if ((sr & UCPD_SR_TXUND) != 0U)
  {
    LL_UCPD_ClearFlag_TxUND(pd_cfg.ucpd);
    pd_tx_busy = 0U;
  }
}

PD_BM_State PD_BM_GetState(void)
{
  return pd_state;
}

uint16_t PD_BM_GetRequestedVoltage(void)
{
  return pd_requested_voltage_mv;
}

uint16_t PD_BM_GetRequestedCurrent(void)
{
  return pd_requested_current_ma;
}

uint8_t PD_BM_GetActiveProfile(void)
{
  return pd_active_profile;
}

uint16_t PD_BM_GetLastSourcePDOCount(void)
{
  return pd_last_pdo_count;
}

static uint32_t PD_BM_Tick(void)
{
  return pd_cfg.get_tick_ms(pd_cfg.user);
}

static uint16_t PD_BM_ProfileCurrent10mA(const PD_BM_Profile *profile)
{
  uint16_t current = (uint16_t)((profile->current_ma + 9U) / 10U);

  if (current == 0U)
  {
    current = 1U;
  }
  if (current > 0x3FFU)
  {
    current = 0x3FFU;
  }

  return current;
}

static uint16_t PD_BM_MakeHeader(uint8_t msg_type, uint8_t ndo, uint8_t msg_id, uint8_t spec_rev)
{
  return (uint16_t)((msg_type & 0x1FU)
      | ((uint16_t)(PD_DATA_ROLE_UFP & 0x01U) << 5U)
      | ((uint16_t)(spec_rev & 0x03U) << 6U)
      | ((uint16_t)(PD_ROLE_SINK & 0x01U) << 8U)
      | ((uint16_t)(msg_id & 0x07U) << 9U)
      | ((uint16_t)(ndo & 0x07U) << 12U));
}

static uint32_t PD_BM_ReadLE32(const uint8_t *buf)
{
  return ((uint32_t)buf[0])
      | ((uint32_t)buf[1] << 8U)
      | ((uint32_t)buf[2] << 16U)
      | ((uint32_t)buf[3] << 24U);
}

static void PD_BM_WriteLE16(uint8_t *buf, uint16_t value)
{
  buf[0] = (uint8_t)(value & 0xFFU);
  buf[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void PD_BM_WriteLE32(uint8_t *buf, uint32_t value)
{
  buf[0] = (uint8_t)(value & 0xFFU);
  buf[1] = (uint8_t)((value >> 8U) & 0xFFU);
  buf[2] = (uint8_t)((value >> 16U) & 0xFFU);
  buf[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static void PD_BM_ResetProtocol(void)
{
  pd_rx_ready = 0U;
  pd_request_pending = 0U;
  pd_tx_busy = 0U;
  pd_tx_msg_id = 0U;
  pd_rx_count = 0U;
  pd_active_cc = PD_CC_NONE;
  pd_active_profile = 0xFFU;
  pd_requested_voltage_mv = 0U;
  pd_requested_current_ma = 0U;
  pd_last_pdo_count = 0U;
  pd_last_get_src_cap_tick = 0U;
  pd_attach_tick = 0U;
  pd_last_src_pdo_count = 0U;
  pd_selected_position = 0U;
  pd_selected_current_10ma = 0U;
  pd_last_spec_rev = PD_SPEC_REV_30;
}

static void PD_BM_EnableDetect(void)
{
  PD_BM_RxDMADisarm();
  LL_UCPD_RxDisable(pd_cfg.ucpd);
  LL_UCPD_RxDMADisable(pd_cfg.ucpd);
  LL_UCPD_TypeCDetectionCC1Disable(pd_cfg.ucpd);
  LL_UCPD_TypeCDetectionCC2Disable(pd_cfg.ucpd);
  LL_UCPD_SetccEnable(pd_cfg.ucpd, LL_UCPD_CCENABLE_NONE);
  LL_UCPD_SetSNKRole(pd_cfg.ucpd);
  LL_UCPD_SetccEnable(pd_cfg.ucpd, LL_UCPD_CCENABLE_CC1CC2);

  LL_UCPD_TypeCDetectionCC1Enable(pd_cfg.ucpd);
  LL_UCPD_TypeCDetectionCC2Enable(pd_cfg.ucpd);

  pd_cfg.ucpd->ICR = UCPD_ICR_TXMSGSENTCF | UCPD_ICR_TXMSGDISCCF | UCPD_ICR_TXMSGABTCF
      | UCPD_ICR_TXUNDCF | UCPD_ICR_RXORDDETCF | UCPD_ICR_RXHRSTDETCF
      | UCPD_ICR_RXOVRCF | UCPD_ICR_RXMSGENDCF | UCPD_ICR_TYPECEVT1CF
      | UCPD_ICR_TYPECEVT2CF;

  pd_cfg.ucpd->IMR = UCPD_IMR_TYPECEVT1IE | UCPD_IMR_TYPECEVT2IE;
  pd_state = PD_BM_STATE_DETACHED;
}

static void PD_BM_Attach(PD_CC cc)
{
  pd_active_cc = (uint8_t)cc;
  pd_rx_count = 0U;
  pd_rx_ready = 0U;

  PD_BM_RxDMADisarm();
  LL_UCPD_RxDisable(pd_cfg.ucpd);
  LL_UCPD_RxDMADisable(pd_cfg.ucpd);
  LL_UCPD_SetccEnable(pd_cfg.ucpd, LL_UCPD_CCENABLE_NONE);
  LL_UCPD_SetCCPin(pd_cfg.ucpd, (cc == PD_CC_1) ? LL_UCPD_CCPIN_CC1 : LL_UCPD_CCPIN_CC2);
  LL_UCPD_SetccEnable(pd_cfg.ucpd, (cc == PD_CC_1) ? LL_UCPD_CCENABLE_CC1 : LL_UCPD_CCENABLE_CC2);
  LL_UCPD_SetRxMode(pd_cfg.ucpd, LL_UCPD_RXMODE_NORMAL);
  PD_BM_RxDMAArm();
  LL_UCPD_RxDMAEnable(pd_cfg.ucpd);
  LL_UCPD_RxEnable(pd_cfg.ucpd);

  pd_cfg.ucpd->ICR = UCPD_ICR_TXMSGSENTCF | UCPD_ICR_TXMSGDISCCF | UCPD_ICR_TXMSGABTCF
      | UCPD_ICR_TXUNDCF | UCPD_ICR_RXORDDETCF | UCPD_ICR_RXHRSTDETCF
      | UCPD_ICR_RXOVRCF | UCPD_ICR_RXMSGENDCF | UCPD_ICR_TYPECEVT1CF
      | UCPD_ICR_TYPECEVT2CF;

  pd_cfg.ucpd->IMR = UCPD_IMR_TYPECEVT1IE | UCPD_IMR_TYPECEVT2IE
      | UCPD_IMR_RXORDDETIE | UCPD_IMR_RXHRSTDETIE
      | UCPD_IMR_RXOVRIE | UCPD_IMR_RXMSGENDIE;

  pd_last_get_src_cap_tick = PD_BM_Tick();
  pd_state = PD_BM_STATE_ATTACHED;
}

static void PD_BM_Detach(void)
{
  PD_BM_ResetProtocol();
  PD_BM_EnableDetect();
}

static void PD_BM_RxDMAArm(void)
{
  pd_cfg.rx_dma_start(pd_rx_buffer, PD_RX_BUFFER_SIZE, pd_cfg.user);
}

static void PD_BM_RxDMADisarm(void)
{
  pd_cfg.rx_dma_stop(pd_cfg.user);
}

static void PD_BM_ProcessMessage(void)
{
  uint16_t header;
  uint8_t msg_type;
  uint8_t ndo;
  uint8_t is_goodcrc;

  if (pd_rx_count < 2U)
  {
    PD_BM_RxDMAArm();
    return;
  }

  header = (uint16_t)pd_rx_buffer[0] | ((uint16_t)pd_rx_buffer[1] << 8U);
  msg_type = PD_HEADER_TYPE(header);
  ndo = PD_HEADER_NDO(header);
  pd_last_spec_rev = PD_HEADER_SPEC(header);
  if (pd_last_spec_rev > PD_SPEC_REV_30)
  {
    pd_last_spec_rev = PD_SPEC_REV_30;
  }

  is_goodcrc = ((ndo == 0U) && (msg_type == PD_MSG_GOODCRC)) ? 1U : 0U;
  if (is_goodcrc == 0U)
  {
    PD_BM_SendGoodCRC(header);
  }

  if ((ndo > 0U) && (msg_type == PD_DATA_SRC_CAP))
  {
    uint8_t count = ndo;

    if (count > 7U)
    {
      count = 7U;
    }
    if ((2U + ((uint16_t)count * 4U)) > pd_rx_count)
    {
      PD_BM_RxDMAArm();
      return;
    }

    for (uint8_t i = 0U; i < count; i++)
    {
      pd_last_src_pdo[i] = PD_BM_ReadLE32(&pd_rx_buffer[2U + ((uint16_t)i * 4U)]);
    }

    pd_last_src_pdo_count = count;
    pd_last_pdo_count = count;
    pd_state = PD_BM_STATE_SRC_CAP_RX;

    if (PD_BM_SelectProfileFromSourceCaps((int8_t)(PD_BM_PROFILE_COUNT - 1U)) != 0U)
    {
      pd_request_pending = 1U;
    }
    else
    {
      pd_state = PD_BM_STATE_ERROR;
    }
  }
  else if ((ndo == 0U) && (msg_type == PD_MSG_ACCEPT))
  {
    pd_state = PD_BM_STATE_ACCEPT_RX;
  }
  else if ((ndo == 0U) && (msg_type == PD_MSG_PS_RDY))
  {
    pd_state = PD_BM_STATE_PS_RDY_RX;
    if (pd_requested_voltage_mv != 0U)
    {
      pd_state = PD_BM_STATE_READY;
    }
  }
  else if ((ndo == 0U) && ((msg_type == PD_MSG_REJECT) || (msg_type == PD_MSG_WAIT)))
  {
    if (PD_BM_RequestNextLowerProfile() != 0U)
    {
      pd_request_pending = 1U;
    }
    else
    {
      pd_state = PD_BM_STATE_ERROR;
    }
  }

  if (pd_active_cc != PD_CC_NONE)
  {
    PD_BM_RxDMAArm();
  }
}

static void PD_BM_SendGoodCRC(uint16_t rx_header)
{
  uint8_t tx[2];
  uint16_t header = PD_BM_MakeHeader(PD_MSG_GOODCRC, 0U, PD_HEADER_MSGID(rx_header), PD_HEADER_SPEC(rx_header));

  PD_BM_WriteLE16(tx, header);
  (void)PD_BM_SendSOP(tx, sizeof(tx));
}

static uint8_t PD_BM_SendSOP(const uint8_t *payload, uint8_t size)
{
  uint32_t start_tick;

  if ((payload == NULL) || (size == 0U))
  {
    return 0U;
  }

  pd_tx_busy = 1U;

  LL_UCPD_WriteTxOrderSet(pd_cfg.ucpd, LL_UCPD_ORDERED_SET_SOP);
  LL_UCPD_SetTxMode(pd_cfg.ucpd, LL_UCPD_TXMODE_NORMAL);
  LL_UCPD_WriteTxPaySize(pd_cfg.ucpd, size);
  LL_UCPD_SendMessage(pd_cfg.ucpd);

  for (uint8_t i = 0U; i < size; i++)
  {
    start_tick = PD_BM_Tick();
    while ((pd_cfg.ucpd->SR & UCPD_SR_TXIS) == 0U)
    {
      if (PD_BM_TimedOut(start_tick, PD_TX_BYTE_TIMEOUT_MS) != 0U)
      {
        pd_tx_busy = 0U;
        return 0U;
      }
    }

    LL_UCPD_WriteData(pd_cfg.ucpd, payload[i]);
  }

  start_tick = PD_BM_Tick();
  while (((pd_cfg.ucpd->SR & (UCPD_SR_TXMSGSENT | UCPD_SR_TXMSGDISC | UCPD_SR_TXMSGABT | UCPD_SR_TXUND)) == 0U)
      && (PD_BM_TimedOut(start_tick, PD_TX_MSG_TIMEOUT_MS) == 0U))
  {
  }

  if ((pd_cfg.ucpd->SR & UCPD_SR_TXMSGSENT) != 0U)
  {
    LL_UCPD_ClearFlag_TxMSGSENT(pd_cfg.ucpd);
    pd_tx_busy = 0U;
    return 1U;
  }

  if ((pd_cfg.ucpd->SR & UCPD_SR_TXMSGDISC) != 0U)
  {
    LL_UCPD_ClearFlag_TxMSGDISC(pd_cfg.ucpd);
  }
  if ((pd_cfg.ucpd->SR & UCPD_SR_TXMSGABT) != 0U)
  {
    LL_UCPD_ClearFlag_TxMSGABT(pd_cfg.ucpd);
  }
  if ((pd_cfg.ucpd->SR & UCPD_SR_TXUND) != 0U)
  {
    LL_UCPD_ClearFlag_TxUND(pd_cfg.ucpd);
  }

  pd_tx_busy = 0U;
  return 0U;
}

static void PD_BM_SendGetSourceCap(void)
{
  uint8_t tx[2];
  uint16_t header = PD_BM_MakeHeader(PD_MSG_GET_SRC_CAP, 0U, pd_tx_msg_id, PD_SPEC_REV_30);

  PD_BM_WriteLE16(tx, header);
  if (PD_BM_SendSOP(tx, sizeof(tx)) != 0U)
  {
    pd_tx_msg_id = (uint8_t)((pd_tx_msg_id + 1U) & 0x07U);
  }
}

static void PD_BM_RequestProfile(uint8_t profile_index)
{
  uint8_t tx[6];
  uint16_t header;
  uint32_t rdo;

  if ((profile_index >= PD_BM_PROFILE_COUNT) || (pd_selected_position == 0U))
  {
    pd_state = PD_BM_STATE_ERROR;
    return;
  }

  rdo = ((uint32_t)pd_selected_position << 28U)
      | ((uint32_t)pd_selected_current_10ma << 10U)
      | ((uint32_t)pd_selected_current_10ma);

  header = PD_BM_MakeHeader(PD_DATA_REQUEST, 1U, pd_tx_msg_id, pd_last_spec_rev);
  PD_BM_WriteLE16(&tx[0], header);
  PD_BM_WriteLE32(&tx[2], rdo);

  if (PD_BM_SendSOP(tx, sizeof(tx)) != 0U)
  {
    pd_state = PD_BM_STATE_REQUEST_SENT;
    pd_tx_msg_id = (uint8_t)((pd_tx_msg_id + 1U) & 0x07U);
  }
  else
  {
    pd_state = PD_BM_STATE_ERROR;
  }
}

static uint8_t PD_BM_SelectProfileFromSourceCaps(int8_t start_index)
{
  if (start_index >= (int8_t)PD_BM_PROFILE_COUNT)
  {
    start_index = (int8_t)PD_BM_PROFILE_COUNT - 1;
  }

  for (int8_t profile_index = start_index; profile_index >= 0; profile_index--)
  {
    const PD_BM_Profile *profile = &pd_cfg.profiles[profile_index];

    if ((profile_index != 0) && (profile->enabled == 0U))
    {
      continue;
    }
    if ((profile->voltage_mv == 0U) || (profile->current_ma == 0U))
    {
      continue;
    }

    for (uint8_t pdo_index = 0U; pdo_index < pd_last_src_pdo_count; pdo_index++)
    {
      uint32_t pdo = pd_last_src_pdo[pdo_index];

      if ((PD_PDO_TYPE(pdo) == 0U) && (PD_PDO_FIXED_VOLT_MV(pdo) == profile->voltage_mv))
      {
        uint16_t requested_current = PD_BM_ProfileCurrent10mA(profile);
        uint16_t source_current = PD_PDO_FIXED_CURR_10MA(pdo);

        if (source_current < requested_current)
        {
          continue;
        }

        pd_active_profile = (uint8_t)profile_index;
        pd_selected_position = (uint8_t)(pdo_index + 1U);
        pd_selected_current_10ma = source_current;
        pd_requested_voltage_mv = profile->voltage_mv;
        pd_requested_current_ma = (uint16_t)(pd_selected_current_10ma * 10U);
        return 1U;
      }
    }
  }

  return 0U;
}

static uint8_t PD_BM_RequestNextLowerProfile(void)
{
  if (pd_active_profile == 0U)
  {
    return 0U;
  }

  return PD_BM_SelectProfileFromSourceCaps((int8_t)pd_active_profile - 1);
}

static void PD_BM_HandleTypeCEvent(void)
{
  if ((pd_state != PD_BM_STATE_DETACHED) && (PD_BM_IsActiveCCOpen() != 0U))
  {
    PD_BM_Detach();
  }
}

static uint8_t PD_BM_IsActiveCCOpen(void)
{
  if (pd_active_cc == PD_CC_1)
  {
    return (LL_UCPD_GetTypeCVstateCC1(pd_cfg.ucpd) == LL_UCPD_SNK_CC1_VOPEN) ? 1U : 0U;
  }

  if (pd_active_cc == PD_CC_2)
  {
    return (LL_UCPD_GetTypeCVstateCC2(pd_cfg.ucpd) == LL_UCPD_SNK_CC2_VOPEN) ? 1U : 0U;
  }

  return 1U;
}

static uint8_t PD_BM_TimedOut(uint32_t start_tick, uint16_t timeout_ms)
{
  return ((PD_BM_Tick() - start_tick) > timeout_ms) ? 1U : 0U;
}
