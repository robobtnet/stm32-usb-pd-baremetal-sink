/*
 * pd_bm.c
 *
 * Minimal bare-metal USB-C Power Delivery sink core for STM32 UCPD.
 * Based on stable SPR baseline (.best) with minimal PD 3.1 EPR additions.
 *
 * Author: Naser Attarzadeh
 * Repository: https://github.com/robobtnet/stm32-usb-pd-baremetal-sink
 *
 * Copyright (c) 2026 Naser Attarzadeh
 * SPDX-License-Identifier: Apache-2.0
 */

#include "pd_bm.h"

#include <stddef.h>
#include <string.h>

#define PD_MSG_GOODCRC          0x01U
#define PD_MSG_ACCEPT           0x03U
#define PD_MSG_REJECT           0x04U
#define PD_MSG_WAIT             0x0CU
#define PD_MSG_SOFT_RESET       0x0DU
#define PD_MSG_NOT_SUPPORTED    0x10U
#define PD_MSG_PS_RDY           0x06U
#define PD_MSG_GET_SRC_CAP      0x07U

#define PD_DATA_SRC_CAP         0x01U
#define PD_DATA_REQUEST         0x02U
#define PD_DATA_EPR_REQUEST     0x09U
#define PD_DATA_EPR_MODE        0x0AU

#define PD_EXT_EXTENDED_CONTROL 0x10U
#define PD_EXT_EPR_SRC_CAP      0x11U

#define PD_EXT_CTRL_EPR_GET_SRC_CAP 1U
#define PD_EXT_CTRL_EPR_GET_SINK_CAP 2U
#define PD_EXT_CTRL_EPR_KEEPALIVE 3U
#define PD_EXT_CTRL_EPR_KEEPALIVE_ACK 4U

#define PD_EPR_ACT_ENTER        0x01U
#define PD_EPR_ACT_ENTER_ACK    0x02U
#define PD_EPR_ACT_ENTER_OK     0x03U
#define PD_EPR_ACT_ENTER_FAIL   0x04U
#define PD_EPR_ACT_EXIT         0x05U
#define PD_EPR_ACT_KEEPALIVE_ACK 0x08U

#define PD_SPEC_REV_30          2U
#define PD_ROLE_SINK            0U
#define PD_DATA_ROLE_UFP        0U

#define PD_RX_BUFFER_SIZE       64U
#define PD_RX_QUEUE_DEPTH       16U
#define PD_DEFAULT_GET_CAP_MS   500U
#define PD_DEFAULT_DEBOUNCE_MS  40U
#define PD_TX_BYTE_TIMEOUT_MS   2U
#define PD_TX_MSG_TIMEOUT_MS    10U
#define PD_EPR_KEEPALIVE_MS     375U
#define PD_EXT_CHUNK_SIZE       26U
#define PD_EPR_FAIL_STREAK_MAX  4U

#define PD_HEADER_TYPE(_h_)     ((uint8_t)((_h_) & 0x1FU))
#define PD_HEADER_EXT(_h_)      ((uint8_t)(((_h_) >> 15U) & 0x01U))
#define PD_HEADER_SPEC(_h_)     ((uint8_t)(((_h_) >> 6U) & 0x03U))
#define PD_HEADER_MSGID(_h_)    ((uint8_t)(((_h_) >> 9U) & 0x07U))
#define PD_HEADER_NDO(_h_)      ((uint8_t)(((_h_) >> 12U) & 0x07U))

#define PD_PDO_TYPE(_pdo_)      (((_pdo_) >> 30U) & 0x03U)
#define PD_PDO_FIXED_VOLT_MV(_pdo_) \
  ((uint16_t)((((uint32_t)(_pdo_) >> 10U) & 0x3FFU) * 50U))
#define PD_PDO_FIXED_CURR_10MA(_pdo_) \
  ((uint16_t)(((uint32_t)(_pdo_)) & 0x3FFU))
#define PD_PDO_FIXED_EPR_CAPABLE(_pdo_) (((_pdo_) >> 23U) & 0x01U)
#define PD_PDO_FIXED_UNCHUNKED_EXT(_pdo_) (((_pdo_) >> 24U) & 0x01U)

#define PD_EPR_PDO_VOLT_MV(_pdo_) \
  ((uint32_t)((((uint32_t)(_pdo_) >> 10U) & 0x3FFU) * 50U))

#define PD_RDO_EPR_CAPABLE      (1UL << 22U)

#define PD_MAX_PDO_COUNT        11U

typedef enum
{
  PD_CC_NONE = 0,
  PD_CC_1,
  PD_CC_2
} PD_CC;

typedef struct
{
  uint8_t data[PD_RX_BUFFER_SIZE];
  uint8_t count;
} PD_RxMessage;

static PD_BM_Config pd_cfg;
static volatile PD_BM_State pd_state = PD_BM_STATE_DETACHED;
static volatile uint8_t pd_rx_ready;
static volatile uint8_t pd_request_pending;
static volatile uint8_t pd_epr_enter_pending;
static volatile uint8_t pd_epr_keepalive_pending;
static volatile uint8_t pd_chunk_request_pending;
static uint8_t pd_chunk_request_msg_type;
static uint8_t pd_chunk_request_number;
static volatile uint8_t pd_tx_busy;
static volatile uint8_t pd_typec_event_pending;
static volatile uint8_t pd_tx_msg_id;
static volatile uint8_t pd_rx_count;
static volatile uint8_t pd_active_cc;
static volatile uint8_t pd_active_profile = 0xFFU;
static volatile uint16_t pd_requested_voltage_mv;
static volatile uint16_t pd_requested_current_ma;
static volatile uint16_t pd_last_pdo_count;
static volatile uint32_t pd_last_get_src_cap_tick;
static volatile uint32_t pd_last_epr_keepalive_tick;
static volatile uint32_t pd_attach_tick;
static uint8_t pd_epr_fail_streak;
static uint8_t pd_epr_negotiating;

static uint8_t pd_rx_buffer[PD_RX_BUFFER_SIZE];
static uint8_t pd_msg_buffer[PD_RX_BUFFER_SIZE];
static uint8_t pd_msg_count;
static PD_RxMessage pd_rx_queue[PD_RX_QUEUE_DEPTH];
static volatile uint8_t pd_rx_queue_head;
static volatile uint8_t pd_rx_queue_tail;
static uint32_t pd_last_src_pdo[PD_MAX_PDO_COUNT];
static uint8_t pd_last_src_pdo_count;
static uint8_t pd_last_spec_rev = PD_SPEC_REV_30;
static uint8_t pd_selected_position;
static uint16_t pd_selected_current_10ma;
static uint32_t pd_selected_pdo;
static uint8_t pd_selected_is_epr;
static uint8_t pd_epr_source_capable;
static uint8_t pd_source_unchunked_capable;
static volatile uint8_t pd_epr_mode_active;
static volatile uint8_t pd_wants_epr;

#define PD_TRACE(_event_, _a_, _b_, _c_, _d_) \
  do { \
    if (pd_cfg.trace != NULL) { \
      pd_cfg.trace((_event_), (uint32_t)(_a_), (uint32_t)(_b_), (uint32_t)(_c_), (uint32_t)(_d_), pd_cfg.user); \
    } \
  } while (0)

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
static void PD_BM_SendControl(uint8_t msg_type);
static uint8_t PD_BM_SendSOP(const uint8_t *payload, uint8_t size);
static void PD_BM_SendGetSourceCap(void);
static void PD_BM_RequestProfile(uint8_t profile_index);
static void PD_BM_RequestEprProfile(void);
static void PD_BM_SendEprModeEnter(void);
static void PD_BM_SendEprKeepAlive(void);
static void PD_BM_SendChunkRequest(uint8_t msg_type, uint8_t chunk_number);
static void PD_BM_ScheduleEprKeepAliveIfDue(void);
static void PD_BM_RxQueuePushFromISR(uint8_t count);
static uint8_t PD_BM_RxQueuePop(void);
static uint8_t PD_BM_SelectProfileFromSourceCaps(int8_t start_index);
static uint8_t PD_BM_SelectSprForEprEntry(void);
static uint8_t PD_BM_SelectEprPdo(void);
static uint8_t PD_BM_RequestNextLowerProfile(void);
static uint8_t PD_BM_ConfigWantsEpr(void);
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

  pd_wants_epr = PD_BM_ConfigWantsEpr();

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
  if (pd_typec_event_pending != 0U)
  {
    __disable_irq();
    pd_typec_event_pending = 0U;
    __enable_irq();
    PD_BM_HandleTypeCEvent();
  }

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
  else if ((pd_state != PD_BM_STATE_READY) && (pd_state != PD_BM_STATE_EPR_READY)
      && (PD_BM_IsActiveCCOpen() != 0U))
  {
    PD_BM_Detach();
    return;
  }

  if (pd_rx_ready != 0U)
  {
    while (PD_BM_RxQueuePop() != 0U)
    {
      PD_BM_ProcessMessage();
      if ((pd_request_pending != 0U)
          || (pd_chunk_request_pending != 0U)
          || (pd_epr_enter_pending != 0U)
          || (pd_epr_keepalive_pending != 0U))
      {
        break;
      }
    }
  }

  if ((pd_request_pending != 0U) && (pd_tx_busy == 0U))
  {
    pd_request_pending = 0U;
    if (pd_selected_is_epr != 0U)
    {
      PD_BM_RequestEprProfile();
    }
    else
    {
      PD_BM_RequestProfile(pd_active_profile);
    }
  }

  if ((pd_chunk_request_pending != 0U) && (pd_tx_busy == 0U))
  {
    uint8_t mt = pd_chunk_request_msg_type;
    uint8_t cn = pd_chunk_request_number;
    pd_chunk_request_pending = 0U;
    PD_BM_SendChunkRequest(mt, cn);
  }

  if ((pd_epr_enter_pending != 0U) && (pd_tx_busy == 0U))
  {
    pd_epr_enter_pending = 0U;
    PD_BM_SendEprModeEnter();
  }

  if ((pd_epr_keepalive_pending != 0U) && (pd_tx_busy == 0U))
  {
    pd_epr_keepalive_pending = 0U;
    PD_BM_SendEprKeepAlive();
  }

  if (((pd_state == PD_BM_STATE_ATTACHED) || (pd_state == PD_BM_STATE_RX_ACTIVITY))
      && ((PD_BM_Tick() - pd_last_get_src_cap_tick) >= pd_cfg.get_source_cap_interval_ms)
      && (pd_tx_busy == 0U)
      && (pd_rx_ready == 0U))
  {
    pd_last_get_src_cap_tick = PD_BM_Tick();
    PD_BM_SendGetSourceCap();
  }

  PD_BM_ScheduleEprKeepAliveIfDue();
}

uint8_t PD_BM_NeedsService(void)
{
  if ((pd_cfg.ucpd == NULL) || (pd_cfg.get_tick_ms == NULL))
  {
    return 0U;
  }

  if ((pd_typec_event_pending != 0U) || (pd_rx_ready != 0U) || (pd_request_pending != 0U)
      || (pd_epr_enter_pending != 0U) || (pd_epr_keepalive_pending != 0U)
      || (pd_chunk_request_pending != 0U))
  {
    return 1U;
  }

  if ((pd_state == PD_BM_STATE_DETACHED) && (pd_attach_tick != 0U)
      && ((PD_BM_Tick() - pd_attach_tick) >= pd_cfg.attach_debounce_ms))
  {
    return 1U;
  }

  if (((pd_state == PD_BM_STATE_ATTACHED) || (pd_state == PD_BM_STATE_RX_ACTIVITY))
      && ((PD_BM_Tick() - pd_last_get_src_cap_tick) >= pd_cfg.get_source_cap_interval_ms)
      && (pd_tx_busy == 0U)
      && (pd_rx_ready == 0U))
  {
    return 1U;
  }

  if ((pd_epr_mode_active != 0U) && (pd_state == PD_BM_STATE_EPR_READY)
      && ((PD_BM_Tick() - pd_last_epr_keepalive_tick) >= PD_EPR_KEEPALIVE_MS)
      && (pd_tx_busy == 0U)
      && (pd_rx_ready == 0U))
  {
    return 1U;
  }

  return 0U;
}

void PD_BM_TimerTickISR(void)
{
  if ((pd_cfg.ucpd == NULL) || (pd_cfg.get_tick_ms == NULL))
  {
    return;
  }

  /* Keep PD transmit out of the timer ISR; this only wakes the main task. */
  PD_BM_ScheduleEprKeepAliveIfDue();
}

void PD_BM_IRQHandler(void)
{
  uint32_t sr = pd_cfg.ucpd->SR;

  if ((sr & UCPD_SR_TYPECEVT1) != 0U)
  {
    LL_UCPD_ClearFlag_TypeCEventCC1(pd_cfg.ucpd);
    pd_typec_event_pending = 1U;
  }

  if ((sr & UCPD_SR_TYPECEVT2) != 0U)
  {
    LL_UCPD_ClearFlag_TypeCEventCC2(pd_cfg.ucpd);
    pd_typec_event_pending = 1U;
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
    PD_TRACE("hard_reset_rx", pd_state, pd_requested_voltage_mv, pd_requested_current_ma, pd_active_cc);
    if ((pd_epr_negotiating != 0U) && (pd_epr_fail_streak < 0xFFU))
    {
      pd_epr_fail_streak++;
      if (pd_epr_fail_streak >= PD_EPR_FAIL_STREAK_MAX)
      {
        pd_wants_epr = 0U;
        PD_TRACE("epr_disabled", pd_epr_fail_streak, 0U, 0U, 0U);
      }
    }
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
    PD_BM_RxQueuePushFromISR(pd_rx_count);
    if (pd_active_cc != PD_CC_NONE)
    {
      PD_BM_RxDMAArm();
    }
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

static uint8_t PD_BM_ConfigWantsEpr(void)
{
  for (uint8_t i = 1U; i < PD_BM_PROFILE_COUNT; i++)
  {
    const PD_BM_Profile *profile = &pd_cfg.profiles[i];
    if ((profile->enabled != 0U) && (profile->voltage_mv > 20000U)
        && (profile->current_ma != 0U))
    {
      return 1U;
    }
  }
  return 0U;
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
  pd_rx_queue_head = 0U;
  pd_rx_queue_tail = 0U;
  pd_msg_count = 0U;
  pd_request_pending = 0U;
  pd_epr_enter_pending = 0U;
  pd_epr_keepalive_pending = 0U;
  pd_chunk_request_pending = 0U;
  pd_chunk_request_msg_type = 0U;
  pd_chunk_request_number = 0U;
  pd_tx_busy = 0U;
  pd_typec_event_pending = 0U;
  pd_tx_msg_id = 0U;
  pd_rx_count = 0U;
  pd_active_cc = PD_CC_NONE;
  pd_active_profile = 0xFFU;
  pd_requested_voltage_mv = 0U;
  pd_requested_current_ma = 0U;
  pd_last_pdo_count = 0U;
  pd_last_get_src_cap_tick = 0U;
  pd_last_epr_keepalive_tick = 0U;
  pd_attach_tick = 0U;
  pd_last_src_pdo_count = 0U;
  pd_selected_position = 0U;
  pd_selected_current_10ma = 0U;
  pd_selected_pdo = 0U;
  pd_selected_is_epr = 0U;
  pd_epr_source_capable = 0U;
  pd_source_unchunked_capable = 0U;
  pd_epr_mode_active = 0U;
  pd_epr_negotiating = 0U;
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
  pd_typec_event_pending = 1U;
  pd_state = PD_BM_STATE_DETACHED;
}

static void PD_BM_Attach(PD_CC cc)
{
  PD_TRACE("attach", cc, LL_UCPD_GetTypeCVstateCC1(pd_cfg.ucpd),
      LL_UCPD_GetTypeCVstateCC2(pd_cfg.ucpd), 0U);

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

static void PD_BM_RxQueuePushFromISR(uint8_t count)
{
  uint16_t header;
  uint8_t msg_type;
  uint8_t ndo;
  uint8_t is_extended;
  uint8_t next;

  if (count < 2U)
  {
    return;
  }
  if (count > PD_RX_BUFFER_SIZE)
  {
    count = PD_RX_BUFFER_SIZE;
  }

  header = (uint16_t)pd_rx_buffer[0] | ((uint16_t)pd_rx_buffer[1] << 8U);
  msg_type = PD_HEADER_TYPE(header);
  ndo = PD_HEADER_NDO(header);
  is_extended = PD_HEADER_EXT(header);

  /*
   * GoodCRC only acknowledges a message we sent. Do not queue it: the source
   * can follow immediately with Accept/PS_RDY/EPR_KeepAlive_Ack, so RX must be
   * armed again as quickly as possible.
   */
  if ((is_extended == 0U) && (ndo == 0U) && (msg_type == PD_MSG_GOODCRC))
  {
    return;
  }

  next = (uint8_t)((pd_rx_queue_head + 1U) % PD_RX_QUEUE_DEPTH);
  if (next == pd_rx_queue_tail)
  {
    PD_TRACE("rx_queue_full", count, PD_RX_QUEUE_DEPTH, pd_active_cc, 0U);
    return;
  }

  pd_rx_queue[pd_rx_queue_head].count = count;
  memcpy(pd_rx_queue[pd_rx_queue_head].data, pd_rx_buffer, count);
  pd_rx_queue_head = next;
  pd_rx_ready = 1U;
}

static uint8_t PD_BM_RxQueuePop(void)
{
  uint8_t tail;

  __disable_irq();
  if (pd_rx_queue_tail == pd_rx_queue_head)
  {
    pd_rx_ready = 0U;
    __enable_irq();
    return 0U;
  }

  tail = pd_rx_queue_tail;
  pd_msg_count = pd_rx_queue[tail].count;
  memcpy(pd_msg_buffer, pd_rx_queue[tail].data, pd_msg_count);
  pd_rx_queue_tail = (uint8_t)((tail + 1U) % PD_RX_QUEUE_DEPTH);
  pd_rx_ready = (pd_rx_queue_tail != pd_rx_queue_head) ? 1U : 0U;
  __enable_irq();

  return 1U;
}

static void PD_BM_ProcessMessage(void)
{
  uint16_t header;
  uint8_t msg_type;
  uint8_t ndo;
  uint8_t is_extended;
  uint8_t is_goodcrc;

  if (pd_msg_count < 2U)
  {
    return;
  }

  header = (uint16_t)pd_msg_buffer[0] | ((uint16_t)pd_msg_buffer[1] << 8U);
  msg_type = PD_HEADER_TYPE(header);
  ndo = PD_HEADER_NDO(header);
  is_extended = PD_HEADER_EXT(header);
  pd_last_spec_rev = PD_HEADER_SPEC(header);
  if (pd_last_spec_rev > PD_SPEC_REV_30)
  {
    pd_last_spec_rev = PD_SPEC_REV_30;
  }

  is_goodcrc = ((is_extended == 0U) && (ndo == 0U) && (msg_type == PD_MSG_GOODCRC)) ? 1U : 0U;
  if (is_goodcrc == 0U)
  {
    (void)PD_BM_SendGoodCRC(header);
    PD_TRACE("rx", header, msg_type, ndo, pd_msg_count);
  }

  if ((is_extended == 0U) && (ndo > 0U) && (msg_type == PD_DATA_SRC_CAP))
  {
    uint8_t count = ndo;

    if (count > PD_MAX_PDO_COUNT)
    {
      count = PD_MAX_PDO_COUNT;
    }
    if ((2U + ((uint16_t)count * 4U)) > pd_msg_count)
    {
      return;
    }

    if ((pd_request_pending != 0U)
        || (pd_state == PD_BM_STATE_REQUEST_SENT)
        || (pd_state == PD_BM_STATE_ACCEPT_RX)
        || (pd_state == PD_BM_STATE_PS_RDY_RX)
        || (pd_state == PD_BM_STATE_EPR_ENTER_SENT)
        || (pd_state == PD_BM_STATE_EPR_READY))
    {
      PD_TRACE("src_cap_ignored", pd_state, pd_request_pending,
          pd_selected_position, PD_HEADER_MSGID(header));
      return;
    }

    for (uint8_t i = 0U; i < count; i++)
    {
      uint8_t pdo_type;
      uint32_t volt_mv;
      uint32_t curr_ma;
      pd_last_src_pdo[i] = PD_BM_ReadLE32(&pd_msg_buffer[2U + ((uint16_t)i * 4U)]);
      pdo_type = (uint8_t)PD_PDO_TYPE(pd_last_src_pdo[i]);
      volt_mv = (pdo_type == 0U) ? PD_PDO_FIXED_VOLT_MV(pd_last_src_pdo[i]) : 0U;
      curr_ma = (pdo_type == 0U)
          ? ((uint32_t)PD_PDO_FIXED_CURR_10MA(pd_last_src_pdo[i]) * 10U)
          : 0U;
      PD_TRACE("spr_pdo", i + 1U, pd_last_src_pdo[i], volt_mv, curr_ma);
    }

    pd_epr_source_capable = ((count > 0U) && (PD_PDO_TYPE(pd_last_src_pdo[0]) == 0U)
        && (PD_PDO_FIXED_EPR_CAPABLE(pd_last_src_pdo[0]) != 0U)) ? 1U : 0U;
    pd_source_unchunked_capable = ((count > 0U) && (PD_PDO_TYPE(pd_last_src_pdo[0]) == 0U)
        && (PD_PDO_FIXED_UNCHUNKED_EXT(pd_last_src_pdo[0]) != 0U)) ? 1U : 0U;
    PD_TRACE("spr_src_cap", count, pd_epr_source_capable, pd_wants_epr, pd_last_src_pdo[0]);
    PD_TRACE("spr_flags", pd_epr_source_capable, pd_source_unchunked_capable, 0U, 0U);

    pd_last_src_pdo_count = count;
    pd_last_pdo_count = count;
    pd_state = PD_BM_STATE_SRC_CAP_RX;
    pd_epr_mode_active = 0U;
    pd_selected_is_epr = 0U;

    if ((pd_epr_source_capable != 0U) && (pd_wants_epr != 0U))
    {
      if (PD_BM_SelectSprForEprEntry() != 0U)
      {
        pd_request_pending = 1U;
      }
      else if (PD_BM_SelectProfileFromSourceCaps((int8_t)(PD_BM_PROFILE_COUNT - 1U)) != 0U)
      {
        pd_request_pending = 1U;
      }
      else
      {
        pd_state = PD_BM_STATE_ERROR;
      }
    }
    else if (PD_BM_SelectProfileFromSourceCaps((int8_t)(PD_BM_PROFILE_COUNT - 1U)) != 0U)
    {
      pd_request_pending = 1U;
    }
    else
    {
      pd_state = PD_BM_STATE_ERROR;
    }
  }
  else if ((is_extended != 0U) && (msg_type == PD_EXT_EXTENDED_CONTROL))
  {
    uint16_t ext_header;
    uint16_t data_size;
    uint8_t ext_type = 0U;
    uint8_t ext_data = 0U;

    if (pd_msg_count < 6U)
    {
      return;
    }

    ext_header = (uint16_t)pd_msg_buffer[2] | ((uint16_t)pd_msg_buffer[3] << 8U);
    data_size = (uint16_t)(ext_header & 0x1FFU);
    if (data_size >= 1U)
    {
      ext_type = pd_msg_buffer[4];
    }
    if (data_size >= 2U)
    {
      ext_data = pd_msg_buffer[5];
    }

    PD_TRACE("ext_ctrl_rx", ext_header, ext_type, ext_data, pd_msg_count);

    if (ext_type == PD_EXT_CTRL_EPR_KEEPALIVE_ACK)
    {
      pd_last_epr_keepalive_tick = PD_BM_Tick();
      PD_TRACE("epr_keepalive_ack", pd_tx_msg_id, pd_state, 0U, 0U);
    }
  }
  else if ((is_extended != 0U) && (msg_type == PD_EXT_EPR_SRC_CAP))
  {
    uint16_t ext_header;
    uint16_t data_size;
    uint8_t chunked;
    uint8_t chunk_number;
    uint8_t request_chunk;

    if (pd_msg_count < 4U)
    {
      return;
    }

    ext_header = (uint16_t)pd_msg_buffer[2] | ((uint16_t)pd_msg_buffer[3] << 8U);
    data_size = (uint16_t)(ext_header & 0x1FFU);
    chunked = (uint8_t)((ext_header >> 15U) & 0x01U);
    request_chunk = (uint8_t)((ext_header >> 10U) & 0x01U);
    chunk_number = (uint8_t)((ext_header >> 11U) & 0x0FU);
    PD_TRACE("epr_src_cap_rx", ext_header, data_size, chunked, chunk_number);

    if ((chunked != 0U) && (request_chunk == 0U))
    {
      uint16_t offset = (uint16_t)((uint16_t)chunk_number * PD_EXT_CHUNK_SIZE);
      uint16_t chunk_bytes = (pd_msg_count > 4U) ? (uint16_t)(pd_msg_count - 4U) : 0U;

      if (offset >= sizeof(pd_last_src_pdo))
      {
        chunk_bytes = 0U;
      }
      else if (offset + chunk_bytes > sizeof(pd_last_src_pdo))
      {
        chunk_bytes = (uint16_t)(sizeof(pd_last_src_pdo) - offset);
      }

      if (chunk_bytes > 0U)
      {
        memcpy(((uint8_t *)pd_last_src_pdo) + offset, &pd_msg_buffer[4], chunk_bytes);
      }

      if ((offset + chunk_bytes) >= data_size)
      {
        uint16_t total_bytes = data_size;
        uint8_t epr_count;
        if (total_bytes > sizeof(pd_last_src_pdo))
        {
          total_bytes = sizeof(pd_last_src_pdo);
        }
        epr_count = (uint8_t)(total_bytes / 4U);
        if (epr_count > PD_MAX_PDO_COUNT)
        {
          epr_count = PD_MAX_PDO_COUNT;
        }
        pd_last_src_pdo_count = epr_count;
        pd_last_pdo_count = epr_count;
        for (uint8_t i = 0U; i < epr_count; i++)
        {
          uint8_t pdo_type = (uint8_t)PD_PDO_TYPE(pd_last_src_pdo[i]);
          uint32_t volt_mv = (pdo_type == 0U) ? PD_PDO_FIXED_VOLT_MV(pd_last_src_pdo[i]) : 0U;
          uint32_t curr_ma = (pdo_type == 0U) ?
              ((uint32_t)PD_PDO_FIXED_CURR_10MA(pd_last_src_pdo[i]) * 10U) : 0U;
          PD_TRACE("epr_pdo", i + 1U, pd_last_src_pdo[i], volt_mv, curr_ma);
        }
        if (PD_BM_SelectEprPdo() != 0U)
        {
          pd_request_pending = 1U;
        }
        else
        {
          pd_state = PD_BM_STATE_ERROR;
        }
      }
      else
      {
        /*
         * Spec PD 3.1 section 6.13: receiver must request the next chunk
         * within tChunkReceiverRequest (~15 ms). Queue the request; the main
         * task will dispatch it on the next iteration. Without this, the
         * source hard-resets after tChunkSenderResponse and we never see the
         * EPR PDOs (28 V / 36 V / 48 V).
         */
        pd_chunk_request_msg_type = PD_EXT_EPR_SRC_CAP;
        pd_chunk_request_number = (uint8_t)(chunk_number + 1U);
        pd_chunk_request_pending = 1U;
      }
    }
  }
  else if ((is_extended == 0U) && (ndo > 0U) && (msg_type == PD_DATA_EPR_MODE))
  {
    uint32_t epr_mdo = PD_BM_ReadLE32(&pd_msg_buffer[2]);
    uint8_t action = (uint8_t)((epr_mdo >> 24U) & 0xFFU);
    PD_TRACE("epr_mode_rx", action, epr_mdo, 0U, 0U);

    if (action == PD_EPR_ACT_ENTER_ACK)
    {
      pd_state = PD_BM_STATE_EPR_ENTER_SENT;
    }
    else if (action == PD_EPR_ACT_ENTER_OK)
    {
      pd_epr_mode_active = 1U;
      pd_last_epr_keepalive_tick = PD_BM_Tick();
    }
    else if (action == PD_EPR_ACT_ENTER_FAIL)
    {
      pd_epr_mode_active = 0U;
      pd_state = (pd_requested_voltage_mv != 0U) ? PD_BM_STATE_READY : PD_BM_STATE_ERROR;
    }
    else if (action == PD_EPR_ACT_KEEPALIVE_ACK)
    {
      /* Source acknowledged our keepalive — refresh our timer reference. */
      pd_last_epr_keepalive_tick = PD_BM_Tick();
    }
  }
  else if ((is_extended == 0U) && (ndo == 0U) && (msg_type == PD_MSG_ACCEPT))
  {
    PD_TRACE("accept", pd_selected_is_epr, pd_requested_voltage_mv,
        pd_requested_current_ma, pd_active_profile);
    pd_state = PD_BM_STATE_ACCEPT_RX;
  }
  else if ((is_extended == 0U) && (ndo == 0U) && (msg_type == PD_MSG_PS_RDY))
  {
    PD_TRACE("ps_rdy", pd_selected_is_epr, pd_requested_voltage_mv,
        pd_requested_current_ma, pd_epr_mode_active);
    if (pd_requested_voltage_mv != 0U)
    {
      if ((pd_selected_is_epr == 0U) && (pd_epr_mode_active == 0U)
          && (pd_epr_source_capable != 0U) && (pd_wants_epr != 0U)
          && (pd_requested_voltage_mv >= 20000U))
      {
        pd_state = PD_BM_STATE_PS_RDY_RX;
        pd_epr_enter_pending = 1U;
      }
      else if (pd_epr_mode_active != 0U)
      {
        pd_state = PD_BM_STATE_EPR_READY;
        pd_last_epr_keepalive_tick = PD_BM_Tick();
        pd_epr_negotiating = 0U;
        pd_epr_fail_streak = 0U;
      }
      else
      {
        pd_state = PD_BM_STATE_READY;
      }
    }
    else
    {
      pd_state = PD_BM_STATE_PS_RDY_RX;
    }
  }
  else if ((is_extended == 0U) && (ndo == 0U) && ((msg_type == PD_MSG_REJECT) || (msg_type == PD_MSG_WAIT)))
  {
    PD_TRACE("reject_wait", msg_type, pd_selected_is_epr, pd_active_profile,
        pd_requested_voltage_mv);
    if (PD_BM_RequestNextLowerProfile() != 0U)
    {
      pd_request_pending = 1U;
    }
    else
    {
      pd_state = PD_BM_STATE_ERROR;
    }
  }
  else if ((is_extended == 0U) && (ndo == 0U) && (msg_type == PD_MSG_NOT_SUPPORTED))
  {
    PD_TRACE("not_supported", pd_state, pd_requested_voltage_mv, pd_selected_is_epr,
        pd_source_unchunked_capable);
  }
  else if ((is_extended == 0U) && (ndo == 0U) && (msg_type == PD_MSG_SOFT_RESET))
  {
    PD_TRACE("soft_reset_rx", pd_state, pd_requested_voltage_mv, pd_selected_is_epr, 0U);
    pd_tx_msg_id = 0U;
    pd_request_pending = 0U;
    pd_epr_enter_pending = 0U;
    pd_epr_keepalive_pending = 0U;
    pd_chunk_request_pending = 0U;
    pd_epr_mode_active = 0U;
    pd_selected_is_epr = 0U;
    PD_BM_SendControl(PD_MSG_ACCEPT);
    pd_state = PD_BM_STATE_ATTACHED;
    pd_last_get_src_cap_tick = PD_BM_Tick() - pd_cfg.get_source_cap_interval_ms;
  }

}

static void PD_BM_SendGoodCRC(uint16_t rx_header)
{
  uint8_t tx[2];
  uint16_t header = PD_BM_MakeHeader(PD_MSG_GOODCRC, 0U,
      PD_HEADER_MSGID(rx_header), PD_HEADER_SPEC(rx_header));

  PD_BM_WriteLE16(tx, header);
  (void)PD_BM_SendSOP(tx, sizeof(tx));
}

static void PD_BM_SendControl(uint8_t msg_type)
{
  uint8_t tx[2];
  uint16_t header = PD_BM_MakeHeader(msg_type, 0U, pd_tx_msg_id, pd_last_spec_rev);

  PD_TRACE("tx_ctrl", msg_type, pd_tx_msg_id, 0U, 0U);
  PD_BM_WriteLE16(tx, header);
  if (PD_BM_SendSOP(tx, sizeof(tx)) != 0U)
  {
    pd_tx_msg_id = (uint8_t)((pd_tx_msg_id + 1U) & 0x07U);
  }
}

static uint8_t PD_BM_SendSOP(const uint8_t *payload, uint8_t size)
{
  uint32_t start_tick;

  if ((payload == NULL) || (size == 0U))
  {
    return 0U;
  }

  /*
   * Clear stale TX status flags before starting a fresh TX. Without this,
   * leftover TXMSGDISC / TXMSGABT / TXUND / TXMSGSENT from prior activity
   * (e.g. a TX completed by polling on the previous call but the next call
   * starts before the bits are cleared) can short-circuit the final wait
   * loop or break TXIS sequencing, which is the most common cause of the
   * intermittent "tx_*_fail" right after RX of a chunked extended message.
   */
  pd_cfg.ucpd->ICR = UCPD_ICR_TXMSGSENTCF | UCPD_ICR_TXMSGDISCCF
      | UCPD_ICR_TXMSGABTCF | UCPD_ICR_TXUNDCF;

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

  PD_TRACE("tx_get_src_cap", pd_tx_msg_id, pd_state, 0U, 0U);
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

  if ((pd_epr_source_capable != 0U) && (pd_wants_epr != 0U))
  {
    rdo |= PD_RDO_EPR_CAPABLE;
  }

  PD_TRACE("tx_request", profile_index, pd_selected_position, rdo, pd_selected_pdo);
  header = PD_BM_MakeHeader(PD_DATA_REQUEST, 1U, pd_tx_msg_id, pd_last_spec_rev);
  PD_BM_WriteLE16(&tx[0], header);
  PD_BM_WriteLE32(&tx[2], rdo);

  uint8_t send_ok = PD_BM_SendSOP(tx, sizeof(tx));
  if (send_ok == 0U)
  {
    PD_TRACE("tx_request_retry", profile_index, pd_selected_position,
        pd_cfg.ucpd->SR, 0U);
    send_ok = PD_BM_SendSOP(tx, sizeof(tx));
  }

  if (send_ok != 0U)
  {
    pd_state = PD_BM_STATE_REQUEST_SENT;
    pd_tx_msg_id = (uint8_t)((pd_tx_msg_id + 1U) & 0x07U);
  }
  else
  {
    PD_TRACE("tx_request_fail", profile_index, pd_selected_position,
        pd_state, pd_cfg.ucpd->SR);
    pd_state = PD_BM_STATE_ERROR;
  }
}

static void PD_BM_RequestEprProfile(void)
{
  uint8_t tx[10];
  uint16_t header;
  uint32_t rdo;
  uint8_t send_ok;

  if ((pd_selected_position == 0U) || (pd_selected_pdo == 0U))
  {
    pd_state = PD_BM_STATE_ERROR;
    return;
  }

  rdo = ((uint32_t)pd_selected_position << 28U)
      | PD_RDO_EPR_CAPABLE
      | ((uint32_t)pd_selected_current_10ma << 10U)
      | ((uint32_t)pd_selected_current_10ma);

  PD_TRACE("tx_epr_request", pd_active_profile, pd_selected_position, rdo, pd_selected_pdo);
  header = PD_BM_MakeHeader(PD_DATA_EPR_REQUEST, 2U, pd_tx_msg_id, pd_last_spec_rev);
  PD_BM_WriteLE16(&tx[0], header);
  PD_BM_WriteLE32(&tx[2], rdo);
  PD_BM_WriteLE32(&tx[6], pd_selected_pdo);

  /*
   * Single-shot retry: the very first TX right after receiving the last chunk
   * of EPR_Source_Capabilities sometimes fails because UCPD TX FIFO sees a
   * stale status bit (see SendSOP). Retrying with cleared flags almost always
   * succeeds and stays well within tSenderResponse (~24 ms).
   */
  send_ok = PD_BM_SendSOP(tx, sizeof(tx));
  if (send_ok == 0U)
  {
    PD_TRACE("tx_epr_request_retry", pd_active_profile, pd_selected_position,
        pd_cfg.ucpd->SR, 0U);
    send_ok = PD_BM_SendSOP(tx, sizeof(tx));
  }

  if (send_ok != 0U)
  {
    pd_state = PD_BM_STATE_REQUEST_SENT;
    pd_last_epr_keepalive_tick = PD_BM_Tick();
    pd_tx_msg_id = (uint8_t)((pd_tx_msg_id + 1U) & 0x07U);
  }
  else
  {
    PD_TRACE("tx_epr_request_fail", pd_active_profile, pd_selected_position,
        pd_state, pd_cfg.ucpd->SR);
    pd_state = PD_BM_STATE_ERROR;
  }
}

static void PD_BM_SendEprModeEnter(void)
{
  uint8_t tx[6];
  uint16_t header;
  uint32_t epr_mdo;
  uint16_t sink_pdp_w = 28U;

  for (uint8_t i = 1U; i < PD_BM_PROFILE_COUNT; i++)
  {
    const PD_BM_Profile *profile = &pd_cfg.profiles[i];
    if ((profile->enabled != 0U) && (profile->voltage_mv > 20000U))
    {
      uint32_t claimed_current_ma = profile->current_ma;
      uint32_t pdp;

      /*
       * Our profile current is the minimum acceptable current, but the stack
       * requests the source PDO current when a matching PDO is available. For
       * 28 V fixed EPR this can be 5 A, so the EPR Enter PDP must budget for
       * that possible request (28 V * 5 A = 140 W). Advertising only 84 W and
       * then requesting 140 W is rejected by stricter sources after PS_RDY.
       */
      if (claimed_current_ma < 5000U)
      {
        claimed_current_ma = 5000U;
      }
      pdp = (((uint32_t)profile->voltage_mv * claimed_current_ma) + 999999U) / 1000000U;
      if (pdp > sink_pdp_w)
      {
        sink_pdp_w = (uint16_t)pdp;
      }
    }
  }
  if (sink_pdp_w < 28U)
  {
    sink_pdp_w = 28U;
  }
  if (sink_pdp_w > 240U)
  {
    sink_pdp_w = 240U;
  }

  epr_mdo = ((uint32_t)PD_EPR_ACT_ENTER << 24U) | ((uint32_t)sink_pdp_w << 16U);

  PD_TRACE("tx_epr_enter", pd_tx_msg_id, sink_pdp_w, pd_requested_voltage_mv, pd_requested_current_ma);
  header = PD_BM_MakeHeader(PD_DATA_EPR_MODE, 1U, pd_tx_msg_id, pd_last_spec_rev);
  PD_BM_WriteLE16(&tx[0], header);
  PD_BM_WriteLE32(&tx[2], epr_mdo);

  uint8_t send_ok = PD_BM_SendSOP(tx, sizeof(tx));
  if (send_ok == 0U)
  {
    PD_TRACE("tx_epr_enter_retry", pd_tx_msg_id, pd_state, pd_cfg.ucpd->SR, 0U);
    send_ok = PD_BM_SendSOP(tx, sizeof(tx));
  }

  if (send_ok != 0U)
  {
    pd_state = PD_BM_STATE_EPR_ENTER_SENT;
    pd_epr_negotiating = 1U;
    pd_tx_msg_id = (uint8_t)((pd_tx_msg_id + 1U) & 0x07U);
  }
  else
  {
    PD_TRACE("tx_epr_enter_fail", pd_tx_msg_id, pd_state, pd_cfg.ucpd->SR, 0U);
  }
}

static void PD_BM_SendEprKeepAlive(void)
{
  uint8_t tx[6];
  uint16_t header;
  uint16_t ext_header;
  uint8_t chunked = (pd_source_unchunked_capable == 0U) ? 1U : 0U;

  /*
   * EPR_KeepAlive is an Extended_Control message, not an EPR_Mode data
   * message. ECDB: Type=3 (EPR_KeepAlive), Data=0. The source answers with
   * Extended_Control Type=4 (EPR_KeepAlive_Ack).
   */
  PD_TRACE("tx_epr_keepalive", pd_tx_msg_id, PD_EXT_CTRL_EPR_KEEPALIVE, chunked, 0U);
  header = PD_BM_MakeHeader(PD_EXT_EXTENDED_CONTROL, 1U, pd_tx_msg_id, pd_last_spec_rev);
  header |= 0x8000U;  /* Extended */
  ext_header = (uint16_t)(2U | ((uint16_t)chunked << 15U));
  PD_BM_WriteLE16(&tx[0], header);
  PD_BM_WriteLE16(&tx[2], ext_header);
  tx[4] = PD_EXT_CTRL_EPR_KEEPALIVE;
  tx[5] = 0U;

  if (PD_BM_SendSOP(tx, sizeof(tx)) != 0U)
  {
    pd_last_epr_keepalive_tick = PD_BM_Tick();
    pd_tx_msg_id = (uint8_t)((pd_tx_msg_id + 1U) & 0x07U);
  }
  else
  {
    PD_TRACE("tx_epr_keepalive_fail", pd_tx_msg_id, 0U, 0U, 0U);
  }
}

/*
 * Send a Chunk_Request Chunked Extended Message asking for the next chunk of
 * an already-in-progress chunked extended message (PD 3.1 spec 6.13.2).
 *
 * Payload layout (6 bytes):
 *   [0..1] PD header  (Extended=1, NDO=1, MessageType=<msg_type>, MsgID, SpecRev)
 *   [2..3] Ext header (Chunked=1, ChunkNumber=N+1, RequestChunk=1, DataSize=0)
 *   [4..5] Padding to round out one 4-byte data object
 */
static void PD_BM_SendChunkRequest(uint8_t msg_type, uint8_t chunk_number)
{
  uint8_t tx[6];
  uint16_t header;
  uint16_t ext_header;

  header = PD_BM_MakeHeader(msg_type, 1U, pd_tx_msg_id, pd_last_spec_rev);
  header |= 0x8000U;  /* Extended */

  ext_header = (uint16_t)((1U << 15U)
      | (((uint16_t)chunk_number & 0x0FU) << 11U)
      | (1U << 10U));

  PD_BM_WriteLE16(&tx[0], header);
  PD_BM_WriteLE16(&tx[2], ext_header);
  tx[4] = 0U;
  tx[5] = 0U;

  PD_TRACE("tx_chunk_req", msg_type, chunk_number, ext_header, pd_tx_msg_id);
  if (PD_BM_SendSOP(tx, sizeof(tx)) != 0U)
  {
    pd_tx_msg_id = (uint8_t)((pd_tx_msg_id + 1U) & 0x07U);
  }
  else
  {
    PD_TRACE("tx_chunk_req_fail", msg_type, chunk_number, 0U, 0U);
  }
}

static void PD_BM_ScheduleEprKeepAliveIfDue(void)
{
  uint32_t now = PD_BM_Tick();

  if ((pd_epr_keepalive_pending == 0U)
      && (pd_epr_mode_active != 0U)
      && (pd_state == PD_BM_STATE_EPR_READY)
      && ((now - pd_last_epr_keepalive_tick) >= PD_EPR_KEEPALIVE_MS)
      && (pd_tx_busy == 0U)
      && (pd_rx_ready == 0U))
  {
    pd_last_epr_keepalive_tick = now;
    pd_epr_keepalive_pending = 1U;
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
    if (profile->voltage_mv > 20000U)
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
        pd_selected_pdo = pdo;
        pd_selected_is_epr = 0U;
        pd_requested_voltage_mv = profile->voltage_mv;
        pd_requested_current_ma = (uint16_t)(pd_selected_current_10ma * 10U);
        PD_TRACE("select", profile_index, pd_selected_position,
            pd_requested_voltage_mv, pd_requested_current_ma);
        return 1U;
      }
    }
  }

  PD_TRACE("select_none", start_index, pd_last_src_pdo_count, pd_epr_mode_active,
      pd_epr_source_capable);
  return 0U;
}

static uint8_t PD_BM_SelectSprForEprEntry(void)
{
  for (uint8_t i = 0U; i < pd_last_src_pdo_count; i++)
  {
    uint32_t pdo = pd_last_src_pdo[i];
    if ((PD_PDO_TYPE(pdo) == 0U) && (PD_PDO_FIXED_VOLT_MV(pdo) == 20000U)
        && (PD_PDO_FIXED_CURR_10MA(pdo) >= 500U))
    {
      pd_active_profile = 0U;
      pd_selected_position = (uint8_t)(i + 1U);
      pd_selected_current_10ma = 500U;
      pd_selected_pdo = pdo;
      pd_selected_is_epr = 0U;
      pd_requested_voltage_mv = 20000U;
      pd_requested_current_ma = 5000U;
      PD_TRACE("select_spr_for_epr", 0U, pd_selected_position,
          pd_requested_voltage_mv, pd_requested_current_ma);
      return 1U;
    }
  }
  return 0U;
}

/*
 * Pick an EPR Fixed PDO from the received EPR_Source_Capabilities.
 *
 * Selection policy (avoid frying loads designed for a specific voltage):
 *   1) Prefer EXACT voltage match to the configured EPR profile (e.g. 28 V).
 *   2) Otherwise, pick the LOWEST PDO whose voltage >= target and whose
 *      current rating >= target. (Not the highest — picking 48 V for a load
 *      that asked for 28 V can be dangerous and is not what the user wants.)
 */
static uint8_t PD_BM_SelectEprPdo(void)
{
  uint8_t best_idx = 0U;
  uint32_t best_volt = 0U;
  uint32_t best_pdo = 0U;
  uint16_t target_voltage = 0U;
  uint16_t target_current_ma = 0U;

  for (uint8_t i = 1U; i < PD_BM_PROFILE_COUNT; i++)
  {
    const PD_BM_Profile *profile = &pd_cfg.profiles[i];
    if ((profile->enabled != 0U) && (profile->voltage_mv > 20000U)
        && (profile->current_ma != 0U)
        && (profile->voltage_mv > target_voltage))
    {
      target_voltage = profile->voltage_mv;
      target_current_ma = profile->current_ma;
      pd_active_profile = i;
    }
  }

  if (target_voltage == 0U)
  {
    PD_TRACE("epr_select_no_profile", pd_last_src_pdo_count, 0U, 0U, 0U);
    return 0U;
  }

  /* Pass 1: exact-voltage match. */
  for (uint8_t i = 0U; i < pd_last_src_pdo_count; i++)
  {
    uint32_t pdo = pd_last_src_pdo[i];
    uint32_t volt;
    uint16_t cur;

    if (PD_PDO_TYPE(pdo) != 0U)
    {
      continue;
    }
    volt = PD_EPR_PDO_VOLT_MV(pdo);
    cur = PD_PDO_FIXED_CURR_10MA(pdo);
    if ((volt == (uint32_t)target_voltage)
        && ((uint32_t)cur * 10U >= (uint32_t)target_current_ma))
    {
      best_idx = (uint8_t)(i + 1U);
      best_volt = volt;
      best_pdo = pdo;
      break;
    }
  }

  /* Pass 2: lowest voltage that meets the target. */
  if (best_idx == 0U)
  {
    uint32_t lowest = 0xFFFFFFFFU;
    for (uint8_t i = 0U; i < pd_last_src_pdo_count; i++)
    {
      uint32_t pdo = pd_last_src_pdo[i];
      uint32_t volt;
      uint16_t cur;

      if (PD_PDO_TYPE(pdo) != 0U)
      {
        continue;
      }
      volt = PD_EPR_PDO_VOLT_MV(pdo);
      cur = PD_PDO_FIXED_CURR_10MA(pdo);
      if ((volt >= (uint32_t)target_voltage)
          && ((uint32_t)cur * 10U >= (uint32_t)target_current_ma)
          && (volt < lowest))
      {
        best_idx = (uint8_t)(i + 1U);
        best_volt = volt;
        best_pdo = pdo;
        lowest = volt;
      }
    }
  }

  if (best_idx == 0U)
  {
    PD_TRACE("epr_select_none", pd_last_src_pdo_count, target_voltage, 0U, 0U);
    return 0U;
  }

  pd_selected_position = best_idx;
  pd_selected_current_10ma = PD_PDO_FIXED_CURR_10MA(best_pdo);
  pd_selected_pdo = best_pdo;
  pd_selected_is_epr = 1U;
  pd_requested_voltage_mv = (uint16_t)best_volt;
  pd_requested_current_ma = (uint16_t)((uint32_t)pd_selected_current_10ma * 10U);
  PD_TRACE("epr_select", pd_active_profile, pd_selected_position,
      pd_requested_voltage_mv, pd_requested_current_ma);
  return 1U;
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
    PD_TRACE("detach", pd_state, pd_requested_voltage_mv, pd_requested_current_ma, 0U);
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
