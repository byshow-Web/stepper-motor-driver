/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
 * @brief          : Y-axis STEP/DIR controller with two limit-sensor inputs.
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#define FRAME_HEADER_1             0xAAU
#define FRAME_HEADER_2             0x55U
#define FRAME_CMD_MOVE_LEGACY      0x01U
#define FRAME_CMD_STOP_LEGACY      0x02U
#define FRAME_CMD_INFO              0x10U
#define FRAME_CMD_MOVE_AXIS         0x11U
#define FRAME_CMD_STOP_AXIS         0x12U
#define FRAME_CMD_GET_STATE         0x13U
#define FRAME_CMD_MOVE_SYNC          0x14U
#define FRAME_CMD_GET_SENSOR          0x15U
#define FRAME_CMD_INFO_REPLY        0x80U
#define FRAME_CMD_STATUS            0x81U
#define FRAME_CMD_SENSOR_REPLY      0x82U
#define FRAME_MAX_PAYLOAD           24U

#define STEPPER_AXIS_COUNT          1U
#define STEPPER_ALL_AXES            0xFFU
#define STEPPER_MIN_PPS             100U
#define STEPPER_MAX_PPS             40000U
#define STEPPER_DEFAULT_ACCEL        1000U
#define STEPPER_MIN_ACCEL            100U
#define STEPPER_MAX_ACCEL            20000U

#define STATUS_ACCEPTED             0x00U
#define STATUS_DONE                 0x01U
#define STATUS_ERROR                0x02U
#define STATUS_STOPPED              0x03U
#define STATUS_BUSY                 0x04U
#define STATUS_READY                0x10U
#define STATUS_STATE                0x20U

static volatile uint8_t rx_state;
static volatile uint8_t rx_command;
static volatile uint8_t rx_length;
static volatile uint8_t rx_index;
static volatile uint8_t rx_crc;
static uint8_t rx_payload[FRAME_MAX_PAYLOAD];
static volatile uint8_t frame_ready;
static uint8_t sensor_last_state[2] = {0xFFU, 0xFFU};
static uint32_t sensor_poll_tick;

volatile StepperAxis stepper_axes[STEPPER_AXIS_COUNT];

void SystemClock_Config(void);
static void MX_USART1_Init(void);
static void Process_Frame(void);
static uint8_t CRC8_Update(uint8_t crc, uint8_t data);
static void USART1_SendByte(uint8_t byte);
static void Transport_Send(const uint8_t *data, uint16_t length);
static void USART1_SendFrame(uint8_t command, const uint8_t *payload, uint8_t length);
static void Send_Info(void);
static void Send_Status(uint8_t axis, uint8_t status, uint16_t request_id, uint32_t executed_steps);
static void Send_AxisState(uint8_t axis);
static void Send_SensorState(void);
static void Send_OneSensorState(uint8_t sensor_id, uint8_t state);
static uint8_t Read_SensorState(uint8_t sensor_id);
static void Sensor_Poll(void);
static void Axis_Start(uint8_t axis, int32_t steps, uint32_t speed_pps, uint32_t accel_pps2, uint16_t request_id);
static void Axis_Stop(uint8_t axis, uint8_t report);
static void Axis_SetPeriod(uint8_t axis, uint32_t speed_pps);
static uint32_t ReadU32(const uint8_t *data);
static uint16_t ReadU16(const uint8_t *data);
static uint32_t AbsI32(int32_t value);

int main(void)
{
  uint8_t axis;
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_Init();
  MX_USB_DEVICE_Init();

  HAL_GPIO_WritePin(YL_STEP_GPIO_Port, YL_STEP_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(YL_DIR_GPIO_Port, YL_DIR_Pin, GPIO_PIN_SET);
  Send_Status(STEPPER_ALL_AXES, STATUS_READY, 0U, 0U);

  while (1)
  {
    Sensor_Poll();
    if (frame_ready)
    {
      Process_Frame();
      frame_ready = 0;
    }
    for (axis = 0; axis < STEPPER_AXIS_COUNT; axis++)
    {
      if (stepper_axes[axis].done_pending)
      {
        stepper_axes[axis].done_pending = 0;
        Send_Status(axis, STATUS_DONE, stepper_axes[axis].request_id, stepper_axes[axis].step_count);
      }
    }
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};
  RCC_PeriphCLKInitTypeDef periph_clk = {0};
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_ON;
  osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  osc.HSIState = RCC_HSI_ON;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();
  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV2;
  clk.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
  periph_clk.PeriphClockSelection = RCC_PERIPHCLK_USB;
  periph_clk.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&periph_clk) != HAL_OK) Error_Handler();
}

static void MX_USART1_Init(void)
{
  GPIO_InitTypeDef gpio = {0};
  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  gpio.Pin = GPIO_PIN_9;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);
  gpio.Pin = GPIO_PIN_10;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &gpio);
  USART1->BRR = HAL_RCC_GetPCLK2Freq() / 115200U;
  USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void USART1_CommandIRQ(void)
{
  uint8_t byte;
  if ((USART1->SR & USART_SR_RXNE) == 0U) return;
  byte = (uint8_t)USART1->DR;
  Protocol_ReceiveByte(byte);
}

/* UART and USB CDC both call this same byte parser. */
void Protocol_ReceiveByte(uint8_t byte)
{
  if (frame_ready) return;
  switch (rx_state)
  {
    case 0: if (byte == FRAME_HEADER_1) rx_state = 1; break;
    case 1: rx_state = (byte == FRAME_HEADER_2) ? 2 : 0; break;
    case 2: rx_command = byte; rx_crc = CRC8_Update(0U, byte); rx_state = 3; break;
    case 3:
      rx_length = byte; rx_crc = CRC8_Update(rx_crc, byte); rx_index = 0;
      rx_state = (byte == 0U) ? 5 : ((byte <= FRAME_MAX_PAYLOAD) ? 4 : 0);
      break;
    case 4:
      rx_payload[rx_index++] = byte; rx_crc = CRC8_Update(rx_crc, byte);
      if (rx_index >= rx_length) rx_state = 5;
      break;
    default:
      if (byte == rx_crc) frame_ready = 1;
      rx_state = 0;
      break;
  }
}

static uint8_t CRC8_Update(uint8_t crc, uint8_t data)
{
  uint8_t bit;
  crc ^= data;
  for (bit = 0; bit < 8U; bit++)
    crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x07U) : (uint8_t)(crc << 1);
  return crc;
}

static void USART1_SendByte(uint8_t byte)
{
  while ((USART1->SR & USART_SR_TXE) == 0U) { }
  USART1->DR = byte;
}

static void USART1_SendFrame(uint8_t command, const uint8_t *payload, uint8_t length)
{
  uint8_t crc = 0U, index, frame[FRAME_MAX_PAYLOAD + 5U];
  frame[0] = FRAME_HEADER_1; frame[1] = FRAME_HEADER_2;
  frame[2] = command; frame[3] = length;
  crc = CRC8_Update(crc, command); crc = CRC8_Update(crc, length);
  for (index = 0; index < length; index++) { frame[4U + index] = payload[index]; crc = CRC8_Update(crc, payload[index]); }
  frame[4U + length] = crc;
  Transport_Send(frame, (uint16_t)length + 5U);
}

static void Transport_Send(const uint8_t *data, uint16_t length)
{
  uint16_t i;
  /* USB is the primary link.  UART is retained as a debug fallback. */
  (void)CDC_Transmit_FS((uint8_t *)data, length);
  for (i = 0U; i < length; i++) USART1_SendByte(data[i]);
}

static void Send_Info(void)
{
  const uint8_t info[] = {1U, 1U, STEPPER_AXIS_COUNT};
  USART1_SendFrame(FRAME_CMD_INFO_REPLY, info, sizeof(info));
}

static void Send_Status(uint8_t axis, uint8_t status, uint16_t request_id, uint32_t executed_steps)
{
  uint8_t payload[8];
  payload[0] = axis; payload[1] = status;
  payload[2] = (uint8_t)request_id; payload[3] = (uint8_t)(request_id >> 8);
  payload[4] = (uint8_t)executed_steps; payload[5] = (uint8_t)(executed_steps >> 8);
  payload[6] = (uint8_t)(executed_steps >> 16); payload[7] = (uint8_t)(executed_steps >> 24);
  USART1_SendFrame(FRAME_CMD_STATUS, payload, sizeof(payload));
}

static void Send_AxisState(uint8_t axis)
{
  Send_Status(axis, stepper_axes[axis].running ? STATUS_STATE : STATUS_DONE,
              stepper_axes[axis].request_id, stepper_axes[axis].step_count);
}

/* Limit inputs are low while their sensor is triggered.
 * Sensor 0 = PA4 (Y home), sensor 1 = PA5 (Y end). */
static uint8_t Read_SensorState(uint8_t sensor_id)
{
  if (sensor_id == 0U)
    return (HAL_GPIO_ReadPin(Y_LIMIT_HOME_GPIO_Port, Y_LIMIT_HOME_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
  return (HAL_GPIO_ReadPin(Y_LIMIT_END_GPIO_Port, Y_LIMIT_END_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

static void Send_OneSensorState(uint8_t sensor_id, uint8_t state)
{
  uint8_t payload[2];
  payload[0] = sensor_id;
  payload[1] = state;
  USART1_SendFrame(FRAME_CMD_SENSOR_REPLY, payload, sizeof(payload));
}

static void Send_SensorState(void)
{
  Send_OneSensorState(0U, Read_SensorState(0U));
  Send_OneSensorState(1U, Read_SensorState(1U));
}

/* Poll every 20 ms.  A changed, stable sensor state is immediately reported
 * to USB CDC/UART, so the upper computer does not need to poll continuously. */
static void Sensor_Poll(void)
{
  uint8_t sensor_id, state;
  if ((HAL_GetTick() - sensor_poll_tick) < 20U) return;
  sensor_poll_tick = HAL_GetTick();
  for (sensor_id = 0U; sensor_id < 2U; sensor_id++)
  {
    state = Read_SensorState(sensor_id);
    if (sensor_last_state[sensor_id] != state)
    {
      sensor_last_state[sensor_id] = state;
      Send_OneSensorState(sensor_id, state);
    }
  }
}

static uint32_t ReadU32(const uint8_t *data)
{
  return (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static uint16_t ReadU16(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t AbsI32(int32_t value)
{
  return (value < 0) ? (uint32_t)(-(value + 1)) + 1U : (uint32_t)value;
}

static void Axis_SetPeriod(uint8_t axis, uint32_t speed_pps)
{
  uint32_t period = (1000000U / (2U * speed_pps)) - 1U;
  if (axis == 0U) __HAL_TIM_SET_AUTORELOAD(&htim2, period);
  else __HAL_TIM_SET_AUTORELOAD(&htim3, period);
}

static void Axis_Start(uint8_t axis, int32_t steps, uint32_t speed_pps, uint32_t accel_pps2, uint16_t request_id)
{
  StepperAxis *state;
  uint32_t start_pps;
  if (axis >= STEPPER_AXIS_COUNT || steps == 0 || speed_pps < STEPPER_MIN_PPS || speed_pps > STEPPER_MAX_PPS ||
      accel_pps2 < STEPPER_MIN_ACCEL || accel_pps2 > STEPPER_MAX_ACCEL)
  {
    Send_Status(axis, STATUS_ERROR, request_id, 0U); return;
  }
  state = (StepperAxis *)&stepper_axes[axis];
  if (state->running) { Send_Status(axis, STATUS_BUSY, request_id, state->step_count); return; }
  if (axis == 0U)
  {
    HAL_GPIO_WritePin(YL_DIR_GPIO_Port, YL_DIR_Pin, steps > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(YL_STEP_GPIO_Port, YL_STEP_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(YR_DIR_GPIO_Port, YR_DIR_Pin, steps > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(YR_STEP_GPIO_Port, YR_STEP_Pin, GPIO_PIN_RESET);
  }
  start_pps = speed_pps / 4U;
  if (start_pps < STEPPER_MIN_PPS) start_pps = STEPPER_MIN_PPS;
  state->target_steps = AbsI32(steps); state->step_count = 0U;
  state->target_pps = speed_pps; state->current_pps = start_pps;
  state->acceleration_pps2 = accel_pps2; state->request_id = request_id;
  state->pulse_is_high = 0U; state->done_pending = 0U; state->running = 1U;
  Axis_SetPeriod(axis, start_pps);
  if (axis == 0U) { __HAL_TIM_SET_COUNTER(&htim2, 0U); HAL_TIM_Base_Start_IT(&htim2); }
  else { __HAL_TIM_SET_COUNTER(&htim3, 0U); HAL_TIM_Base_Start_IT(&htim3); }
  Send_Status(axis, STATUS_ACCEPTED, request_id, 0U);
}

static void Axis_Stop(uint8_t axis, uint8_t report)
{
  StepperAxis *state;
  if (axis >= STEPPER_AXIS_COUNT) return;
  state = (StepperAxis *)&stepper_axes[axis];
  if (state->running)
  {
    state->running = 0U; state->pulse_is_high = 0U;
    if (axis == 0U) { HAL_TIM_Base_Stop_IT(&htim2); HAL_GPIO_WritePin(YL_STEP_GPIO_Port, YL_STEP_Pin, GPIO_PIN_RESET); }
    else { HAL_TIM_Base_Stop_IT(&htim3); HAL_GPIO_WritePin(YR_STEP_GPIO_Port, YR_STEP_Pin, GPIO_PIN_RESET); }
  }
  if (report) Send_Status(axis, STATUS_STOPPED, state->request_id, state->step_count);
}

void Stepper_TimerElapsed(TIM_TypeDef *timer_instance)
{
  uint8_t axis = (timer_instance == TIM2) ? 0U : ((timer_instance == TIM3) ? 1U : STEPPER_ALL_AXES);
  StepperAxis *state;
  uint32_t remaining, braking_steps, delta;
  if (axis == STEPPER_ALL_AXES) return;
  state = (StepperAxis *)&stepper_axes[axis];
  if (!state->running) return;
  if (state->pulse_is_high)
  {
    if (axis == 0U) HAL_GPIO_WritePin(YL_STEP_GPIO_Port, YL_STEP_Pin, GPIO_PIN_RESET);
    else HAL_GPIO_WritePin(YR_STEP_GPIO_Port, YR_STEP_Pin, GPIO_PIN_RESET);
    state->pulse_is_high = 0U;
    if (state->step_count >= state->target_steps)
    {
      state->running = 0U;
      if (axis == 0U) HAL_TIM_Base_Stop_IT(&htim2); else HAL_TIM_Base_Stop_IT(&htim3);
      state->done_pending = 1U;
      return;
    }
    remaining = state->target_steps - state->step_count;
    braking_steps = (state->current_pps * state->current_pps) / (2U * state->acceleration_pps2);
    delta = state->acceleration_pps2 / state->current_pps;
    if (delta == 0U) delta = 1U;
    if (remaining <= braking_steps && state->current_pps > STEPPER_MIN_PPS)
      state->current_pps = (state->current_pps > STEPPER_MIN_PPS + delta) ? state->current_pps - delta : STEPPER_MIN_PPS;
    else if (state->current_pps < state->target_pps)
      state->current_pps = (state->current_pps + delta < state->target_pps) ? state->current_pps + delta : state->target_pps;
    Axis_SetPeriod(axis, state->current_pps);
    return;
  }
  if (axis == 0U) HAL_GPIO_WritePin(YL_STEP_GPIO_Port, YL_STEP_Pin, GPIO_PIN_SET);
  else HAL_GPIO_WritePin(YR_STEP_GPIO_Port, YR_STEP_Pin, GPIO_PIN_SET);
  state->pulse_is_high = 1U;
  state->step_count++;
}

static void Process_Frame(void)
{
  uint8_t axis;
  int32_t steps;
  uint32_t speed, acceleration;
  uint16_t request_id;
  if (rx_command == FRAME_CMD_INFO && rx_length == 0U) { Send_Info(); return; }
  if (rx_command == FRAME_CMD_MOVE_AXIS && rx_length == 15U)
  {
    axis = rx_payload[0]; steps = (int32_t)ReadU32(&rx_payload[1]); speed = ReadU32(&rx_payload[5]);
    acceleration = ReadU32(&rx_payload[9]); request_id = ReadU16(&rx_payload[13]);
    Axis_Start(axis, steps, speed, acceleration, request_id); return;
  }
  if (rx_command == FRAME_CMD_STOP_AXIS && rx_length == 1U)
  {
    axis = rx_payload[0];
    if (axis == STEPPER_ALL_AXES) { Axis_Stop(0U, 1U); }
    else if (axis < STEPPER_AXIS_COUNT) Axis_Stop(axis, 1U);
    else Send_Status(axis, STATUS_ERROR, 0U, 0U);
    return;
  }
  if (rx_command == FRAME_CMD_GET_STATE && rx_length == 1U)
  {
    axis = rx_payload[0];
    if (axis == STEPPER_ALL_AXES) { Send_AxisState(0U); }
    else if (axis < STEPPER_AXIS_COUNT) Send_AxisState(axis);
    else Send_Status(axis, STATUS_ERROR, 0U, 0U);
    return;
  }
  if (rx_command == FRAME_CMD_GET_SENSOR && rx_length == 0U)
  {
    Send_SensorState(); return;
  }
  /* This Y-axis firmware intentionally does not accept the two-axis sync command. */
  /* Legacy commands keep old one-axis tooling usable. */
  if (rx_command == FRAME_CMD_MOVE_LEGACY && rx_length == 8U)
  {
    Axis_Start(0U, (int32_t)ReadU32(&rx_payload[0]), ReadU32(&rx_payload[4]), STEPPER_DEFAULT_ACCEL, 0U); return;
  }
  if (rx_command == FRAME_CMD_STOP_LEGACY && rx_length == 0U)
  {
    Axis_Stop(0U, 1U); return;
  }
  Send_Status(STEPPER_ALL_AXES, STATUS_ERROR, 0U, 0U);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
