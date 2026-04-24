/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "peripheral_lib.h"
#include "dual_stepper.h"
#include "usart.h"

/* USER CODE BEGIN PTD */
typedef enum
{
  CTRL_MODE_IDLE = 0U,
  CTRL_MODE_TRACK
} CtrlMode_t;

typedef enum
{
  RX_WAIT_AA = 0U,
  RX_WAIT_FF,
  RX_WAIT_FUNC,
  RX_WAIT_LEN,
  RX_WAIT_DATA,
  RX_WAIT_SC,
  RX_WAIT_AC
} K230_RxState_t;
/* USER CODE END PTD */

/* USER CODE BEGIN PD */
#define K230_FRAME_WIDTH 640U
#define K230_FRAME_HEIGHT 360U
#define K230_CENTER_X ((int16_t)(K230_FRAME_WIDTH / 2U))
#define K230_CENTER_Y ((int16_t)(K230_FRAME_HEIGHT / 2U))
#define K230_FRAME_TIMEOUT_MS 120U

#define K230_FRAME_HEAD1 0xAAU
#define K230_FRAME_HEAD2 0xFFU
#define K230_FRAME_FUNC_POINT 0xF2U
#define K230_FRAME_LEN_POINT 4U

#define INPUT_LIMIT_X 160
#define INPUT_LIMIT_Y 100

#define AIM_LOCK_DEADBAND_X 8
#define AIM_LOCK_DEADBAND_Y 8
#define AIM_RELEASE_DEADBAND_X 16
#define AIM_RELEASE_DEADBAND_Y 16
#define AIM_MOTION_DEADBAND_X 9
#define AIM_MOTION_DEADBAND_Y 9
#define AIM_LOCK_STABLE_FRAMES 2U

#define LASER_DEADBAND_X  50
#define LASER_DEADBAND_Y  50

#define ABS(x) ((x) < 0 ? -(x) : (x))

#define MOTOR_CMD_LIMIT_X 12
#define MOTOR_CMD_LIMIT_Y 12
#define AIM_STEP_DIV_X 8
#define AIM_STEP_DIV_Y 8
#define AIM_STEP_DIV_SLOW_X 14
#define AIM_STEP_DIV_SLOW_Y 14
#define AIM_STEP_DIV_FINE_X 24
#define AIM_STEP_DIV_FINE_Y 24
#define AIM_SLOW_BAND_X 24
#define AIM_SLOW_BAND_Y 16
#define AIM_FINE_BAND_X 8
#define AIM_FINE_BAND_Y 6
#define AIM_PULSE_WIDTH_US 1500U
#define AIM_PULSE_WIDTH_SLOW_US 2500U
#define AIM_PULSE_WIDTH_FINE_US 5000U


/* USER CODE END PD */

/* USER CODE BEGIN PV */
static uint8_t g_uart_rx_byte = 0U;
static volatile uint32_t g_uart_total_bytes = 0U;
static volatile uint32_t g_uart_checksum_fail_count = 0U;
static volatile uint32_t g_uart_valid_frame_count = 0U;
static volatile uint16_t g_uart_last_parsed_x = 320U;
static volatile uint16_t g_uart_last_parsed_y = 180U;

static K230_RxState_t g_rx_state = RX_WAIT_AA;
static uint8_t g_rx_func = 0U;
static uint8_t g_rx_len = 0U;
static uint8_t g_rx_data[K230_FRAME_LEN_POINT] = {0};
static uint8_t g_rx_data_idx = 0U;
static uint8_t g_rx_sc = 0U;
static uint8_t g_rx_ac = 0U;
static volatile CtrlMode_t g_ctrl_mode = CTRL_MODE_IDLE;
static volatile uint32_t g_last_msg_tick = 0U;
static volatile uint8_t g_boot_centered = 0U;
static volatile uint32_t g_led_blink_start = 0U;
/* USER CODE END PV */

void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
static void Uart_StartReceiveIT(void);
static void K230_ResetParser(void);
static void K230_ParseByte(uint8_t b);

static void LedOn(void);
static void LedOff(void);
static void LedPulseVisible(uint32_t ms);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

static void Uart_StartReceiveIT(void)
{
  UART_HandleTypeDef* uart_handle = &huart2;
  (void)HAL_UART_Receive_IT(uart_handle, &g_uart_rx_byte, 1U);
}

static void K230_ResetParser(void)
{
  g_rx_state = RX_WAIT_AA;
  g_rx_func = 0U;
  g_rx_len = 0U;
  g_rx_data_idx = 0U;
  g_rx_sc = 0U;
  g_rx_ac = 0U;
}

static void K230_ParseByte(uint8_t b)
{
  uint8_t calc_sc = 0U;
  uint8_t calc_ac = 0U;
  uint8_t i = 0U;

  g_uart_total_bytes++;

  switch (g_rx_state)
  {
    case RX_WAIT_AA:
      if (b == K230_FRAME_HEAD1) {
        g_rx_state = RX_WAIT_FF;
        g_led_blink_start = HAL_GetTick();
      }
      break;

    case RX_WAIT_FF:
      if (b == K230_FRAME_HEAD2) g_rx_state = RX_WAIT_FUNC;
      else if (b == K230_FRAME_HEAD1) g_rx_state = RX_WAIT_FF;
      else g_rx_state = RX_WAIT_AA;
      break;

    case RX_WAIT_FUNC:
      g_rx_func = b;
      g_rx_state = RX_WAIT_LEN;
      break;

    case RX_WAIT_LEN:
      g_rx_len = b;
      if ((g_rx_func == K230_FRAME_FUNC_POINT) && (g_rx_len == K230_FRAME_LEN_POINT))
      {
        g_rx_data_idx = 0U;
        g_rx_state = RX_WAIT_DATA;
      }
      else K230_ResetParser();
      break;

    case RX_WAIT_DATA:
      g_rx_data[g_rx_data_idx++] = b;
      if (g_rx_data_idx >= g_rx_len) g_rx_state = RX_WAIT_SC;
      break;

    case RX_WAIT_SC:
      g_rx_sc = b;
      g_rx_state = RX_WAIT_AC;
      break;

    case RX_WAIT_AC:
      g_rx_ac = b;

      calc_sc = 0U; calc_ac = 0U;
      calc_sc ^= K230_FRAME_HEAD1;   calc_ac = (calc_ac + K230_FRAME_HEAD1) & 0xFFU;
      calc_sc ^= K230_FRAME_HEAD2;   calc_ac = (calc_ac + K230_FRAME_HEAD2) & 0xFFU;
      calc_sc ^= g_rx_func;          calc_ac = (calc_ac + g_rx_func) & 0xFFU;
      calc_sc ^= g_rx_len;           calc_ac = (calc_ac + g_rx_len) & 0xFFU;
      for (i = 0U; i < g_rx_len; ++i) {
        calc_sc ^= g_rx_data[i];
        calc_ac = (calc_ac + g_rx_data[i]) & 0xFFU;
      }

      if ((g_rx_func == K230_FRAME_FUNC_POINT) && (calc_sc == g_rx_sc) && (calc_ac == g_rx_ac))
      {
        g_uart_last_parsed_x = (uint16_t)(g_rx_data[0] | (g_rx_data[1] << 8));
        g_uart_last_parsed_y = (uint16_t)(g_rx_data[2] | (g_rx_data[3] << 8));
        g_uart_valid_frame_count += 1U;
        g_ctrl_mode = CTRL_MODE_TRACK;
        g_last_msg_tick = HAL_GetTick();
        g_led_blink_start = HAL_GetTick();
        g_boot_centered = 1U;
      }
      else
      {
        g_uart_checksum_fail_count++;
      }
      K230_ResetParser();
      break;

    default:
      K230_ResetParser();
      break;
  }
}

static void LedOn(void)
{
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

static void LedOff(void)
{
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

static void LedPulseVisible(uint32_t ms)
{
  LedOn();
  HAL_Delay(ms);
  LedOff();
  HAL_Delay(ms);
}

/* USER CODE END 0 */

int main(void)
{
  uint16_t cmd_x = 320U;
  uint16_t cmd_y = 180U;
  int16_t err_x = 0;
  int16_t err_y = 0;
  int16_t motor_cmd_x = 0;
  int16_t motor_cmd_y = 0;
  uint16_t pulse_width_us = AIM_PULSE_WIDTH_US;
  uint32_t handled_frame_count = 0U;
  uint32_t latest_frame_count = 0U;
  uint32_t last_frame_tick = 0U;
  uint32_t stable_lock_frames = 0U;
  uint8_t laser_enabled = 0U;
  uint8_t aim_locked = 0U;
  CtrlMode_t ctrl_mode = CTRL_MODE_IDLE;

  HAL_Init();
  SystemClock_Config();

  peripheral_system_init();

  LedOff();
  peripheral_laser_off();

  DualStepper_SetHoldEnabled(1U);
  peripheral_motor_enable(MOTOR_AXIS_1, 1U);
  peripheral_motor_enable(MOTOR_AXIS_2, 1U);

  K230_ResetParser();
  Uart_StartReceiveIT();
  last_frame_tick = HAL_GetTick();

  while (1)
  {
    /* Copy shared data with interrupt protection */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    latest_frame_count = g_uart_valid_frame_count;
    cmd_x = g_uart_last_parsed_x;
    cmd_y = g_uart_last_parsed_y;
    ctrl_mode = g_ctrl_mode;
    __set_PRIMASK(primask);

    /* Check for message timeout */
    if ((ctrl_mode == CTRL_MODE_TRACK) &&
        ((HAL_GetTick() - g_last_msg_tick) >= K230_FRAME_TIMEOUT_MS))
    {
      ctrl_mode = CTRL_MODE_IDLE;
      g_ctrl_mode = CTRL_MODE_IDLE;
    }

    /* LED blink debug indicator for UART frame reception */
    if (g_led_blink_start != 0U) {
      uint32_t elapsed = HAL_GetTick() - g_led_blink_start;
      if (elapsed < 50U) {
        LedOn();
      } else {
        LedOff();
        g_led_blink_start = 0U;
      }
    }

    if ((ctrl_mode == CTRL_MODE_TRACK) && (latest_frame_count != handled_frame_count))
    {
      handled_frame_count = latest_frame_count;
      last_frame_tick = HAL_GetTick();

      err_x = (int16_t)cmd_x - K230_CENTER_X;
      err_y = (int16_t)cmd_y - K230_CENTER_Y;

      if (err_x > INPUT_LIMIT_X) err_x = INPUT_LIMIT_X;
      if (err_x < -INPUT_LIMIT_X) err_x = -INPUT_LIMIT_X;
      if (err_y > INPUT_LIMIT_Y) err_y = INPUT_LIMIT_Y;
      if (err_y < -INPUT_LIMIT_Y) err_y = -INPUT_LIMIT_Y;

      /* Lock detection */
      if ((ABS(err_x) <= AIM_LOCK_DEADBAND_X) && (ABS(err_y) <= AIM_LOCK_DEADBAND_Y))
      {
        if (stable_lock_frames < AIM_LOCK_STABLE_FRAMES) stable_lock_frames += 1U;
      }
      else stable_lock_frames = 0U;

      if (stable_lock_frames >= AIM_LOCK_STABLE_FRAMES) aim_locked = 1U;
      else if ((ABS(err_x) >= AIM_RELEASE_DEADBAND_X) || (ABS(err_y) >= AIM_RELEASE_DEADBAND_Y))
        aim_locked = 0U;

      /* Motion deadband */
      if (ABS(err_x) <= AIM_MOTION_DEADBAND_X) err_x = 0;
      if (ABS(err_y) <= AIM_MOTION_DEADBAND_Y) err_y = 0;

      /* Multi-zone speed control */
      pulse_width_us = AIM_PULSE_WIDTH_US;
      if ((ABS(err_x) <= AIM_FINE_BAND_X) && (ABS(err_y) <= AIM_FINE_BAND_Y))
      {
        motor_cmd_x = err_x / AIM_STEP_DIV_FINE_X;
        motor_cmd_y = err_y / AIM_STEP_DIV_FINE_Y;
        pulse_width_us = AIM_PULSE_WIDTH_FINE_US;
      }
      else if ((ABS(err_x) <= AIM_SLOW_BAND_X) && (ABS(err_y) <= AIM_SLOW_BAND_Y))
      {
        motor_cmd_x = err_x / AIM_STEP_DIV_SLOW_X;
        motor_cmd_y = err_y / AIM_STEP_DIV_SLOW_Y;
        pulse_width_us = AIM_PULSE_WIDTH_SLOW_US;
      }
      else
      {
        motor_cmd_x = err_x / AIM_STEP_DIV_X;
        motor_cmd_y = err_y / AIM_STEP_DIV_Y;
      }

      if ((motor_cmd_x == 0) && (err_x != 0)) motor_cmd_x = (err_x > 0) ? 1 : -1;
      if ((motor_cmd_y == 0) && (err_y != 0)) motor_cmd_y = (err_y > 0) ? 1 : -1;

      if (motor_cmd_x > MOTOR_CMD_LIMIT_X) motor_cmd_x = MOTOR_CMD_LIMIT_X;
      if (motor_cmd_x < -MOTOR_CMD_LIMIT_X) motor_cmd_x = -MOTOR_CMD_LIMIT_X;
      if (motor_cmd_y > MOTOR_CMD_LIMIT_Y) motor_cmd_y = MOTOR_CMD_LIMIT_Y;
      if (motor_cmd_y < -MOTOR_CMD_LIMIT_Y) motor_cmd_y = -MOTOR_CMD_LIMIT_Y;

      /* Only send new command when motors are idle (non-blocking) */
      if ((motor_cmd_x != 0) || (motor_cmd_y != 0))
      {
        /* Shrink critical section: only protect IsMoving() check to avoid
         * race condition. MoveAxes() contains DWT_DelayUs(~1300us) which
         * must NOT run with IRQs disabled, otherwise UART/UART2 RX data
         * will be lost. */
        uint32_t primask_save = __get_PRIMASK();
        __disable_irq();
        uint8_t need_move = !DualStepper_IsMoving();
        __set_PRIMASK(primask_save);

        if (need_move)
        {
          DualStepper_MoveAxes(motor_cmd_y, motor_cmd_x, pulse_width_us);
        }
      }

    }
    else
    {
      aim_locked = 0U;
      stable_lock_frames = 0U;
    }

    /* Laser control: direct deadband check, outside new-frame block */
    if (g_boot_centered && ctrl_mode == CTRL_MODE_TRACK &&
        (ABS(err_x) <= LASER_DEADBAND_X) && (ABS(err_y) <= LASER_DEADBAND_Y))
    {
      if (laser_enabled == 0U) { peripheral_laser_on(); laser_enabled = 1U; }
    }
    else
    {
      if (laser_enabled != 0U) { peripheral_laser_off(); laser_enabled = 0U; }
    }

    /* Keep the main loop running fast — no blocking */
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  uint32_t flash_latency = FLASH_LATENCY_2;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
    flash_latency = FLASH_LATENCY_2;
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, flash_latency) != HAL_OK) Error_Handler();
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    K230_ParseByte(g_uart_rx_byte);
    Uart_StartReceiveIT();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    K230_ResetParser();
    Uart_StartReceiveIT();
  }
}

/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1) { LedPulseVisible(100U); }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif
