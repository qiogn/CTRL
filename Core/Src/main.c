/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "dual_stepper.h"
#include "peripheral_lib.h"

/* USER CODE BEGIN PTD */
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
/* K230 -> STM32 : AA FF F2 04 xL xH yL yH SC AC */
#define K230_FRAME_HEAD1 0xAAU
#define K230_FRAME_HEAD2 0xFFU
#define K230_FRAME_FUNC_CMD 0xF2U
#define K230_FRAME_LEN_CMD 4U
#define K230_FRAME_WIDTH 640U
#define K230_FRAME_HEIGHT 360U
#define K230_CENTER_X ((int16_t)(K230_FRAME_WIDTH / 2U))
#define K230_CENTER_Y ((int16_t)(K230_FRAME_HEIGHT / 2U))

/* 输入限制：最大偏移量 */
#define INPUT_LIMIT_X 160
#define INPUT_LIMIT_Y 100

/* 死区：中心区域不响应 */
#define CENTER_DEADBAND_X 6
#define CENTER_DEADBAND_Y 6

/* 绝对值宏 */
#define ABS(x) ((x) < 0 ? -(x) : (x))

/* 电机命令限制 */
#define MOTOR_CMD_LIMIT_X 24
#define MOTOR_CMD_LIMIT_Y 24
/* USER CODE END PD */

/* USER CODE BEGIN PV */
static uint8_t g_uart_rx_byte = 0U;
static volatile uint32_t g_uart_valid_frame_count = 0U;
static volatile uint16_t g_uart_last_parsed_x = 320U;
static volatile uint16_t g_uart_last_parsed_y = 180U;

static K230_RxState_t g_rx_state = RX_WAIT_AA;
static uint8_t g_rx_func = 0U;
static uint8_t g_rx_len = 0U;
static uint8_t g_rx_data[K230_FRAME_LEN_CMD] = {0};
static uint8_t g_rx_data_idx = 0U;
static uint8_t g_rx_sc = 0U;
static uint8_t g_rx_ac = 0U;

/* USER CODE END PV */

void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
static void Uart_StartReceiveIT(void);
static void K230_ResetParser(void);
static void K230_ParseByte(uint8_t b);

static void LedOn(void);
static void LedOff(void);
static void LedPulseVisible(uint32_t ms);

static void MotorDisableAll(void);
static void FirmwareTagBlink(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* lower motor: PB0/PB1/PB10 */
#define MOTOR_LOWER_EN_PORT EN2_GPIO_Port
#define MOTOR_LOWER_EN_PIN EN2_Pin
#define MOTOR_LOWER_STEP_PORT Step2_GPIO_Port
#define MOTOR_LOWER_STEP_PIN Step2_Pin
#define MOTOR_LOWER_DIR_PORT Dir2_GPIO_Port
#define MOTOR_LOWER_DIR_PIN Dir2_Pin

/* upper motor: PA5/PA6/PA7 */
#define MOTOR_UPPER_EN_PORT EN1_GPIO_Port
#define MOTOR_UPPER_EN_PIN EN1_Pin
#define MOTOR_UPPER_STEP_PORT Step1_GPIO_Port
#define MOTOR_UPPER_STEP_PIN Step1_Pin
#define MOTOR_UPPER_DIR_PORT Dir1_GPIO_Port
#define MOTOR_UPPER_DIR_PIN Dir1_Pin


static void LaserOff(void)
{
  /* PB1低电平关闭激光继电器 */
  HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);
}

static void Uart_StartReceiveIT(void)
{
  (void)HAL_UART_Receive_IT(&huart2, &g_uart_rx_byte, 1U);
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

  switch (g_rx_state)
  {
    case RX_WAIT_AA:
      if (b == K230_FRAME_HEAD1)
      {
        g_rx_state = RX_WAIT_FF;
      }
      break;

    case RX_WAIT_FF:
      if (b == K230_FRAME_HEAD2)
      {
        g_rx_state = RX_WAIT_FUNC;
      }
      else if (b == K230_FRAME_HEAD1)
      {
        g_rx_state = RX_WAIT_FF;
      }
      else
      {
        g_rx_state = RX_WAIT_AA;
      }
      break;

    case RX_WAIT_FUNC:
      g_rx_func = b;
      g_rx_state = RX_WAIT_LEN;
      break;

    case RX_WAIT_LEN:
      g_rx_len = b;
      if (g_rx_len == K230_FRAME_LEN_CMD)
      {
        g_rx_data_idx = 0U;
        g_rx_state = RX_WAIT_DATA;
      }
      else
      {
        K230_ResetParser();
      }
      break;

    case RX_WAIT_DATA:
      g_rx_data[g_rx_data_idx++] = b;
      if (g_rx_data_idx >= K230_FRAME_LEN_CMD)
      {
        g_rx_state = RX_WAIT_SC;
      }
      break;

    case RX_WAIT_SC:
      g_rx_sc = b;
      g_rx_state = RX_WAIT_AC;
      break;

    case RX_WAIT_AC:
      g_rx_ac = b;

      calc_sc = 0U;
      calc_ac = 0U;

      calc_sc = (uint8_t)((calc_sc + K230_FRAME_HEAD1) & 0xFFU); calc_ac = (uint8_t)((calc_ac + calc_sc) & 0xFFU);
      calc_sc = (uint8_t)((calc_sc + K230_FRAME_HEAD2) & 0xFFU); calc_ac = (uint8_t)((calc_ac + calc_sc) & 0xFFU);
      calc_sc = (uint8_t)((calc_sc + g_rx_func) & 0xFFU);        calc_ac = (uint8_t)((calc_ac + calc_sc) & 0xFFU);
      calc_sc = (uint8_t)((calc_sc + g_rx_len) & 0xFFU);         calc_ac = (uint8_t)((calc_ac + calc_sc) & 0xFFU);

      for (i = 0U; i < K230_FRAME_LEN_CMD; ++i)
      {
        calc_sc = (uint8_t)((calc_sc + g_rx_data[i]) & 0xFFU);
        calc_ac = (uint8_t)((calc_ac + calc_sc) & 0xFFU);
      }

      if ((g_rx_func == K230_FRAME_FUNC_CMD) &&
          (calc_sc == g_rx_sc) &&
          (calc_ac == g_rx_ac))
      {
        g_uart_last_parsed_x = (uint16_t)(g_rx_data[0] | (g_rx_data[1] << 8));
        g_uart_last_parsed_y = (uint16_t)(g_rx_data[2] | (g_rx_data[3] << 8));
        g_uart_valid_frame_count += 1U;
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


static void MotorDisableAll(void)
{
  HAL_GPIO_WritePin(MOTOR_LOWER_EN_PORT, MOTOR_LOWER_EN_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOTOR_UPPER_EN_PORT, MOTOR_UPPER_EN_PIN, GPIO_PIN_SET);
}

static void FirmwareTagBlink(void)
{
  uint32_t i = 0U;

  for (i = 0U; i < 3U; ++i)
  {
    LedPulseVisible(80U);
  }
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
  uint32_t handled_frame_count = 0U;
  uint32_t latest_frame_count = 0U;

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  DualStepper_Init();

  LedOff();
  MotorDisableAll();
  LaserOff();  /* 确保激光关闭 */
  HAL_Delay(150U);
  FirmwareTagBlink();

  /* 简单电机测试：让两个电机各转50步 */
  DualStepper_MoveAxes(50, 50, 1000);
  HAL_Delay(500U);

  K230_ResetParser();
  Uart_StartReceiveIT();

  while (1)
  {
    __disable_irq();
    latest_frame_count = g_uart_valid_frame_count;
    cmd_x = g_uart_last_parsed_x;
    cmd_y = g_uart_last_parsed_y;
    __enable_irq();

    if (latest_frame_count != handled_frame_count)
    {
      handled_frame_count = latest_frame_count;

      /* 计算相对于中心的偏移 */
      err_x = (int16_t)cmd_x - K230_CENTER_X;
      err_y = (int16_t)cmd_y - K230_CENTER_Y;

      /* 限制偏移范围 */
      if (err_x > INPUT_LIMIT_X) err_x = INPUT_LIMIT_X;
      if (err_x < -INPUT_LIMIT_X) err_x = -INPUT_LIMIT_X;
      if (err_y > INPUT_LIMIT_Y) err_y = INPUT_LIMIT_Y;
      if (err_y < -INPUT_LIMIT_Y) err_y = -INPUT_LIMIT_Y;

      /* 应用死区：中心区域不响应 */
      if (ABS(err_x) <= CENTER_DEADBAND_X) err_x = 0;
      if (ABS(err_y) <= CENTER_DEADBAND_Y) err_y = 0;

      /* 简单映射：超过死区则移动1步 */
      motor_cmd_x = (err_x > 0) ? 1 : ((err_x < 0) ? -1 : 0);
      motor_cmd_y = (err_y > 0) ? 1 : ((err_y < 0) ? -1 : 0);

      /* 发送电机控制命令 */
      /* motor1 = 竖直轴(Y), motor2 = 水平轴(X) */
      DualStepper_SetTrackingCommand(motor_cmd_y, motor_cmd_x);
    }
    else
    {
      /* 没有新数据时短暂延时 */
      HAL_Delay(1U);
    }
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
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();
    }
    flash_latency = FLASH_LATENCY_2;
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, flash_latency) != HAL_OK)
  {
    Error_Handler();
  }
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
  while (1)
  {
    LedPulseVisible(100U);
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
