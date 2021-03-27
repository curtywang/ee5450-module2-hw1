/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (only the threads are in this file)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dfsdm.h"
#include "i2c.h"
#include "octospi.h"
#include "rng.h"
#include "spi.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "sensor_telemetry_setup.h"
#include "sensor_telemetry.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE BEGIN PTD */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static TX_EVENT_FLAGS_GROUP global_event_flags;
TX_BYTE_POOL global_byte_pool;
ULONG global_memory_area[(sizeof(struct global_data_t) / sizeof(ULONG)) + 10 * sizeof(ULONG)];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void thread_network_setup(ULONG global_data_ulong);
_Noreturn void blink_PA_5(ULONG global_data_ulong);
_Noreturn void blink_PB_14(ULONG global_data_ulong);
_Noreturn void thread_temperature(ULONG global_data_ulong);
_Noreturn void thread_accelerometer(ULONG global_data_ulong);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief interrupt handler to set software interrupts
 * @param GPIO_Pin
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_13) {
        tx_event_flags_set(&global_event_flags, EVT_BUTTON_PRESSED, TX_OR);
    }
    else if (GPIO_Pin == GPIO_PIN_1) {
        SPI_WIFI_ISR();
    }
}


/**
 * @brief handle the spi3 interrupt (for Wifi/BLE)
 */
void SPI3_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&hspi);
}


/**
 * @brief function for defining the ThreadX application
 * @param first_unused_memory
 */
void tx_application_define(void* first_unused_memory) {
    char* pointer = TX_NULL;
    volatile UINT status;

    systick_interval_set(TX_TIMER_TICKS_PER_SECOND);

    status = tx_event_flags_create(&global_event_flags, "global event flags");
    status = tx_byte_pool_create(&global_byte_pool, "global byte pool",
                                 global_memory_area, sizeof(struct global_data_t) + 10 * sizeof(ULONG));

    /* allocate global data structure */
    status = tx_byte_allocate(&global_byte_pool, (VOID**)&pointer,
                              sizeof(struct global_data_t), TX_NO_WAIT);
    struct global_data_t* global_data = (struct global_data_t*)pointer;
    global_data->interval_mqtt = 1;
    global_data->interval_accelerometer = 1;
    global_data->interval_temperature = 1;
    global_data->interval_ld1 = 1;
    global_data->interval_ld2 = 2;

    /* initialize synchronization primitives */
    status = tx_mutex_create(&global_data->mutex_mqtt, "MQTT client mutex", TX_NO_INHERIT);
    status = tx_mutex_create(&global_data->mutex_i2c2, "I2C channel 2 mutex", TX_NO_INHERIT);
    status = tx_mutex_create(&global_data->mutex_network_reset, "network setup mutex", TX_NO_INHERIT);

    status = tx_byte_pool_create(&global_data->byte_pool_0, "byte pool 0",
                        global_data->memory_area, BYTE_POOL_SIZE);

    status = tx_byte_allocate(&global_data->byte_pool_0, (VOID **) &pointer, STACK_SIZE, TX_NO_WAIT);
    status = tx_thread_create(&global_data->thread_network_setup, "thread net test",
                              thread_network_setup, (ULONG)global_data,
                              pointer, STACK_SIZE,
                              1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
        printf("thread creation failed\r\n");

    tx_byte_allocate(&global_data->byte_pool_0, (VOID **) &pointer, STACK_SIZE, TX_NO_WAIT);
    status = tx_thread_create(&global_data->threads[0], "thread 0",
                              blink_PA_5, (ULONG)global_data,
                              pointer, STACK_SIZE,
                              3, 3, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
        printf("thread creation failed\r\n");

    tx_byte_allocate(&global_data->byte_pool_0, (VOID **) &pointer, STACK_SIZE, TX_NO_WAIT);
    status = tx_thread_create(&global_data->threads[1], "thread 1",
                              blink_PB_14, (ULONG)global_data,
                              pointer, STACK_SIZE,
                              3, 3, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
        printf("thread creation failed\r\n");

    tx_byte_allocate(&global_data->byte_pool_0, (VOID **) &pointer, STACK_SIZE, TX_NO_WAIT);
    status = tx_thread_create(&global_data->threads[2], "thread 2",
                              thread_temperature, (ULONG)global_data,
                              pointer, STACK_SIZE,
                              2, 2, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
        printf("thread creation failed\r\n");

    tx_byte_allocate(&global_data->byte_pool_0, (VOID **) &pointer, STACK_SIZE, TX_NO_WAIT);
    status = tx_thread_create(&global_data->threads[3], "thread 3",
                              thread_accelerometer, (ULONG)global_data,
                              pointer, STACK_SIZE,
                              2, 2, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
        printf("thread creation failed\r\n");
}


/**
 * @brief setup the network by connecting to the Wifi network and using nxduo to test MQTT
 * @param global_data_ulong ->interval_mqtt: interval of sending out mqtt messages
 */
void thread_network_setup(ULONG global_data_ulong) {
    struct global_data_t* global_data = (struct global_data_t*)global_data_ulong;
    volatile UINT status;
    uint32_t tick_interval = 100 * global_data->interval_mqtt;
    ULONG current_tick;
    const size_t message_size = 30;
    char message[message_size + 1];
    const size_t topic_size = 30;
    char topic[topic_size + 1];
    memset(topic, 0, topic_size + 1);
    memset(message, 0, message_size + 1);

    status = setup_wifi(false);
    if (status != 0)
        return;
    status = setup_nx_wifi(global_data);
    if (status != 0)
        return;
    status = setup_nx_mqtt_and_connect(global_data);
    if (status != 0)
        return;

    snprintf(topic, topic_size, "board_test/hello");
    snprintf(message, message_size, "board says Hello!!!");
    status = send_nx_mqtt_message(global_data, topic, message);
    if (status != 0)
        return;

    for (size_t i = 0; i < 7; i++) {
        current_tick = tx_time_get();
        snprintf(message, message_size, "hello the time is: %lu", current_tick);
        status = send_nx_mqtt_message(global_data, topic, message);
        if (status != 0)
            return;
        tx_thread_sleep(tick_interval / 4);
    }

    tx_event_flags_set(&global_event_flags, EVT_WIFI_READY, TX_OR);
}


/**
 * @brief thread that blinks PA5 at given interval.
 * @param global_data_ulong ->interval_ld1: interval of blinking ld1 at 50% duty cycle
 */
_Noreturn void blink_PA_5(ULONG global_data_ulong) {
    struct global_data_t* global_data = (struct global_data_t*)global_data_ulong;
    uint32_t ticks_duty_cycle = (global_data->interval_ld1 * 100) / 2;
    while (1) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        tx_thread_sleep(ticks_duty_cycle);  // this is in ticks, which is default 100 per second.
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        tx_thread_sleep(ticks_duty_cycle);
    }
}


/**
 * @brief thread that blinks PB14 at given interval.
 * @param global_data_ulong ->interval_ld2: interval of blinking ld2 at 50% duty cycle
 */
_Noreturn void blink_PB_14(ULONG global_data_ulong) {
    struct global_data_t* global_data = (struct global_data_t*)global_data_ulong;
    uint32_t period_ticks = (global_data->interval_ld2 * 100) / 2;
    while(1) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        tx_thread_sleep(period_ticks);  // this is in ticks, which is default 100 per second.
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        tx_thread_sleep(period_ticks);
    }
}


/**
 * @brief utility for resetting the network setup thread
 * @param global_data: pointer to the global data strcuture
 */
void reset_network_thread(struct global_data_t* global_data) {
    ULONG actual_flags;
    // the mutex check that another thread isn't resetting
    UINT status = tx_mutex_get(&global_data->mutex_network_reset, TX_NO_WAIT);
    tx_event_flags_set(&global_event_flags, EVT_WIFI_READY, TX_AND_CLEAR);
    if (status == TX_SUCCESS) {
        status = tx_thread_reset(&global_data->thread_network_setup);  // reset network setup thread
        printf("%d", status);
        status = tx_thread_resume(&global_data->thread_network_setup);  // start the network setup thread
    }
    tx_event_flags_get(&global_event_flags, EVT_WIFI_READY, TX_AND, &actual_flags, TX_WAIT_FOREVER);
    tx_mutex_put(&global_data->mutex_network_reset);  // if not owned, will error out anyway
}


/**
 * @brief thread that reads the temperature sensor value at given interval
 * @param global_data_ulong ->interval_temperature: interval in seconds to read the tsensor
 */
_Noreturn void thread_temperature(ULONG global_data_ulong) {
    UINT status;
    ULONG actual_flags;
    struct global_data_t* global_data = (struct global_data_t*)global_data_ulong;
    uint32_t period_ticks = global_data->interval_temperature * 100;

    const size_t message_size = 40;
    char message[message_size + 1];
    const size_t topic_size = 30;
    char topic[topic_size + 1];
    memset(topic, 0, topic_size + 1);
    memset(message, 0, message_size + 1);
    snprintf(topic, topic_size, "board_test/temperature");

    BSP_TSENSOR_Init();
    tx_event_flags_get(&global_event_flags, EVT_WIFI_READY, TX_AND, &actual_flags, TX_WAIT_FOREVER);

    while (1) {
        get_temperature_message(global_data, message, message_size);
        status = send_nx_mqtt_message(global_data, topic, message);
        if (status != NX_SUCCESS) {
            reset_network_thread(global_data);
        }
        tx_thread_sleep(period_ticks);
    }
}


/**
 * @brief thread that reads the accelerometer XYZ at given interval
 * @param global_data_ulong ->interval_accelerometer: interval in seconds to read the accelerometer
 */
_Noreturn void thread_accelerometer(ULONG global_data_ulong) {
    UINT status;
    ULONG actual_flags;
    struct global_data_t* global_data = (struct global_data_t*)global_data_ulong;
    uint32_t period_ticks = (global_data->interval_accelerometer * 100);

    const size_t message_size = 40;
    char message[message_size + 1];
    const size_t topic_size = 30;
    char topic[topic_size + 1];
    memset(topic, 0, topic_size + 1);
    memset(message, 0, message_size + 1);
    snprintf(topic, topic_size, "board_test/accelerometer");

    ACCELERO_StatusTypeDef xl_status = BSP_ACCELERO_Init();
    if (xl_status != ACCELERO_OK)
        printf("ERROR!");
    tx_event_flags_get(&global_event_flags, EVT_WIFI_READY, TX_AND, &actual_flags, TX_WAIT_FOREVER);

    while (1) {
        get_accelerometer_message(global_data, message, message_size);
        status = send_nx_mqtt_message(global_data, topic, message);
        if (status != NX_SUCCESS) {
            reset_network_thread(global_data);
        }
        tx_thread_sleep(period_ticks);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_DFSDM1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_OCTOSPI1_Init();
  MX_SPI3_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_USB_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */
  tx_kernel_enter();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_DFSDM1|RCC_PERIPHCLK_USB
                              |RCC_PERIPHCLK_RNG|RCC_PERIPHCLK_ADC
                              |RCC_PERIPHCLK_OSPI;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  PeriphClkInit.OspiClockSelection = RCC_OSPICLKSOURCE_SYSCLK;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
