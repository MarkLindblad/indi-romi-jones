// Robot Template app
//
// Framework for creating applications that control the Kobuki robot

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_spi.h"
//
// // #include "app_uart.h"
// // // #if defined (UART_PRESENT)
// // // #include "nrf_uart.h"
// // // #endif
// // // #if defined (UARTE_PRESENT)
// // // #include "nrf_uarte.h"
// // // #endif
// // #include "nrf_uart.h"
//
#include "buckler.h"
#include "display.h"
// #include "kobukiActuator.h"
// #include "kobukiSensorPoll.h"
// #include "kobukiSensorTypes.h"
// #include "kobukiUtilities.h"
// #include "lsm9ds1.h"
// #include <nrf_serial.h>
// // #include <unistd.h>
// // #include "ydlidar_sdk.h"
//
//
// #define UART_TX_BUF_SIZE 128                        /**< UART TX buffer size. */
// #define UART_RX_BUF_SIZE 128                       /**< UART RX buffer size. */
// #define RX_PIN_NUMBER 10
// #define TX_PIN_NUMBER 8
// #define RTS_PIN_NUMBER 7
// #define CTS_PIN_NUMBER 9
// #define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED
//
//
// NRF_SERIAL_DRV_UART_CONFIG_DEF(m_uart0_drv_config,
//                       BUCKLER_UART_RX, BUCKLER_UART_TX,
//                       0, 0,
//                       NRF_UART_HWFC_DISABLED, NRF_UART_PARITY_EXCLUDED,
//                       NRF_UART_BAUDRATE_115200,
//                       UART_DEFAULT_CONFIG_IRQ_PRIORITY);
//
// #define SERIAL_FIFO_TX_SIZE 512
// #define SERIAL_FIFO_RX_SIZE 512
//
//
//
// #define SERIAL_BUFF_TX_SIZE 1
// #define SERIAL_BUFF_RX_SIZE 1
//
// NRF_SERIAL_BUFFERS_DEF(serial_buffs, SERIAL_BUFF_TX_SIZE, SERIAL_BUFF_RX_SIZE);
// NRF_SERIAL_QUEUES_DEF(serial_queues2, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE);
//
//
// NRF_SERIAL_CONFIG_DEF(serial_config, NRF_SERIAL_MODE_DMA,
//                       &serial_queues2, &serial_buffs, NULL, NULL);
//
//
//
//
//
//
// // void uart_error_handle(nrf_uart_event_t * p_event)
// // {
// //     if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
// //     {
// //         APP_ERROR_HANDLER(p_event->data.error_communication);
// //     }
// //     else if (p_event->evt_type == APP_UART_FIFO_ERROR)
// //     {
// //         APP_ERROR_HANDLER(p_event->data.error_code);
// //     }
// // }
//
//
//
// I2C manager
// NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);
//
// typedef enum {
//   OFF,
//   DRIVING,
// } robot_state_t;
//
//
// int main(void) {
//
//
//   NRF_SERIAL_UART_DEF(serial_uart, 0);
//   nrf_gpio_pin_dir_set(BUCKLER_UART_RX, NRF_GPIO_PIN_DIR_OUTPUT);
//
//   ret_code_t error_code = NRF_SUCCESS;
//   int status;
//
//   nrf_serial_init(&serial_uart, &m_uart0_drv_config, &serial_config);
//   // status = nrf_serial_flush(&serial_uart, NRF_SERIAL_MAX_TIMEOUT);
//   // if(status != NRF_SUCCESS) {
//   //   printf("flush error: %d\n", status);
//   //   return status;
//   // }
//   // status = nrf_serial_rx_drain(&serial_uart);
//   // if(status != NRF_SUCCESS) {
//   //   printf("rx drain error: %d\n", status);
//   //   return status;
//   // }
//
//   // initialize RTT library
//   error_code = NRF_LOG_INIT(NULL);
//   APP_ERROR_CHECK(error_code);
//   NRF_LOG_DEFAULT_BACKENDS_INIT();
//   printf("Log initialized!\n");
//
//   // initialize LEDs
//   nrf_gpio_pin_dir_set(23, NRF_GPIO_PIN_DIR_OUTPUT);
//   nrf_gpio_pin_dir_set(24, NRF_GPIO_PIN_DIR_OUTPUT);
//   nrf_gpio_pin_dir_set(25, NRF_GPIO_PIN_DIR_OUTPUT);
//
//   // // initialize display
//   // nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
//   // nrf_drv_spi_config_t spi_config = {
//   //   .sck_pin = BUCKLER_LCD_SCLK,
//   //   .mosi_pin = BUCKLER_LCD_MOSI,
//   //   .miso_pin = BUCKLER_LCD_MISO,
//   //   .ss_pin = BUCKLER_LCD_CS,
//   //   .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
//   //   .orc = 0,
//   //   .frequency = NRF_DRV_SPI_FREQ_4M,
//   //   .mode = NRF_DRV_SPI_MODE_2,
//   //   .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
//   // };
//   // error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
//   // APP_ERROR_CHECK(error_code);
//   // display_init(&spi_instance);
//   // display_write("Hello, Human!", DISPLAY_LINE_0);
//   // printf("Display initialized!\n");
//   //
//   // // initialize i2c master (two wire interface)
//   // nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
//   // i2c_config.scl = BUCKLER_SENSORS_SCL;
//   // i2c_config.sda = BUCKLER_SENSORS_SDA;
//   // i2c_config.frequency = NRF_TWIM_FREQ_100K;
//   // error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
//   // APP_ERROR_CHECK(error_code);
//   // lsm9ds1_init(&twi_mngr_instance);
//   // printf("IMU initialized!\n");
//
//   // initialize Kobuki
//   // kobukiInit();
//   // printf("Kobuki initialized!\n");
//
//   // configure initial state
//   // robot_state_t state = OFF;
//   // KobukiSensors_t sensors = {0};
//   uint8_t lidarData = 0;
//
//   //for testing uart
//   while (true)
//     {
//       status = nrf_serial_read(&serial_uart, &lidarData, 1, NULL, 100);
//
//
//         // for (int i = 0; i < 32; i++){
//         //     printf("%X",lidarData[i] );
//         //   }
//           printf("%X\n", lidarData);
//           status = nrf_serial_flush(&serial_uart, NRF_SERIAL_MAX_TIMEOUT);
//           status = nrf_serial_rx_drain(&serial_uart);
//
//     }
//   // printf("Hi!\n");
//
//   // loop forever, running state machine
//   // while (1) {
//   //   // read sensors from robot
//   //   kobukiSensorPoll(&sensors);
//   //
//   //   // delay before continuing
//   //   // Note: removing this delay will make responses quicker, but will result
//   //   //  in printf's in this loop breaking JTAG
//   //   nrf_delay_ms(1);
//   //
//   //   // handle states
//   //   switch(state) {
//   //   	printf("test\n");
//   //     case OFF: {
//   //       // transition logic
//   //       if (is_button_pressed(&sensors)) {
//   //         state = DRIVING;
//   //       } else {
//   //         // perform state-specific actions here
//   //         display_write("OFF", DISPLAY_LINE_0);
//   //
//   //         state = OFF;
//   //       }
//   //       break; // each case needs to end with break!
//   //     }
//   //
//   //     case DRIVING: {
//   //       // transition logic
//   //       if (is_button_pressed(&sensors)) {
//   //         state = OFF;
//   //       } else {
//   //         // perform state-specific actions here
//   //         display_write("DRIVING", DISPLAY_LINE_0);
//   //
//   //         state = DRIVING;
//   //       }
//   //       break; // each case needs to end with break!
//   //     }
//   //
//   //     // add other cases here
//   //
//   //   }
//   // }
// }
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
// #include "bsp.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED


#define MAX_TEST_DATA_BYTES     (1U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

int main(void)
{
    uint32_t err_code;
  // initialize RTT library
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    printf("Log initialized!\n");

    nrf_gpio_pin_dir_set(BUCKLER_UART_RX, NRF_GPIO_PIN_DIR_INPUT);
    nrf_gpio_pin_dir_set(BUCKLER_UART_TX, NRF_GPIO_PIN_DIR_OUTPUT);



    const app_uart_comm_params_t comm_params =
      {
          BUCKLER_UART_RX,
          BUCKLER_UART_TX,
          0,
          0,
          UART_HWFC,
          false,
          NRF_UART_BAUDRATE_115200
          // #if defined (UART_PRESENT)
          //           NRF_UART_BAUDRATE_115200
          // #else
          //           NRF_UARTE_BAUDRATE_115200
          // #endif
                };
      APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code);

    printf("\r\nUART example started.\r\n");

   while (true)
   {
       uint8_t cr;
        while (app_uart_get(&cr) != NRF_SUCCESS);
       // while (app_uart_put(cr) != NRF_SUCCESS);
        printf("%x\n",cr );


   }
 }
