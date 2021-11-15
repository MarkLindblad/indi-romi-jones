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

#include "app_uart.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif


#include "buckler.h"
#include "display.h"
#include "kobukiActuator.h"
#include "kobukiSensorPoll.h"
#include "kobukiSensorTypes.h"
#include "kobukiUtilities.h"
#include "lsm9ds1.h"


#define UART_TX_BUF_SIZE 8                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 8                         /**< UART RX buffer size. */
#define RX_PIN_NUMBER 10
#define TX_PIN_NUMBER 8
#define RTS_PIN_NUMBER NULL
#define CTS_PIN_NUMBER NULL
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED
#define NRF_UARTE_BAUDRATE_115200 115200

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


// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

typedef enum {
  OFF,
  DRIVING,
} robot_state_t;


int main(void) {
  ret_code_t error_code = NRF_SUCCESS;
  uint32_t err_code;

  const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          UART_HWFC,
          false,
          NRF_UARTE_BAUDRATE_115200
        };

  APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code);

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  // initialize LEDs
  nrf_gpio_pin_dir_set(23, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(24, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(25, NRF_GPIO_PIN_DIR_OUTPUT);

  // // initialize display
  // nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
  // nrf_drv_spi_config_t spi_config = {
  //   .sck_pin = BUCKLER_LCD_SCLK,
  //   .mosi_pin = BUCKLER_LCD_MOSI,
  //   .miso_pin = BUCKLER_LCD_MISO,
  //   .ss_pin = BUCKLER_LCD_CS,
  //   .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
  //   .orc = 0,
  //   .frequency = NRF_DRV_SPI_FREQ_4M,
  //   .mode = NRF_DRV_SPI_MODE_2,
  //   .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  // };
  // error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  // APP_ERROR_CHECK(error_code);
  // display_init(&spi_instance);
  // display_write("Hello, Human!", DISPLAY_LINE_0);
  // printf("Display initialized!\n");
  //
  // // initialize i2c master (two wire interface)
  // nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  // i2c_config.scl = BUCKLER_SENSORS_SCL;
  // i2c_config.sda = BUCKLER_SENSORS_SDA;
  // i2c_config.frequency = NRF_TWIM_FREQ_100K;
  // error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  // APP_ERROR_CHECK(error_code);
  // lsm9ds1_init(&twi_mngr_instance);
  // printf("IMU initialized!\n");

  // initialize Kobuki
  // kobukiInit();
  // printf("Kobuki initialized!\n");

  // configure initial state
  robot_state_t state = OFF;
  KobukiSensors_t sensors = {0};

  //for testing uart
  while (true)
    {
      printf("Here\n" );
        uint8_t cr;
        while (app_uart_get(&cr) != NRF_SUCCESS);
        // while (app_uart_put(cr) != NRF_SUCCESS);
        printf("%c\n",cr );
    }
  // printf("Hi!\n");

  // loop forever, running state machine
  // while (1) {
  //   // read sensors from robot
  //   kobukiSensorPoll(&sensors);
  //
  //   // delay before continuing
  //   // Note: removing this delay will make responses quicker, but will result
  //   //  in printf's in this loop breaking JTAG
  //   nrf_delay_ms(1);
  //
  //   // handle states
  //   switch(state) {
  //   	printf("test\n");
  //     case OFF: {
  //       // transition logic
  //       if (is_button_pressed(&sensors)) {
  //         state = DRIVING;
  //       } else {
  //         // perform state-specific actions here
  //         display_write("OFF", DISPLAY_LINE_0);
  //
  //         state = OFF;
  //       }
  //       break; // each case needs to end with break!
  //     }
  //
  //     case DRIVING: {
  //       // transition logic
  //       if (is_button_pressed(&sensors)) {
  //         state = OFF;
  //       } else {
  //         // perform state-specific actions here
  //         display_write("DRIVING", DISPLAY_LINE_0);
  //
  //         state = DRIVING;
  //       }
  //       break; // each case needs to end with break!
  //     }
  //
  //     // add other cases here
  //
  //   }
  // }
}
