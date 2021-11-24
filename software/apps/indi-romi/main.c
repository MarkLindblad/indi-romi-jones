

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

#include "app_pwm.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED

#include "buckler.h"
#include "display.h"
#include "kobukiActuator.h"
#include "kobukiSensorPoll.h"
#include "kobukiSensorTypes.h"
#include "kobukiUtilities.h"
#include "lsm9ds1.h"
#include <nrf_serial.h>

#include "YDLidar.h"
#include "simple_ble.h"



#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */
// #define UART_RX_BUF_SIZE 512                         /**< UART RX buffer size. */


//pwm instance
APP_PWM_INSTANCE(PWM1,1);
static volatile bool pwm_ready;            // A flag indicating PWM status.


// BLE configuration
// This is mostly irrelevant since we are scanning only
static simple_ble_config_t ble_config = {
        // BLE address is c0:98:e5:49:00:00
        .platform_id       = 0x49,    // used as 4th octet in device BLE address
        .device_id         = 0x1234,  // Last two octets of device address
        .adv_name          = "Indi-Romi Jones", // irrelevant in this example
        .adv_interval      = MSEC_TO_UNITS(2000, UNIT_0_625_MS), // send a packet once per x second (minimum is 20 ms)
        .min_conn_interval = MSEC_TO_UNITS(500, UNIT_1_25_MS), // irrelevant if advertising only
        .max_conn_interval = MSEC_TO_UNITS(1000, UNIT_1_25_MS), // irrelevant if advertising only
};
simple_ble_app_t* simple_ble_app;

//4607eda0-f65e-4d59-a9ff-84420d87a4ca
static simple_ble_service_t robot_service = {{
    .uuid128 = {0xca,0xa4,0x87,0x0d,0x42,0x84,0xff,0xA9,
                0x59,0x4D,0x5e,0xf6,0xa0,0xed,0x07,0x46}
}}; 
 //0xa0ed
static simple_ble_char_t dist_char = {.uuid16 = 0xa0ee};
static float ble_distance = 0.0;

static simple_ble_char_t angle_char = {.uuid16 = 0xa0f};
static int ble_angle = 90;



void pwm_ready_callback(uint32_t pwm_id)    // PWM callback function
{
    pwm_ready = true;
}


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
void lidar_speed(uint8_t value){
  while (app_pwm_channel_duty_set(&PWM1, 0, value) == NRF_ERROR_BUSY);
}
void start_lidar(){
  app_pwm_enable(&PWM1);
}

void stop_lidar(){
  app_pwm_disable(&PWM1);
}
const app_uart_comm_params_t comm_params =
  {
      NRF_GPIO_PIN_MAP(0,11),
      BUCKLER_UART_TX,
      0,
      0,
      UART_HWFC,
      false,
      NRF_UART_BAUDRATE_115200

            };

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);




typedef enum {
  OFF,
  DRIVING,
} robot_state_t;


int main(void) {
  uint32_t err_code;
  APP_UART_FIFO_INIT(&comm_params,
                     UART_RX_BUF_SIZE,
                     UART_TX_BUF_SIZE,
                     uart_error_handle,
                     // APP_IRQ_PRIORITY_LOWEST,
                     APP_IRQ_PRIORITY_HIGHEST,
                     err_code);
  APP_ERROR_CHECK(err_code);

  ret_code_t error_code = NRF_SUCCESS;

  // setup ble
  simple_ble_app = simple_ble_init(&ble_config);

  simple_ble_add_service(&robot_service);

  simple_ble_add_characteristic(1, 1, 0, 0, // read, write, notify, vlen
    sizeof(ble_distance), (uint8_t*)&ble_distance,
    &robot_service, &dist_char);

  simple_ble_add_characteristic(1, 1, 0, 0, // read, write, notify, vlen
    sizeof(ble_angle), (uint8_t*)&ble_angle,
    &robot_service, &angle_char);



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

  // // initialize i2c master (two wire interface)
  // nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  // i2c_config.scl = BUCKLER_SENSORS_SCL;
  // i2c_config.sda = BUCKLER_SENSORS_SDA;
  // i2c_config.frequency = NRF_TWIM_FREQ_100K;
  // error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  // APP_ERROR_CHECK(error_code);
  // lsm9ds1_init(&twi_mngr_instance);
  // printf("IMU initialized!\n");

  //initialize PWM w/ period 100us
   app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(100, NRF_GPIO_PIN_MAP(0,12));
   err_code = app_pwm_init(&PWM1,&pwm1_cfg,pwm_ready_callback);
   APP_ERROR_CHECK(err_code);

   lidar_speed(20);
   start_lidar();
   printf("started Lidar\n");
  // initialize Kobuki
  // kobukiInit();
  // printf("Kobuki initialized!\n");

  // configure initial state
  robot_state_t state = OFF;
  KobukiSensors_t sensors = {0};

  simple_ble_adv_only_name();

  printf("\r\nUART example started.\r\n");
  uint8_t cr[32] = {0};
  int i = 0;

  while (true)
  {   
    // while (app_uart_get(cr + i) != NRF_SUCCESS);
    // // printf("%X",cr[i] );
    // i++;
    // if (i >=32) {
    //   for (int i = 0; i < 32; i++) {
    //     printf("%X",cr[i]);
    //   }
    //   printf("\n");
    //   i = 0;
    //   uint8_t cr[32] = {0};
      app_uart_flush();
      printf("HERE\n");
     if (waitScanDot(100) == RESULT_OK){
       scanPoint point = getCurrentScanPoint();
       int angle = (int) point.angle % 360;
       if ( -3 + 0 <=  angle && angle <= 3 + 0 ) {
            // printf("angle: %d distance %.2f quality: %d\n",angle, point.distance/10, point.quality );
            ble_distance = point.distance;
            ble_angle = angle;
          }

     }
      // Sleep while SoftDevice handles BLE
      power_manage();
     // nrf_delay_ms(100);
    }
  }

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
// }





// sample lidar byte stream
// AA5502895764784788F000000000000000000000000000000000000000000000000000000000000
// AA550289D9521A3614B0000000000000000000000
// 4484C41441C42042843443C44444C45446046447047C4
// BC4C84D04AA5501A79A317ACDB4A0000000000000000
// 00000000000000000000000000
// AA551173AC0
// AA55028DAE8B791D5000000000000000000000
// 000000000000000000000000000911951991B11B51B91
// AA55028E5759154A6EBD1C51C91CD1D51D91E11E5
// 2D21521922122923123924124925125926126D279281291
// 2F12131132133133D3AA55028B11525234C7249351359361369
// 38D39539939D3A53A93B13B93BD3C93D03DC30000000
// 000E240C1419142244F24EE3F9354
// AA550287F231331B96F11419
// 4714814994AD48A45646147548949D4A94A949947E479471
// 42541549454FD3F93F93FD3541543642D4
// AA550286531F1
// 43543143142D43143143142D42942D43143543943D43D441
// 45545D45545D48246645945D46546D4714794854954A94AA
// 4D14E14F1415D5215315495615795955B15CD5E95E636
// 6D700000000002A49D2529553F95396527D517550124F14E91
// 35AA550282F4DAF5A646E9D3521362A4C254C664E5A501253FE4D2556D5526
// 5F262E560A66A60A258B62CBE2EA62DCD2CA62AC929852951291D2A362B72
// 40DA32AC33CE3BAA5502895B99682057B23D643E462546E145A9458145B5
// 44C14475453642C400284344444504584007A40000000
// 30CD30AD302373A2B5628B225AA55028F3687F761460AA235A21A61F8E1E0
// 0005E56055C56056856856C57057457057C58C500000
// 0000000000000000000
