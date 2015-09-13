/*
 * hw.hpp
 *
 * Created: 6.10.2014 14:24:31
 *  Author: kubas
 */ 

AVRLIB_DEFINE_XMEGA_PIN(debug_tx_pin, PORTF, 3);
AVRLIB_DEFINE_XMEGA_PIN(debug_rx_pin, PORTF, 2);
USART_t& debug_usart = USARTF0;
#define debug_usart_rxc_vect USARTF0_RXC_vect
#define debug_usart_txc_vect USARTF0_TXC_vect
#define debug_usart_dre_vect USARTF0_DRE_vect

AVRLIB_DEFINE_XMEGA_PIN(driver_tx_pin, PORTC, 3);
AVRLIB_DEFINE_XMEGA_PIN(driver_rx_pin, PORTC, 2);
USART_t& driver_usart = USARTC0;
#define driver_usart_rxc_vect USARTC0_RXC_vect
#define driver_usart_txc_vect USARTC0_TXC_vect
#define driver_usart_dre_vect USARTC0_DRE_vect

AVRLIB_DEFINE_XMEGA_PIN(position_sensor_tx_pin, PORTC, 7);
AVRLIB_DEFINE_XMEGA_PIN(position_sensor_rx_pin, PORTC, 6);
USART_t& position_sensor_usart = USARTC1;
#define position_sensor_usart_rxc_vect USARTC1_RXC_vect
#define position_sensor_usart_txc_vect USARTC1_TXC_vect
#define position_sensor_usart_dre_vect USARTC1_DRE_vect

AVRLIB_DEFINE_XMEGA_PIN(bluetooth_tx_pin, PORTE, 3);
AVRLIB_DEFINE_XMEGA_PIN(bluetooth_rx_pin, PORTE, 2);
USART_t& bluetooth_usart = USARTE0;
#define bluetooth_usart_rxc_vect USARTE0_RXC_vect
#define bluetooth_usart_txc_vect USARTE0_TXC_vect
#define bluetooth_usart_dre_vect USARTE0_DRE_vect

AVRLIB_DEFINE_XMEGA_PIN(controller_tx_pin, PORTD, 7);
AVRLIB_DEFINE_XMEGA_PIN(controller_rx_pin, PORTD, 6);
USART_t& controller_usart = USARTD1;
#define controller_usart_rxc_vect USARTD1_RXC_vect
#define controller_usart_txc_vect USARTD1_TXC_vect
#define controller_usart_dre_vect USARTD1_DRE_vect
// multiplexed with USB

AVRLIB_DEFINE_XMEGA_PIN(rs485_tx_pin, PORTD, 3);
AVRLIB_DEFINE_XMEGA_PIN(rs485_rx_pin, PORTD, 2);
AVRLIB_DEFINE_XMEGA_PIN(rs485_dir_pin, PORTD, 1);
USART_t& rs485_usart = USARTD0;
#define rs485_usart_rxc_vect USARTD0_RXC_vect
#define rs485_usart_txc_vect USARTD0_TXC_vect
#define rs485_usart_dre_vect USARTD0_DRE_vect


AVRLIB_DEFINE_XMEGA_PIN(sck, PORTE, 7);
AVRLIB_DEFINE_XMEGA_PIN(miso, PORTE, 6);
AVRLIB_DEFINE_XMEGA_PIN(mosi, PORTE, 5);
AVRLIB_DEFINE_XMEGA_PIN(ss, PORTE, 4);

AVRLIB_DEFINE_XMEGA_PIN(left_enc_cs, PORTB, 0);  // GPIO5
AVRLIB_DEFINE_XMEGA_PIN(right_enc_cs, PORTB, 1); // GPIO6


AVRLIB_DEFINE_XMEGA_PIN(pwr_en, PORTE, 1);

AVRLIB_DEFINE_XMEGA_PIN(off_btn, PORTE, 0);


AVRLIB_DEFINE_XMEGA_PIN(led0_pin, PORTR, 0);
AVRLIB_DEFINE_XMEGA_PIN(led1_pin, PORTR, 1);

AVRLIB_DEFINE_XMEGA_PIN(led2p_pin, PORTC, 0);
AVRLIB_DEFINE_XMEGA_PIN(led2n_pin, PORTC, 1);

AVRLIB_DEFINE_XMEGA_PIN(repro_pin, PORTD, 0);
TC0_t& repro_timer = TCD0;
static const uint8_t repro_timer_channel = 0;


static const uint8_t left_motor_thermistor_channel = 4;		// GPIO1
static const uint8_t right_motor_thermistor_channel = 5;	// GPIO2
static const uint8_t power_mosfets_thermistor_channel = 6;	// GPIO3
static const uint8_t driver_thermistor_channel = 7;			// GPIO4

static const uint8_t flyer_feeder_sensor_channel = 3; // GPIO0


AVRLIB_DEFINE_XMEGA_PIN(flyer_feeder_sensor_led, PORTC, 5);

AVRLIB_DEFINE_XMEGA_PIN(flyer_feeder_magnet_pin, PORTC, 4);
TC1_t& flyer_feeder_magnet_timer = TCC1;
TC1_t& flyer_feeder_sensor_timer = TCC1;
static const uint8_t flyer_feeder_magnet_timer_channel = 0;
static const uint8_t flyer_feeder_sensor_timer_channel = 1;

PORT_t& flyer_feeder_motor_port = PORTF;
static const uint8_t flyer_feeder_motor_gp = 4;
static const uint8_t flyer_feeder_motor_gm = 0xF0;


AVRLIB_DEFINE_XMEGA_PIN(servo0_pin, PORTF, 0);
AVRLIB_DEFINE_XMEGA_PIN(servo1_pin, PORTF, 1);
AVRLIB_DEFINE_XMEGA_PIN(servo2_pin, PORTD, 4);
AVRLIB_DEFINE_XMEGA_PIN(servo3_pin, PORTD, 5);

TC0_t& servo01_timer = TCF0;
static const uint8_t servo0_timer_channel = 0;
static const uint8_t servo1_timer_channel = 1;

static const uint8_t servo01_current_channel = 0;

TC1_t& servo23_timer = TCD1;
static const uint8_t servo2_timer_channel = 0;
static const uint8_t servo3_timer_channel = 1;

static const uint8_t servo23_current_channel = 1;


static const uint8_t main_board_power_channel = 2; // unused - powered from stabilized 5 V