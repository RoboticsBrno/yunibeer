/*
 * Yunibeer
 *     main_board.cpp
 *
 * Created: 6.10.2014 13:01:31
 *  Author: kubas
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

#include "avrlib/xmega_pin.hpp"
#include "avrlib/bootseq.hpp"
#include "avrlib/sync_usart.hpp"
#include "avrlib/async_usart.hpp"
#include "avrlib/uart_xmega.hpp"
#include "avrlib/string.hpp"
#include "avrlib/format.hpp"
#include "avrlib/command_parser.hpp"
#include "avrlib/timer_xmega.hpp"
#include "avrlib/counter.hpp"
#include "avrlib/stopwatch.hpp"
#include "avrlib/math.hpp"
#include "avrlib/pair.hpp"
#include "avrlib/sequence_detector.hpp"

#include "hw.hpp"
#include "eeprom.hpp"

using avrlib::format;
using avrlib::format_spgm;
using avrlib::send;
using avrlib::send_spgm;
using avrlib::string;
using avrlib::clamp;

#define nop() __asm__ volatile ("nop")

// usarts

#define bootseq avrlib::nobootseq

typedef avrlib::async_usart<avrlib::uart_xmega, 16, 128, bootseq, uint8_t> debug_port_type;
debug_port_type debug_port;
ISR(debug_usart_rxc_vect) { debug_port.intr_rx(); }
	
typedef avrlib::async_usart<avrlib::uart_xmega, 32, 32, bootseq, uint8_t> bluetooth_port_type;
bluetooth_port_type bluetooth_port;
ISR(bluetooth_usart_rxc_vect) { bluetooth_port.intr_rx(); }
	
typedef avrlib::async_usart<avrlib::uart_xmega, 32, 32, bootseq, uint8_t> controller_port_type;
controller_port_type controller_port;
ISR(controller_usart_rxc_vect) { controller_port.intr_rx(); }
	
typedef avrlib::async_usart<avrlib::uart_xmega, 32, 32, avrlib::nobootseq, uint8_t> position_sensor_port_type;
position_sensor_port_type position_sensor_port;
ISR(position_sensor_usart_rxc_vect) { position_sensor_port.intr_rx(); }
	
typedef avrlib::async_usart<avrlib::uart_xmega, 32, 32, avrlib::nobootseq, uint8_t> driver_port_type;
driver_port_type driver_port;
ISR(driver_usart_rxc_vect) { driver_port.intr_rx(); }
ISR(driver_usart_dre_vect) { driver_port.intr_tx(); }
	
typedef avrlib::async_usart<avrlib::uart_xmega, 32, 32, avrlib::nobootseq, uint8_t> rs485_type;
rs485_type rs485;
ISR(rs485_usart_rxc_vect) { rs485.intr_rx(); }

template <typename main_port_type, typename redir1_type, typename redir2_type>
class debug_t
{
public:
	debug_t(main_port_type& main_port, redir1_type& redir1_port, redir2_type& redir2_port, uint8_t * const eeprom_addr)
		:main_port(main_port), redir1_port(redir1_port), redir2_port(redir2_port), selected_port(0),
		 selected_port_eeprom_addr(eeprom_addr), input_enable_seq("debug_in_en")
	{}
	void init() // has to be called after eeprom mapping
	{
		selected_port = *selected_port_eeprom_addr;
	}
	void redirect(const uint8_t& port_index)
	{
		selected_port = port_index;
		*selected_port_eeprom_addr = selected_port;
		flush_eeprom_page_buffer(selected_port_eeprom_addr);
	}
	uint8_t get_port_index() const
	{
		return selected_port;
	}
	void write(const char& ch)
	{
		main_port.write(ch);
		switch(selected_port & 0x7F)
		{
			case  1: redir1_port.write(ch); break;
			case  2: redir2_port.write(ch); break;
		}
	}
	void process_tx()
	{
		main_port.process_tx();
		switch(selected_port & 0x7F)
		{
			case  1: redir1_port.process_tx(); break;
			case  2: redir2_port.process_tx(); break;
		}
	}
	void flush()
	{
		main_port.flush();
		switch(selected_port & 0x7F)
		{
			case  1: redir1_port.flush(); break;
			case  2: redir2_port.flush(); break;
		}
	}
	bool empty() const
	{
		if(!main_port.empty())
			return false;
		switch(selected_port)
		{
			case 0x01:
			case 0x02: return false;
			case 0x81: return redir1_port.empty();
			case 0x82: return redir2_port.empty();
			default:   return   main_port.empty();
		}
	}
	char read()
	{
		if(!main_port.empty())
			return main_port.read();
		switch(selected_port)
		{
			case 0x01:
				if(input_enable_seq(redir1_port.rx_buffer().top()))
					selected_port |= 0x80;
				return 0;
			case 0x02:
				if(input_enable_seq(redir2_port.rx_buffer().top()))
					selected_port |= 0x80;
				return 0;
			case 0x81: return redir1_port.read();
			case 0x82: return redir2_port.read();
			default:   return   main_port.read();
		}
	}
private:
	main_port_type& main_port;
	redir1_type& redir1_port;
	redir2_type& redir2_port;
	uint8_t selected_port;
	uint8_t * const selected_port_eeprom_addr;
	avrlib::sequence_detector_t<> input_enable_seq;
};

debug_t<debug_port_type, bluetooth_port_type, controller_port_type>
	debug(debug_port, bluetooth_port, controller_port, debug_port_eeprom_addr);


uint8_t ReadCalibrationByte(const uint8_t& index)
{
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	uint8_t result = pgm_read_byte(index);
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;
	return result;
}


#include "time.hpp"
#include "roboclaw.hpp"
#include "flyer_feeder.hpp"

typedef led0_pin led0;

int main(void)
{
    led0_pin::make_low();
	led1_pin::make_low();
	
	led2n_pin::make_low();
	led2p_pin::make_low();
	
	led0::make_high(); // only alias for led0_pin
	
	pwr_en::make_high();
	pwr_en::pinctrl(PORT_OPC_WIREDANDPULL_gc);
	
	off_btn::pullup();
	off_btn::make_inverted();
	
	debug_rx_pin::pullup();
	debug_tx_pin::make_high();
	
	driver_rx_pin::pullup();
	driver_tx_pin::make_high();
	
	position_sensor_rx_pin::pullup();
	position_sensor_tx_pin::make_high();
	
	bluetooth_rx_pin::pullup();
	bluetooth_tx_pin::make_high();
	
	controller_rx_pin::pullup();
	controller_tx_pin::make_high();
	
	rs485_rx_pin::pullup();
	rs485_tx_pin::make_high();
	rs485_dir_pin::make_low();
	
	OSC.CTRL = OSC_RC32MEN_bm | OSC_RC2MEN_bm | OSC_RC32KEN_bm;
	uint32_t timeout_cnt = 0;
	while ((OSC.STATUS & OSC_RC32MRDY_bm) == 0)
	{
		if(++timeout_cnt == 1000000)
		{
			for(uint8_t i = 0; i != 7; ++i)
			{
				led0::toggle();
				_delay_ms(8000); // 0.5 s because still 2 MHz clock, instead of 32 MHz
			}
			CCP = CCP_IOREG_gc;
			RST_CTRL = RST_SWRST_bm;
		}
	}
	CCP = CCP_IOREG_gc;
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc;
	OSC.CTRL = OSC_RC32MEN_bm | OSC_RC32KEN_bm;
	timeout_cnt = 0;
	while ((OSC.STATUS & OSC_RC32KRDY_bm) == 0)
	{
		if(++timeout_cnt == 1000000)
		{
			for(uint8_t i = 0; i != 9; ++i)
			{
				led0::toggle();
				_delay_ms(500);
			}
			CCP = CCP_IOREG_gc;
			RST_CTRL = RST_SWRST_bm;
		}
	}
	DFLLRC32M.CTRL = DFLL_ENABLE_bm;
	
	NVM.CTRLB = NVM_EEMAPEN_bm;
	while((NVM.STATUS & NVM_NVMBUSY_bm) != 0){};
		
	debug.init();
	    
	debug_port.usart().open(debug_usart						, (-3<<12)|131, true); // 115200
	bluetooth_port.usart().open(bluetooth_usart				, (-3<<12)|131, true);
	controller_port.usart().open(controller_usart			, (-3<<12)|131, true);
	position_sensor_port.usart().open(position_sensor_usart	, (-3<<12)|131, true);
	rs485.usart().open(rs485_usart							, (-3<<12)|131, true);
	driver_port.usart().open(driver_usart					, ( 2<<12)| 12, true); // 38400
	driver_port.async_tx(true);
	_delay_ms(1);
	
	format_spgm(debug, PSTR("\nYunibeer\n\tRST_STATUS = %x2\n")) % uint8_t(RST_STATUS);
	debug.flush();
	RST_STATUS = 0x3F;
	
	base_timer_type::timer_type::tov_interrupt(true);
	base_timer_type::timer_type::clock_source(avrlib::timer_fosc_1);
	
	PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	sei();
	
	led0::set_low();
	
	ss::pullup();
	mosi::make_low();
	miso::pulldown();
	sck::make_low();
	
	left_enc_cs::make_low();
	wait(msec(1));
	left_enc_cs::set_high();
	wait(usec(1));
	right_enc_cs::make_low();
	wait(msec(1));
	right_enc_cs::set_high();
	
	roboclaw_t<driver_port_type> motors(driver_port, 128);
	
	timeout second_timeout(sec(1));
	
	char ch = 0;
	
	timeout driver_read_timeout(sec(1));
	driver_read_timeout.cancel();
	
	timeout off_timeout(sec(3));
	off_timeout.cancel();
	
	avrlib::timed_command_parser<base_timer_type> cmd_parser(base_timer, msec(10));
	
	timeout emergency_brake_timeout(msec(500));
	emergency_brake_timeout.force();
	
	timeout led_send_delay(msec(200));
	
	motors.set_pwm_resolution(motors.pwm_10bit);
	
	feeder_t<flyer_feeder_magnet_pin, flyer_feeder_sensor_led> feeder(
		motor_t(flyer_feeder_motor_port, flyer_feeder_motor_gm),
		magnet_t<flyer_feeder_magnet_pin>(),
		optosensor_t<flyer_feeder_sensor_led>(flyer_feeder_threshold_eeprom_addr, flyer_feeder_sensor_channel));
	button_t feed_button(1<<0);
	
	for(;;)
	{
		if(!debug.empty())
		{
			ch = debug.read();
			switch(ch)
			{
				case '\r':
					debug.write('\n');
					break;
					
				case 'i':
					driver_port.write(128);
					driver_port.write(21);
					driver_port.flush();
					driver_read_timeout.restart();
					for(uint8_t cnt = 0; 1; ++cnt)
					{
						if(driver_read_timeout)
						{
							driver_read_timeout.cancel();
							send(debug, "timeout\n");
							break;
						}
						ch = driver_port.read();
						if(ch == 0)
						{
							format(debug, "\n received %  chars\n") % cnt;
							break;
						}
						debug.write(ch);
					}
					break;
					
				case 'O':
					pwr_en::set_low();
					for(;;);
					
				case 'A':
				case 'B':
				case 'C':
					debug.redirect((ch - 'A') | 0x80);
					break;
					
				case 'D':
					debug.redirect(debug.get_port_index() & 0x7F);
					break;
					
				case 'f':
					feeder.feed();
					break;
					
				case 'R':
					feeder.reset();
					break;
					
				case '+':
					feeder.paper_sensor().increase_threshold();
					format(debug, "pwm: % \n") % feeder.paper_sensor().get_threshold();
					break;
					
				case '-':
					feeder.paper_sensor().decrease_threshold();
					format(debug, "pwm: % \n") % feeder.paper_sensor().get_threshold();
					break;
					
				case 'E':
					feeder.paper_sensor().save_threshold();
					break;
				
				case 'a':
					feeder.automatic();
					break;
					
				case 'm':
					feeder.automatic(false);
					break;
			}
		}
		
		led0::set_value(feeder.paper_sensor());
		
		if(!driver_port.empty())
		{
			format(debug, "rec %x2 \n") % uint8_t(driver_port.read());
		}
		
		if (!bluetooth_port.empty())
		{
			ch = bluetooth_port.read();
			switch(cmd_parser.push_data(ch))
			{
			case 1:
				if(cmd_parser.size() == 9)
				{
					int8_t fwd = cmd_parser[3];
					int8_t cross = cmd_parser[1];

					if (fwd < -8)
						fwd += 8;
					else if (fwd > 8)
						fwd -= 8;
					else
						fwd = 0;

					if (cross < -8)
						cross += 8;
					else if (cross > 8)
						cross -= 8;
					else
						cross = 0;

					cross /= 2;

					uint8_t buttons = cmd_parser[8];

					uint8_t slowGetter = 0;
					if((buttons & (1<<2)) != 0 && (buttons & (1<<3)) != 0)
						slowGetter = 0;
					else if((buttons & (1<<2)) == 0 && (buttons & (1<<3)) == 0)
						slowGetter = 2;
					else
						slowGetter = 1;
						
					int16_t left_target = clamp((fwd + cross) * 8, -1023, 1023);
					int16_t right_target = clamp((fwd - cross) * 8, -1023, 1023);
					
					motors.set_left_power ( left_target >> slowGetter/*, (buttons & 1) == 0*/);
					motors.set_right_power(right_target >> slowGetter/*, (buttons & 1) == 0*/);
					
					feed_button.process(buttons);
					if(feed_button.changed())
						feeder.automatic(feed_button);
					
					emergency_brake_timeout.restart();
				}
				break;
					
				default:
					emergency_brake_timeout.start();
					break;
				
			case 255:
				break;
			}
		}
		
		if (emergency_brake_timeout)
		{
			motors.set_power(0, 0);
			emergency_brake_timeout.cancel();
			if(feed_button.changed())
				feeder.automatic();
		}

		if (led_send_delay && !emergency_brake_timeout)
		{
			led_send_delay.restart();
		}
		
		if(off_btn::read())
		{
			if(off_timeout)
			{
				pwr_en::set_low();
				for(;;);
			}
			else if(!off_timeout.running())
			{
				off_timeout.restart();
			}
			led2p_pin::set_low();
			led2n_pin::set_high();
		}
		else
		{
			off_timeout.cancel();
			led2n_pin::set_low();
			led2p_pin::set_high();
		}
		
		if(second_timeout)
		{
			second_timeout.ack();
			led1_pin::toggle();
		}
		
		feeder.process();
		
		debug.process_tx();
		bluetooth_port.process_tx();
		controller_port.process_tx();
		position_sensor_port.process_tx();
		rs485.process_tx();
	}
}