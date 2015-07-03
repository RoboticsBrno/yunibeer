/*
 * main_board.cpp
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

#include "hw.hpp"

using avrlib::format;
using avrlib::format_spgm;
using avrlib::send;
using avrlib::send_spgm;
using avrlib::string;

#define nop() __asm__ volatile ("nop")

// usarts

typedef avrlib::async_usart<avrlib::uart_xmega, 16, 128, avrlib::nobootseq, uint8_t> debug_type;
debug_type debug;
ISR(debug_usart_rxc_vect) { debug.intr_rx(); }
	
typedef avrlib::async_usart<avrlib::uart_xmega, 32, 32, avrlib::nobootseq, uint8_t> bluetooth_port_type;
bluetooth_port_type bluetooth_port;
ISR(bluetooth_usart_rxc_vect) { bluetooth_port.intr_rx(); }
	
typedef avrlib::async_usart<avrlib::uart_xmega, 32, 32, avrlib::nobootseq, uint8_t> controller_port_type;
controller_port_type controller_port;
ISR(controller_usart_rxc_vect) { controller_port.intr_rx(); }
	
typedef avrlib::async_usart<avrlib::uart_xmega, 32, 32, avrlib::nobootseq, uint8_t> position_sensor_port_type;
position_sensor_port_type position_sensor_port;
ISR(position_sensor_usart_rxc_vect) { position_sensor_port.intr_rx(); }
	
typedef avrlib::async_usart<avrlib::uart_xmega, 32, 32, avrlib::nobootseq, uint8_t> driver_port_type;
driver_port_type driver_port;
ISR(driver_usart_rxc_vect) { driver_port.intr_rx(); }
	
typedef avrlib::async_usart<avrlib::uart_xmega, 32, 32, avrlib::nobootseq, uint8_t> rs485_type;
rs485_type rs485;
ISR(rs485_usart_rxc_vect) { rs485.intr_rx(); }
	
	
// timing
	
AVRLIB_XMEGA_TIMER(timerc0, TCC0);
typedef avrlib::counter<timerc0, uint32_t, uint32_t, false> base_timer_type;
base_timer_type base_timer;
ISR(TCC0_OVF_vect) { base_timer.tov_interrupt(); }

struct stopwatch
	:avrlib::stopwatch<base_timer_type>
{
	stopwatch(bool run = true)
		:avrlib::stopwatch<base_timer_type>(base_timer)
	{
		if(run)
			return;
		stop();
		clear();
	}
};

struct timeout
	:avrlib::timeout<base_timer_type>
{
	timeout(avrlib::timeout<base_timer_type>::time_type timeout)
		:avrlib::timeout<base_timer_type>(base_timer, timeout)
	{
	}
};

void wait(base_timer_type::time_type time)
{
	avrlib::wait(base_timer, time);
}

template <typename Process>
void wait(base_timer_type::time_type time, Process process)
{
	avrlib::wait(base_timer, time, process);
}

template <typename Process>
void wait(base_timer_type::time_type time, Process process, int)
{
	avrlib::wait(base_timer, time, process, 0);
}

#define  sec(value) (32000000UL*value)
#define msec(value) (32000UL*value)
#define usec(value) (32UL*value)

// main

typedef led0_pin led0;

int main(void)
{
    led0_pin::make_low();
	led1_pin::make_low();
	
	led2n_pin::make_low();
	led2p_pin::make_low();
	
	led0::make_high(); // only alias for led0_pin
	
	pwr_en::make_high();
	
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
	    
	debug.usart().open(debug_usart							, (-3<<12)|131, true);
	bluetooth_port.usart().open(bluetooth_usart				, (-3<<12)|131, true);
	controller_port.usart().open(controller_usart			, (-3<<12)|131, true);
	position_sensor_port.usart().open(position_sensor_usart	, (-3<<12)|131, true);
	driver_port.usart().open(driver_usart					, ( 2<<12)| 12, true);
	rs485.usart().open(rs485_usart							, (-3<<12)|131, true);
	_delay_ms(1);
	
	format_spgm(debug, PSTR("\nYunibeer\n\tRST_STATUS = %x2\n")) % uint8_t(RST_STATUS);
	debug.flush();
	RST_STATUS = 0x3F;
	
	base_timer_type::timer_type::tov_interrupt(true);
	base_timer_type::timer_type::clock_source(avrlib::timer_fosc_1);
	
	PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	sei();
	
	led0::set_low();
	
	timeout second_timeout(sec(1));
	
	char ch = 0;
	
	timeout driver_read_timeout(sec(1));
	driver_read_timeout.cancel();
	
	timeout off_timeout(sec(3));
	off_timeout.cancel();
	
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
					
				case 1:
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
					
				case 2:
					pwr_en::set_low();
					for(;;);
					break;
				
				default:
					bluetooth_port.write(ch);
					break;
			}
		}
		
		if (!bluetooth_port.empty())
		{
			debug.write(bluetooth_port.read());
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
		
		debug.process_tx();
		bluetooth_port.process_tx();
		controller_port.process_tx();
		position_sensor_port.process_tx();
		driver_port.process_tx();
		rs485.process_tx();
	}
}