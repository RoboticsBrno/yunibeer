#ifndef FLYER_FEEDER_HPP
#define FLYER_FEEDER_HPP

class motor_t
{
public:
	typedef int32_t counter_type;
	motor_t(PORT_t& port, const uint8_t& pins)
		:m_port(port), m_pins_mask(pins),
		 m_speed(c_start_speed), m_step_timeout(m_speed), m_acc_timeout(c_acc_step),
		 m_step_index(0), m_dir(true), m_step_counter(0), m_next(false)
	{
		m_port.OUTCLR = m_pins_mask;
		m_port.DIRSET = m_pins_mask;
		m_step_timeout.cancel();
		m_acc_timeout.cancel();
		for(uint8_t i = 0, j = 1; j != 0 && i != c_phases; j <<= 1)
			if((m_pins_mask & j) != 0)
				m_phase_table[i++] = j;
		uint8_t last = m_phase_table[0];
		for(uint8_t i = 1; i != c_phases; ++i)
		{
			uint8_t temp = m_phase_table[i];
			m_phase_table[i] |= last;
			last = temp;
		}
		m_phase_table[0] |= last;
	}
	void process()
	{
		if(m_step_timeout || m_next)
		{
			m_step_timeout.ack();
			m_step_timeout.set_timeout(m_speed);
			m_port.OUT = (m_port.OUT & ~m_pins_mask) | m_phase_table[m_step_index];
			if(m_dir)
			{
				if(++m_step_index == c_phases)
					m_step_index = 0;
				++m_step_counter;
			}
			else
			{
				if(--m_step_index == -1)
					m_step_index = c_phases - 1;
				--m_step_counter;
			}
			m_next = false;
		}
		if(m_acc_timeout)
		{
			m_acc_timeout.ack();
			if(m_speed > c_max_speed)
				m_speed -= c_steepd_step;
			else
				m_acc_timeout.cancel();
		}
	}
	void start()
	{
		m_speed = c_start_speed;
		m_step_timeout.set_timeout(m_speed);
		m_step_timeout.restart();
		m_acc_timeout.restart();
		m_port.OUTSET = m_pins_mask;
	}
	void stop()
	{
		m_step_timeout.cancel();
		m_acc_timeout.cancel();
		m_port.OUTCLR = m_pins_mask;
	}
	void reset()
	{
		m_step_counter = 0;
	}
	int32_t steps()
	{
		return m_step_counter;
	}
	void emergency_break()
	{
		m_step_timeout.cancel();
		m_acc_timeout.cancel();
		m_port.OUTCLR = m_pins_mask;
	}
	bool running()
	{
		return m_step_timeout.running() && !m_acc_timeout.running();
	}
	void next()
	{
		m_step_timeout.force();
		m_next = true;
	}
	
private:
	static const uint8_t c_phases = 4;

	PORT_t& m_port;
	const uint8_t m_pins_mask;
	uint8_t m_phase_table[c_phases];
	
	timeout::time_type m_speed;
	timeout m_step_timeout;
	timeout m_acc_timeout;
	int8_t m_step_index;
	bool m_dir;
	counter_type m_step_counter;
	bool m_next;
	
	static const timeout::time_type c_start_speed = usec(5000);
	static const timeout::time_type c_max_speed = usec(3700);
	static const timeout::time_type c_steepd_step = usec(50);
	static const timeout::time_type c_acc_step = msec(25);
	
};

template<typename pin>
struct magnet_t
{
	magnet_t()
	{
		pin::make_low();
	}
	void on()
	{
		pin::set_high();
	}
	void off()
	{
		pin::set_low();
	}
	bool state()
	{
		return pin::read();
	}
	void emergency_break()
	{
		pin::set_low();
	}
};

template<typename led_pin>
class optosensor_t
{
public:
	optosensor_t(uint8_t * const threshold_ptr, const uint8_t& adc_channel)
		:m_threshold_ptr(threshold_ptr), m_threshold(*m_threshold_ptr & 0x3F), m_ac(adc_channel < 8 ? ACA : ACB)
	{
		m_ac.CTRLB = get_threshold();
		if(adc_channel < 8)
		{
			*(&PORTA.PIN0CTRL + adc_channel) = PORT_OPC_PULLUP_gc | PORT_ISC_INPUT_DISABLE_gc;
			m_ac.AC0MUXCTRL = (adc_channel << AC_MUXPOS_gp) | AC_MUXNEG_SCALER_gc;
		}
		else
		{
			*(&PORTB.PIN0CTRL + adc_channel - 8) = PORT_OPC_PULLUP_gc | PORT_ISC_INPUT_DISABLE_gc;
			m_ac.AC0MUXCTRL = ((adc_channel - 8) << AC_MUXPOS_gp) | AC_MUXNEG_SCALER_gc;
		}
		m_ac.AC0CTRL = AC_HYSMODE_SMALL_gc | AC_ENABLE_bm;
		led_pin::make_high();
	}
	void increase_threshold()
	{
		if(m_threshold < 63)
			++m_threshold;
		m_ac.CTRLB = m_threshold;
	}
	void decrease_threshold()
	{
		if(m_threshold > 0)
			--m_threshold;
		m_ac.CTRLB = m_threshold;
	}
	uint8_t get_threshold() const
	{
		return m_threshold;
	}
	void save_threshold()
	{
		*m_threshold_ptr = m_threshold;
		flush_eeprom_page_buffer(m_threshold_ptr);
	}
	void led_on()
	{
		led_pin::set_high();
	}
	void led_off()
	{
		led_pin::set_low();
	}
	operator bool()
	{
		return (m_ac.STATUS & AC_AC0STATE_bm) == 0;
	}
private:
	uint8_t * const m_threshold_ptr;
	uint8_t m_threshold;
	AC_t& m_ac;
};

template<typename magnet_pin, typename optosensor_led_pin>
class feeder_t
{
	enum state_t { IDLE, START, START_MOTOR, START_MAGNET, PAUSE_MAGNET, STOP_MAGNET, CHECK_SUCCESS, STOP_MOTOR, WAIT_FOR_NEXT, STOPPED };
public:
	feeder_t(const motor_t& motor, const magnet_t<magnet_pin>& magnet, const optosensor_t<optosensor_led_pin>& paper_sensor)
		:m_motor(motor), m_magnet(magnet), m_paper_sensor(paper_sensor), m_state(IDLE), m_count(0),
		 m_magnet_pause_counter(0), m_magnet_pause_timeout(c_magnet_pause_time), m_error_counter(0), m_automatic(false)
	{
		m_magnet_pause_timeout.cancel();
	}
	void process()
	{
		m_motor.process();
		switch(m_state)
		{
			case IDLE:
				break;
			
			case START:
				m_motor.start();
				m_state = START_MOTOR;
				break;
				
			case START_MOTOR:
				if(m_motor.running())
					m_state = START_MAGNET;
				break;
			
			case START_MAGNET:
				m_magnet_pause_counter = c_magnet_on;
				m_motor.reset();
				m_magnet.on();
				m_state = PAUSE_MAGNET;
				break;
				
			case PAUSE_MAGNET:
				if(m_motor.steps() > m_magnet_pause_counter)
				{
					m_magnet.off();
					m_magnet_pause_timeout.restart();
					m_state = STOP_MAGNET;
				}
				break;
				
			case STOP_MAGNET:
				if(m_magnet_pause_timeout)
				{
					m_magnet_pause_timeout.cancel();
					if(m_magnet_pause_counter < c_magnet_off)
					{
						m_magnet.on();
						m_magnet_pause_counter += c_magnet_pause;
						m_state = PAUSE_MAGNET;
					}
					else
					{
						m_state = CHECK_SUCCESS;
					}
					
				}
				break;
				
			case CHECK_SUCCESS:
				if(m_motor.steps() < c_check_success)
					break;
				if(m_paper_sensor)
				{
					m_state = STOP_MOTOR;
				}
				else if(++m_error_counter < c_errors_limit)
				{
					m_state = START_MAGNET;
				}
				else
				{
					m_motor.stop();
					m_state = IDLE;
					m_automatic = false;
					m_count = 0;
					//led.blink(msec(200));
					send(debug, "Out of paper!\n");
				}
				break;
				
			case STOP_MOTOR:
				if(m_motor.steps() < c_motor_on)
					break;
				if(m_automatic)
				{
					m_motor.stop();
					m_state = WAIT_FOR_NEXT;
					++m_count;
				}
				else
				{
					if(--m_count == 0)
					{
						m_motor.stop();
						m_state = IDLE;
					}
					else
					{
						m_state = START_MAGNET;
					}
				}
				break;
				
			case WAIT_FOR_NEXT:
				if(!m_paper_sensor)
					m_state = START;
				break;
				
			case STOPPED:
				break;
				
		}
	}
	void feed()
	{
		if(m_automatic)
			automatic(false);
		if(m_state != STOPPED)
			++m_count;
		if(m_state == IDLE)
			m_state = START;
		m_automatic = false;
	}
	uint8_t automatic(const bool& enable = true)
	{
		uint8_t cnt = 0;
		if(!enable)
		{
			while(m_state != WAIT_FOR_NEXT)
			{
				process();
				cnt = m_count;
				if(m_error_counter == c_errors_limit || m_state == STOPPED || m_state == IDLE)
				{
					m_count = 0;
					return cnt;
				}
			}
			m_automatic = false;
			m_state = IDLE;
			m_count = 0;
			return cnt;
		}
		if(m_automatic)
			return m_count;
		while(m_state != IDLE && m_state != WAIT_FOR_NEXT)
		{
			process();
			cnt = m_count;
			if(m_error_counter == c_errors_limit || m_state == STOPPED)
			{
				m_count = 0;
				return cnt;
			}
		}
		m_automatic = true;
		m_count = 0;
		if(m_state != WAIT_FOR_NEXT)
		{
			if(m_paper_sensor)
				m_state = WAIT_FOR_NEXT;
			else
				m_state = START;
		}
		return 0;
	}
	void emergency_break()
	{
		m_motor.emergency_break();
		m_magnet.emergency_break();
		m_count = 0;
		m_state = STOPPED;
	}
	void reset()
	{
		m_state = IDLE;
		m_error_counter = 0;
		m_automatic = false;
// 		if(led.blinking())
// 			led.off();
	}
	void clear()
	{
		if(m_count > 1)
			m_count = 1;
	}
	bool is_automatic() const
	{
		return m_automatic;
	}
	
	optosensor_t<optosensor_led_pin>& paper_sensor() { return m_paper_sensor; }
	
private:
	motor_t m_motor;
	magnet_t<magnet_pin> m_magnet;
	optosensor_t<optosensor_led_pin> m_paper_sensor;
	state_t m_state;
	uint8_t m_count;
	motor_t::counter_type m_magnet_pause_counter;
	timeout m_magnet_pause_timeout;
	uint8_t m_error_counter;
	bool m_automatic;
	
	static const motor_t::counter_type c_magnet_on = 48*24/6;
	static const motor_t::counter_type c_magnet_pause = 48*24/60;
	static const motor_t::counter_type c_magnet_off = c_magnet_on + c_magnet_pause * 1;
	static const timeout::time_type c_magnet_pause_time = msec(20);
	static const motor_t::counter_type c_motor_on = 48*24*2;
	static const motor_t::counter_type c_check_success = c_motor_on / 4;
	static const uint8_t c_errors_limit = 3;
};

class button_t
{
public:
	button_t(const uint8_t& mask)
		:m_mask(mask), m_old(0), m_changed(false), m_clicked(false), m_filter_timeout(msec(500))
	{
		m_filter_timeout.cancel();
	}
	void process(uint8_t value)
	{
		value &= m_mask;
		if(value != m_old)
			m_filter_timeout.restart();
		if(m_filter_timeout)
		{
			m_changed = true;
			if(value)
				m_clicked = true;
			m_filter_timeout.cancel();
		}
		m_old = value;
	}
	bool clicked()
	{
		if(!m_clicked)
			return false;
		m_clicked = false;
		return true;
	}
	bool changed()
	{
		if(!m_changed)
			return false;
		m_changed = false;
		return true;
	}
	operator bool() const
	{
		return m_old != 0;
	}
	
private:
	const uint8_t m_mask;
	uint8_t m_old;
	bool m_changed;
	bool m_clicked;
	timeout m_filter_timeout;
};

#endif