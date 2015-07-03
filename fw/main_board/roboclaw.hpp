#ifndef ROBOCLAW_HPP

template <class Usart>
class roboclaw_t
{
public:
	typedef int16_t power_type;
	
	enum pwm_resolution_t :uint8_t
	{
		pwm_8bit = 0,
		pwm_9bit = 1,
		pwm_10bit = 2,
		pwm_11bit = 3,
		pwm_12bit = 4,
		pwm_13bit = 5,
		pwm_14bit = 6
	};

	roboclaw_t(Usart& usart, const uint8_t& address)
		:m_usart(usart), m_address(address)
	{}
		
	void set_power(const power_type& l, const power_type& r)
	{
		write_cmd(34, l, r);
	}
	
	void set_left_power(const power_type& power)
	{
		write_cmd(32, power);
	}
	
	void set_right_power(const power_type& power)
	{
		write_cmd(33, power);
	}
	
	void set_pwm_resolution(const pwm_resolution_t& res)
	{
		write_cmd(48, res);
	}
	
private:
	void write_cmd(const uint8_t& cmd, const uint8_t& value)
	{
		m_usart.write(m_address);
		m_usart.write(cmd);
		m_usart.write(value);
		m_usart.write((m_address + cmd + value) & 0x7F);
	}
	
	void write_cmd(const uint8_t& cmd, const int16_t& value)
	{
		uint8_t l = value & 0xFF;
		uint8_t h = value >> 8;
		m_usart.write(m_address);
		m_usart.write(cmd);
		m_usart.write(h);
		m_usart.write(l);
		m_usart.write((m_address + cmd + h + l) & 0x7F);
	}
	
	void write_cmd(const uint8_t& cmd, const int16_t& value1, const int16_t& value2)
	{
		uint8_t l1 = value1 & 0xFF;
		uint8_t h1 = value1 >> 8;
		uint8_t l2 = value2 & 0xFF;
		uint8_t h2 = value2 >> 8;
		m_usart.write(m_address);
		m_usart.write(cmd);
		m_usart.write(h1);
		m_usart.write(l1);
		m_usart.write(h2);
		m_usart.write(l2);
		m_usart.write((m_address + cmd + h1 + l1 + h2 + l2) & 0x7F);
	}
	
	void write_sign(int8_t value, const uint8_t& pos_cmd, const uint8_t& neg_cmd)
	{
		if(value >= 0)
		{
			write_cmd(pos_cmd, value);
		}
		else
		{
			if(value == -128)
				++value;
			write_cmd(neg_cmd, -value);
		}
	}

	Usart& m_usart;
	const uint8_t m_address;
};

#endif