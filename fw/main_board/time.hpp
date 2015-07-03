#ifndef TIME_HPP
#define TIME_HPP

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

#endif