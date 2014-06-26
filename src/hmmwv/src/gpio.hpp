#ifndef GPIO_HPP
#define GPIO_HPP

#include <cstdlib>
#include <vector>
#include <iostream>
#include <fstream>

#include <stdio.h>
#include <string.h>
#include <unistd.h>

class GPIO
{
public:
	/**
	 * PWM pins.
	 * Assigned int values are path indices in the _pwmPinPaths list.
	 */
	enum PwmPin {
		P8_13 = 0,
		P8_19 = 1,
		P9_14 = 2,
		P9_16 = 3
	};

	/**
	 * Plain GPIOs.
	 * Assigned int values are pin indices in sysfs.
	 */
	enum Pin {
		P8_10 = 68,
		P8_12 = 44,
		P8_15 = 47,
		P8_17 = 27,
		P9_17 = 4,
		P9_18 = 5,
		P9_21 = 3,
		P9_23 = 49,
		P9_24 = 15,
		P9_26 = 14,
		P9_41 = 20,
		P9_42 = 7
	};

	GPIO();
	~GPIO();
	
	/**
	 * @brief Sets a GPIO pin to the specified value.
	 * 
	 * @param pin The GPIO to set.
	 * @param value Desired pin value.
	 */
	void setPin(const Pin pin, const bool value);

	/**
	 * @brief Sets a PWM pin's duty cycle.
	 * 
	 * @param pin The PWM pin to set.
	 * @param duty Duty cycle percentage, 0 < duty < 1.
	 */
	void setPwm(const PwmPin pin, const float duty);
	
private:
	bool containsPin(const Pin pin);
	void exportPin(const Pin pin);
	int echo(const std::string target, const int value);
	int echo(const std::string target, const char *value);
	std::string matchPath(std::string pattern);
	inline std::string append(const std::string base, const std::string suffix)
	{
		std::string tmp(base);
		tmp.append(suffix);
		return tmp;
	}

	const unsigned int PWM_PERIOD;
	std::vector<int> _exportedPins;
	std::vector<std::string> _pwmPinPaths;
};

#endif
