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
	enum PwmPin {
		P8_13 = 0, // int values are path indices in _pwmPinPaths
		P8_19 = 1,
		P9_14 = 2,
		P9_16 = 3
	};

	enum Pin {
		P8_10 = 68, // int values are pin indices in sysfs
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
	
	void setPin(const Pin pin, const bool value);
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
