#ifndef ENGINE_HPP
#define ENGINE_HPP

#include "gpio.hpp"

class Engine
{
public:
	Engine(GPIO *gpio, const GPIO::Pin enablePin, const GPIO::Pin directionPin,
		const GPIO::PwmPin speedPin);
	~Engine();

	/**
	* @brief here be docs
	*/
	void start(const int direction, const float speed = 0.0f);

private:
	GPIO *_gpio;
	const GPIO::Pin _enablePin;
	const GPIO::Pin _directionPin;
	const GPIO::PwmPin _speedPin;
	int _lastDirection;
};

#endif
