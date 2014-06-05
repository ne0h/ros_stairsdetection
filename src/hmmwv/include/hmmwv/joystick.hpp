#ifndef JOYSTICK_HPP
#define JOYSTICK_HPP

#include <SDL/SDL.h>
#include <string>
#include <vector>

#include "joystickevent.hpp"

class Joystick {
public:
	Joystick();
	~Joystick();

	std::string getName();
	JoystickEvent getEvent();

private:
	SDL_Joystick *m_joystick;
	std::string m_name;
};

#endif
