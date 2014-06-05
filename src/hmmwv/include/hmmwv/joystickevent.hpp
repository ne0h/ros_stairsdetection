#ifndef JOYSTICKEVENT_HPP
#define JOYSTICKEVENT_HPP

#include <vector>

class JoystickEvent {
public:
	JoystickEvent(std::vector<bool> buttons, std::vector<short> axis);
	~JoystickEvent();

	std::vector<bool>  getButtons();
	std::vector<short> getAxis();

private:
	std::vector<bool>  m_buttons;
	std::vector<short> m_axis;
};

#endif
