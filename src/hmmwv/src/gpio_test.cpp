#include <iostream>
#include "gpio.hpp"

using namespace std;

int main(int argc, char **argv)
{
	GPIO gpio;
	/*gpio.setPwm(GPIO::P8_13, 1.0);
	cin.get();
	gpio.setPwm(GPIO::P8_13, .5);
	cin.get();
	gpio.setPwm(GPIO::P8_13, 0);
	cin.get();*/

	bool on = true;

	while(true) {
		gpio.setPin(GPIO::P9_18, on);
		cout << on;
		on = !on;
		cin.get();
	}
	return 0;
}
