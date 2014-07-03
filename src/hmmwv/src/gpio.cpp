/**
 * Steps to do for pwm
 * 1. "echo am33xx_pwm > /sys/devices/bone_capemgr.9/slots"
 * 2. "echo bone_pwm_P8_13 > /sys/devices/bone_capemgr.9/slots"
 * 3. "cd /sys/devices/ocp.3/pwm_test_P8_13.16/"
 * 4. "echo 0 > duty" for max value or "echo 500000 > duty" for min value
 */

#include "gpio.hpp"
#include <cassert>
#include <glob.h>
#include <sstream>

using namespace std;

GPIO::GPIO() :
	PWM_PERIOD(50000) // nanoseconds = 2000 Hz
{
	// Enable the pwm pins
	echo("/sys/devices/bone_capemgr.9/slots", "am33xx_pwm");
	echo("/sys/devices/bone_capemgr.9/slots", "bone_pwm_P8_13");
	echo("/sys/devices/bone_capemgr.9/slots", "bone_pwm_P8_19");
	echo("/sys/devices/bone_capemgr.9/slots", "bone_pwm_P9_14");
	echo("/sys/devices/bone_capemgr.9/slots", "bone_pwm_P9_16");

	// Do not change the order of pins here. The PIN enum is used to access
	// these paths, so their order is important.
	cout << "Finding PWM pin paths:\n";
	string path = matchPath("/sys/devices/ocp.*/pwm_test_P8_13.*/");
	cout << path << endl;
	_pwmPinPaths.push_back(path);
	path = matchPath("/sys/devices/ocp.*/pwm_test_P8_19.*/");
	cout << path << endl;
	_pwmPinPaths.push_back(path);
	path = matchPath("/sys/devices/ocp.*/pwm_test_P9_14.*/");
	cout << path << endl;
	_pwmPinPaths.push_back(path);
	path = matchPath("/sys/devices/ocp.*/pwm_test_P9_16.*/");
	cout << path << endl;
	_pwmPinPaths.push_back(path);

	// Set PWM period for both outputs
	int result = echo(append(_pwmPinPaths[P8_13], "period"), PWM_PERIOD);
	if(result != 0) {
		cout << "At least one echo failed\n";
	}
	result = echo(append(_pwmPinPaths[P8_19], "period"), PWM_PERIOD);
	if(result != 0) {
		cout << "At least one echo failed\n";
	}
	result = echo(append(_pwmPinPaths[P9_14], "period"), PWM_PERIOD);
	if(result != 0) {
		cout << "At least one echo failed\n";
	}
	result = echo(append(_pwmPinPaths[P9_16], "period"), PWM_PERIOD);
	if(result != 0) {
		cout << "At least one echo failed\n";
	}
}

GPIO::~GPIO()
{

}

void GPIO::setPin(const Pin pin, const bool value)
{
	assert(value == 0 || value == 1);

	// already exported?
	if (!containsPin(pin)) {
		exportPin(pin);
	}
	
	stringstream ss;
	ss << "/sys/class/gpio/gpio" << pin << "/value";
	string pinPath = ss.str();
	echo(pinPath, value);
}

/* Duty cycle in percent */
void GPIO::setPwm(const PwmPin pin, const float dutyPerc)
{
	/*
	https://groups.google.com/forum/#!topic/beagleboard/qma8bMph0yM

	This will connect PWM to pin P9_14 and generate on the pin  ~2MHz
	waveform with 50% duty.

	modprobe pwm_test
	echo am33xx_pwm > /sys/devices/bone_capemgr.9/slots
	echo bone_pwm_P9_14 > /sys/devices/bone_capemgr.9/slots
	echo 500 > /sys/devices/ocp.2/pwm_test_P9_14.16/period
	echo 250 > /sys/devices/ocp.2/pwm_test_P9_14.16/duty
	*/

	assert(dutyPerc >= 0.0);
	assert(dutyPerc <= 1.0);

	// The beaglebone interprets duty == period as 0 V output and duty == 0 results in
	// 3.3 V output, so we invert the value here to make 0 % duty correspond to 0 V output.
	const int duty = (1.0 - dutyPerc) * 500000;
	int result = 0;
	result = echo(append(_pwmPinPaths[pin], "duty"), duty);

	if(result != 0) {
		cout << "At least one echo failed\n";
	}
}

bool GPIO::containsPin(const Pin pin)
{
	for (vector<int>::iterator it = _exportedPins.begin();
			it != _exportedPins.end(); it++) {
		if (*it == pin) {
			return true;
		}
	}
	
	return false;
}

void GPIO::exportPin(const Pin pin)
{
	// Construct the base path to the pin
	stringstream ss;
  	ss << "/sys/class/gpio/gpio" << pin << "/";
  	string basePath = ss.str();
  	
  	echo("/sys/class/gpio/export", pin);
  	echo(append(basePath, "direction"), "out");

// WTF?

	/*strcpy(setValue, GPIOString);
	if(fwrite(&setValue, sizeof(char), 2, outputHandle) == -1)
		cerr << "kaputt\n";
	fclose(outputHandle);*/
	
	_exportedPins.push_back(pin);
}

int GPIO::echo(const string target, const int value)
{
	ofstream file(target.c_str());
	if(!file) {
		cerr << "Could not open " << target << endl;
		return -1;
	}

	file << value;
	file.close();
	return 0;
}

int GPIO::echo(const string target, const char *value)
{
	ofstream file(target.c_str());
	if(!file) {
		cerr << "Could not open " << target << endl;
		return -1;
	}

	file << value;
	file.close();
	return 0;
}

string GPIO::matchPath(const std::string pattern)
{

	// Find the pwm pin paths (they change on every reboot...)
	// int glob(const char *pattern, int flags,
	//			int (*errfunc) (const char *epath, int eerrno),
	//			glob_t *pglob);
	glob_t foundPaths;
	int result = glob(pattern.c_str(), GLOB_ERR | GLOB_MARK, NULL, &foundPaths);

	switch(result) {
	case 0:
		if(foundPaths.gl_pathc != 1) {
			cout << "Warning: PWM pin path pattern (tm) matched more than one path. Taking the first one.\n";
		}
		return string(foundPaths.gl_pathv[0]);
	case GLOB_NOSPACE:
		cerr << "Call to glob ran out of memory!\n";
		break;
	case GLOB_ABORTED:
		cerr << "Glob encountered a read error (are we root?)\n";
		break;
	case GLOB_NOMATCH:
		cerr << "Glob couldn't match a path\n";
		break;
	default:
		cerr << "Unexpected glob error!\n";
	}

	return string();
}
