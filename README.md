
# lmlm
Hardwares:
- Arduino Uno R4 wifi
- BNO055 breakout board
- DC motor supports control RPM by voltage
- Input signal that captures the active state of LMLM's relay: brew, refill, paddle switch on

Software:
- Tested on Arduino IDE
- Requirements libraries:
	- Adafruit Unified Sensor:
	- ArduinoBLE:
		- Modify BLEDevice class of the library to add the following function:
void BLEDevice::setMaxMtu(int maxMtu)
{
	ATT.setMaxMtu(maxMtu);
}
