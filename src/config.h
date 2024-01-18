#define EEPROM_SIZE 512
#define CALIBRATION_X_LOW_ADDRESS 10
#define CALIBRATION_X_HIGH_ADDRESS 14
#define CALIBRATION_Y_LOW_ADDRESS 18
#define CALIBRATION_Y_HIGH_ADDRESS 24


#define SAMPLES 7



#define JOYSTICK_WAKE_THRESHOLD 3

#define JOYSTICK_ERROR_PERCENT 2


unsigned long lastSysWorkerTick = 0;
unsigned long sysWorkerTickInterval = 1000;
unsigned long scnTickInterval = 10;