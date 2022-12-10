#include "sensor.h"
#include <Arduino-ICM20948.h>
#include "../lib/ICM20948/TapDetector/TapDetector.h"

class ICM20948Sensor : public Sensor
{
public:
    ICM20948Sensor() = default;
    ~ICM20948Sensor() override = default;

    void motionSetup() override final;
    void setupSensor(uint8_t sensorId, uint8_t addr)  override final;


    void motionLoop() override final;
    void sendData() override final;
    void startCalibration(int calibrationType) override final;

    void save_bias(bool repeat);

private:
    bool auxiliary{false};
    unsigned long lastData = 0;
    uint8_t addr = 0x69;
    int bias_save_counter = 0;
    uint8_t ICM_address;
    bool ICM_found = false;
    bool ICM_init = false;
    bool newData = false;
    bool newTap;
    ArduinoICM20948 imu;
    ArduinoICM20948Settings icmSettings;
#ifdef ESP32
    Preferences prefs;
    Timer<> timer = timer_create_default();
#endif
    TapDetector tapDetector;
};