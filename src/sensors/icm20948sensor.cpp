#include "sensor.h"
#include "network/network.h"
#include <i2cscan.h>
#include "Icm20948sensor.h"
#include "calibration.h"

// seconds after previous save (from start) when calibration (DMP Bias) data will be saved to NVS. Increments through the list then stops; to prevent unwelcome eeprom wear.
int bias_save_periods[] = {120, 180, 300, 600, 600}; // 2min + 3min + 5min + 10min + 10min (no more saves after 30min)

#ifndef USE_6_AXIS
#define USE_6_AXIS true
#endif

#ifndef ENABLE_TAP
#define ENABLE_TAP false
#endif

void ICM20948Sensor::save_bias(bool repeat)
{
#if ESP32 && defined(SAVE_BIAS)
    if (SAVE_BIAS)
    {
        int bias_a[3], bias_g[3], bias_m[3];

        icm20948.getGyroBias(bias_g);
        icm20948.getMagBias(bias_m);
        icm20948.getAccelBias(bias_a);

        bool accel_set = bias_a[0] && bias_a[1] && bias_a[2];
        bool gyro_set = bias_g[0] && bias_g[1] && bias_g[2];
        bool mag_set = bias_m[0] && bias_m[1] && bias_m[2];

#ifdef FULL_DEBUG
        Serial.println("bias gyro result:");
        Serial.println(bias_g[0]);
        Serial.println(bias_g[1]);
        Serial.println(bias_g[2]);
        Serial.println("end gyro");

        Serial.println("bias accel result:");
        Serial.println(bias_a[0]);
        Serial.println(bias_a[1]);
        Serial.println(bias_a[2]);
        Serial.println("end accel");

        Serial.println("bias mag result:");
        Serial.println(bias_m[0]);
        Serial.println(bias_m[1]);
        Serial.println(bias_m[2]);
        Serial.println("end mag");
#endif

        if (accel_set)
        {
            // Save accel
            prefs.putInt(auxiliary ? "ba01" : "ba00", bias_a[0]);
            prefs.putInt(auxiliary ? "ba11" : "ba10", bias_a[1]);
            prefs.putInt(auxiliary ? "ba21" : "ba20", bias_a[2]);

#ifdef FULL_DEBUG
            Serial.println("Wrote Accel Bias");
#endif
        }

        if (gyro_set)
        {
            // Save gyro
            prefs.putInt(auxiliary ? "bg01" : "bg00", bias_g[0]);
            prefs.putInt(auxiliary ? "bg11" : "bg10", bias_g[1]);
            prefs.putInt(auxiliary ? "bg21" : "bg20", bias_g[2]);

#ifdef FULL_DEBUG
            Serial.println("Wrote Gyro Bias");
#endif
        }

        if (mag_set)
        {
            // Save mag
            prefs.putInt(auxiliary ? "bm01" : "bm00", bias_m[0]);
            prefs.putInt(auxiliary ? "bm11" : "bm10", bias_m[1]);
            prefs.putInt(auxiliary ? "bm21" : "bm20", bias_m[2]);

#ifdef FULL_DEBUG
            Serial.println("Wrote Mag Bias");
#endif
        }
    }

    if (repeat)
    {
        bias_save_counter++;
        // Possible: Could make it repeat the final timer value if any of the biases are still 0. Save strategy could be improved.
        if (sizeof(bias_save_periods) != bias_save_counter)
        {
            timer.in(
                bias_save_periods[bias_save_counter] * 1000, [](void *arg) -> bool
                { ((ICM20948Sensor*)arg)->save_bias(true); return false; },
                this);
        }
    }

#endif // ifdef ESP32
}

void ICM20948Sensor::setupSensor(uint8_t sensorId, uint8_t addr)
{
    this->addr = addr;
    this->auxiliary = auxiliary;
    this->sensorOffset = {Quat(Vector3(0, 0, 1), ((int)auxiliary) == 0 ? IMU_ROTATION : IMU_ROTATION)};
    this->tapDetector = TapDetector(3, []() {}); // Tripple tap
    this->sensorId = sensorId;
    this->working = true;

    icmSettings = {
        .sensorId = sensorId,
        .i2c_speed = I2C_SPEED,             // i2c clock speed
        .i2c_address = 0x69,                // i2c address (0x69 / 0x68)
        .is_SPI = false,                    // Enable SPI, if disable use i2c
        .cs_pin = 10,                       // SPI chip select pin
        .spi_speed = 7000000,               // SPI clock speed in Hz, max speed is 7MHz
        .mode = 1,                          // 0 = low power mode, 1 = high performance mode
        .enable_gyroscope = true,           // Enables gyroscope output
        .enable_accelerometer = true,       // Enables accelerometer output
        .enable_magnetometer = true,        // Enables magnetometer output // Enables quaternion output
        .enable_gravity = true,            // Enables gravity vector output
        .enable_linearAcceleration = true, // Enables linear acceleration output
        .enable_quaternion6 = USE_6_AXIS,   // Enables quaternion 6DOF output
        .enable_quaternion9 = !USE_6_AXIS,  // Enables quaternion 9DOF output
        .enable_har = false,                // Enables activity recognition
        .enable_steps = false,              // Enables step counter
        .enable_step_detector = false,      // Probably not working
        .gyroscope_frequency = 100,           // Max frequency = 225, min frequency = 1
        .accelerometer_frequency = 100,     // Max frequency = 225, min frequency = 1
        .magnetometer_frequency = 50,        // Max frequency = 70, min frequency = 1
        .gravity_frequency = 100,             // Max frequency = 225, min frequency = 1
        .linearAcceleration_frequency = 100,  // Max frequency = 225, min frequency = 1
        .quaternion6_frequency = 100,       // Max frequency = 225, min frequency = 50
        .quaternion9_frequency = 100,       // Max frequency = 225, min frequency = 50
        .har_frequency = 50,                // Max frequency = 225, min frequency = 50
        .steps_frequency = 50,              // Max frequency = 225, min frequency = 50
        .step_detector_frequency = 100,     // Max frequency = 225, min frequency = 50
    };
    Serial.println("Initialising an IMC20948");
}

void ICM20948Sensor::motionSetup()
{

    working = false;
    if (Connected)
    {
        int rc = imu.init(icmSettings);
        if (rc == 0)
        {

            lastData = millis();
            working = true;
            ICM_init = true;
        }
        else
        { // Some error on init
            ICM_init = false;
            Serial.println("ICM init error");
        }
    }
    else
    {
        Serial.println("No ICM found");
    }
}

void ICM20948Sensor::motionLoop()
{
    if (Connected && ICM_init)
    {

        int r = imu.task();

        if (r == 0)
        {
            if (imu.quat6DataIsReady())
            {
                imu.readQuat6Data(&quaternion.w, &quaternion.x, &quaternion.y, &quaternion.z);
                quaternion *= sensorOffset; // May prefer to use icm20948 mount matrix option?
                newData = true;
                lastData = millis();
            }
            else if (imu.quat9DataIsReady())
            {
                imu.readQuat9Data(&quaternion.w, &quaternion.x, &quaternion.y, &quaternion.z);
                quaternion *= sensorOffset; // May prefer to use icm20948 mount matrix option?
                newData = true;
                lastData = millis();
            }

            if (imu.accelDataIsReady())
            {
                float x, y, z;
                imu.readAccelData(&x, &y, &z);

                newTap |= tapDetector.update(z);
            }
        }
        else
        { // Some err reported
            Serial.print("imu.task() returnd error : ");
            Serial.println(r);
        }
    }
    else
    {
        Serial.println("Not Found or initialised");
    }

    if (lastData + 1000 < millis())
    {
        working = false;
        lastData = millis();
        Serial.print("[ERR] Sensor timeout ");
        Serial.println(addr);
    }
}

void ICM20948Sensor::sendData()
{
    if (newData)
    {
        newData = false;
        Network::sendRotationData(&quaternion, DATA_TYPE_NORMAL, 0, auxiliary);
#ifdef FULL_DEBUG
        // Serial.print("[DBG] Quaternion: ");
        // Serial.print(quaternion.x);
        // Serial.print(",");
        // Serial.print(quaternion.y);
        // Serial.print(",");
        // Serial.print(quaternion.z);
        // Serial.print(",");
        // Serial.println(quaternion.w);
#endif
    }

    if (newTap)
    {
        DataTransfer::sendByte(PACKET_TAP);
        newTap = false;
    }
}

void ICM20948Sensor::startCalibration(int calibrationType)
{
// 20948 does continuous calibration and no need to use for ESP32 as it auto saves bias values

// If ESP32, manually force a new save
#ifdef ESP32
    save_bias(false);
#endif
    // TODO: If 8266, save the current bias values to eeprom
    //#ifdef 8266
    // Types are int, device config saves float - need to save and load like mpu6050 does
    //#endif
}
