/*
 * Copyright (c) 2018-2020 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <stdint.h>

#include <string>
#include <vector>

#include <frc/ErrorBase.h>
#include <frc/SpeedController.h>
#include <hal/Types.h>

#include "rev/CANError.h"
#include "rev/ControlType.h"

namespace rev {

class CANSparkMaxLowLevel : public frc::ErrorBase, public frc::SpeedController {
    friend class CANPIDController;
    friend class CANDigitalInput;
    friend class CANEncoder;
    friend class CANAnalog;

public:

    static const uint8_t kAPIMajorVersion;
    static const uint8_t kAPIMinorVersion;
    static const uint8_t kAPIBuildVersion;
    static const uint32_t kAPIVersion;

    enum class MotorType { kBrushed = 0, kBrushless = 1 };

    enum class ParameterStatus {
        kOK = 0,
        kInvalidID = 1,
        kMismatchType = 2,
        kAccessMode = 3,
        kInvalid = 4,
        kNotImplementedDeprecated = 5,
    };

    enum class PeriodicFrame { kStatus0 = 0, kStatus1 = 1, kStatus2 = 2 };

    struct PeriodicStatus0 {
        double appliedOutput;
        uint16_t faults;
        uint16_t stickyFaults;
        MotorType motorType;
        bool isFollower;
        uint8_t lock;
        uint8_t roboRIO;
        uint8_t isInverted;
        uint64_t timestamp;
    };

    struct PeriodicStatus1 {
        double sensorVelocity;
        uint8_t motorTemperature;
        double busVoltage;
        double outputCurrent;
        uint64_t timestamp;
    };

    struct PeriodicStatus2 {
        double sensorPosition;
        double iAccum;
        uint64_t timestamp;
    };

    enum class TelemetryID{
        kBusVoltage = 0,
        kOutputCurrent,
        kVelocity,
        kPosition,
        kIAccum,
        kAppliedOutput,
        kMotorTemp,
        kFaults,
        kStickyFaults,
        kAnalogVoltage,
        kAnalogPosition, 
        kAnalogVelocity,
        kAltEncPosition,
        kAltEncVelocity,
        kTotalStreams
    };

    struct TelemetryMessage {
        TelemetryID id;
        float value = 0;
        uint64_t timestamp = 0;
        const char* name;
        const char* units;
        float lowerBnd;
        float upperBnd;
    };

    /**
     * Create a new SPARK MAX Controller
     *
     * @param deviceID The device ID.
     * @param type     The motor type connected to the controller. Brushless
     *                 motors must be connected to their matching color and the
     *                 hall sensor plugged in. Brushed motors must be connected
     *                 to the Red and Black terminals only.
     */
    explicit CANSparkMaxLowLevel(int deviceID, MotorType type);
    
    /**
     * Closes the SPARK MAX Controller
     */
    ~CANSparkMaxLowLevel();

    /**
     * Get the firmware version of the SPARK MAX.
     *
     * @return uint32_t Firmware version integer. Value is represented as 4
     * bytes, Major.Minor.Build H.Build L
     *
     */
    uint32_t GetFirmwareVersion();

    uint32_t GetFirmwareVersion(bool& isDebugBuild);

    /**
     * Get the firmware version of the SPARK MAX as a string.
     *
     * @return std::string Human readable firmware version string
     *
     */
    std::string GetFirmwareString();

    /**
     * Get the unique serial number of the SPARK MAX. Currently not implemented.
     *
     * @return std::vector<uint8_t> Vector of bytes representig the unique
     * serial number
     *
     */
    std::vector<uint8_t> GetSerialNumber();

    /**
     * Get the configured Device ID of the SPARK MAX.
     *
     * @return int device ID
     *
     */
    int GetDeviceId() const;

	/**
	 * Get the motor type setting from when the SparkMax was created.
	 * 
	 * This does not use the Get Parameter API which means it does not read 
	 * what motor type is stored on the SparkMax itself. Instead, it reads
	 * the stored motor type from when the SparkMax object was first created.
	 * 
	 * @return MotorType Motor type setting
	 */
    MotorType GetInitialMotorType();

    /**
     * Set the motor type connected to the SPARK MAX.
     *
     * This uses the Set Parameter API and should be used infrequently.
     * The parameter does not presist unless burnFlash() is called.
     * The recommended method to configure this parameter is to use the
     * SPARK MAX GUI to tune and save parameters.
     *
     * @param type The type of motor connected to the controller. Brushless
     * motors must be connected to their matching color and the hall sensor
     * plugged in. Brushed motors must be connected to the Red and Black
     * terminals only.
     *
     * @return CANError Set to CANError::kOk if successful
     *
     */
    CANError SetMotorType(MotorType type);

    /**
     * Get the motor type setting for the SPARK MAX.
     *
     * This uses the Get Parameter API and should be used infrequently. This
     * function uses a non-blocking call and will return a cached value if the
     * parameter is not returned by the timeout. The timeout can be changed by
     * calling SetCANTimeout(int milliseconds)
     *
     * @return MotorType Motor type setting
     *
     */
    MotorType GetMotorType();

    /**
     * Set the rate of transmission for periodic frames from the SPARK MAX
     *
     * Each motor controller sends back three status frames with different
     * data at set rates. Use this function to change the default rates.
     *
     * Defaults:
     * Status0 - 10ms
     * Status1 - 20ms
     * Status2 - 50ms
     *
     * This value is not stored in the FLASH after calling burnFlash()
     * and is reset on powerup.
     *
     * Refer to the SPARK MAX reference manual on details for how and when
     * to configure this parameter.
     *
     * @param frameID   The frame ID can be one of PeriodicFrame type
     * @param periodMs The rate the controller sends the frame to the
     * controller.
     *
     * @return CANError Set to CANError::kOk if successful
     *
     */
    CANError SetPeriodicFramePeriod(PeriodicFrame frame, int periodMs);


	/**
	 * Set the control frame send period for the native CAN Send thread. To
     * disable periodic sends, set periodMs to 0.
     * 
	 * @param periodMs The send period in milliseconds between 1ms and 100ms
     * or set to 0 to disable periodic sends. Note this is not updated until 
	 * the next call to Set() or SetReference().
     * 
	 */
    void SetControlFramePeriodMs(int periodMs);

    /**
     * Restore motor controller parameters to factory default
     *
     * @param persist If true, burn the flash with the factory default
     * parameters
     *
     * @return CANError Set to CANError::kOk if successful
     */
    CANError RestoreFactoryDefaults(bool persist = false);

    /**
     * Allow external controllers to recieve control commands over USB. 
     * For example, a configuration where the heartbeat (and enable/disable)
     * is sent by the main controller, but control frames are sent by
     * other CAN devices over USB.
     * 
     * This is global for all controllers on the same bus.
     * 
     * This does not disable sending control frames from this device. To prevent
     * conflicts, do not enable this feature and also send Set() for SetReference()
     * from the controllers you wish to control.
     *
     * @param enable Enable or disable external control
     *
     */
    static void EnableExternalUSBControl(bool enable);

#ifndef __FRC_ROBORIO__
    /**
     * Send enabled or disabled command to controllers. This is global for all 
     * controllers on the same bus, and will only work for non-roboRIO targets
     * in non-competiton use. This function will also not work if a roboRIO is
     * present on the CAN bus.
     * 
     * This does not disable sending control frames from this device. To prevent
     * conflicts, do not enable this feature and also send Set() for SetReference()
     * from the controllers you wish to control.
     *
     * @param enable Enable or disable external control
     *
     */
    static void SetEnable(bool enable);
#endif

protected:
    CANError SetEncPosition(double value);
    CANError SetIAccum(double value);

    struct FollowConfig {
        uint32_t leaderArbId;
        union {
            struct {
                uint32_t rsvd1 : 18;
                uint32_t invert : 1;
                uint32_t rsvd2 : 5;
                uint32_t predefined : 8;
            } config;
            uint32_t configRaw;
        };
    };

    void* m_sparkMax;

    PeriodicStatus0 GetPeriodicStatus0();
    PeriodicStatus1 GetPeriodicStatus1();
    PeriodicStatus2 GetPeriodicStatus2();

    CANError SetFollow(FollowConfig config);

    CANError SetpointCommand(double value,
                             ControlType ctrl = ControlType::kDutyCycle,
                             int pidSlot = 0, double arbFeedforward = 0, int arbFFUnits = 0);
    
    float GetSafeFloat(float f);

    MotorType m_motorType;

private:
    const int m_deviceID;
};

}  // namespace rev
