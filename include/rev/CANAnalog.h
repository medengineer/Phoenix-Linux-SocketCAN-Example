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

#include "rev/CANError.h"
#include "rev/CANSensor.h"

#include <stdint.h>

namespace rev {

class CANSparkMax;

class CANAnalog : public CANSensor {
public:
    /**
     * Analog sensors have the ability to either be absolute or relative.
     * By default, GetAnalog() will return an absolute analog sensor, but
     * it can also be configured to be a relative sensor instead. 
     */
    enum class AnalogMode { kAbsolute = 0, kRelative = 1 };

    /**
    * Constucts a CANAnalog.
    * 
    * @param device The Spark Max to which the analog sensor is attached.
    */
    explicit CANAnalog(CANSparkMax& device, AnalogMode mode);

    CANAnalog(CANAnalog&&) = default;
    CANAnalog& operator=(CANAnalog&&) = default;
    
    CANAnalog(const CANAnalog& rhs) = default;

    /**
     * Get the voltage of the analog sensor.
     * 
     * @return Voltage of the sensor 
     */
    double GetVoltage();

    /**
      * Get the position of the motor. Returns value in the native unit 
      * of 'volt' by default, and can be changed by a scale factor 
      * using setPositionConversionFactor().
      * 
      * @return Position of the sensor in volts
      */
    double GetPosition();

    /** 
     * Get the velocity of the motor. Returns value in the native units of
     * 'volts per second' by default, and can be changed by a 
     * scale factor using setVelocityConversionFactor().
     * 
     * @return Velocity of the sensor in volts per second
     */
    double GetVelocity();

    /**
	 * Set the conversion factor for the position of the analog sensor.
     * By default, revolutions per volt is 1. Changing the position conversion
	 * factor will also change the position units.
     * 
     * @param factor The conversion factor which will be multiplied by volts
     * 
     * @return CANError Set to CANError.kOK if successful
     */
    CANError SetPositionConversionFactor(double factor);

    /**
     * Get the current conversion factor for the position of the analog
     * sensor. 
     * 
     * @return Analog position conversion factor
     */
    double GetPositionConversionFactor();

    /**
     * Set the conversion factor for the veolocity of the analog sensor.
     * By default, revolutions per volt second is 1. Changing the velocity 
	 * conversion factor will also change the velocity units.
     * 
     * @param factor The conversion factor which will be multipled by volts per second
     * 
     * @return CANError Set to CANError.kOK is successful
     */
    CANError SetVelocityConversionFactor(double factor);

    /**
     * Get the current conversion factor for the velocity of the analog
     * sensor. 
     * 
     * @return Analog velocity conversion factor
     */
    double GetVelocityConversionFactor();
    
    /**
     * Set the average sampling depth for a quadrature encoder. This value
     * sets the number of samples in the average for velocity readings. This
     * can be any value from 1 to 64.
     * 
     * When the SparkMax controller is in Brushless mode, this 
     * will not change any behavior.
     * 
     * @param depth The average sampling depth between 1 and 64 (default)
     * 
     * @return CANError.kOK if successful
     */
    CANError SetAverageDepth(uint32_t depth);

    /**
     * Set the measurement period for velocity measurements of a quadrature encoder.
     * When the SparkMax controller is in Brushless mode, this will not
     * change any behavior.
     * 
     * The basic formula to calculate velocity is change in positon / change in time.
     * This parameter sets the change in time for measurement.
     * 
     * @param period_us Measurement period in milliseconds. This number may be
     * between 1 and 100 (default).
     * 
     * @return CANError.kOK if successful
     */
    CANError SetMeasurementPeriod(uint32_t period_ms);

    /**
     * Get the averafe sampling depth for a quadrature encoder. 
     * 
     * @return The average sampling depth
     */
    uint32_t GetAverageDepth();

    /**
     * Get the number of samples for reading from a quadrature encoder. This value
     * sets the number of samples in the average for velocity readings.
     * 
     * @return Measurement period in microseconds
     */ 
    uint32_t GetMeasurementPeriod();
    
    /**
     * Set the phase of the CANSensor so that it is set to be in 
     * phase with the motor itself. This only works for quadrature
     * encoders. This will throw an error if the user tries to set
     * inverted while the SparkMax is Brushless and using the hall 
     * effect sensor.
     * 
     * @param inverted The phase of the encoder
     * 
     * @return CANError.kOK if successful
     */
    CANError SetInverted(bool inverted) override;

    /**
     * Get the phase of the CANSensor. This will just return false
     * if the user tries to get inverted while the SparkMax is 
     * Brushless and using the hall effect sensor.
     * 
     * @return The phase of the encoder
     */
    bool GetInverted() const override;

private:

    AnalogMode m_mode;

protected:
    int GetID() const override;
};

}  // namespace rev