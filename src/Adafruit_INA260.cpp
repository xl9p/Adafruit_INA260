/*!
 *  @file Adafruit_INA260.cpp
 *
 *  @mainpage Adafruit INA260 I2C Current and Power sensor
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for the INA260 I2C Current and Power sensor
 *
 * 	This is a library for the Adafruit INA260 breakout:
 * 	http://www.adafruit.com/products/4226
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  @section dependencies Dependencies
 *
 *  This library depends on the Adafruit BusIO library
 *
 *  @section author Author
 *
 *  Bryan Siepert for Adafruit Industries
 *
 * 	@section license License
 *
 * 	BSD (see license.txt)
 *
 * 	@section  HISTORY
 *
 *     v1.0 - First release
 */



#include "Arduino.h"
#include <Wire.h>

#include "Adafruit_INA260.h"

/*!
 *    @brief  Instantiates a new INA260 class
 */
Adafruit_INA260::Adafruit_INA260(TwoWire *theWire, uint8_t i2cAddress) {
  this->i2cDevice = std::make_unique<Adafruit_I2CDevice>(i2cAddress, theWire);

  this->configReg.emplace(this->i2cDevice.get(), (uint8_t) INA260_REGISTERS::CONFIG, 2, MSBFIRST);
  this->maskEnableReg.emplace(this->i2cDevice.get(), (uint8_t) INA260_REGISTERS::MASK_ENABLE, 2, MSBFIRST);
  this->alertLimitReg.emplace(this->i2cDevice.get(), (uint8_t) INA260_REGISTERS::ALERT_LIMIT, 2, MSBFIRST);

  this->currentReg.emplace(this->i2cDevice.get(), (uint8_t) INA260_REGISTERS::CURRENT, 2, MSBFIRST);
  this->votlageReg.emplace(this->i2cDevice.get(), (uint8_t) INA260_REGISTERS::BUSVOLTAGE, 2, MSBFIRST);
  this->powerReg.emplace(this->i2cDevice.get(), (uint8_t) INA260_REGISTERS::POWER, 2, MSBFIRST);

  this->dieRegister.emplace(this->i2cDevice.get(), (uint8_t) INA260_REGISTERS::DIE_UID, 2, MSBFIRST);
  this->mfgRegister.emplace(this->i2cDevice.get(), (uint8_t) INA260_REGISTERS::MFG_UID, 2, MSBFIRST);
}

/*!
 *    @brief  Sets up the HW
 *    @param  i2c_address The I2C address to be used.
 *    @param  theWire The Wire object to be used for I2C connections.
 *    @return True if initialization was successful, otherwise false.
 */
INA260SensorOperation Adafruit_INA260::begin(const INA260_Config &sensorConfig) {
  if (!this->isInitialized) {
    if (!this->i2cDevice->begin()) {
      return INA260SensorOperation(SENSOR_STATUS_I2C_FAIL);
    }
    

    Adafruit_BusIO_RegisterBits deviceId = Adafruit_BusIO_RegisterBits(&this->dieRegister.value(), 12, (uint8_t)INA260_DIE_UID_OFFSETS::DID);

    uint32_t mfg = this->mfgRegister.value().read();
    uint32_t devId = deviceId.read();

    // make sure we're talking to the right chip
    if ((mfg != (uint16_t)INA260_DEFINITIONS::MANUFACTURER_ID) || (devId != (uint16_t)INA260_DEFINITIONS::DEVICE_ID)) {
      return INA260SensorOperation(SENSOR_STATUS_BAD_ADDR);
    }


    bool configModeRes = Adafruit_BusIO_RegisterBits(&this->configReg.value(), 3U,
                        (uint8_t) INA260_CONFIG_OFFSETS::MODE).write((uint8_t)sensorConfig.mode);
    bool configAvgRes = Adafruit_BusIO_RegisterBits(&this->configReg.value(), 3U,
                        (uint8_t) INA260_CONFIG_OFFSETS::AVG).write((uint8_t)sensorConfig.averagingCount);
    bool configVbusCTRes = Adafruit_BusIO_RegisterBits(&this->configReg.value(), 3U,
                        (uint8_t) INA260_CONFIG_OFFSETS::VBUSCT).write((uint8_t)sensorConfig.voltageConversionTime);
    bool configCCTRes = Adafruit_BusIO_RegisterBits(&this->configReg.value(), 3U,
                        (uint8_t) INA260_CONFIG_OFFSETS::ISHCT).write((uint8_t)sensorConfig.currentConversionTime);
    bool maskEnableLEnableRes = Adafruit_BusIO_RegisterBits(&this->maskEnableReg.value(), 1U,
                        (uint8_t) INA260_MASK_ENABLE_OFFSETS::LEN).write((uint8_t)sensorConfig.alertLatchState);
    bool maskEnableLTypeRes = Adafruit_BusIO_RegisterBits(&this->maskEnableReg.value(), 6U,
                        (uint8_t) INA260_MASK_ENABLE_OFFSETS::CNVR).write((uint8_t)sensorConfig.alertLatchType);
    bool maskEnableLPolarityRes = Adafruit_BusIO_RegisterBits(&this->maskEnableReg.value(), 1U,
                        (uint8_t) INA260_MASK_ENABLE_OFFSETS::APOL).write((uint8_t)sensorConfig.alertLatchPolarity);

    bool maskEnableAlertLimitRes = Adafruit_BusIO_RegisterBits(&this->alertLimitReg.value(), 16U, 0U).write((int16_t)(sensorConfig.alertThreshold / 1.25f));

    if (!(configModeRes && configAvgRes && configVbusCTRes && configCCTRes && maskEnableLEnableRes && maskEnableLTypeRes && maskEnableLPolarityRes && maskEnableAlertLimitRes)) {
      return INA260SensorOperation(SENSOR_STATUS_CONFIG_SET_FAIL);
    }

    this->configuration = sensorConfig;
    this->isInitialized = true;

    switch(this->reset().sensorStatus) {
      case SENSOR_STATUS_OP_OK: {
        return INA260SensorOperation(SENSOR_STATUS_OP_OK);
      }
      default: {
        this->isInitialized = false;
				return INA260SensorOperation(SENSOR_STATUS_I2C_FAIL);
      }
    }
  }
  return INA260SensorOperation(SENSOR_STATUS_ALREADY_INIT);
}


/*!
    @brief Reads and scales the current value of the Current register.
    @return The current current measurement in mA
*/
INA260SensorOperation Adafruit_INA260::readCurrent(void) {
  if (this->isInitialized) {
    uint32_t rawData = this->currentReg.value().read();

    if (rawData == 0xFFFFFFFF) {
      return INA260SensorOperation(SENSOR_STATUS_DATA_BAD);
    }

    return INA260SensorOperation(SENSOR_STATUS_DATA_OK, ((int16_t)rawData * 1.25f));
  }
  return INA260SensorOperation(SENSOR_STATUS_NOT_INIT);
}


/*!
    @brief Reads and scales the current value of the Bus Voltage register.
    @return The current bus voltage measurement in mV
*/
INA260SensorOperation Adafruit_INA260::readVoltage(void) {
  if (this->isInitialized) {
    uint32_t rawData = this->votlageReg.value().read();

    if (rawData == 0xFFFFFFFF) {
      return INA260SensorOperation(SENSOR_STATUS_DATA_BAD);
    }

    return INA260SensorOperation(SENSOR_STATUS_DATA_OK, ((int16_t)rawData * 1.25f));
  }
  return INA260SensorOperation(SENSOR_STATUS_NOT_INIT);
}


/*!
    @brief Reads and scales the current value of the Power register.
    @return The current Power calculation in mW
*/
INA260SensorOperation Adafruit_INA260::readPower(void) {
  if (this->isInitialized) {
    uint32_t rawData = this->powerReg.value().read();

    if (rawData == 0xFFFFFFFF) {
      return INA260SensorOperation(SENSOR_STATUS_DATA_BAD);
    }

    return INA260SensorOperation(SENSOR_STATUS_DATA_OK, ((int16_t)rawData * 10.0f));
  }
  return INA260SensorOperation(SENSOR_STATUS_NOT_INIT);
}


/*!
    @brief Resets the harware. 
    All registers are set to default values, the same as a power-on reset.
*/
INA260SensorOperation Adafruit_INA260::reset(void) {
  if (this->isInitialized) {
    Adafruit_BusIO_RegisterBits reset = Adafruit_BusIO_RegisterBits(&this->configReg.value(), 1U, (uint8_t)INA260_CONFIG_OFFSETS::RESET);
    if (!reset.write(1)) {
      return INA260SensorOperation(SENSOR_STATUS_I2C_FAIL);
    }
    delay(5);
    return INA260SensorOperation(SENSOR_STATUS_OP_OK);
  }
  return INA260SensorOperation(SENSOR_STATUS_NOT_INIT);
}

INA260SensorOperation Adafruit_INA260::end(void) {
  if (this->isInitialized) {
    
    return INA260SensorOperation(SENSOR_STATUS_NOT_IMPLEMENTED);
  }
  return INA260SensorOperation(SENSOR_STATUS_NOT_INIT);
}


/*!
    @brief Sets a new measurement mode
    @param new_mode The new mode to be set
*/
INA260SensorOperation Adafruit_INA260::setMode(INA260_MeasurementMode new_mode) {
  if (this->isInitialized) {
    Adafruit_BusIO_RegisterBits mode = Adafruit_BusIO_RegisterBits(&this->configReg.value(), 3U, (uint8_t)INA260_CONFIG_OFFSETS::MODE);
    if (!mode.write((uint8_t)new_mode)) {
      return INA260SensorOperation(SENSOR_STATUS_I2C_FAIL);
    }
    this->configuration.mode = new_mode;
    return INA260SensorOperation(SENSOR_STATUS_OP_OK);
  }
  return INA260SensorOperation(SENSOR_STATUS_NOT_INIT);
}


/*!
    @brief Sets the number of averaging samples
    @param count The number of samples to be averaged
*/
INA260SensorOperation Adafruit_INA260::setAveragingCount(INA260_AveragingCount count) {
  if (this->isInitialized) {
    Adafruit_BusIO_RegisterBits averaging_count = Adafruit_BusIO_RegisterBits(&this->configReg.value(), 3U, (uint8_t)INA260_CONFIG_OFFSETS::AVG);
    if (!averaging_count.write((uint8_t)count)) {
      return INA260SensorOperation(SENSOR_STATUS_I2C_FAIL);
    }
    this->configuration.averagingCount = count;
    return INA260SensorOperation(SENSOR_STATUS_OP_OK);
  }
  return INA260SensorOperation(SENSOR_STATUS_NOT_INIT);
}


/*!
    @brief Sets the current conversion time
    @param time The new current conversion time
*/
INA260SensorOperation Adafruit_INA260::setCurrentConversionTime(INA260_ConversionTime time) {
  if (this->isInitialized) {
    Adafruit_BusIO_RegisterBits current_conversion_time = Adafruit_BusIO_RegisterBits(&this->configReg.value(), 3U, (uint8_t)INA260_CONFIG_OFFSETS::ISHCT);
    if (!current_conversion_time.write((uint8_t)time)) {
      return INA260SensorOperation(SENSOR_STATUS_I2C_FAIL);
    }
    this->configuration.currentConversionTime = time;
    return INA260SensorOperation(SENSOR_STATUS_OP_OK);
  }
  return INA260SensorOperation(SENSOR_STATUS_NOT_INIT);
}


/*!
    @brief Sets the bus voltage conversion time
    @param time The new bus voltage conversion time
*/
INA260SensorOperation Adafruit_INA260::setVoltageConversionTime(INA260_ConversionTime time) {
  if (this->isInitialized) {
    Adafruit_BusIO_RegisterBits voltage_conversion_time = Adafruit_BusIO_RegisterBits(&this->configReg.value(), 3U, (uint8_t)INA260_CONFIG_OFFSETS::VBUSCT);
    if (!voltage_conversion_time.write((uint8_t)time)) {
      return INA260SensorOperation(SENSOR_STATUS_I2C_FAIL);
    }
    this->configuration.voltageConversionTime = time;
    return INA260SensorOperation(SENSOR_STATUS_OP_OK);
  }
  return INA260SensorOperation(SENSOR_STATUS_NOT_INIT);
}


/*!
    @brief Checks if the most recent one shot measurement has completed
    @return true if the conversion has completed
*/
INA260SensorOperation Adafruit_INA260::conversionReady(void) {
  if (this->isInitialized) {
    Adafruit_BusIO_RegisterBits conversion_ready = Adafruit_BusIO_RegisterBits(&this->maskEnableReg.value(), 1U, (uint8_t)INA260_MASK_ENABLE_OFFSETS::CVRF);
    return INA260SensorOperation(SENSOR_STATUS_OP_OK, (bool)conversion_ready.read());
  }
  return INA260SensorOperation(SENSOR_STATUS_NOT_INIT);
}


/*!
    @brief Sets which parameter asserts the ALERT pin
    @param alert The parameter which asserts the ALERT pin
    @note The method writes 6 different offsets at a time.
*/
INA260SensorOperation Adafruit_INA260::setAlertType(INA260_AlertType alert) {
  if (this->isInitialized) {
    Adafruit_BusIO_RegisterBits alert_type = Adafruit_BusIO_RegisterBits(&this->maskEnableReg.value(), 6U, (uint8_t)INA260_MASK_ENABLE_OFFSETS::CNVR);
    if (!alert_type.write((uint8_t)alert)) {
      return INA260SensorOperation(SENSOR_STATUS_I2C_FAIL);
    }
    this->configuration.alertLatchType = alert;
    return INA260SensorOperation(SENSOR_STATUS_OP_OK);
  }
  return INA260SensorOperation(SENSOR_STATUS_NOT_INIT);
}


/*!
    @brief Sets the Alert Limit
    @param limit The new limit that triggers the alert
*/
INA260SensorOperation Adafruit_INA260::setAlertLimit(float limit) {
  if (this->isInitialized) {
    Adafruit_BusIO_RegisterBits alert_limit = Adafruit_BusIO_RegisterBits(&this->alertLimitReg.value(), 16U, 0U);
    if (!alert_limit.write((int16_t)(limit / 1.25f))) {
      return INA260SensorOperation(SENSOR_STATUS_I2C_FAIL);
    }
    this->configuration.alertThreshold = limit;
    return INA260SensorOperation(SENSOR_STATUS_OP_OK);
  }
  return INA260SensorOperation(SENSOR_STATUS_NOT_INIT);
}


/*!
    @brief Sets Alert Polarity Bit
    @param polarity The polarity of the alert pin
*/
INA260SensorOperation Adafruit_INA260::setAlertPolarity(INA260_AlertPolarity polarity) {
  if (this->isInitialized) {
    Adafruit_BusIO_RegisterBits alert_polarity = Adafruit_BusIO_RegisterBits(&this->maskEnableReg.value(), 1U, (uint8_t)INA260_MASK_ENABLE_OFFSETS::APOL);
    if (!alert_polarity.write((uint8_t)polarity)) {
      return INA260SensorOperation(SENSOR_STATUS_I2C_FAIL);
    }
    this->configuration.alertLatchPolarity = polarity;
    return INA260SensorOperation(SENSOR_STATUS_OP_OK);
  }
  return INA260SensorOperation(SENSOR_STATUS_NOT_INIT);
}


/*!
    @brief Sets Alert Latch Bit
    @param state The parameter which asserts the ALERT pin
*/
INA260SensorOperation Adafruit_INA260::setAlertLatch(INA260_AlertLatch state) {
  if (this->isInitialized) {
    Adafruit_BusIO_RegisterBits alert_latch = Adafruit_BusIO_RegisterBits(&this->maskEnableReg.value(), 1U, (uint8_t)INA260_MASK_ENABLE_OFFSETS::LEN);
    if (!alert_latch.write((uint8_t)state)) {
      return INA260SensorOperation(SENSOR_STATUS_I2C_FAIL);
    }
    this->configuration.alertLatchState = state;
    return INA260SensorOperation(SENSOR_STATUS_OP_OK);
  }
  return INA260SensorOperation(SENSOR_STATUS_NOT_INIT);
}


/*!
    @brief Checks if the Alert Flag is set
    @return true if the flag is set
*/
INA260SensorOperation Adafruit_INA260::alertFunctionFlag(void) {
  if (this->isInitialized) {
    Adafruit_BusIO_RegisterBits alert_function_flag = Adafruit_BusIO_RegisterBits(&this->maskEnableReg.value(), 1U, (uint8_t)INA260_MASK_ENABLE_OFFSETS::AFF);
    return INA260SensorOperation(SENSOR_STATUS_OP_OK, ((bool)alert_function_flag.read()));
  }
  return INA260SensorOperation(SENSOR_STATUS_NOT_INIT);
}


INA260SensorOperation Adafruit_INA260::getConfiguration(bool update) {
  if (update) {
    if (this->isInitialized) {
      // Adafruit_BusIO_RegisterBits(&this->configReg, 3U,
      //                     (uint8_t) INA260_CONFIG_OFFSETS::MODE).read();
      // Adafruit_BusIO_RegisterBits(&this->configReg, 3U,
      //                     (uint8_t) INA260_CONFIG_OFFSETS::AVG).read();
      // Adafruit_BusIO_RegisterBits(&this->configReg, 3U,
      //                     (uint8_t) INA260_CONFIG_OFFSETS::VBUSCT).read();
      // Adafruit_BusIO_RegisterBits(&this->configReg, 3U,
      //                     (uint8_t) INA260_CONFIG_OFFSETS::ISHCT).read();
      
      // Adafruit_BusIO_RegisterBits(&this->maskEnableReg, 1U,
      //                     (uint8_t) INA260_MASK_ENABLE_OFFSETS::LEN).read();
      // Adafruit_BusIO_RegisterBits(&this->maskEnableReg, 6U,
      //                     (uint8_t) INA260_MASK_ENABLE_OFFSETS::CNVR).read();
      // Adafruit_BusIO_RegisterBits(&this->maskEnableReg, 1U,
      //                     (uint8_t) INA260_MASK_ENABLE_OFFSETS::APOL).read();

      // Adafruit_BusIO_RegisterBits(&this->alertLimitReg, 16U, 0U).read();
      return INA260SensorOperation(SENSOR_STATUS_NOT_IMPLEMENTED);
    }
    return INA260SensorOperation(SENSOR_STATUS_NOT_INIT);
  } else {
    return INA260SensorOperation(SENSOR_STATUS_OP_OK, this->configuration);
  }
}


Adafruit_INA260::~Adafruit_INA260() {

}