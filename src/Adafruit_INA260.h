/*!
 *  @file Adafruit_INA260.h
 *
 * 	I2C Driver for INA260 Current and Power sensor
 *
 * 	This is a library for the Adafruit INA260 breakout:
 * 	http://www.adafruit.com/products/4226
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#pragma once

#include <memory>
#include <optional>

#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_I2CDevice.h>
#include <Adafruit_BusIO_Register.h>

#include <SensorCommon.h>

/**
 * INA260 default values.
 */
enum class INA260_DEFINITIONS : uint16_t {
  I2CADDR_DEFAULT = 0x40,   ///< INA260 default i2c address
  MANUFACTURER_ID = 0x5449, ///< Default manufacturer ID
  DEVICE_ID       = 0x227   ///< Default device ID
};


/**
 * INA260 hardware registers. (Datasheet 8.6)
 */
enum class INA260_REGISTERS : uint8_t {
  CONFIG       = 0x00, ///< Configuration register
  CURRENT      = 0x01, ///< Current measurement register (signed) in mA
  BUSVOLTAGE   = 0x02, ///< Bus voltage measurement register in mV
  POWER        = 0x03, ///< Power calculation register in mW
  MASK_ENABLE  = 0x06, ///< Interrupt/Alert setting and checking register
  ALERT_LIMIT  = 0x07, ///< Alert limit value register
  MFG_UID      = 0xFE, ///< Manufacturer ID Register
  DIE_UID      = 0xFF  ///< Die ID and Revision Register
};


/**
 * INA260 register offsets. (Datasheet 8.6)
 */
/* Configuration Register Field */
enum class INA260_CONFIG_OFFSETS : uint8_t {
  MODE   = 0x00, ///< Mode offset
  ISHCT  = 0x03, ///< Current conversion time offset
  VBUSCT = 0x06, ///< Bus voltage conversion time offset
  AVG    = 0x09, ///< Averaging mode
  RESET  = 0x0F  ///< Reset bit
};

/* Mask/Enable Register Field */
enum class INA260_MASK_ENABLE_OFFSETS : uint8_t {
  OVF   = 0x02, ///< Math overflow flag
  CVRF  = 0x03, ///< Conversion ready
  /* Alert configuration*/
  LEN   = 0x00, ///< Alert latch enable
  APOL  = 0x01, ///< Alert polarity bit
  AFF   = 0x04, ///< Alert function flag
  CNVR  = 0x0A, ///< Conversion ready
  POL   = 0x0B, ///< Power over-limit
  BUL   = 0x0C, ///< Bus voltage under-voltage
  BOL   = 0x0D, ///< Bus voltage over-voltage
  UCL   = 0x0E, ///< Under current limit
  OCL   = 0x0F  ///< Over current limit
};

/* Die ID Register Field */
enum class INA260_DIE_UID_OFFSETS : uint8_t {
  RID = 0x00, ///< Die revision ID
  DID = 0x04  ///< Device ID
};


/**
 * @brief Mode options.
 *
 * Allowed values for setMode.
 */
enum class INA260_MeasurementMode : uint8_t {
  SHUTDOWN    = 0x00, /** SHUTDOWN: Minimize quiescient current and turn off current into the device inputs. Set another mode to exit shutown mode */
  TRIGGERED   = 0x03, /** TRIGGERED: Trigger a one-shot measurement of current and bus voltage. Set the TRIGGERED mode again to take a new measurement */
  CONTINUOUS  = 0x07  /** CONTINUOUS: (Default) Continuously update the current, bus voltage and power registers with new measurements */
};


/**
 * @brief Conversion Time options.
 *
 * Allowed values for setCurrentConversionTime and setVoltageConversionTime.
 */
enum class INA260_ConversionTime : uint8_t{
  TIME_140_us,   ///< Measurement time: 140us
  TIME_204_us,    ///< Measurement time: 204us
  TIME_332_us,    ///< Measurement time: 332us
  TIME_558_us,   ///< Measurement time: 558us
  TIME_1_1_ms,   ///< Measurement time: 1.1ms (Default)
  TIME_2_116_ms, ///< Measurement time: 2.116ms
  TIME_4_156_ms, ///< Measurement time: 4.156ms
  TIME_8_244_ms, ///< Measurement time: 8.224ms
};


/**
 * @brief Averaging Count options.
 *
 * Allowed values forsetAveragingCount.
 */
enum class INA260_AveragingCount : uint8_t {
  COUNT_1,    ///< Window size: 1 sample (Default)
  COUNT_4,    ///< Window size: 4 samples
  COUNT_16,   ///< Window size: 16 samples
  COUNT_64,   ///< Window size: 64 samples
  COUNT_128,  ///< Window size: 128 samples
  COUNT_256,  ///< Window size: 256 samples
  COUNT_512,  ///< Window size: 512 samples
  COUNT_1024, ///< Window size: 1024 samples
};


/**
 * @brief Alert trigger options.
 *
 * Allowed values for setAlertType.
 */
enum class INA260_AlertType : uint8_t {
  CONVERSION_READY  = 0x1,  ///< Trigger on conversion ready
  OVERPOWER         = 0x2,  ///< Trigger on power over limit
  UNDERVOLTAGE      = 0x4,  ///< Trigger on bus voltage under limit
  OVERVOLTAGE       = 0x8,  ///< Trigger on bus voltage over limit
  UNDERCURRENT      = 0x10, ///< Trigger on current under limit
  OVERCURRENT       = 0x20, ///< Trigger on current over limit
  NONE              = 0x0,  ///< Do not trigger alert pin (Default)
};


/**
 * @brief Alert pin polarity options.
 *
 * Allowed values for setAlertPolarity.
 */
enum class INA260_AlertPolarity : uint8_t {
  NORMAL    = 0x0, ///< Active high open-collector (Default)
  INVERTED  = 0x1, ///< Active low open-collector
};


/**
 * @brief Alert pin latch options.
 *
 * Allowed values for setAlertLatch.
 */
enum class INA260_AlertLatch : uint8_t {
  ENABLED     = 0x1, ///< Alert will latch until Mask/Enable register is read
  TRANSPARENT = 0x0, ///< Alert will reset when fault is cleared
};

typedef struct _ina260_config {
  INA260_MeasurementMode mode = INA260_MeasurementMode::CONTINUOUS;
  INA260_ConversionTime currentConversionTime = INA260_ConversionTime::TIME_558_us;
  INA260_ConversionTime voltageConversionTime = INA260_ConversionTime::TIME_558_us;
  INA260_AveragingCount averagingCount = INA260_AveragingCount::COUNT_1;

  INA260_AlertLatch alertLatchState = INA260_AlertLatch::TRANSPARENT;
  INA260_AlertPolarity alertLatchPolarity = INA260_AlertPolarity::NORMAL;
  INA260_AlertType alertLatchType = INA260_AlertType::NONE;
  float alertThreshold = 0.0f;
  bool alertCallbackAttached = false;
} INA260_Config;

using INA260SensorOperation = SensorOperation<INA260_Config>;

/**
 * @brief  Class that stores state and functions for interacting with INA260 Current and Power Sensor
 */
class Adafruit_INA260 {
public:
  Adafruit_INA260(TwoWire *theWire = &Wire, uint8_t i2cAddress = (uint8_t) INA260_DEFINITIONS::I2CADDR_DEFAULT);

  INA260SensorOperation begin(const INA260_Config &sensorConfig);
  INA260SensorOperation end(void);

  INA260SensorOperation reset(void);

  INA260SensorOperation readCurrent(void);
  INA260SensorOperation readVoltage(void);
  INA260SensorOperation readPower(void);

  INA260SensorOperation getConfiguration(bool update);

  INA260SensorOperation setMode(INA260_MeasurementMode mode);
  INA260SensorOperation setCurrentConversionTime(INA260_ConversionTime time);
  INA260SensorOperation setVoltageConversionTime(INA260_ConversionTime time);
  INA260SensorOperation setAveragingCount(INA260_AveragingCount count);

  INA260SensorOperation setAlertLimit(float limit);
  INA260SensorOperation setAlertLatch(INA260_AlertLatch state);
  INA260SensorOperation setAlertPolarity(INA260_AlertPolarity polarity);
  INA260SensorOperation setAlertType(INA260_AlertType alert);

  INA260SensorOperation conversionReady(void);
  INA260SensorOperation alertFunctionFlag(void);
  
  ~Adafruit_INA260();
private:
  bool isInitialized = false;

  INA260_Config configuration;

  std::unique_ptr<Adafruit_I2CDevice> i2cDevice;

  /* INA260 register objects */
  std::optional<Adafruit_BusIO_Register> configReg;
  std::optional<Adafruit_BusIO_Register> maskEnableReg;
  std::optional<Adafruit_BusIO_Register> alertLimitReg;

  std::optional<Adafruit_BusIO_Register> currentReg;
  std::optional<Adafruit_BusIO_Register> votlageReg;
  std::optional<Adafruit_BusIO_Register> powerReg;

  std::optional<Adafruit_BusIO_Register> dieRegister;
  std::optional<Adafruit_BusIO_Register> mfgRegister;
};
