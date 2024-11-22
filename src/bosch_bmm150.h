/*****************************************************************************/
/**
 * @file       bosch_bmm150.h
 * @brief      bosch bmm150 sensor library for arduino
 * @author     André Grüttner
 * @copyright  Please see the accompanying LICENSE.txt file.
 * @date       2024-11-22
 */
/*****************************************************************************/
#pragma once

/*****************************************************************************/
#include "BMM150_SensorAPI/bmm150.h"
#include "BMM150_SensorAPI/bmm150_defs.h"

#include <Arduino.h>
#include <Wire.h>

#include <cstdint>

/*****************************************************************************/
namespace andrgrue::sensor {

/*****************************************************************************/

/**
 * @brief bosch bmm150 sensor driver for Arduino Nano uController
 */
class bosch_bmm150 {
  // data types
public:
  /**
   * @brief sensor data
   */
  struct Data {
    float    x;
    float    y;
    float    z;
    float    variance;
    uint64_t timestamp;
    uint8_t  flags;
  };

  /// device information
  struct DevInfo {
    TwoWire* wire;
    uint8_t  dev_addr;
  };

  // construction
public:
  /**
   * @brief constructor
   * @param wire I2C bus
   * @param address I2C device address of sensor
   * @param irqPin interrupt pin name
   * @param magneticfieldVariance magnetic field variance
   */
  bosch_bmm150(TwoWire&       wire,
               const uint8_t& address = BMM150_DEFAULT_I2C_ADDRESS,
               const PinName& irqPin  = NC,
               const float    magneticfieldVariance = 0.09);
  virtual ~bosch_bmm150() = default;

  // operations
public:
  /**
   * @brief initialize sensor
   * @param cb interrupt callback
   */
  bool initialize(const uint8_t& mag_freq = BMM150_DATA_RATE_10HZ
#ifdef __MBED__
                  ,
                  mbed::Callback<void(void)> cb = nullptr
#endif
  );
  void terminate();

  /**
   * @brief enable interrupt generation
   * interrupt pin must be configured
   */
  void enableInterrupt();

  /**
   * @brief disable interrupt generation
   */
  void disableInterrupt();

#if __MBED__
  /**
   * @brief link interrupt callback
   * @param cb interrupt callback
   * @return true if successful
   */
  bool interruptCallback(mbed::Callback<void()> cb);
#endif

  /**
   * @brief check if new data is available
   * @return true if new data is available
   */
  bool magneticfieldAvailable();
  /**
   * @brief read gyroscope
   * @param x x-axis magnetic field in micro Tesla
   * @param y y-axis magnetic field in micro Tesla
   * @param z z-axis magnetic field in micro Tesla
   */
  int magneticfield(float& x, float& y, float& z);
  int magneticfield(Data& data);
  /// Sampling rate of the sensor.
  float magneticfieldSampleRate();

protected:
  int8_t configure_sensor(struct bmm150_dev* dev);
#if __MBED__
  void interruptHandler();
#endif

private:
  // data
public:
protected:
private:
  TwoWire*      wire_;
  const uint8_t address_ {0};
  const PinName irqPin_ {NC};
  bool          initialized_ {false};
#ifdef __MBED__
  mbed::Callback<void(void)> cb_;
#endif
  volatile uint64_t timestamp_ns_ {0};

  DevInfo           mag_dev_info;
  struct bmm150_dev bmm1;

  uint8_t mag_freq_ {BMM150_DATA_RATE_10HZ};

  /**
   * @brief variance of the sensor values
   *  --> stdev = 0.3uT
   *  --> variance = 0.09uT^2
   */
  const float magneticfieldVariance_ {0.0f};
};

/*****************************************************************************/
}  // namespace andrgrue::sensor
