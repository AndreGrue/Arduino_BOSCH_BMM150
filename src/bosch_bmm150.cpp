/*****************************************************************************/
/**
 * @file       bosch_bmm150.cpp
 * @brief      bosch bmm150 sensor library for arduino
 * @author     André Grüttner
 * @copyright  Please see the accompanying LICENSE.txt file.
 * @date       2024-11-22
 */
/*****************************************************************************/
#include "bosch_bmm150.h"

#include "Arduino.h"
#include "Wire.h"

#ifdef __MBED__
#  include "drivers/InterruptIn.h"
#  include "mbed_events.h"
#  include "mbed_shared_queues.h"

static events::EventQueue accel_gyro_queue(10 * EVENTS_EVENT_SIZE);
#endif

/*****************************************************************************/
namespace andrgrue::sensor {

/*****************************************************************************/

static int8_t bmm1_i2c_read(uint8_t  reg_addr,
                            uint8_t* reg_data,
                            uint32_t len,
                            void*    intf_ptr) {
  if ((reg_data == NULL) || (len == 0) || (len > 32)) {
    return -1;
  }
  uint8_t bytes_received;

  bosch_bmm150::DevInfo* dev_info = (bosch_bmm150::DevInfo*)intf_ptr;
  uint8_t                dev_id   = dev_info->dev_addr;

  dev_info->wire->beginTransmission(dev_id);
  dev_info->wire->write(reg_addr);
  if (dev_info->wire->endTransmission(false) == 0) {
    bytes_received = dev_info->wire->requestFrom(dev_id, len);
    // Optionally, throw an error if bytes_received != len
    for (uint16_t i = 0; i < bytes_received; i++) {
      reg_data[i] = dev_info->wire->read();
    }
  } else {
    return -1;
  }

  return 0;
}

static int8_t bmm1_i2c_write(uint8_t        reg_addr,
                             const uint8_t* reg_data,
                             uint32_t       len,
                             void*          intf_ptr) {
  if ((reg_data == NULL) || (len == 0) || (len > 32)) {
    return -1;
  }

  bosch_bmm150::DevInfo* dev_info = (bosch_bmm150::DevInfo*)intf_ptr;
  uint8_t                dev_id   = dev_info->dev_addr;
  dev_info->wire->beginTransmission(dev_id);
  dev_info->wire->write(reg_addr);
  for (uint16_t i = 0; i < len; i++) {
    dev_info->wire->write(reg_data[i]);
  }
  if (dev_info->wire->endTransmission() != 0) {
    return -1;
  }

  return 0;
}

static void bmm1_delay_us(uint32_t period, void* intf_ptr) {
  delayMicroseconds(period);
}

/*****************************************************************************/

bosch_bmm150::bosch_bmm150(TwoWire&       wire,
                           const uint8_t& address,
                           const PinName& irqPin,
                           const float    magneticfieldVariance)
    : wire_(&wire)
    , address_(address)
    , irqPin_(irqPin)
    , magneticfieldVariance_(magneticfieldVariance) {}

bool bosch_bmm150::initialize(const uint8_t& mag_freq
#ifdef __MBED__
                              ,
                              mbed::Callback<void(void)> cb
#endif
) {
  mag_freq_ = mag_freq;

#ifdef __MBED__
  if (!interruptCallback(cb)) {
    initialized_ = false;
    return false;
  }
#endif
  enableInterrupt();

  bmm1.chip_id  = address_;
  bmm1.read     = bmm1_i2c_read;
  bmm1.write    = bmm1_i2c_write;
  bmm1.delay_us = bmm1_delay_us;
  bmm1.intf     = BMM150_I2C_INTF;
  bmm1.intf_ptr = &mag_dev_info;

  mag_dev_info.wire     = wire_;
  mag_dev_info.dev_addr = address_;

  int8_t bmm150InitResult = bmm150_init(&bmm1);
  if (bmm150InitResult != BMM150_OK) {
    return false;
  }

  int8_t bmm150ConfigResult = configure_sensor(&bmm1);
  if (bmm150ConfigResult != BMM150_OK) {
    return false;
  }

  bool success =
      (bmm150InitResult == BMM150_OK) && (bmm150ConfigResult == BMM150_OK);
  initialized_ = success;
  return success;
}

void bosch_bmm150::terminate() { initialized_ = false; }

void bosch_bmm150::enableInterrupt() {
  // configured as an input
  if (NC != irqPin_) {
    pinMode(irqPin_, INPUT);
  }
}

void bosch_bmm150::disableInterrupt() {}

#ifdef __MBED__

bool bosch_bmm150::interruptCallback(mbed::Callback<void(void)> cb) {
  if (NC != irqPin_ && nullptr != cb) {
    static mbed::InterruptIn irq(irqPin_, PullDown);
    static rtos::Thread      event_t(osPriorityHigh, 768, nullptr, "events");
    cb_          = cb;
    osStatus ret = event_t.start(
        callback(&accel_gyro_queue, &events::EventQueue::dispatch_forever));
    if (osOK == ret) {
      irq.rise(mbed::callback(this, &bosch_bmm150::interruptHandler));
      return true;
    }
    return false;
  }
  return true;
}

void bosch_bmm150::interruptHandler() {
  if (initialized_ && cb_) {
    timestamp_ns_ = micros() * 1000ULL;
    accel_gyro_queue.call(cb_);
  }
}

#endif

int bosch_bmm150::magneticfield(float& x, float& y, float& z) {
  struct bmm150_mag_data mag_data;
  int const              rc = bmm150_read_mag_data(&mag_data, &bmm1);
  x                         = mag_data.x;
  y                         = mag_data.y;
  z                         = mag_data.z;

  if (rc == BMM150_OK)
    return 1;
  else
    return 0;
}

int bosch_bmm150::magneticfield(Data& data) {
  data.variance = magneticfieldVariance_;
  if (0 == timestamp_ns_) {
    data.timestamp = micros() * 1000ULL;
  } else {
    data.timestamp = timestamp_ns_;
    timestamp_ns_  = 0;
  }
  data.flags = 0;
  return magneticfield(data.x, data.y, data.z);
}

bool bosch_bmm150::magneticfieldAvailable() {
  bmm150_get_interrupt_status(&bmm1);
  return (bmm1.int_status & BMM150_INT_ASSERTED_DRDY) != 0;
}

float bosch_bmm150::magneticfieldSampleRate() {
  struct bmm150_settings settings;
  bmm150_get_sensor_settings(&settings, &bmm1);
  switch (settings.data_rate) {
  case BMM150_DATA_RATE_10HZ:
    return 10;
  case BMM150_DATA_RATE_02HZ:
    return 2;
  case BMM150_DATA_RATE_06HZ:
    return 6;
  case BMM150_DATA_RATE_08HZ:
    return 8;
  case BMM150_DATA_RATE_15HZ:
    return 15;
  case BMM150_DATA_RATE_20HZ:
    return 20;
  case BMM150_DATA_RATE_25HZ:
    return 25;
  case BMM150_DATA_RATE_30HZ:
    return 30;
  }
  return 0;
}

int8_t bosch_bmm150::configure_sensor(struct bmm150_dev* dev) {
  /* Status of api are returned to this variable. */
  int8_t                 rslt;
  struct bmm150_settings settings;

  /* Set powermode as normal mode */
  settings.pwr_mode = BMM150_POWERMODE_NORMAL;
  rslt              = bmm150_set_op_mode(&settings, dev);

  if (rslt == BMM150_OK) {
    /* Setting the preset mode as Low power mode
     * i.e. data rate = 10Hz, XY-rep = 1, Z-rep = 2
     */
    settings.preset_mode = BMM150_PRESETMODE_REGULAR;
    // rslt = bmm150_set_presetmode(&settings, dev);

    if (rslt == BMM150_OK) {
      /* Map the data interrupt pin */
      settings.int_settings.drdy_pin_en = 0x01;
      // rslt = bmm150_set_sensor_settings(BMM150_SEL_DRDY_PIN_EN, &settings,
      // dev);

      settings.data_rate = mag_freq_;
      rslt = bmm150_set_sensor_settings(BMM150_SEL_DATA_RATE, &settings, dev);
    }
  }
  return rslt;
}

/*****************************************************************************/
}  // namespace andrgrue::sensor
