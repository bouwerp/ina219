#include <unistd.h>
#include "ina219.h"
#include "pigpio.h"

/*!
 *  @brief  Sends a single command byte over I2C
 *  @param  reg
 *          register address
 *  @param  value
 *          value to write
 */
void Adafruit_INA219::wireWriteRegister(uint8_t reg, uint16_t value) {
    i2cWriteWordData(_i2c_handle, reg, value);
}

/*!
 *  @brief  Reads a 16 bit values over I2C
 *  @param  reg
 *          register address
 *  @param  *value
 *          read value
 */
void Adafruit_INA219::wireReadRegister(uint8_t reg, uint16_t *value) {
    // sleep for 1ms
    usleep(1000); // Max 12-bit conversion time is 586us per sample
    *value =  i2cReadWordData(_i2c_handle, reg);
}

/*!
 *  @brief  Set power save mode according to parameters
 *  @param  on
 *          boolean value
 */
void Adafruit_INA219::powerSave(bool on) {
    uint16_t current;
    wireReadRegister(INA219_REG_CONFIG, &current);
    uint8_t next;
    if (on) {
        next = current | INA219_CONFIG_MODE_POWERDOWN;
    } else {
        next = current & ~INA219_CONFIG_MODE_POWERDOWN;
    }
    wireWriteRegister(INA219_REG_CONFIG, next);
}

/*!
 *  @brief  Instantiates a new INA219 class
 *  @param addr the I2C address the device can be found on. Default is 0x40
 */
Adafruit_INA219::Adafruit_INA219(uint8_t addr) {
    gpioInitialise();
    _i2c_handle = i2cOpen(1, addr, 0);
    ina219_i2caddr = addr;
    ina219_currentDivider_mA = 0;
    ina219_powerMultiplier_mW = 0;
}

/*!
 *  @brief  Gets the raw bus voltage (16-bit signed integer, so +-32767)
 *  @return the raw bus voltage reading
 */
uint16_t Adafruit_INA219::getBusVoltage_raw() {
    uint16_t value;
    wireReadRegister(INA219_REG_BUSVOLTAGE, &value);
    // flip MSB and LSB
    value = ((value & 0xFF00u) >> 8u) + ((value & 0x00FFu) << 8u);
    // remove OVF and CNVR bits and shift right to adjust
    return (int16_t)((value & 0xFFF8u) >> 1u);
}

/*!
 *  @brief  Gets the raw shunt voltage (16-bit signed integer, so +-32767)
 *  @return the raw shunt voltage reading
 */
uint16_t Adafruit_INA219::getShuntVoltage_raw() {
    uint16_t value;
    wireReadRegister(INA219_REG_SHUNTVOLTAGE, &value);
    // flip MSB and LSB
    value = ((value & 0xFF00u) >> 8u) + ((value & 0x00FFu) << 8u);
    return (uint16_t)(value);
}

/*!
 *  @brief  Gets the raw current value (16-bit signed integer, so +-32767)
 *  @return the raw current reading
 */
uint16_t Adafruit_INA219::getCurrent_raw() {
    uint16_t value;
    wireReadRegister(INA219_REG_CURRENT, &value);
    return (uint16_t)value;
}

/*!
 *  @brief  Gets the raw power value (16-bit signed integer, so +-32767)
 *  @return raw power reading
 */
uint16_t Adafruit_INA219::getPower_raw() {
    uint16_t value;
    wireReadRegister(INA219_REG_POWER, &value);
    return (uint16_t)value;
}

/*!
 *  @brief  Gets the shunt voltage in mV (so +-327mV)
 *  @return the shunt voltage converted to millivolts
 */
float Adafruit_INA219::getShuntVoltage_mV() {
    uint16_t value;
    value = getShuntVoltage_raw();
    return value * 0.01;
}

/*!
 *  @brief  Gets the shunt voltage in volts
 *  @return the bus voltage converted to volts
 */
float Adafruit_INA219::getBusVoltage_V() {
    uint16_t value = getBusVoltage_raw();
    return ((float)value)*0.001;
}

/*!
 *  @brief  Gets the current value in mA, taking into account the
 *          config settings and current LSB
 *  @return the current reading convereted to milliamps
 */
float Adafruit_INA219::getCurrent_mA() {
    float valueDec = getCurrent_raw();
    valueDec /= ina219_currentDivider_mA;
    return valueDec;
}

/*!
 *  @brief  Gets the power value in mW, taking into account the
 *          config settings and current LSB
 *  @return power reading converted to milliwatts
 */
float Adafruit_INA219::getPower_mW() {
    float valueDec = getPower_raw();
    valueDec *= ina219_powerMultiplier_mW;
    return valueDec;
}

void Adafruit_INA219::setCalibration(uint16_t calibration, uint32_t currentDivider, float powerMultiplier) {
    ina219_currentDivider_mA = currentDivider;
    ina219_powerMultiplier_mW = powerMultiplier;
    ina219_calValue = calibration;
    wireWriteRegister(INA219_REG_CALIBRATION, calibration);

}

void Adafruit_INA219::setConfiguration(uint16_t configuration) {
    ina219_configValue = configuration;
    wireWriteRegister(INA219_REG_CONFIG, configuration);
}