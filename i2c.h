//
// Created by Pieter Bouwer on 2019-08-20.
//

#include <cstdint>
#include <string>

#ifndef INA219_I2C_H
#define INA219_I2C_H

class I2C {
public:
    /**
     * @param string device
     */
    I2C(const std::string&);
    ~I2C();
    virtual void beginTransmission(uint8_t);
    virtual void writeByte(uint8_t data);
    virtual uint8_t readByte();
private:
    int file_i2c;
};

#endif //INA219_I2C_H
