//
// Created by Pieter Bouwer on 2019-08-20.
//


#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port
#include <cstdio>
#include <iostream>
#include <fstream>
#include "i2c.h"

I2C::I2C(const std::string& device) {
    if ((file_i2c = open(device.c_str(), O_RDWR)) < 0)
    {
        printf("Failed to open the i2c bus");
        return;
    }
}

I2C::~I2C() {
    close(file_i2c);
}

void I2C::beginTransmission(uint8_t address) {
    if (ioctl(file_i2c, I2C_SLAVE, address) < 0)
    {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        return;
    }
}

void I2C::writeByte(uint8_t data) {
    if (write(file_i2c, &data, 1) != 1)
    {
        printf("Failed to write to the i2c bus.\n");
    }
}

uint8_t I2C::readByte() {
    uint8_t d;
    if (read(file_i2c, &d, 1) != 1)
    {
        printf("Failed to read from the i2c bus.\n");
    }
    return d;
}
