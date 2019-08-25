INA219 I2C library for Raspberry Pi
===================================

This is a simple port of the Adafruit INA219 library for C++ to be used on a Raspberry Pi.

Pre-requisites:
- Cross compiler tools (e.g. Crosstool NG)
- Toolchain file referencing your cross compiling tools

# Using the library

```c++
    Adafruit_INA219 ina219(0x40);
// calibration calculated with a current LSB of 1e-4
// Vbus_Max = 16V
// Vshunt_Max = 0.32V
// R_Shunt = 0.1Ohm
// MaxExpected_I = 2A
// Min_LSB = MaxExpected_I / 32767
// Max_LSB = MaxExpected_I / 4096
// Min_LSB < Current_LSB < Max_LSB (But close to Min_LSB)
// Current_LSB = 1e-4
// Calibration_Value = 4096 / (1e4 * (Current_LSB * R_Shunt)) = 4096

    ina219.setCalibration(0x1000, 10, 2);
//           PGA  BADC   SADC   Mode
//    [0]0[0][1 1][111 0][001 1][111]
//    1		    f      1      f
    ina219.setConfiguration(0x1f1f);
    cout << "shunt voltage: " << ina219.getShuntVoltage_mV() << " [mV]" << endl;
    cout << "bus voltage: " << ina219.getBusVoltage_V() << " [V]" << endl;
    cout << "current: " << ina219.getCurrent_mA() << " [mA]" << endl;
    cout << "power: " << ina219.getPower_mW() << " [mW]" << endl;
```