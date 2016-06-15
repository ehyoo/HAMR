#!/usr/bin/env python

# HTU21D lib for Edison using mraa by Chris Nelson, October 2015
# Clues from Sparkfun and Adafruit implementations


import mraa as m
import time


class HAMR:
    x = None

    # Commands
    # TRIGGER_TEMP_MEASURE_HOLD = 0xE3
    # TRIGGER_RH_MEASURE_HOLD =   0xE5
    TEST_ARDUINO =              0x06
    # WRITE_USER_REG =            0xE6
    # SOFT_RESET =                0xFE

    # # MRAA I2C bus frequency modes.  These enums should probably be picked up from a header file.
    I2C_STD =  0
    I2C_FAST = 1
    I2C_HIGH = 2

    # Constructor
    def __init__(self, bus, debug=False, freq=I2C_STD):
        # self.x = m.I2c(bus, raw=True)  # raw=True forces manual bus selection, vs. board default
        self.x = m.I2c(bus) # note i had to remove the raw keyword as it did not work. maybe needs updating?
        
        self.debug = debug
        # if self.debug:
        print self.x
        self.x.frequency(freq)  #default to I2C_STD (up to 100kHz).  Other options:  I2C_FAST (up to 400kHz), I2C_HIGH (up to 3.4MHz)
        self.x.address(0x05) # Address of the HTU21D sensor


    # test function to read arduino
    def read_arduino(self):
        self.x.address(0x05)
        n = self.x.readReg(self.TEST_ARDUINO)
        # if self.debug or deb:
        print "reading arduino: {}".format (hex(n))
        return n

    def write_arduino(self):
        n = self.x.writeByte(5)
        print n
        return n


    # def readUserReg(self, deb=False):
    #     "Read the user register byte"
    #     n = self.x.readReg(self.READ_USER_REG)
    #     if self.debug or deb:
    #         print "HTU21D Read User reg: {}".format (hex(n))
    #     return n


    # def writeUserReg(self, value, deb=False):
    #     "Write the user register byte"
    #     n = self.x.writeReg(self.WRITE_USER_REG, value)  # returns 0x00 on ack, and 0x05 on no response
    #     if self.debug or deb:
    #         print "HTU21D Write User reg: {}, Status: {}, UserReg read back: {}".format (hex(value), hex(n), hex(self.x.readReg(self.READ_USER_REG)))
    #     return n


    # def softReset (self, deb=False):
    #     "Issue soft reset, and wait"
    #     n = self.x.writeByte(self.SOFT_RESET)  # returns 0x00 on ack, and 0x05 on no response
    #     time.sleep (0.05)  # spec requires 15ms.  Set to 50ms, just because I'm not in that much of a hurry.
    #     if self.debug or deb:
    #         print "HTU21D Soft reset status: {}, UserReg read back: {}".format (hex(n), hex(self.x.readReg(self.READ_USER_REG)))
    #     return n


    # def readTemperatureData(self, deb=False):

    #     "Read 3 temperature bytes from the sensor"
    #     try:
    #         bytes3 = self.x.readBytesReg (self.TRIGGER_TEMP_MEASURE_HOLD, 3)
    #     except:
    #         if self.debug or deb:
    #             print "HTU21D readTemperatureData readBytesReg failed"
    #         return -256  # I2C read error
            
    #     # if self.debug or deb:
    #         # print "HTU21D Read temp returned:  {} {} {}".format (hex(bytes3[0]),hex(bytes3[1]),hex(bytes3[2]))
           
    #     return temp















    # def readRHData(self, deb=False):
    #     "Read 3 relative humidity bytes from the sensor"
    #     try:
    #         bytes3 = self.x.readBytesReg (self.TRIGGER_RH_MEASURE_HOLD, 3)
    #     except:
    #         if self.debug or deb:
    #             print "HTU21D readRHData readBytesReg failed"
    #         return -256  # I2C read error
            
    #     if self.debug or deb:
    #         print "HTU21D Read RH returned:    {} {} {}".format (hex(bytes3[0]),hex(bytes3[1]),hex(bytes3[2]))

    #     # CRC Check
    #     if not self.crc8check(bytes3):
    #         if self.debug or deb:
    #             print "HTU21D Read RH CRC error"
    #         return -255  # bad CRC

    #     MSB = bytes3[0]
    #     LSB = bytes3[1]
    #     rawRH = ((MSB <<8) | LSB) & 0xFFFC  # Lower two bits of LSB are status bits to be cleared
    #     if self.debug or deb:
    #         print "HTU21D Raw RH:              {0:x} hex, {0:d} decimal".format (rawRH)

    #     # Apply datasheet forumula:  RH = -6 + 125 * <sensor data> / 2^16
    #     RH = -6.0 + (125 * rawRH / 2**16)
    #     if self.debug or deb:
    #         print "HTU21D Calculated RH:       {}".format (RH)

    #     return RH
