"""
qwiic_as6212
============
Python module for the [SparkFun Digital Temperature Sensor Breakout - AS6212 (Qwiic)](https://www.sparkfun.com/products/18521)

This python package is a port of the existing [SparkFun Qwiic AS6212 Sensor Arduino Library](https://github.com/sparkfun/SparkFun_TMP102_Arduino_Library/tree/master/examples)

This package can be used in conjunction with the overall [SparkFun qwiic Python Package](https://github.com/sparkfun/Qwiic_Py)

New to qwiic? Take a look at the entire [SparkFun qwiic ecosystem](https://www.sparkfun.com/qwiic).

"""
from __future__ import print_function, division

import qwiic_i2c

#======================================================================
# Basic setup of I2C commands and available I2C Addresses
#
#
# The name of this device - note this is private
_DEFAULT_NAME = "Qwiic AS6212 Sensor"

AS6212_DEFAULT_ADDRESS = 0x48

#Internal Register Addresses
TVAL_REG      =       0x0    #Temperature Register
CONFIG_REG    =       0x1    #Configuration Register
TLOW_REG      =       0x2    #Low Temperature Threshold
THIGH_REG     =       0x3    #High Temperature Threshold

#Helpful preset definitions for configuration register
DEFAULTM  =       0x40A0   #Default state
SLEEPMODE =       0x41A0   #Sleep Mode
SLEEPSS   =       0xC1A0   #Sleep Mode Single Shot

# Register values (MSB -> LSB)
SINGLESHOT	=	0x8000	#15
CFAULT_1		=	0x0800	#12
CFAULT_0		=	0x0400	#11
POLARITY		=	0x0200	#10
INTERRUPT		=	0x0100	#9
SLEEP			=	0x0080	#8
CONVER_RATE_1 =	0x0040	#7
CONVER_RATE_0 =	0x0020	#6
ALERT			=	0x0010	#5

AS6212_RESOLUTION = 0.0078125

AS6212_CONFIG_BIT_ALERT = 5
AS6212_CONFIG_BIT_CONVERSION_RATE_0 = 6
AS6212_CONFIG_BIT_CONVERSION_RATE_1 = 7
AS6212_CONFIG_BIT_SLEEP_MODE = 8
AS6212_CONFIG_BIT_INTERRUPT_MODE = 9
AS6212_CONFIG_BIT_ALERT_POL = 10
AS6212_CONFIG_BIT_CONSECUTIVE_FAULTS_0 = 11
AS6212_CONFIG_BIT_CONSECUTIVE_FAULTS_1 = 12
AS6212_CONFIG_BIT_SINGLE_SHOT = 15

#Address can be set using jumpers on bottom of board (default: 0x48)
_AVAILABLE_I2C_ADDRESS = [0x48, 0x44, 0x45, 0x46, 0x47, 0x49, 0x4A, 0x4B]

###############################################################################
###############################################################################
# Some devices have multiple available addresses - this is a list of these addresses.
# NOTE: The first address in this list is considered the default I2C address for the
# device.

class QwiicAs6212Sensor(object):
    """
    QwiicAs6212Sensor

        :param address: The I2C address to use for the device.
                        If not provided, the default address is used.
        :param i2c_driver: An existing i2c driver object. If not provided
                        a driver object is created.
        :return: The AS6212 Sensor device object.
        :rtype: Object
    """
    device_name         = _DEFAULT_NAME
    available_addresses = _AVAILABLE_I2C_ADDRESS

    AS6212_MODE_COMPARATOR = 0
    AS6212_MODE_INTERRUPT = 1

    AS6212_CONVERSION_CYCLE_TIME_125MS = 3
    AS6212_CONVERSION_CYCLE_TIME_250MS = 2
    AS6212_CONVERSION_CYCLE_TIME_1000MS = 1
    AS6212_CONVERSION_CYCLE_TIME_4000MS = 0

    AS6212_ALERT_ACTIVE_HIGH = 1
    AS6212_ALERT_ACTIVE_LOW = 0

    # Constructor
    def __init__(self, address=None, i2c_driver=None):

        # Did the user specify an I2C address?
        self.address = self.available_addresses[0] if address is None else address

        # load the I2C driver if one isn't provided

        if i2c_driver is None:
            self._i2c = qwiic_i2c.getI2CDriver()
            if self._i2c is None:
                print("Unable to load I2C driver for this platform.")
                return
        else:
            self._i2c = i2c_driver


    # ----------------------------------
    # is_connected()
    #
    # Is an actual board connected to our system?

    def is_connected(self):
        """
            Determine if a Soil MoistureSensor device is conntected to the system..
            :return: True if the device is connected, otherwise False.
            :rtype: bool
        """        
        return qwiic_i2c.isDeviceConnected(self.address)

    connected = property(is_connected)

    # ----------------------------------
    # begin()
    #
    # Initialize the system/validate the board.
    def begin(self):
        """
            Initialize the operation of the Soil Moisture Sensor module
            :return: Returns true of the initialization was successful, otherwise False.
            :rtype: bool
        """
        # Set variables
        self.tempC = 0
        self.tempF = 0
        
        # Basically return True if we are connected...
        return self.is_connected()

    #****************************************************************************#
    #
    #   Sensor functions
    #
    # ****************************************************************************#
    def get_address(self):
        """
        Returns the device address
        """
        return self.address

    def read_2_byte_register(self, register_to_read):
        """
        Reads two bytes of data from a desired register.
        Combines them into a single 16 bit value
        Returns single value
        """
        data = self._i2c.readBlock(self.address, register_to_read, 2)

        #Combine bytes to create a single signed int
        return ( (data[0] << 8 ) | data[1] )

    def write_register(self, reg, data):
        data_bytes = [0,0]
        data_bytes[1] |= (data & 0xFF)
        data_bytes[0] |= data >> 8
        print(data_bytes)
        self._i2c.writeBlock(self.address, reg, data_bytes)

    def read_config(self):
        return self.read_2_byte_register(CONFIG_REG)

    def write_config(self, targetState):
        self.write_register(CONFIG_REG, targetState)
        
    def read_temp_c(self):
        """
        Reads the results from the sensor
        :rtype: integer
        """
        digitalTempC = self.read_2_byte_register(TVAL_REG)
                
        if (digitalTempC < 32768):
                finalTempC = digitalTempC * 0.0078125
        if (digitalTempC >= 32768):
                finalTempC = ((digitalTempC - 1) * 0.0078125) * -1

        self.tempC = finalTempC
        return self.tempC
        
    def read_temp_f(self):
        """
        Reads the results from the sensor
        :rtype: integer
        """

        self.tempF = self.read_temp_c() * 9.0 / 5.0 + 32.0
        return self.tempF

    def set_conversion_cycletime(self, cycletime):
        """
        sets the conversion cylce time (aka convertion rate) in the config register
        valid settings are:
        
        AS6212_CONVERSION_CYCLE_TIME_125MS
        AS6212_CONVERSION_CYCLE_TIME_250MS
        AS6212_CONVERSION_CYCLE_TIME_1000MS
        AS6212_CONVERSION_CYCLE_TIME_4000MS
        """
        #discard out of range values
        if cycletime > 3 or cycletime < 0:
            return nan
        configReg = self.read_config()
        configBit_6 = self.bit_read(cycletime, 0)
        configBit_7 = self.bit_read(cycletime, 1)
        configReg = self.bit_write(configReg, AS6212_CONFIG_BIT_CONVERSION_RATE_0, configBit_6)
        configReg = self.bit_write(configReg, AS6212_CONFIG_BIT_CONVERSION_RATE_1, configBit_7)
        self.write_config(configReg)      

    def set_alert_polarity(self, polarity):
        """
        Set the polarity of Alert
        0 - Active LOW
        1 - Active HIGH
        """
        configReg = self.read_config()
        configReg = self.bit_write(configReg, AS6212_CONFIG_BIT_ALERT_POL, polarity)
        self.print_binary(configReg)
        
        self.write_config(configReg)















          
    def sleep(self):
        """
        Switch sensor to low power mode
        """
        sleepValue = self._i2c.readByte(self.address, CONFIG_REG)
        sleepValue |= 0x01	# Set SD (bit 0 of first byte)
        self._i2c.writeByte(self.address, CONFIG_REG, sleepValue)



    def alert(self):
        """
        Returns state of Alert register
        """
        alert = self._i2c.readByte(self.address, CONFIG_REG)
        alert &= 0x20   #Clear everything but the alert bit (bit 5)
        return alert>>5
        
    def one_shot(self, setOneShot = 0):
        """
        Sets the SingleShot Register. Returns 1 after the conversion is complete
        """
        registerByte = self._i2c.readByte(self.address, CONFIG_REG)
        if(setOneShot == 1):
                registerByte |= (1<<7)
                self._i2c.writeByte(self.address, CONFIG_REG, registerByte)
                return 0
        else:
                registerByte &= (1<<7)
                return (registerByte>>7)


    def set_low_temp_c(self, temperature):
        """
        Sets T_LOW (degrees C) alert threshold
        """
        if(temperature > 150.0):
                temperature = 150.0
        if(temperature < -55.0):
                temperature = -55.0
                
        registerByte = self._i2c.readBlock(self.address, CONFIG_REG, 2)
        
        #Check if temperature should be 12 or 13 bits
        # 0 - temp data will be 12 bits
        # 1 - temp data will be 13 bits
        extendedMode = (registerByte[1]&0x10) >> 4	
                                                                
        #Convert analog temperature to digital value
        temperature = temperature/0.0625
        
        if(extendedMode):	#13-bit mode
                registerByte[0] = int(temperature)>>5
                registerByte[1] = (int(temperature)<<3)
        else:
                registerByte[0] = int(temperature)>>4
                registerByte[1] = int(temperature)<<4
              
        self._i2c.writeBlock(self.address, T_LOW_REGISTER, registerByte)

    def set_high_temp_c(self, temperature):
        """
        Sets T_LOW (degrees C) alert threshold
        """

        if(temperature > 150.0):
                temperature = 150.0
        if(temperature < -55.0):
                temperature = -55.0
        registerByte = self._i2c.readBlock(self.address, CONFIG_REG, 2)
        
        #Check if temperature should be 12 or 13 bits
        # 0 - temp data will be 12 bits
        # 1 - temp data will be 13 bits
        extendedMode = (registerByte[1]&0x10) >> 4	
                                                                
        #Convert analog temperature to digital value
        temperature = temperature/0.0625
                
        if(extendedMode):	#13-bit mode
                registerByte[0] = int(temperature)>>5
                registerByte[1] = (int(temperature)<<3)
        else:
                registerByte[0] = int(temperature)>>4
                registerByte[1] = int(temperature)<<4
                
        self._i2c.writeBlock(self.address, T_HIGH_REGISTER, registerByte)

    def set_low_temp_f(self, temperature):
        """
        Sets T_LOW (degrees F) alert threshold
        """
        new_temp = (temperature - 32)*5/9    # Convert temperature to C
        self.set_low_temp_c(new_temp)           # Set T_LOW

        
    def set_high_temp_f(self, temperature):
        """
        Sets T_HIGH (degrees F) alert threshold
        """
        new_temp = (temperature - 32)*5/9    # Convert temperature to C
        self.set_high_temp_c(new_temp)           # Set T_HIGH


    def read_low_temp_c(self):
        """
        Gets T_LOW (degrees C) alert threshold
        """
        configByte = self._i2c.readBlock(self.address, CONFIG_REG, 2)

        # 0 - temp data will be 12 bits
        # 1 - temp data will be 13 bits
        extendedMode = (configByte[1]&0x10)>>4	

        lowTempByte = self._i2c.readBlock(self.address, T_LOW_REGISTER, 2)

        if(lowTempByte[0] == 0xFF and lowTempByte[1] == 0xFF):
                return NAN

        if (extendedMode):
                digitalTemp = ((lowTempByte[0]) << 5) | (lowTempByte[1] >> 3)
                if(digitalTemp > 0xFFF):
                        digitalTemp |= 0xE000
        else:
                digitalTemp = ((lowTempByte[0]) << 4) | (lowTempByte[1] >> 4)
                if(digitalTemp > 0x7FF):
                        digitalTemp |= 0xF000
                        
        return digitalTemp*0.0625    
        
                    
    def read_high_temp_c(self):
        """
        Gets T_HIGH (degrees C) alert threshold
        """
        configByte = self._i2c.readBlock(self.address, CONFIG_REG, 2)

        # 0 - temp data will be 12 bits
        # 1 - temp data will be 13 bits
        extendedMode = (configByte[1]&0x10)>>4	

        highTempByte = self._i2c.readBlock(self.address, T_HIGH_REGISTER, 2)

        if(highTempByte[0] == 0xFF and highTempByte[1] == 0xFF):
                return NAN

        if (extendedMode):
                digitalTemp = ((highTempByte[0]) << 5) | (highTempByte[1] >> 3)
                if(digitalTemp > 0xFFF):
                        digitalTemp |= 0xE000
        else:
                digitalTemp = ((highTempByte[0]) << 4) | (highTempByte[1] >> 4)
                if(digitalTemp > 0x7FF):
                        digitalTemp |= 0xF000
                        
        return digitalTemp*0.0625


    def read_low_temp_f(self):
        """
        Reads T_LOW register in F
        """
        return self.read_low_temp_c()*9.0/5.0 + 32.0
        
    def read_high_temp_f(self):
        """
        Reads T_HIGH register in F		
        """
        
        return self.read_high_temp_c()*9.0/5.0 + 32.0



            
        
    def set_alert_mode(self, mode):
        """
	// Set Alert type
        // 0 - Comparator Mode: Active from temp > T_HIGH until temp < T_LOW
        // 1 - Thermostat Mode: Active when temp > T_HIGH until any read operation occurs
	
        """
        configByte = self._i2c.readByte(self.address, CONFIG_REG)
        
        #Load new conversion rate
        configByte &= 0xFD            # Clear old TM bit (bit 1 of first byte)
        configByte |= mode<<1	        # Shift in new TM bit

        self._i2c.writeByte(self.address, CONFIG_REG, configByte)


















    def set_faults(self, faults):
        """
	    Set the number of consecutive faults
	    1 - 1 fault
	    2 - 2 faults
	    3 - 3 faults
	    4 - 4 faults
        """
        if (faults > 4) or (faults < 1):
            return NaN
        faults = faults - 1 # consecutive faults value is stored in just 2 bits in the config reg,
        # so we must convert from "human readable" ints 1-4 to stored values (0-3).
        configReg = self.read_config()
        configBit_11 = self.bit_read(faults, 0)
        configBit_12 = self.bit_read(faults, 1)
        configReg = self.bit_write(configReg, AS6212_CONFIG_BIT_CONSECUTIVE_FAULTS_0, configBit_11)
        configReg = self.bit_write(configReg, AS6212_CONFIG_BIT_CONSECUTIVE_FAULTS_1, configBit_12)
        self.setConfig(configReg)


    def bit_read(self, value, bit):
        return (((value) >> (bit)) & 0x01)

    def bit_set(self, value, bit):
        return ((value) | (1 << (bit)))

    def bit_clear(self, value, bit):
        return ((value) & ~(1 << (bit)))

    def bit_write(self, value, bit, bitvalue):
        if (bitvalue == 1):
            return self.bit_set(value, bit)
        if (bitvalue == 0):
            return self.bit_clear(value, bit)
    
    def print_config_binary(self):
        configReg = self.read_config()
        print("\nConfig: ")
        configBin = ""
        for i in range(15, -1, -1):
            configBin += str(self.bit_read(configReg, i))
            if i == 8:
                configBin += " "
        print(configBin)

    def print_binary(self, data):
        configReg = self.read_config()
        print("\ndata: ")
        dataBin = ""
        for i in range(15, -1, -1):
            dataBin += str(self.bit_read(data, i))
            if i == 8:
                dataBin += " "
        print(dataBin)        
        



 
