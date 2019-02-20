#!/usr/bin/env python3

from smbus2 import SMBus
from BQ34Z100Fields import *
import time, struct, math, sys
import numpy as np

def floatToXemicsF4(number): #Probably breaks with 0 and infinity
    exponent = int(math.floor(math.log2(abs(number))+1))-24 #Round towards -infinity
    mantissa = int(abs(number)/(2**exponent))&0x7FFFFF
    exponent = exponent + 128 + 24
    result = (exponent<<24) | mantissa | (int(number<0)<<23)
    return list(struct.pack('>L', result))
    

def xemicsF4ToFloat(bytes):
    unpackedbytes = struct.unpack('>L', bytearray(bytes))[0]
    exponent = ((unpackedbytes&0xFF000000)>>24)-128-24
    mantissa = (unpackedbytes&0x00FFFFFF)|0x800000
    sign = 1 if (unpackedbytes&0x800000 == 0) else -1
    result = mantissa * (2**exponent) * sign
    return result

def testFloatConversion():
    failed = False    
    for x in np.linspace(-18, 18, 4*36+1):
        if abs(2**x-xemicsF4ToFloat(floatToXemicsF4(2**x)))>(2**(-23+abs(x))):
            print(str(2**x) + " failed with result of " + str(xemicsF4ToFloat(floatToXemicsF4(2**x))))
            failed = True
    return failed
    
testFloatConversion()

def writeDataFlashField(i2cbus, field, value):
    i2cbus.write_byte_data(0x55, ExtendedCommands.BLOCKDATACONTROL.value, 0)
    i2cbus.write_byte_data(0x55, ExtendedCommands.DATAFLASHCLASS.value, field.value[0])
    i2cbus.write_byte_data(0x55, ExtendedCommands.DATAFLASHBLOCK.value, field.value[1]//32)

    size = None
    if(field.value[2] != '4'):
        size = struct.calcsize(field.value[2])
    else:
        size = 4
    
    time.sleep(0.0002)
    oldchecksum = i2cbus.read_byte_data(0x55, ExtendedCommands.BLOCKDATACHECKSUM.value)
    oldvalue = i2cbus.read_i2c_block_data(0x55, ExtendedCommands.BLOCKDATA.value + (field.value[1]%32), size)
    

    newvalue = None

    if(field.value[2] != '4'):
        newvalue = struct.pack(field.value[2], value)
    else:
        pass
    
    newchecksum = None
    quicksum = True
    if(quicksum):
        temp = (255-oldchecksum-(sum(oldvalue)%256))%256
        newchecksum = 255 - (temp+sum(newvalue))%256

    i2cbus.write_i2c_block_data(0x55, ExtendedCommands.BLOCKDATA.value + field.value[1]%32, list(newvalue))
    i2cbus.write_byte_data(0x55, ExtendedCommands.BLOCKDATACHECKSUM.value, newchecksum)
    time.sleep(0.001)

def readDataFlashField(i2cbus, field):
    i2cbus.write_byte_data(0x55, ExtendedCommands.BLOCKDATACONTROL.value, 0)
    i2cbus.write_byte_data(0x55, ExtendedCommands.DATAFLASHCLASS.value, field.value[0])
    i2cbus.write_byte_data(0x55, ExtendedCommands.DATAFLASHBLOCK.value, field.value[1]//32)

    size = None
    if(field.value[2] != '4'):
        size = struct.calcsize(field.value[2])
    else:
        size = 4
    
    time.sleep(0.0002)
    value = i2cbus.read_i2c_block_data(0x55, ExtendedCommands.BLOCKDATA.value + (field.value[1]%32), size)
    return struct.unpack(field.value[2], bytearray(value))[0]

def resetGauge(i2cbus):
    i2cbus.write_i2c_block_data(0x55, StandardCommands.CONTROL.value, [0x00, 0x41])

bus = SMBus(1)

print("Set design capacity: ")
writeDataFlashField(bus, DataFlashFields.DESIGNCAPACITY, 20000)
print("Set design energy scale: ")
writeDataFlashField(bus, DataFlashFields.DESIGNENERGYSCALE, 10)
print("Set Design Energy: ")
writeDataFlashField(bus, DataFlashFields.DESIGNENERGY, 31428)
print("Set series cells: ")
writeDataFlashField(bus, DataFlashFields.NUMBERSERIESCELLS, 6)
print("Set divider: ")
writeDataFlashField(bus, DataFlashFields.VOLTAGEDIVIDER, 28962)
print("Set pack config: ")
writeDataFlashField(bus, DataFlashFields.PACKCONFIGURATION, 0b0010100111100001)
print("Set LED config: ")
writeDataFlashField(bus, DataFlashFields.LEDCOMMCONFIGURATION, 0b01001011)
resetGauge(bus)
print("Design capacty: " + str(readDataFlashField(bus, DataFlashFields.DESIGNCAPACITY)))
print("Design energy scale: " + str(readDataFlashField(bus, DataFlashFields.DESIGNENERGYSCALE)))
print("Design energy: " + str(readDataFlashField(bus, DataFlashFields.DESIGNENERGY)))
print("Series cells: " + str(readDataFlashField(bus, DataFlashFields.NUMBERSERIESCELLS)))
print("Divider: " + str(readDataFlashField(bus, DataFlashFields.VOLTAGEDIVIDER)))

while True:
    b = bus.read_i2c_block_data(0x55, StandardCommands.VOLTAGE.value, 2)
    print(struct.unpack('H', bytearray(b))[0], end="\r")
    time.sleep(0.05)
