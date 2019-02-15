#!/usr/bin/env python3

from enum import Enum

class StandardCommands(Enum):
    CONTROL = 0x00 #R/W, Control register, has subcommands
    STATEOFCHARGE = 0x02 #R, Percent SOC, 0 to 100
    MAXERROR = 0x03 #R, Percent margin of SOC
    REMAININGCAPACITY = 0x04 #R, mAH remaining (scaled)
    FULLCHARGECAPACITY = 0x06 #R, mAH when full (scaled)
    VOLTAGE = 0x08 #R, unsigned mV
    AVERAGECURRENT = 0x0A #R, signed mA (scaled)
    TEMPERATURE = 0x0C #R, Temperature from chosen source in 0.1K
    FLAGS = 0x0E #R, Error/status flags
    FLAGSB = 0x12 #R, More error/status flags
    CURRENT = 0x10 #R, signed mA (scaled)

class ExtendedCommands(Enum):
    ATIMETOEMPTY = 0x18 #R, remaining discharge time at AVERAGECURRENT amps in minutes, 65535 if not discharging
    ATIMETOFULL = 0x1A #R, remining charge time at AVERAGECURRENT amps in minutes, 65535 if not charging
    PASSEDCHARGE = 0x1C #R, mAH since last "IT simulation"
    DOD0TIME = 0x1E #R, minutes since last Depth-Of-Discharge0 update
    AVAILABLEENERGY = 0x24 #R, (10?)mWH ewmaining in battery
    AVERAGEPOWER = 0x26 #R, unsigned (10?)mW average power of current discharge
    SERIALNUMBER = 0x28 #R, programmed serial number
    INTERNALTEMPERATURE = 0x2A #R, internal temperature of device in 0.1K
    CYCLECOUNT = 0x2C #R, Number of cycles battery has experienced
    STATEOFHEALTH = 0x2E #R, Full charge capacity as percentage of design capacity
    CHARGEVOLTAGE = 0x30 #R, Recommended charging voltage based on temperature
    CHARGECURRENT = 0x32 #R, Recommended charging current based on temperature
    PACKCONFIGURATION = 0x3A #R, "Selected features of the device pertaining to various features"
    DESIGNCAPACITY = 0x3C #R, Design capacity in mAH
    DATAFLASHCLASS = 0x3E #W, Sets data flash "class" to be accessed (UNSEALED ONLY)
    DATAFLASHBLOCK = 0x3F #W, 0x00 sets authentication data, 0x01 sets manufacturer data
    BLOCKDATA = 0x40 #R/W, Read and write current data flash block
    BLOCKDATACHECKSUM = 0x60 #R/W, Checksum of data read/written to data flash block
    AUTHENTICATEDATA = 0x40 #R/W, challenge/response data for authentication
    AUTHENTICATECHECKSUM = 0x54 #R/W, checksum of authentication data written to device
    BLOCKDATACONTROL = 0x61 #R/W, 0x00 sets normal access, 0x01 sets SEALED mode operation
    GRIDNUMBER = 0x62 #R, "active grid point"
    LEARNEDSTATUS = 0x63 #R, enabled/disabled status of QMax and Impedence Track
    DODEOC = 0x64 #R, Depth of Discharge at end of charge
    QSTART = 0x66 #R, IT calculated initial capacity mAH
    TRUERC = 0x68 #R, "True remaining capacity" mAH from IT without smoothing
    TRUEFCC = 0x6A #R, "True full charge" capacity mAH from IT without smoothing
    STATETIME = 0x6C #R, Seconds since last state change (CHARGE, DISCHARGE, REST)
    QMAXPASSEDQ = 0x6E #R, Capacity mAH since last Qmax DOD update
    DOD0 = 0x70 #R, Depth of discharge during last Open Circuit Voltage reading
    QMAXDOD0 = 0x72 #R, DOD0 to be used for next Qmax update of cell 1, only valid when VOK=1
    QMAXTIME = 0x74 #R, 16ths of an hour since last Qmax DOD update

#FLASH TRANSACTION PROCEDURE
#Unseal device
#Write 0x00 to BLOCKDATACONTROL
#Write subclass ID to DATAFLASHCLASS
#Write block offset (offset mod 32) to DATAFLASHBLOCK
#Read/Write data from within BLOCKDATA
#To finalize a write, write the new checksum of the block to BLOCKDATACHECKSUM, TI says checksum can be recomputed from the old checksum and the new data (see page 22 of the datasheet)
#Checksum is 255- the 8 bit sum of the block
#Reset gauge by writing reset command to control register
#Gauge will reseal if previously sealed, otherwise seal the gauge if you dare
#TI says the block memory can be read/written out of bounds because that never causes security holes

class DataFlashFields(Enum):
    #Format is a tuple of (subclass ID, offset)
    #SAFETY
    OTCHARGE = (2, 0) #Charge overtemperature threshold (0.1 degC, 0-1200)
    OTCHARGETIME = (2, 2) #Charge overtemperature trip time (Set to 0 to disable) (Seconds, 0-60)
    OTCHARGERECOVERY = (2, 3) #Threshold to clear charge overtemperature flag when under (0.1 degC, 0-1200)
    OTDISCHARGE = (2, 5) #Discharge overtemperature threshold (0.1 degC, 0-1200)
    OTDISCHARGETIME = (2, 7) #Discharge overtemperature trip time (Set to 0 to disable)(Seconds, 0-60)
    OTDISCHARGERECOVERY = (2, 8) #Threshold to clear discharge overtemperature (0.1 degC, 0-1200)
    #CHARGE INHIBIT CONFIG
    CHARGEINHIBITTEMPLOW = (32, 0) #(0.1 degC, -400 to 1200)
    CHARGEINHIBITTEMPHIGH = (32, 2) #(0.1 degC, -400 to 1200)
    CHARGEINHIBITHYSTERESIS = (32, 4) #(0.1 degC, 0 to 100)
    #CHARGE
    SUSPENDLOWTEMP = (34, 0) #(0.1 degC, -400 to -50)
    SUSPENDHIGHTEMP = (34, 2) #(0.1 degC, -400 to 550)
    PBEFFEFFICIENCY = (34, 4) #(Percent. 0 to 100)
    PBTEMPCOEFFICIENT = (34, 5) #(Percent, 0 to 0.07812, 32 bit float)
    PBDROPOFFPERCENT = (34, 9) #(Percent, 0 to 100)
    PBREDUCTIONRATE = (34, 10) #(Percent, 0 to 1.25, 32 bit float)
    #CHARGE TERMINATION
    TAPERCURRENT = (36, 0) #(mA, 0 to 1000)
    MINTAPERCAPACITY = (36, 2) #(mAh, 0 to 1000)
    CELLTAPERVOLTAGE = (36, 4) #(mV, 0 to 1000)
    CURRENTTAPERWINDOW = (36, 6) #(Seconds, 0 to 60)
    TCASET = (36, 7) #(Percent, -1 to 100)
    TCACLEAR = (36, 8) #(Percent, -1 to 100)
    FCSET = (36, 9) #(Percent, -1 to 100)
    FCCLEAR = (36, 10) #(Percent, -1 to 100)
    DODATEOCDELTAT = (36, 11) #(0.1 degC, 0 to 1000)
    NIMHDELTATEMP = (36, 13) #(0.1 degC, 0 to 255)
    NIMHDELTATEMPTIME = (36, 15) #(Seconds, 0 to 1000)
    NIMHHOLDOFFTIME = (36, 17) #(Seconds, 0 to 1000)
    NIMHHOLDOFFCURRENT = (36, 19) #(mA, 0 to 32000)
    NIMHHOLDOFFTEMP = (36, 21) #(0.1 degC, 0 to 1000)
    NIMHCELLNEGATIVEDELTAVOLT = (36, 23) #(mV, 0 to 100)
    NIMHCELLNEGATIVEDELTATIME = (36, 24) #(Seconds, 0 to 255)
    NIMHNEGATIVEDELTAQUALVOLT = (36, 25) #(mV, 0 to 32767)
    #DATA
    MANUFACTUREDATE = (48, 2) #(Day + Mo*32 + (Yr-1980)*256, 0 to 65535)
    SERIALNUMBER = (48, 4) #(hex, 0 to 0xFFFF)
    CYCLECOUNT = (48, 6) #(counts, 0 to 65535)
    CCTHRESHOLD = (48, 8) #(mAh, 100 to 32767) Threshold for incrementing cycle count on discharge
    MAXERRORLIMIT = (48, 10) #(Percent, 0 to 100)
    DESIGNCAPACITY = (48, 11) #(mAh, 0 to 32767) 
    DESIGNENERGY = (48, 13) #(mWh, 0 to 32767)
    SOHLOADI = (48, 15) #(mA, -32767 to -400)
    CELLCHARGEVOLTAGET1T2 = (48, 17) #(mV, 0 to 4600)
    CELLCHARGEVOLTAGET2T3 = (48, 19) #(mV, 0 to 4600)
    CELLCHARGEVOLTAGET3T4 = (48, 21) #(mV, 0 to 4600)
    CHARGECURRENTT1T2 = (48, 23) #(Percent, 0 to 100)
    CHARGECURRENTT2T3 = (48, 24) #(Percent, 0 to 100)
    CHARGECURRENTT3T4 = (48, 25) #(Percent, 0 to 100)
    JEITAT1 = (48, 26) #(deg C, -128 to 127)
    JEITAT2 = (48, 27) #(deg C, -128 to 127)
    JEITAT3 = (48, 28) #(deg C, -128 to 127)
    JEITAT4 = (48, 29) #(deg C, -128 to 127)
    DESIGNENERGYSCALE = (48, 30) #(number, 0 to 255)
    DEVICENAME = (48, 31) #(12 character string, "bq34z100-G1")
    MANUFACTURERNAME = (48, 43) #(12 character string, "Texas Inst.")
    DEVICECHEMISTRY = (48, 55) #(5 character string, "LION") Probably changes if you use the bqstudio tool to set chemistry
    #DISCHARGE
    SOC1SETTHESHOLD = (49, 0) #(mAh, 0 to 65535)
    SOC1CLEARTHRESHOLD = (49, 2) #(mAh, 0 to 65535)
    SOCFSETTHREHOLD = (49, 4) #(mAh, 0 to 65535)
    SOCFCLEARTHRESHOLD = (49, 6) #(mAh, 0 to 65535)
    CELLBLSETVOLTTHRESHOLD = (49, 8) #(mV, 0 to 5000)
    CELLBLSETVOLTTIME = (49, 10) #(seconds, 0 to 60)
    CELLBLCLEARVOLTTHRESHOLD = (49, 11) #(mV, 0 to 5000)
    CELLBHSETVOLTTHRESHOLD = (49, 13) #(mV, 0 to 5000)
    CELLBHVOLTTIME = (49, 15) #(seconds, 0 to 60)
    CELLBHCLEARVOLTTHRESHOLD = (49, 16) #(mV, 0 to 5000)
    CYCLEDELTA = (49, 21) #(0.01%, 0 to 255)
    #MANUFACTURER DATA
    PACKLOTCODE = (56, 0) #(hex, 0x0 to 0xFFFF)
    PCBLOTCODE = (56, 2) #(hex, 0x0 to 0xFFFF)
    FIRMWAREVERSION = (56, 4) #(hex, 0x0 to 0xFFFF)
    HARDWAREREVISION = (56, 6) #(hex, 0x0 to 0xFFFF)
    CELLREVISION = (56, 8) #(hex, 0x0 to 0xFFFF)
    DFCONFIGREVISION = (56, 10) #(hex, 0x0 to 0xFFFF)
    #LIFETIME DATA
    LIFETIMEMAXTEMP = (59, 0) #(0.1 degC, 0 to 1400)
    LIFETIMEMINTEMP = (59, 2) #(0.1 degC, -600 to 1400)
    LIFETIMEMAXCHARGECURRENT = (59, 4) #(mA, -32767 to 32767)
    LIFETIMEMAXDISCHARGECURRENT = (59, 6) #(mA, -32767 to 32767)
    LIFETIMEMAXPACKVOLTAGE = (59, 8) #(20 mV, 0 to 65535)
    LIFETIMEMINPACKVOLTAGE = (59, 10) #(20 mV, 0 to 65535)
    #LIFETIME TEMP SAMPLES
    LTFLASHCOUNT = (60, 0) #(counts, 0 to 65535)
    #REGISTERS
    PACKCONFIGURATION = (64, 0) #(2 bytes flags)
    PACKCONFIGURATIONB = (64, 2) #(1 byte flags)
    PACKCONFIGURATIONC = (64, 3) #(1 byte flags)
    LEDCOMMCONFIGURATION = (64, 4) #(1 byte flags)
    ALERTCONFIGURATION = (64, 5) #(2 bytes flags)
    NUMBERSERIESCELLS = (64, 7) #(number, 0 to 100)
    #LIFETIME RESOLUTION
    LTTEMPRES = (66, 0) #(0.1 degC, 0 to 255)
    LTCURRES = (66, 1) #(mA, 0 to 255)
    LTVOLTAGERES = (66, 2) #(20 mV, 0 to 255)
    LTUPDATETIME = (66, 3) #(seconds, 0 to 65535)
    #LED DISPLAY
    LEDHOLDTIME = (67, 0) #(number, 0 to 255)
    #POWER
    FLASHUPDATEOKCELLVOLT = (68, 0) #(mV, 0 to 4200)
    SLEEPCURRENT = (68, 2) #(mA, 0 to 100)
    FSWAIT = (68, 11) #(seconds, 0 to 255)
    #MANUFACTURER INFO, 32 bytes of whatever you want I guess
    MANUFACTUERERINFOSTART = (58, 0)
    #IT CONFIG
    LOADSELECT = (80, 0) #(number, 0 to 255) 
    LOADMODE = (80, 1) #(number, 0 to 255) 0 is constant current, 1 is constant power
    RESCURRENT = (80, 10) #(mA, 0 to 1000)
    MAXRESFACTOR = (80, 14) #(number, 0 to 255)
    MINRESFACTOR = (80, 15) #(number, 0 to 255)
    RAFILTER = (80, 17) #(number, 0 to 1000)
    MINPASSEDCHARGENIMHLA1STQMAX = (80, 47) #(percent, 0 to 100) a real mouthfull
    MAXQMAXCHANGE = (80, 49) #(percent, 0 to 255)
    CELLTERMINATEVOLTAGE = (80, 53) #(mV, 1000 to 3700)
    CELLTERMVDELTA = (80, 55) #(mV, 0 to 4200)
    RESRELAXTIME = (80, 58) #(seconds, 0 to 65534)
    USERRATEMA = (80, 62) #(mA, -32767 to 32767)
    USERRATEPWR = (80, 64) #(mW/cW, -32767 to 32767)
    RESERVECAPMAH = (80, 66) #(mAh, 0 to 9000)
    RESERVEENERGY = (80, 68) #(mWh/cWh, 0 to 14000)
    MAXSCALEBACKGRID = (80, 72) #(number, 0 to 15)
    CELLMINDELTAV = (80, 73) #(mV, 0 to 65535)
    RAMAXDELTA = (80, 75) #(percent, 0 to 255)
    DESIGNRESISTANCE = (80, 76) #(milliOhm, 1 to 32767)
    REFERENCEGRID = (80, 78) #(0 to 14) what does this mean
    QMAXDELTAPERCENT = (80, 79) #(mAh, 0 to 100)
    MAXRESSCALE = (80, 80) #(number, 0 to 32767)
    MINRESSCALE = (80, 82) #(number, 0 to 32767)
    FASTSCALESTARTSOC = (80, 84) #(percent, 0 to 100)
    CHARGEHYSVSHIFT = (80, 89) #(mV, 0 to 2000)
    SMOOTHRELAXTIME = (80, 91) #(seconds, 1 to 32767)
    #CURRENT THRESHOLDS
    DISCHARGECURRENTTHRESHOLD = (81, 0) #(mA, 0 to 2000)
    CHARGECURRENTTHRESHOLD = (81, 2) #(mA, 0 to 2000)
    QUITCURRENT = (81, 4) #(mA, 0 to 1000)
    DISCHARGERELAXTIME = (81, 6) #(seconds, 0 to 8191)
    CHARGERELAXTIME = (81, 8) #(seconds, 0 to 255)
    CELLMAXIRCORRECT = (81, 9) #(mV, 0 to 1000)
    #STATE
    QMAXCELL0 = (82, 0) #(mAh, 0 to 32767)
    CYCLECOUNT = (82, 2) #(number, 0 to 65535)
    UPDATESTATUS = (82, 4) #(number, 0 to 6)
    CELLVATCHARGETERM = (82, 5) #(mV, 0 to 5000)
    AVGILASTRUN = (82, 7) #(mA, -32767 to 32767)
    AVGPLASTRUN = (82, 9) #(mW, -32767 to 32767)
    CELLDELTAVOLTAGE = (82, 11) #(mV, -32767 to 32767)
    TRISE = (82, 13) #(number, 0 to 32767)
    TTIMECONSTANT = (82, 15) #(number, 0 to 32767)
    #RA TABLE, don't touch
    RA0FLAG = (88, 0) #(Hex, 0x0 to 0xFFFF)
    RA0 = (88, 2) #(Lots of numbers)
    RA0XFLAG = (89, 0) #(hex, 0x0 to 0xFFFF)
    RA0X = (89, 2) #(Lots of numbers)
    #CALIBRATION DATA
    CCGAIN = (104, 0) #(milliOhms, 1e-1 to 4e1, 32 bit float)
    CCDELTA = (104, 4) #(milliOhms, 2.98e4 to 1.19e6, 32 bit float)
    CCOFFSET = (104, 8) #(number, -32767 to 32767)
    BOARDOFFSET = (104, 10) #(number, -128 to 127)
    INTTEMPOFFSET = (104, 11) #(0.1 degC, -128 to 127)
    EXTTEMPOFFSET = (104, 12) #(0.1 degC, -128 to 127)
    VOLTAGEDIVIDER = (104, 14) #(mV, 0 to 65535)
    #CALIBRATION CURRENT
    DEADBAND = (107, 1) #(mA, 0 to 255)
    #SECURITY
    SEALEDTOUNSEALED = (112, 0) #(hex, 0 to 0xFFFF FFFF, default 0x36720414)
    UNSEALEDTOFULL = (112, 4) #(hex, 0 to 0xFFFF FFFF, default 0xFFFF FFFF)
    AUTHKEY3 = (112, 8) #(hex, 0 to 0xFFFF FFFF, default 0x1234 5678)
    AUTHKEY2 = (112, 12) #(hex, 0 to 0xFFFF FFFF, default 0x89AB CDEF)
    AUTHKEY1 = (112, 16) #(hex, 0 to 0xFFFF FFFF, default 0xFEDC BA98)
    AUTHKEY0 = (112, 20) #(hex, 0 to 0xFFFF FFFF, default 0x7654 3210)


