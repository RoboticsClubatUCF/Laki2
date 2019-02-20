#!/usr/bin/env python3

from enum import Enum

#Note that some bullshit means TI uses a different floating point format from the rest of the world
#8 bits exponent, 1 bit sign, 23 bits mantissa
#Exponent is converted to a number as 2^(expo-128-24), multiply by sign, set bit 24 of mantissa to 1

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
#Write block offset (offset/32) to DATAFLASHBLOCK
#Read/Write data from within BLOCKDATA
#To finalize a write, write the new checksum of the block to BLOCKDATACHECKSUM, TI says checksum can be recomputed from the old checksum and the new data (see page 22 of the datasheet)
#Checksum is 255- the 8 bit sum of the block
#Reset gauge by writing reset command to control register
#Gauge will reseal if previously sealed, otherwise seal the gauge if you dare
#TI says the block memory can be read/written out of bounds because that never causes security holes

class DataFlashFields(Enum):
    #Format is a tuple of (subclass ID, offset, struct format)
    #SAFETY
    OTCHARGE = (2, 0, '>h') #Charge overtemperature threshold (0.1 degC, 0-1200)
    OTCHARGETIME = (2, 2, '>B') #Charge overtemperature trip time (Set to 0 to disable) (Seconds, 0-60)
    OTCHARGERECOVERY = (2, 3, '>h') #Threshold to clear charge overtemperature flag when under (0.1 degC, 0-1200)
    OTDISCHARGE = (2, 5, '>h') #Discharge overtemperature threshold (0.1 degC, 0-1200)
    OTDISCHARGETIME = (2, 7, '>B') #Discharge overtemperature trip time (Set to 0 to disable)(Seconds, 0-60)
    OTDISCHARGERECOVERY = (2, 8, '>h') #Threshold to clear discharge overtemperature (0.1 degC, 0-1200)
    #CHARGE INHIBIT CONFIG
    CHARGEINHIBITTEMPLOW = (32, 0, '>h') #(0.1 degC, -400 to 1200)
    CHARGEINHIBITTEMPHIGH = (32, 2, '>h') #(0.1 degC, -400 to 1200)
    CHARGEINHIBITHYSTERESIS = (32, 4, '>h') #(0.1 degC, 0 to 100)
    #CHARGE
    SUSPENDLOWTEMP = (34, 0, '>h') #(0.1 degC, -400 to -50)
    SUSPENDHIGHTEMP = (34, 2, '>h') #(0.1 degC, -400 to 550)
    PBEFFEFFICIENCY = (34, 4, '>B') #(Percent. 0 to 100)
    PBTEMPCOEFFICIENT = (34, 5, '4') #(Percent, 0 to 0.07812, 32 bit float)
    PBDROPOFFPERCENT = (34, 9, '>B') #(Percent, 0 to 100)
    PBREDUCTIONRATE = (34, 10, '4') #(Percent, 0 to 1.25, 32 bit float)
    #CHARGE TERMINATION
    TAPERCURRENT = (36, 0, '>h') #(mA, 0 to 1000)
    MINTAPERCAPACITY = (36, 2, '>h') #(mAh, 0 to 1000)
    CELLTAPERVOLTAGE = (36, 4, '>h') #(mV, 0 to 1000)
    CURRENTTAPERWINDOW = (36, 6, '>B') #(Seconds, 0 to 60)
    TCASET = (36, 7, '>b') #(Percent, -1 to 100)
    TCACLEAR = (36, 8, '>b') #(Percent, -1 to 100)
    FCSET = (36, 9, '>b') #(Percent, -1 to 100)
    FCCLEAR = (36, 10, '>b') #(Percent, -1 to 100)
    DODATEOCDELTAT = (36, 11, '>h') #(0.1 degC, 0 to 1000)
    NIMHDELTATEMP = (36, 13, '>h') #(0.1 degC, 0 to 255)
    NIMHDELTATEMPTIME = (36, 15, '>H') #(Seconds, 0 to 1000)
    NIMHHOLDOFFTIME = (36, 17, '>H') #(Seconds, 0 to 1000)
    NIMHHOLDOFFCURRENT = (36, 19, '>h') #(mA, 0 to 32000)
    NIMHHOLDOFFTEMP = (36, 21, '>h') #(0.1 degC, 0 to 1000)
    NIMHCELLNEGATIVEDELTAVOLT = (36, 23, '>B') #(mV, 0 to 100)
    NIMHCELLNEGATIVEDELTATIME = (36, 24, '>B') #(Seconds, 0 to 255)
    NIMHNEGATIVEDELTAQUALVOLT = (36, 25, '>h') #(mV, 0 to 32767)
    #DATA
    MANUFACTUREDATE = (48, 2, '>H') #(Day + Mo*32 + (Yr-1980)*256, 0 to 65535)
    SERIALNUMBER = (48, 4, '>H') #(hex, 0 to 0xFFFF)
    CYCLECOUNT = (48, 6, '>H') #(counts, 0 to 65535)
    CCTHRESHOLD = (48, 8, '>h') #(mAh, 100 to 32767) Threshold for incrementing cycle count on discharge
    MAXERRORLIMIT = (48, 10, '>B') #(Percent, 0 to 100)
    DESIGNCAPACITY = (48, 11, '>h') #(mAh, 0 to 32767) 
    DESIGNENERGY = (48, 13, '>h') #(mWh, 0 to 32767)
    SOHLOADI = (48, 15, '>h') #(mA, -32767 to -400)
    CELLCHARGEVOLTAGET1T2 = (48, 17, '>H') #(mV, 0 to 4600)
    CELLCHARGEVOLTAGET2T3 = (48, 19, '>H') #(mV, 0 to 4600)
    CELLCHARGEVOLTAGET3T4 = (48, 21, '>H') #(mV, 0 to 4600)
    CHARGECURRENTT1T2 = (48, 23, '>B') #(Percent, 0 to 100)
    CHARGECURRENTT2T3 = (48, 24, '>B') #(Percent, 0 to 100)
    CHARGECURRENTT3T4 = (48, 25, '>B') #(Percent, 0 to 100)
    JEITAT1 = (48, 26, '>b') #(deg C, -128 to 127)
    JEITAT2 = (48, 27, '>b') #(deg C, -128 to 127)
    JEITAT3 = (48, 28, '>b') #(deg C, -128 to 127)
    JEITAT4 = (48, 29, '>b') #(deg C, -128 to 127)
    DESIGNENERGYSCALE = (48, 30, '>B') #(number, 0 to 255)
    DEVICENAME = (48, 31, '11s') #(12 character string, "bq34z100-G1\0")
    MANUFACTURERNAME = (48, 43, '11s') #(12 character string, "Texas Inst.\0")
    DEVICECHEMISTRY = (48, 55, '4s') #(5 character string, "LION\0") Probably changes if you use the bqstudio tool to set chemistry
    #DISCHARGE
    SOC1SETTHESHOLD = (49, 0, '>H') #(mAh, 0 to 65535)
    SOC1CLEARTHRESHOLD = (49, 2, '>H') #(mAh, 0 to 65535)
    SOCFSETTHREHOLD = (49, 4, '>H') #(mAh, 0 to 65535)
    SOCFCLEARTHRESHOLD = (49, 6, '>H') #(mAh, 0 to 65535)
    CELLBLSETVOLTTHRESHOLD = (49, 8, '>h') #(mV, 0 to 5000)
    CELLBLSETVOLTTIME = (49, 10, '>B') #(seconds, 0 to 60)
    CELLBLCLEARVOLTTHRESHOLD = (49, 11, '>h') #(mV, 0 to 5000)
    CELLBHSETVOLTTHRESHOLD = (49, 13, '>h') #(mV, 0 to 5000)
    CELLBHVOLTTIME = (49, 15, '>B') #(seconds, 0 to 60)
    CELLBHCLEARVOLTTHRESHOLD = (49, 16, '>h') #(mV, 0 to 5000)
    CYCLEDELTA = (49, 21, '>B') #(0.01%, 0 to 255)
    #MANUFACTURER DATA
    PACKLOTCODE = (56, 0, '>H') #(hex, 0x0 to 0xFFFF)
    PCBLOTCODE = (56, 2, '>H') #(hex, 0x0 to 0xFFFF)
    FIRMWAREVERSION = (56, 4, '>H') #(hex, 0x0 to 0xFFFF)
    HARDWAREREVISION = (56, 6, '>H') #(hex, 0x0 to 0xFFFF)
    CELLREVISION = (56, 8, '>H') #(hex, 0x0 to 0xFFFF)
    DFCONFIGREVISION = (56, 10, '>H') #(hex, 0x0 to 0xFFFF)
    #LIFETIME DATA
    LIFETIMEMAXTEMP = (59, 0, '>h') #(0.1 degC, 0 to 1400)
    LIFETIMEMINTEMP = (59, 2, '>h') #(0.1 degC, -600 to 1400)
    LIFETIMEMAXCHARGECURRENT = (59, 4, '>h') #(mA, -32767 to 32767)
    LIFETIMEMAXDISCHARGECURRENT = (59, 6, '>h') #(mA, -32767 to 32767)
    LIFETIMEMAXPACKVOLTAGE = (59, 8, '>H') #(20 mV, 0 to 65535)
    LIFETIMEMINPACKVOLTAGE = (59, 10, '>H') #(20 mV, 0 to 65535)
    #LIFETIME TEMP SAMPLES
    LTFLASHCOUNT = (60, 0, '>H') #(counts, 0 to 65535)
    #REGISTERS
    PACKCONFIGURATION = (64, 0, '>H') #(2 bytes flags)
    #RESCAP, CAL_EN, SCALED, RSVD, VOLTSEL (Internal divider when 0), IWAKE, RSNS1, RSNS0, SLEEP, RMFCC, NiDT, NiDV, QPCCLEAR, GNDSEL (Chooses ground reference between pin VSS and 10), TEMPS (Internal temp when 0)

    PACKCONFIGURATIONB = (64, 2, 'B') #(1 byte flags)
    #CHarGe DOD EOC, RSVD, V consistency ENabled, RSVD, JEITA, LiFePO relax, DOD weighting (LiFePo), Fast convergence

    PACKCONFIGURATIONC = (64, 3, 'B') #(1 byte flags)
    #SOH display, RSOC Hold (during discharge, disable if smoothing enabled), Fast Filter near EDV, Fast sleep sampling, Keep RC and RSOC after 0, RELAX_JUMP_OK, RELAX_SMOOTH_OK, SMOOTH RSOC

    LEDCOMMCONFIGURATION = (64, 4, 'B') #(1 byte flags)
    #EXT_LED[3:0], LED_ON, LED_MODE[2:0]
    #LED_MODE: 0-No LEDS, 1-Single LED, 2-Four LEDs, 3-External LEDS w/I2C, 4-External LEDS w/HDQ
    #LED_ON locks display on in modes 2-4, LED_SOH selects between RSOC and SOH
    #EXT_LED is set to number of LEDs - 1

    ALERTCONFIGURATION = (64, 5, 'H') #(2 bytes flags)
    NUMBERSERIESCELLS = (64, 7, '>B') #(number, 0 to 100)
    #LIFETIME RESOLUTION
    LTTEMPRES = (66, 0, '>B') #(0.1 degC, 0 to 255)
    LTCURRES = (66, 1, '>B') #(mA, 0 to 255)
    LTVOLTAGERES = (66, 2, '>B') #(20 mV, 0 to 255)
    LTUPDATETIME = (66, 3, '>H') #(seconds, 0 to 65535)
    #LED DISPLAY
    LEDHOLDTIME = (67, 0, '>B') #(number, 0 to 255)
    #POWER
    FLASHUPDATEOKCELLVOLT = (68, 0, '>h') #(mV, 0 to 4200) 2800mV * number of cells * 5000/Voltage divider
    SLEEPCURRENT = (68, 2, '>h') #(mA, 0 to 100)
    FSWAIT = (68, 11, '>B') #(seconds, 0 to 255)
    #MANUFACTURER INFO, 32 bytes of whatever you want I guess
    MANUFACTUERERINFOSTART = (58, 0, '32c')
    #IT CONFIG
    LOADSELECT = (80, 0, '>B') #(number, 0 to 255) 
    LOADMODE = (80, 1, '>B') #(number, 0 to 255) 0 is constant current, 1 is constant power
    RESCURRENT = (80, 10, '>h') #(mA, 0 to 1000)
    MAXRESFACTOR = (80, 14, '>B') #(number, 0 to 255)
    MINRESFACTOR = (80, 15, '>B') #(number, 0 to 255)
    RAFILTER = (80, 17, '>h') #(number, 0 to 1000)
    MINPASSEDCHARGENIMHLA1STQMAX = (80, 47) #(percent, 0 to 100) a real mouthfull
    MAXQMAXCHANGE = (80, 49, 'B') #(percent, 0 to 255)
    CELLTERMINATEVOLTAGE = (80, 53, '>h') #(mV, 1000 to 3700)
    CELLTERMVDELTA = (80, 55, '>h') #(mV, 0 to 4200)
    RESRELAXTIME = (80, 58) #(seconds, 0 to 65534)
    USERRATEMA = (80, 62, '>h') #(mA, -32767 to 32767)
    USERRATEPWR = (80, 64, '>h') #(mW/cW, -32767 to 32767)
    RESERVECAPMAH = (80, 66, '>h') #(mAh, 0 to 9000)
    RESERVEENERGY = (80, 68, '>h') #(mWh/cWh, 0 to 14000)
    MAXSCALEBACKGRID = (80, 72, '>B') #(number, 0 to 15)
    CELLMINDELTAV = (80, 73, '>H') #(mV, 0 to 65535)
    RAMAXDELTA = (80, 75, '>B') #(percent, 0 to 255)
    DESIGNRESISTANCE = (80, 76, '>h') #(milliOhm, 1 to 32767)
    REFERENCEGRID = (80, 78, '>B') #(0 to 14) what does this mean
    QMAXDELTAPERCENT = (80, 79, '>B') #(mAh, 0 to 100)
    MAXRESSCALE = (80, 80, '>H') #(number, 0 to 32767)
    MINRESSCALE = (80, 82, '>H') #(number, 0 to 32767)
    FASTSCALESTARTSOC = (80, 84, '>B') #(percent, 0 to 100)
    CHARGEHYSVSHIFT = (80, 89, '>h') #(mV, 0 to 2000)
    SMOOTHRELAXTIME = (80, 91, '>h') #(seconds, 1 to 32767)
    #CURRENT THRESHOLDS
    DISCHARGECURRENTTHRESHOLD = (81, 0, '>h') #(mA, 0 to 2000)
    CHARGECURRENTTHRESHOLD = (81, 2, '>h') #(mA, 0 to 2000)
    QUITCURRENT = (81, 4, '>h') #(mA, 0 to 1000)
    DISCHARGERELAXTIME = (81, 6, '>H') #(seconds, 0 to 8191)
    CHARGERELAXTIME = (81, 8, '>B') #(seconds, 0 to 255)
    CELLMAXIRCORRECT = (81, 9, '>H') #(mV, 0 to 1000)
    #STATE
    QMAXCELL0 = (82, 0 , '>h') #(mAh, 0 to 32767)
    STATECYCLECOUNT = (82, 2, '>H') #(number, 0 to 65535)
    UPDATESTATUS = (82, 4, '>B') #(number, 0 to 6)
    CELLVATCHARGETERM = (82, 5, '>h') #(mV, 0 to 5000)
    AVGILASTRUN = (82, 7, '>h') #(mA, -32767 to 32767)
    AVGPLASTRUN = (82, 9, '>h') #(mW, -32767 to 32767)
    CELLDELTAVOLTAGE = (82, 11, '>h') #(mV, -32767 to 32767)
    TRISE = (82, 13, 'h') #(number, 0 to 32767)
    TTIMECONSTANT = (82, 15, '>h') #(number, 0 to 32767)
    #RA TABLE, don't touch
    RA0FLAG = (88, 0, '>H') #(Hex, 0x0 to 0xFFFF)
    RA0 = (88, 2, '>15h') #(Lots of numbers)
    RA0XFLAG = (89, 0, '>H') #(hex, 0x0 to 0xFFFF)
    RA0X = (89, 2, '>15h') #(Lots of numbers)
    #CALIBRATION DATA
    CCGAIN = (104, 0, '4') #(milliSeiverts, 1e-1 to 4e1, 32 bit float) Value is 4.768/(sense resistance mOhms), might be not exact due to calibration steps
    CCDELTA = (104, 4, '4') #(milliSeiverts, 2.98e4 to 1.19e6, 32 bit float) Value is 5677445/(sense resistance mOhms), might not be exact due to calibration steps
    CCOFFSET = (104, 8, '>h') #(number, -32767 to 32767)
    BOARDOFFSET = (104, 10, '>b') #(number, -128 to 127)
    INTTEMPOFFSET = (104, 11, '>b') #(0.1 degC, -128 to 127)
    EXTTEMPOFFSET = (104, 12, '>b') #(0.1 degC, -128 to 127)
    VOLTAGEDIVIDER = (104, 14, '>H') #(mV, 0 to 65535) Battery voltage when input is 900mV (adjust to calibrate voltage)
    #CALIBRATION CURRENT
    DEADBAND = (107, 1, '>B') #(mA, 0 to 255)
    #SECURITY
    SEALEDTOUNSEALED = (112, 0, '>I') #(hex, 0 to 0xFFFF FFFF, default 0x36720414)
    UNSEALEDTOFULL = (112, 4, '>I') #(hex, 0 to 0xFFFF FFFF, default 0xFFFF FFFF)
    AUTHKEY3 = (112, 8, '>I') #(hex, 0 to 0xFFFF FFFF, default 0x1234 5678)
    AUTHKEY2 = (112, 12, '>I') #(hex, 0 to 0xFFFF FFFF, default 0x89AB CDEF)
    AUTHKEY1 = (112, 16, '>I') #(hex, 0 to 0xFFFF FFFF, default 0xFEDC BA98)
    AUTHKEY0 = (112, 20, '>I') #(hex, 0 to 0xFFFF FFFF, default 0x7654 3210)


