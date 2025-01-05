# DEYE Inverter Communication Project

## Project Overview
ESP32-based communication system for DEYE Inverter with the following architecture:
```
[this:ESP32]--UART-->[ESP32 AT-System]--WIFI-->[Deye 600/800 Inverter]
    ||
   SPI
    ||
[TFT_DISPLAY]
```

## Version
Current Version: 0.5.3

## System Components
- ESP32-S or WROOM (tested with ESP32-D0WD-V3 revision v3.0)
- ESP32 with AT-Firmware
- TFT Display (240x320)
- WIFI Connection
- Deye SUN300/600/800-G3-EU230 Inverter

## Register Reading Commands
| Parameter                  | Read_CMD | RegAdd | NumOfReg | CRC16 |
|---------------------------|----------|---------|----------|--------|
| Active Power Regulation   | 0103     | 0028    | 0001     | 0402  |
| Radiator Temperature      | 0103     | 005A    | 0001     | A419  |
| Current Power             | 0103     | 0056    | 0001     | 641A  |
| Daily Yield               | 0103     | 003C    | 0001     | 4406  |
| Total Yield               | 0103     | 003F    | 0002     | F407  |

## Version History

### v0.5.3
- Added ESP32 Logo

### v0.5.1
- Added APR Write function:
  - Testing toggle between 2% and 100%
  - Using additional button to force update
- Added Current measurement HW comm SW (not integrated)
- Added Version info in display
- Added Runtime

## TODO List
### Priority 1
- Documentation
  - Comments
  - AT-Command Structure
  - ESP32 AT-CMDs
  - Register addresses

### Priority 2
- Error notification if inverter is offline (Done: tbd use ping method later)
- Refine function and print structure
- Add other Button inputs
- CRC16 Modbus calculation

### Priority 3
- Move WIFI information to external header
- Update wifi state, notify if connection not possible

### Additional Tasks
- Add Date and Time to inform latest update
- Check date and update (not needed if current measurement is implemented)

## Next Development Steps
- Add Current Measurement
- Add adapt APR according to Current (power) measurement

## To Be Tested
- Current power and yield range (pending sun conditions)

## Command Structure Documentation

### Read Commands
```
DATAGRAM    := ATCMD + MODBUSLEN + SEPERATOR + MODBUSMSG + MODBUSCRC + NEWLINE
ATCMD       := AT+INVDATA=
MODBUSLEN   := len(MODBUSMSG + MODBUSCRC)
MODBUSCRC   := crc(MODBUSMSG)
SEPERATOR   := ,
MODBUSMSG   := SLAVE + FCODE + STARTADDR + REGSIZE
SLAVE       := 01
FCODE       := 03
STARTADDR   := FFFF
REGSIZE     := 0001
VALUELEN    := len(VALUE)
VALUE       := 0000
NEWLINE     := \n
```

### Write Commands
```
DATAGRAM    := ATCMD + MODBUSLEN + SEPERATOR + MODBUSMSG + MODBUSCRC + NEWLINE
ATCMD       := AT+INVDATA=
MODBUSLEN   := len(MODBUSMSG + MODBUSCRC)
MODBUSCRC   := crc(MODBUSMSG)
SEPERATOR   := ,
MODBUSMSG   := SLAVE + FCODE + STARTADDR + REGSIZE + VALUELEN + VALUE
SLAVE       := 01
FCODE       := 10
STARTADDR   := FFFF
REGSIZE     := 0001
VALUELEN    := len(VALUE)
VALUE       := 0000
NEWLINE     := \n
```
