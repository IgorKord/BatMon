# Protocol Switching Implementation Summary

## Changes Implemented in Simulator.cs

### 1. ASCII_button_CheckedChanged (Line ~1554)
**Purpose**: Switch from DNP or Modbus to ASCII protocol

**Implementation**:
- **From DNP**: Sends DNP control point 48 (0x30) via `DNP_ControlPoint_Operate()`
  - Control point 48 switches to ASCII temporarily (not saved to EEPROM)
  - Based on firmware DNP.c lines 1490-1525

- **From Modbus**: Writes value 3 (ASCII_CMDS) to register 48 (0x30) via `Modbus_WriteRegister()`
  - Register 48 switches to ASCII temporarily (not saved to EEPROM)
  - Based on firmware main.c lines 2445-2458

### 2. DNPButton_CheckedChanged (Line ~1612)
**Purpose**: Switch from ASCII or Modbus to DNP3 protocol

**Implementation**:
- **From ASCII**: Sends ASCII command `prot>dnp3\r` via serial port
  - Saves to EEPROM
  - Based on firmware ASCII_commands.c lines 1338-1405

- **From Modbus**: Writes value 1 (DNP3) to register 51 (0x33) via `Modbus_WriteRegister()`
  - Register 51 switches to DNP3 and saves to EEPROM
  - Based on firmware main.c lines 2445-2458

### 3. ModbusButton_CheckedChanged (Line ~1668)
**Purpose**: Switch from ASCII or DNP to Modbus protocol

**Implementation**:
- **From ASCII**: Sends ASCII command `prot>modb\r` via serial port
  - Saves to EEPROM
  - Based on firmware ASCII_commands.c lines 1338-1405

- **From DNP**: Sends DNP control point 50 (0x32) via `DNP_ControlPoint_Operate()`
  - Control point 50 switches to MODBUS and saves to EEPROM
  - Based on firmware DNP.c lines 1490-1525

## New Helper Functions Added (After line ~870)

### DNP_ControlPoint_Operate()
**Purpose**: Send DNP Direct Operate command for control points

**Parameters**:
- `unit_addr`: Meter address
- `control_point`: Control point number (48, 49, or 50)
- `value`: Control value (typically 1)

**Message Structure**:
- Function Code: 0x05 (Direct Operate)
- Object: Group 12 (Binary Output)
- Variation: 1 (CROB - Control Relay Output Block)
- Includes proper DNP framing with CRC

**Control Points**:
- Point 48 (0x30) → ASCII (temporary)
- Point 49 (0x31) → SETUP (temporary)
- Point 50 (0x32) → MODBUS (saves to EEPROM)

### Modbus_WriteRegister()
**Purpose**: Send Modbus Write Single Register command

**Parameters**:
- `meter_addr`: Meter address
- `register_addr`: Register address (48 or 51)
- `value`: Protocol code (1=DNP3, 2=MODBUS, 3=ASCII)

**Message Structure**:
- Function Code: 0x06 (Write Single Register)
- Includes Modbus CRC

**Registers**:
- Register 48 (0x30) → ASCII (temporary)
- Register 51 (0x33) → DNP3 (saves to EEPROM)

## Protocol Persistence to EEPROM

| Protocol | Saved to EEPROM? |
|----------|------------------|
| ASCII    | ❌ NO - Temporary only (production/testing mode) |
| SETUP    | ❌ NO - Temporary only (startup/configuration mode) |
| DNP3     | ✅ YES - Normal customer protocol |
| MODBUS   | ✅ YES - Normal customer protocol |

## Firmware Protocol Values

Per firmware enums in GLOBALS.H:
```c
SETUP = 0x00
DNP3 = 0x01
MODBUS = 0x02
ASCII_CMDS = 0x03
ASCII_MENU = 0x04
```

## Testing Recommendations

1. **ASCII → DNP**: Test with meter address 2, should send DNP control point command
2. **ASCII → Modbus**: Test with meter address 2, should send ASCII command
3. **DNP → ASCII**: Test DNP control point 48 operation
4. **DNP → Modbus**: Test DNP control point 50 operation
5. **Modbus → ASCII**: Test Modbus write register 48
6. **Modbus → DNP**: Test Modbus write register 51
7. **Verify EEPROM**: Confirm DNP3/MODBUS save to EEPROM, ASCII/SETUP do not

## Notes

- All protocol switches include 100ms delay (`Tools.msSleep(100)`) for firmware processing
- Protocol is only switched if `monitor_port_open == true`
- Protocol variable is updated AFTER command is sent to firmware
- Commands are protocol-specific - correct command type is selected based on current protocol
