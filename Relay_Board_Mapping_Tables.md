# Relay Board (ATmega88PA-AU) - Pin Mapping Tables

## Table 1: Display_Info.Relays_state Mapping

| Bit # in Display_Info.Relays_state | Mapped to Relay K# (or pulse) | Activates Port Name, Bit # | ATmega88PA-AU Pin # (TQFP-32) |
|:---------------------------------:|:-----------------------------:|:-------------------------:|:-----------------------------:|
| 0 | K1 (Plus Ground Fault) | Port D, Bit 2 | Pin 32 |
| 1 | K2 (Minus Ground Fault) | Port B, Bit 0 | Pin 12 |
| 2 | K3 (High Battery Fault) | Port B, Bit 2 | Pin 14 |
| 3 | K4 (Low Bat/AC Loss Fault) | Port B, Bit 1 | Pin 13 |
| 4 | BB_PULSE (Excitation Pulse) | Port C, Bit 3 | Pin 26 |
| 5 | (unused) | - | - |
| 6 | (unused) | - | - |
| 7 | (unused) | - | - |

**Notes:**
- K1, K2, K3 are Normally Open relays
- K4 is a Normally Closed relay (inverse logic)
- BB_PULSE is used for ground fault detection excitation

---

## Table 2: Display_Info.ExtLED_state Mapping

| Bit # in Display_Info.ExtLED_state | Mapped to LED # (Description) | Activates Port Name, Bit # | ATmega88PA-AU Pin # (TQFP-32) |
|:----------------------------------:|:-----------------------------:|:-------------------------:|:-----------------------------:|
| 0 | LED0 (+Ground Fault) | Port D, Bit 0 | Pin 30 |
| 1 | LED1 (-Ground Fault) | Port D, Bit 1 | Pin 31 |
| 2 | (used by K1 relay) | Port D, Bit 2 | Pin 32 |
| 3 | LED4 (Ripple V/I, HiZ) | Port D, Bit 3 | Pin 1 |
| 4 | LED5 (AC Power Loss) | Port D, Bit 4 | Pin 2 |
| 5 | LED3 (Low Battery) | Port D, Bit 5 | Pin 9 |
| 6 | LED2 (High Battery) | Port D, Bit 6 | Pin 10 |
| 7 | (unused) | Port D, Bit 7 | Pin 11 |

**Notes:**
- All LEDs are active high
- LEDs are connected via J1 connector (8-pin socket on back of relay board)
- The LED ordering matches the physical top-to-bottom order on the annunciator panel
- ExtLED_state byte is directly applied to Port D (LED_PORT)

---

## Communication Protocol

**TWI Message from Comm Board to Relay Board:**
- Command: `TWI_MSG_ALARMS` (0x0E)
- BYTE_1: Message type (TWI_MSG_ALARMS)
- BYTE_2: Relays_state (relay control + BB_PULSE)
- BYTE_3: ExtLED_state (LED control)

**TWI Response from Relay Board to Comm Board:**
- Command: `TWI_RELAY_BOARD_STATUS` (0x0C)
- BYTE_1: Message type
- BYTE_2: alarm_bd_status (AC power loss bit, external reset button bit)
- BYTE_3: 0 (unused)

---

## Hardware Connections

### J1 Connector Pin Assignments (8-pin socket)
| J1 Pin | Port | Function |
|:------:|:----:|:---------|
| Pin 1 | PD0 | LED0 (+Ground Fault) |
| Pin 2 | GND | Ground |
| Pin 3 | PD1 | LED1 (-Ground Fault) |
| Pin 4 | +5V | Supply voltage |
| Pin 5 | PD6 | LED2 (High Battery) |
| Pin 6 | PD3 | LED4 (Ripple/HiZ) |
| Pin 7 | PD5 | LED3 (Low Battery) |
| Pin 8 | PD4 | LED5 (AC Loss) |

---

**Document Version:** 1.2  
**Date:** 2026  
**Project:** 826-501C Relay Board Firmware  
**Processor:** ATmega88PA-AU (TQFP-32 package)  
**Last Update:** Corrected LED5 (AC Loss) mapping to PD4/Pin 2
