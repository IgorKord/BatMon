/*****************************************************************************
*
* Filename: HARDWARE.h
*
* $Source:  $
* Last Saved Revision Number $Revision:  $
* Last Saved Revision $Date: $
* Last Saved Revision $Author:  $
*
* Original Author:  Igor Kordunsky
* Date: 05/31/2023
*
* File Description:	All hardware definitions, macros and function definitions
// device     Flash  EEPROM  RAM   case   maxGPIO
// ATmega644   64K   2K      4K   TQFP44  32
// Xtal clock  16 MHz
*****************************************************************************/
#ifndef _HARDWARE_
#define _HARDWARE_


#define Bit_0                    1 //0x0001
#define Bit_1                    2 //0x0002
#define Bit_2                    4 //0x0004
#define Bit_3                    8 //0x0008
#define Bit_4                   16 //0x0010
#define Bit_5                   32 //0x0020
#define Bit_6                   64 //0x0040
#define Bit_7                  128 //0x0080

#define Bit_8                  256 //0x0100
#define Bit_9                  512
#define Bit_10                1024
#define Bit_11                2048
#define Bit_12                4096
#define Bit_13                8192
#define Bit_14               16384
#define Bit_15               32768

#define Bit_16               65536
#define Bit_17              131072
#define Bit_18              262144
#define Bit_19              524288
#define Bit_20             1048576
#define Bit_21             2097152
#define Bit_22             4194304
#define Bit_23             8388608

#define Bit_24            16777216
#define Bit_25            33554432
#define Bit_26            67108864
#define Bit_27           134217728
#define Bit_28           268435456
#define Bit_29           536870912
#define Bit_30          1073741824
#define Bit_31          2147483648


/****  V E R Y   I M P O R T A N T   C O N S T A N T S **************/
/********************************************************************/
#define MASTER_CLOCK                16000000 //Hz, 16 MHz, ONE INSTRUCTION TAKES 62.5 ns
#define TIMING_INTERRUPT_SETTING    199 // IK20250925 changed from 124 to 199. 0x7C gives 1000 us TIMER 2 interrupt for all timing counters variables
//#define TICK_TIMER_RELOAD_VAL		(long)((MASTER_CLOCK*TICK_PERIOD_uSec)/1000)		// 26 = 10uSec, 261 = 0.1mSec rate,  418=>0.16 ms
//#define CTRL_LOOP_RELOAD_VAL		(Uint32)((MASTER_CLOCK*1000)/16/LOOP_RATE)-1 // 41779*1000/16/12000-1 = 2611.187 for 1KHz, 261 for 10KHz, 216 for 12KHz, 212 for 12228 Hz (2048 scope rate, eazy for FFT)
//#define AD_DA_RELOAD_VAL			(Uint32)((MASTER_CLOCK*1000)/MAX_ADC_CHANNELS/ADC_AVERAGING/LOOP_RATE) // 41779*1000/12/12/2000 = 108 // 43 uSec rate (~- 15uSec rate is limit? )
#define LPF_factor       0.02f
#define TIMERS_0_PRESCALER_VAL  64 // Timer_0 creates ADC interrupt; divider must be one of (1, 8, 64, 256, or 1024)
#if (TIMERS_0_PRESCALER_VAL == 1)
  #define TIMER_0_PRESCALER_SET      0x01
#else
  #if(TIMERS_0_PRESCALER_VAL == 8)
	#define TIMER_0_PRESCALER_SET      0x02
  #else
	#if(TIMERS_0_PRESCALER_VAL == 64)
	  #define TIMER_0_PRESCALER_SET      0x03
	#else
	  #if(TIMERS_0_PRESCALER_VAL == 256)
		#define TIMER_0_PRESCALER_SET      0x04
	  #else
		#if(TIMERS_0_PRESCALER_VAL == 1024)
		  #define TIMER_0_PRESCALER_SET      0x05
		#else
		  #error SELECT PRESCALER for Timer 0 FROM THIS LIST (1, 8, 64, 256, or 1024)
		#endif
	  #endif
	#endif
  #endif
#endif

#define TIMER_2_PRESCALER_VAL  128 // Timer_2 creates 1 ms interrupt; divider must be one of (1, 8, 32, 64, 128, 256, or 1024)
#if (TIMER_2_PRESCALER_VAL == 1)
  #define TIMER_2_PRESCALER_SET      0x01
#else
  #if(TIMER_2_PRESCALER_VAL == 8)
	#define TIMER_2_PRESCALER_SET      0x02
  #else
	#if(TIMER_2_PRESCALER_VAL == 32)
	  #define TIMER_2_PRESCALER_SET      0x03
	#else
	  #if(TIMER_2_PRESCALER_VAL == 64)
		#define TIMER_2_PRESCALER_SET      0x04
	  #else
		 #if(TIMER_2_PRESCALER_VAL == 128)
		  #define TIMER_2_PRESCALER_SET      0x05
		 #else
		  #if(TIMER_2_PRESCALER_VAL == 256)
			#define TIMER_2_PRESCALER_SET      0x06
		  #else
			#if(TIMER_2_PRESCALER_VAL == 1024)
			  #define TIMER_2_PRESCALER_SET      0x07
			#else
			  #error SELECT PRESCALER for Timer 0 FROM THIS LIST (1, 8, 32, 64, 128, 256, or 1024)
			#endif
		  #endif
		#endif
	  #endif
	#endif
  #endif
#endif
/*----------- OUTPUTS --------------------*/
//#define XMT_ENABLE       PORTD
//#define CLOCK_SIG        PORTB
//#define AD_ADDR          PORTD
/*---------- OUTPUT MASKS ----------------*/

// UCSRA === USART Control and Status Register A, bits
// SFR_B_N(0x0B, UCSRA, RXC, TXC, UDRE, FE, DOR, PE, U2X, MPCM)

#define RXC              Bit_7  // 0x80 USART Receive Complete. This flag bit is set when there are unread data in the receive buffer and cleared when
								//    the receive buffer is empty (that is, does not contain any unread data).If the Receiver is disabled, the receive
								//    buffer will be flushed and consequently the RXCn bit will become zero.The RXCn Flag can be used to generate
								//    a Receive Complete interrupt (see description of the RXCIEn bit).
#define TXC              Bit_6  // 0x40 USART Transmit Complete. This flag bit is set when the entire frame in the Transmit Shift Register has been shifted out
								//    and there are no new data currently present in the transmit buffer(UDRn).The TXCn Flag bit is automatically
								//    cleared when a transmit complete interrupt is executed, or it can be cleared by writing a one to its bit location.
								//    The TXCn Flag can generate a Transmit Complete interrupt (see description of the TXCIEn bit)
#define UDRE             Bit_5  // 0x20 USART Data Register Empty. indicates if the transmit buffer (UDR) is ready to receive new data.
								//    If UDRE is one, the buffer is empty, and therefore ready to be written. The UDRE Flag can generate a Data Register empty Interrupt.
#define FRAME_ERROR_BIT  Bit_4  // 0x10 Frame Error
#define DOR              Bit_3  // 0x08 Data OverRun
#define PARITY_ERROR_BIT Bit_2  // 0x04 Parity Error
#define U2X              Bit_1  // 0x02 Double the USART Transmission Speed
#define MPCM             Bit_0  // 0x01 Multi-processor Communication Mode


// UCSRB === USART Control and Status Register B, bits
// SFR_B_N(0x0A, UCSRB, RXCIE, TXCIE, UDRIE, RXEN, TXEN, UCSZ2, RXB8, TXB8)

#define RXCIE            Bit_7 // 0x80 RX Complete Interrupt Enable
#define TXCIE            Bit_6 // 0x40 TX Complete Interrupt Enable
#define UDRIE            Bit_5 // 0x20 USART Data Register Empty Interrupt Enable
#define RXEN             Bit_4 // 0x10 Receiver Enable = 1, disable = 0; usage: UCSR0B &= ~RXEN; // immediately disable RS232 RCV *****
							   //    In contrast to the Transmitter, disabling of the Receiver will be immediate. Data from ongoing
							   //    receptions will therefore be lost. When disabled (that is, the RXEN is set to zero) the Receiver
							   //    will no longer override the normal function of the RxD port pin. The receiver buffer FIFO will be
							   //    flushed when the receiver is disabled. Remaining data in the buffer will be lost
#define TXEN             Bit_3 // 0x08 Transmitter Enable =1, disable = 0
							   //    The disabling of the transmitter (setting the TXEN to zero) will not become effective until ongoing
							   //    and pending transmissions are completed, that is, when the transmit Shift Register and transmit
							   //    Buffer Register do not contain data to be transmitted. When disabled, the transmitter will no longer override the TxD pin.
#define UCSZ2            Bit_2 // 0x04 Character Size
#define RXB8             Bit_1 // 0x02 Receive Data Bit 8
#define TXB8             Bit_0 // 0x01 Transmit Data Bit 8

#define CLOCK_1          0x80           // 0x08
#define CLOCK_0          0x7F           // 0xF7

/*---------- PORT SETUP ------------------*/
// 'x' is the port name, A, B, C, D
// 'n' is the bit, 0 to 7
// DDRx Data Direction Register (initial value is 0). If DDxn is written logic 1, Pxn is configured as Output, zero (0) means input
// PORTx Port x Data Register (initial value is 0). if PORTxn is written 1 when it is input, pull-up resistor is activated
// PINx Input Pin Address Register

											//I = input; O = output; A = Alternate/Not used
									// bit#   7 6 5 4 3 2 1 0
#define PORTA_DDR        0xD8               //O O I O O A A I
#define PORTA_INIT       0x21               //with pull-ups on inputs
#define PORTB_DDR        0x1A               //0 0 0 1 1 0 1 0 << Bit 1 (U1, pin 34) - output, XMT_REV_LED3 IK20250523 - USED FOR Timing measurement, Bit 3 (U1, pin 43) - output, XMT_REV_LED2 >>> goes via ribbon cable to bi-color LED on front board
//#define PORTB_DDR      0x0A               //A A A A O I O I
#define PORTB_INIT       0x05               //Pull-ups on inputs
#define PORTC_DDR        0x00               //A A I I I I I I
#define PORTC_INIT       0x38               //Pull-ups on inputs only
#define PORTD_DDR        0xBC               //1 0 1 1 1 1 0 0 << Bit 7 (U1, pin 16) - output, XTM/REV_LED_1 >>> goes via ribbon cable to bi-color LED on front board
//#define PORTD_DDR      0xBC               //O I O O O O A A
#define PORTD_INIT       0x4C               //Pull-ups on inputs only


/* -------------TWI - Two Wire Interface ------*/

/**********************************************************
TWI Status Register - TWSR
TWSR - TWI Status Register
Name : TWSR
Offset : 0x71
Reset : 0xF8
Property:
The TWSR only contains relevant status information when the TWI Interrupt Flag is asserted.
At all other times, the TWSR contains a special status code indicating that no relevant status information is available.

Bits[7:3] - TWS : TWI Status
These five bits reflect the status of the TWI logic and the Two - Wire Serial Bus.
The different status codes are described later in this section.
Note that the value read from TWSR contains both the 5 - bit status value and the 2 - bit prescaler value.
The application designer should mask the prescaler bits to zero when checking the Status bits.
This makes status checking independent of prescaler setting.
Bit 2 - Reserved Bit. This bit is reserved and will always read as zero.
Bits [1:0] - TWPS: TWI Prescaler Bits. These bits can be read and written, and control the bit rate prescaler.
**********************************************************/

#define TWS7             Bit_7 // TWI Status Bit 7
#define TWS6             Bit_6 // TWI Status Bit 6
#define TWS5             Bit_5 // TWI Status Bit 5
#define TWS4             Bit_4 // TWI Status Bit 4
#define TWS3             Bit_3 // TWI Status Bit 3
#define TWSRna           Bit_2 // Bit 2 - Reserved Bit (always read as zero)
//TWPS1 TWPS0 Prescaler Value           |  =1 | =4  | =16 | =64 |
#define TWPS1            Bit_1 // Bit 1 |  0  |  0  |  1  |  1  |
#define TWPS0            Bit_0 // Bit 0 |  0  |  1  |  0  |  1  |
/* The SCL frequency in MASTER mode is generated according to the following equation
SCL frequency = (CPU Clock frequency) / (16 + 2*(TWBR) * PrescalerValue) */

#define TWSR_STATUS_MASK      0xF8
#define TWSR_PRESCALER_MASK   0x03
#define TWSR_STATUS_NOT_AVAIL TWSR_STATUS_MASK

/**********************************************************
TWI Control Register - TWCR
Name: TWCR
Offset: 0x74
Reset: 0x00
Property:
The TWCR is used to control the operation of the TWI.It is used to enable the TWI, to initiate a
master access by applying a START condition to the bus, to generate a receiver acknowledge,
to generate a stop condition, and to control halting of the bus while the data to be written to the
bus are written to the TWDR. It also indicates a write collision if data is attempted written to
TWDR while the register is inaccessible.

The TWINT Flag must be cleared by software by writing a logic one to it. Note that this flag is not
automatically cleared by hardware when executing the interrupt routine. Also note that clearing
this flag starts the operation of the TWI, so all accesses to the TWI Address Register (TWAR),
TWI Status Register (TWSR), and TWI Data Register (TWDR) must be complete before clearing this flag.
As long as the TWINT Flag is set, the SCL line is held low. This allows the application software to
complete its tasks before allowing the TWI transmission to continue.
**********************************************************/
#define TWINT            Bit_7 // 0x80 TWI Interrpt Flag
/*This bit is set by hardware when the TWI has finished its current job and expects application software
response. If the I-bit in SREG and TWIE in TWCR are set, the MCU will jump to the TWI Interrupt Vector.
While the TWINT Flag is set, the SCL low period is stretched. The TWINT Flag must be cleared by
software by writing a logic '1' (one) to it.
Note that this flag is not automatically cleared by hardware when executing the interrupt routine. Also
note that clearing this flag starts the operation of the TWI, so all accesses to the TWI Address Register
(TWAR), TWI Status Register (TWSR), and TWI Data Register (TWDR) must be complete before clearing this flag.*/

#define TWEA             Bit_6 // 0x40 TWI Enable Acknowledge Bit
/*The TWEA bit controls the generation of the acknowledge pulse. If the TWEA bit is written to one, the
ACK pulse is generated on the TWI bus if the following conditions are met:
1. The device's own slave address has been received.
2. A general call has been received, while the TWGCE bit in the TWAR is set.
3. A data byte has been received in Master Receiver or Slave Receiver mode.
By writing the TWEA bit to zero, the device can be virtually disconnected from the 2-wire Serial Bus
temporarily. Address recognition can then be resumed by writing the TWEA bit to one again.*/

#define TWSTA            Bit_5 // 0x20 TWI START Condition Bit
/*The application writes the TWSTA bit to one when it desires to become a Master on the 2-wire Serial Bus.
The TWI hardware checks if the bus is available, and generates a START condition on the bus if it is free.
However, if the bus is not free, the TWI waits until a STOP condition is detected, and then generates a
new START condition to claim the bus Master status. TWSTA must be cleared by software when the
START condition has been transmitted. */

#define TWSTO            Bit_4 // 0x10 TWI STOP Condition Bit
/*Writing the TWSTO bit to one in Master mode will generate a STOP condition on the 2-wire Serial Bus.
When the STOP condition is executed on the bus, the TWSTO bit is cleared automatically. In Slave
mode, setting the TWSTO bit can be used to recover from an error condition. This will not generate a
STOP condition, but the TWI returns to a well-defined unaddressed Slave mode and releases the SCL
and SDA lines to a high impedance state.*/

#define TWCC             Bit_3 // 0x08 TWI Write Collision Flag
/*The TWWC bit is set when attempting to write to the TWI Data Register - TWDR when TWINT is low.
This flag is cleared by writing the TWDR Register when TWINT is high.*/

#define TWEN             Bit_2 // 0x04 TWI Enable Bit
/*The TWEN bit enables TWI operation and activates the TWI interface. When TWEN is written to one, the
TWI takes control over the I/O pins connected to the SCL and SDA pins, enabling the slew-rate limiters
and spike filters. If this bit is written to zero, the TWI is switched off and all TWI transmissions are
terminated, regardless of any ongoing operation.*/

#define TWIna            Bit_1 // 0x02 Bit 1 - Reserved Bit

#define TWIE             Bit_0 // 0x01 TWI Interrupt Enable.
/*When this bit is written to one, and the I - bit in SREG is set, the TWI interrupt request will be
activated for as long as the TWINT Flag is high.*/


/* ------------- MCUSR - MCU Status Register -------------------------------------------
The MCU Status Register provides information on which reset source caused an MCU reset.

Bit          7    6    5    4    3    2    1    0
0x34 (0x54)  -    -    -  JTRF WDRF BORF EXTRF PORF
Read/Write   R    R    R   R/W  R/W  R/W  R/W  R/W
Init Value   0    0    0   See Bit Description */

#define JTRF            Bit_4 // JTAG Reset Flag
/* This bit is set if a reset is being caused by a logic one in the JTAG Reset Register selected by
the JTAG instruction AVR_RESET. This bit is reset by a Power-on Reset, or by writing a logic
zero to the flag.*/
#define WDRF            Bit_3 // Watchdog Reset Flag
/*This bit is set if a Watchdog Reset occurs.The bit is reset by a Power - on Reset, or by writing a
logic zero to the flag. */
#define BORF            Bit_2 // Brown-out Reset Flag
/*This bit is set if a Brown-out Reset occurs. The bit is reset by a Power-on Reset, or by writing a
logic zero to the flag. */
#define EXTRF           Bit_1 // External Reset Flag
/*This bit is set if an External Reset occurs.The bit is reset by a Power - on Reset, or by writing a
logic zero to the flag. */
#define PORF            Bit_0 // Power-on Reset Flag
/*This bit is set if a Power-on Reset occurs. The bit is reset only by writing a logic zero to the flag.
To make use of the Reset Flags to identify a reset condition, the user should read and then
Reset the MCUSR as early as possible in the program. If the register is cleared before another
reset occurs, the source of the reset can be found by examining the Reset Flags. */

/* ------- WDTCSR - Watchdog Timer Control Register  ---------------------------------------
Bit          7    6    5    4    3    2    1    0
(0x60)     WDIF WDIE WDP3 WDCE WDE  WDP2 WDP1 WDP0
Read/Write R/W  R/W  R/W  R/W  R/W  R/W  R/W  R/W
Init Value  0     0    0    0    X    0    0    0 */

#define WDIF            Bit_7 // Watchdog Interrupt Flag
/*This bit is set when a time-out occurs in the Watchdog Timer and the Watchdog Timer is configured for interrupt.
WDIF is cleared by hardware when executing the corresponding interrupt
handling vector. Alternatively, WDIF is cleared by writing a logic one to the flag. When the I-bit in
SREG and WDIE are set, the Watchdog Time-out Interrupt is executed.*/

#define WDIE            Bit_6 // Watchdog Interrupt Enable
/*When this bit is written to one and the I-bit in the Status Register is set, the Watchdog Interrupt is
enabled. If WDE is cleared in combination with this setting, the Watchdog Timer is in Interrupt
Mode, and the corresponding interrupt is executed if time-out in the Watchdog Timer occurs.
If WDE is set, the Watchdog Timer is in Interrupt and System Reset Mode. The first time-out in
the Watchdog Timer will set WDIF. Executing the corresponding interrupt vector will clear WDIE
and WDIF automatically by hardware (the Watchdog goes to System Reset Mode). This is useful for keeping the
Watchdog Timer security while using the interrupt. To stay in Interrupt and
System Reset Mode, WDIE must be set after each interrupt. This should however not be done
within the interrupt service routine itself, as this might compromise the safety-function of the
Watchdog System Reset mode. If the interrupt is not executed before the next time-out, a System Reset will be applied.*/

#define WDCE            Bit_4 // Watchdog Change Enable
/*This bit is used in timed sequences for changing WDE and prescaler bits. To clear the WDE bit,
and/or change the prescaler bits, WDCE must be set.
Once written to one, hardware will clear WDCE after four clock cycles.*/

#define WDE             Bit_3 //  Watchdog System Reset Enable
/*WDE is overridden by WDRF in MCUSR. This means that WDE is always set when WDRF is
set. To clear WDE, WDRF must be cleared first. This feature ensures multiple resets during conditions causing failure, and a safe start-up after the failure.*/

// Bits 5, 2:0 - WDP3:0: Watchdog Timer Prescaler 3, 2, 1 and 0
#define WDP3            Bit_5
#define WDP2            Bit_2
#define WDP1            Bit_1
#define WDP0            Bit_0
/* The WDP3:0 bits determine the Watchdog Timer prescaling when the Watchdog Timer is running.
The different prescaling values and their corresponding time-out periods are shown in table

__________________________________________________________________________________
Bit_5 | Bit_2 | Bit_1 | Bit_0 |
WDP3  | WDP2  | WDP1  | WDP0  |Number of WDT Osc Cycles | Typ Time-out at VCC = 5.0V
  0   |   0   |   0   |   0   | 2K (2048) cycles        |     16 ms
  0   |   0   |   0   |   1   | 4K (4096) cycles        |     32 ms
  0   |   0   |   1   |   0   | 8K (8192) cycles        |     64 ms
  0   |   0   |   1   |   1   | 16K (16384) cycles      |    0.125s
  0   |   1   |   0   |   0   | 32K (32768) cycles      |    0.25s
  0   |   1   |   0   |   1   | 64K (65536) cycles      |    0.5s
  0   |   1   |   1   |   0   | 128K (131072) cycles    |    1.0s
  0   |   1   |   1   |   1   | 256K (262144) cycles    |    2.0s
  1   |   0   |   0   |   0   | 512K (524288) cycles    |    4.0s
  1   |   0   |   0   |   1   | 1024K (1048576) cycles  |    8.0s
  1   |   0   |   1   |   0   |
  1   |   0   |   1   |   1   |
  1   |   1   |   0   |   0   |  Reserved
  1   |   1   |   0   |   1   |
  1   |   1   |   1   |   0   |
  1   |   1   |   1   |   1   |
*/


/***********************************************************************/
#define ADEN             Bit_7  //  ADC Enable. Writing this bit to one enables the ADC. By writing it to zero, the ADC is turned off.
								// Turning the ADC off while a conversion is in progress, will terminate this conversion.
#define ADSC             Bit_6 //  ADC Start Conversion bit
#ifndef PC	   //ATMEL

  #define TestBit_on_off(BIT)  GP0DAT = BIT
  #define TestBit_on(BIT)      GP0SET = BIT
  #define TestBit_off(BIT)     GP0CLR = BIT
  #define TestP12_on  GP1SET = P02_BIT //P1.2, P1.3 are strobe signals for external port lines created by 74HCT75
  #define TestP12_off GP1CLR = P02_BIT //can be used as test signals

#else  //PC
extern uint8 EEPROM[0x800];
extern void initEEsimulator(void);

#ifndef UBRR0	// = 103;                                //9600 baud
long UBRR0;		// 12-bits register in ATMEL
#endif

#ifndef CLKPR // = 0x80;                                // Clock Prescale Register
long CLKPR;
  #endif

#ifndef UCSRC // = 0x86;                              // no UART_parity, async, 1 stop
long UCSRC;
  #endif

  #ifndef UCSRA // = 0x20
long UCSRA;
  #endif

  #ifndef UCSR0A //= 0x20;                              //redundant because it is set a reset anyway
long UCSR0A;
  #endif

  #ifndef UCSRB // = 0x98;                              //enable rcv & xmt,
long UCSRB;
  #endif

  #ifndef UCSR0B // = 0x98;                              //enable rcv & xmt,
long UCSR0B;
  #endif


  #ifndef UCSR0C // = 0x06;                              //no UART_parity, async, 1 stop,
long UCSR0C;
  #endif

  #ifndef UBRRL // = 47;                                //9600 baud
long UBRRL;
  #endif

#ifndef UDR// = 47;                                  //9600 baud
long UDR;
#endif

#ifndef UDR0 // = Tx_char;                           //UART Tx register
volatile Uchar UDR0;
#endif

#ifndef TWSR // status register
volatile Uchar TWSR;
#endif

#ifndef TWDR // Rx data register
volatile Uchar TWDR;
#endif

#ifndef TWAR // address register, set all call
long TWAR;
#endif

#ifndef TWBR // TWI baud rate register, 77 kHz
long TWBR;
#endif

#ifndef TWCR // control register
long TWCR;
#endif

#ifndef SREG // status register
long SREG;
#endif

#ifndef PORTA // PORT A register
long PORTA;
#endif

#ifndef PORTB // PORT B register
long PORTB;
#endif

#ifndef PORTC // PORT C register
long PORTC;
#endif

#ifndef PORTD // PORT D register
long PORTD;
#endif

#ifndef PORTE // PORT D register
long PORTE;
#endif

#ifndef PORTF // PORT D register
long PORTF;
#endif

#ifndef PORTG // PORT D register
long PORTG;
#endif

#ifndef DDRA // I I I I I I I I
long DDRA;
#endif

#ifndef DDRB // I I I I I I I I
long DDRB;
#endif

#ifndef DDRC // I I I I I I TWI TWI
long DDRC;
#endif

#ifndef DDRD // I I I I I I O I
long DDRD;
#endif

#ifndef DDRE // I I I I I I O I
long DDRE;
#endif

#ifndef DDRF // I I I I I I O I
long DDRF;
#endif

#ifndef DDRG // I I I I I I O I
long DDRG;
#endif

#ifndef PORTA_DDR // I I I I I I I I
long PORTA_DDR;
#endif

#ifndef PORTB_DDR  // I I I I I I I I
long PORTB_DDR;
#endif

#ifndef PORTC_DDR  // I I I I I I TWI TWI
long PORTC_DDR;
#endif

#ifndef PORTD_DDR  // I I I I I I O I
long PORTD_DDR;
#endif

#ifndef WDTCR // enable watchdog for 2048
long WDTCR;
#endif

#ifndef MCUCSR
long MCUCSR;
#endif

#ifndef WDTCSR
long WDTCSR;
#endif

#ifndef MCUSR
long MCUSR;
#endif

#ifndef TCNT1 // Set it for 1 ms intervals.
long TCNT1; //= 0xE0BF; in init.c
#endif

#ifndef TIMSK1 // Set it for 1 ms intervals.
long TIMSK1; //= 0x01; in init.c
#endif

#ifndef OCR2
long OCR2; // = 114;                                // for 1 ms
#endif

/*-------- Timer 1 used for 4-20ma (PWM)---*/
#ifndef ICR1 //
long ICR1;
#endif

#ifndef TCCR1A
long TCCR1A; // =0x00;                             // normal counter
#endif

#ifndef TCCR1B
long TCCR1B; // =0x82;                             // no prescale
#endif

#ifndef TCCR1C
long TCCR1C; // =??;                             // no prescale
#endif

/*-------- Timer 0 -System  Timer ----*/

#ifndef TCCR0A
long TCCR0A; // 0x24;                              // Timer/Counter Control Register A
#endif

#ifndef TCCR0B
long TCCR0B; // 0x25;                              // Timer/Counter Control Register B
#endif

#ifndef TCNT0
long TCNT0; // 0x26;                              // Timer/Counter Register
#endif

#ifndef OCR0A
long OCR0A; // 0x27;                              // Output Compare Register A
#endif

#ifndef OCR0B
long OCR0B; // 0x48;                              // Output Compare Register B
#endif

#ifndef TIMSK0
long TIMSK0; // 0x6E;                              // Timer/Counter Interrupt Mask Register
#endif

#ifndef TIFR0
long TIFR0; // 0x35;                              // Timer/Counter 0 Interrupt Flag Register
#endif

/*-------- Timer 2 -System  Timer ----*/
#ifndef TCCR2
long TCCR2; // 0x2C;                              // clear on compare
#endif

#ifndef TCCR2A
long TCCR2A; // = 0x02;                             //normal counter, clear tmr on compare
#endif

#ifndef TCCR2B
long TCCR2B; // = 0x02;                             //divide by 8
#endif

#ifndef ASSR
long ASSR; // ??
#endif

#ifndef TIMSK
long TIMSK; // = 0x80; enable timer 2 compare int
#endif

#ifndef TIMSK2
long TIMSK2; // = 0x80; enable timer 2 compare int
#endif

#ifndef ACSR
long ACSR; // = 0x44 ??
#endif

#ifndef ADMUX // ADC Multiplexer
long ADMUX;
#endif

#ifndef ADCSRA // ADC control
long ADCSRA;
#endif

#ifndef ADCL // ADC result lower byte
long ADCL;
#endif

#ifndef ADCH // ADC result higher byte
long ADCH;
#endif

#ifndef PINB //
long PINB;
#endif

#ifndef PINC //
long PINC;
#endif

#ifndef EEAR //
long EEAR;
#endif

#ifndef EEDR //
volatile Uchar EEDR;
#endif

#ifndef EECR //
volatile Uchar EECR;
#endif

#ifndef OCR1A //
long OCR1A;
#endif

#ifndef OCR2A //
long OCR2A;
#endif

#ifndef TCNT2 //
long TCNT2;
#endif

#ifndef PING                 // Thet state of 'UP' button,  pressed - not pressed
long PING;
#endif

#ifndef PINF                 // Thet state of 'limit' button,  pressed - not pressed
long PINF;
#endif

/*****************************************************************/

	#ifdef GP0DAT
	  #undef GP0DAT
long GP0DAT;
	#endif
	#ifdef GP1DAT
	  #undef GP1DAT
long GP1DAT;
	#endif
	#ifdef GP2DAT
	  #undef GP2DAT
long GP2DAT;
	#ifdef GP2SET
	  #undef GP2SET
	#endif
long GP2CLR;
	#ifdef GP2CLR
	  #undef GP2CLR
	#endif
long GP2SET;
	#endif
	#ifdef GP3DAT
	  #undef GP3DAT
long GP3DAT;
	#endif
	#ifdef GP4DAT
	  #undef GP4DAT
long GP4DAT;
	#endif
  #define TestBit_on_off(BIT)	// GP0DAT = BIT
  #define TestBit_on(BIT)		// GP0SET = BIT
  #define TestBit_off(BIT)		// GP0CLR = BIT
  #define TestP12_on			// GP1SET = P02_BIT //P1.2, P1.3 are strobe signals for external port lines
  #define TestP12_off			// GP1CLR = P02_BIT //can be used as test signals
#endif

// Port bit definitions
#define  PD0  Bit_0 // Port D, bit 0
#define  PD1  Bit_1 // Port D, bit 1
#define  PD2  Bit_2 // Port D, bit 2
#define  PD3  Bit_3 // Port D, bit 3
#define  PD4  Bit_4 // Port D, bit 4
#define  PD5  Bit_5 // Port D, bit 5
#define  PD6  Bit_6 // Port D, bit 6
#define  PD7  Bit_7 // Port D, bit 7

// 8-pin J1 socket on relay board (on back side of can)

#define J1_PIN1 PD0  // Port D, bit 0
#define J1_PIN2 GND  // common ground
#define J1_PIN3 PD1  // Port D, bit 1
#define J1_PIN4 PLS_5V  // supply voltage out
#define J1_PIN5 PD6  // Port D, bit 6
#define J1_PIN6 PD3  // Port D, bit 3
#define J1_PIN7 PD5  // Port D, bit 5
#define J1_PIN8 PD6  // Port D, bit 8

// Anuncitor's LEDs names on the panel, in the top-to-bottom order
#ifdef STANDARD_PANEL
#define EXT_LED_PLS_GND_FAULT        J1_PIN1 // TOP LED on panel
#define EXT_LED_MNS_GND_FAULT        J1_PIN3
#define EXT_LED_HIGH_BAT_FAULT       J1_PIN5
#define EXT_LED_LOW_BAT_FAULT        J1_PIN7
#define EXT_LED_RippleV_C_HiZ_FAULT  J1_PIN6 // combination of Ripple voltage, Ripple current, High Impedance
#define EXT_LED_LOSS_AC_FAULT        J1_PIN8 // Bottom LED on panel
#endif
#ifdef DUAL_VOLTAGE_ALARM_TRIP
#define EXT_LED_PLS_GND_FAULT        J1_PIN1 // TOP LED on panel
#define EXT_LED_MNS_GND_FAULT        J1_PIN3
#define LED_EMERGENCY_LOW_BAT        J1_PIN5 //
#define EXT_LED_HIGH_BAT_FAULT       J1_PIN7
#define EXT_LED_LOW_BAT_FAULT        J1_PIN6 //
#define EXT_LED_RippleV_C_HiZ_FAULT  J1_PIN8 // Bottom LED on panel

// included into bottom LED #define EXT_LED_LOSS_AC_FAULT        J1_PIN6 // combination of Ripple voltage, Ripple current, High Impedance
#endif

#define TestBit_pulse(BIT)\
 GP0SET = BIT;\
 GP0CLR = BIT;
#define STROBE 	 TestP37_off; TestP37_on; Delay_uS(4); TestP37_off;

//*********************************** HARDWARE: STATUS LED on Header Board ***************************
//#define     BLINK_RATE  (Bit_10 * TICKS_IN_mSEC)
//*********************************** HARDWARE: LED on Front Panel ***********************************
//#define     LED_REG     GP2DAT //P2.0 - P2.1

//#ifdef PC //for PC
//#endif  //for PC

#endif //_HARDWARE_
