#include "DNP.h"

//---- Event variables
uint8  event_type;
uint8  point_state;
uint8  event_cntr;
uint8  ovr_flow;
//-!- IK20240108 need only 11 * 2 bytes, not 11*8!
// event takes 2 bytes: event_type is coded by bit (only 8 event types), point state = ON, OFF == 1 or 0
volatile uint8 event_array[32];				// need 11 * 8 for a ten event buffer
uint8 num_of_events;						// How many events requested (firmware keeps only 10 events in event_array[])
uint8 events_requested;
//-!- IK20250812 do these variables need to be volatile? they are not modified in interrupt
/*volatile*/ uint8  iien1;		// IIN byte 1
/*volatile*/ uint8  iien2;		// IIN byte 2
/*volatile*/ uint8  length;		// IK20250808 why there two length variables? dnp_length and length?
/*volatile*/ uint8  dnp_length;
/*volatile*/ uint8  DNPqualifier;
/*volatile*/ uint8  DNPbroadcast;
/*volatile*/ uint8  pending_confirm;			//-!- IK20241226 can be a bit, two states
/*volatile*/ uint8  dll_status;					// in DNP3 protocol DLL stands for "Data Link Layer"
/*volatile*/ uint8  variation;					// holds variation of the object in DNP3 protocol
/*volatile*/ uint8  all_stations_msg;			// indicates if this is a DNP broadcast message to all stations
/*volatile*/ uint8  events_record_num = 0;		// IK20250709 added explicit initialization //points to next available slot in saving events

uint8  DNPstart;
uint8  DNPstop;
uint8  sequence_num;						// last_sequence;
uint16 send_dnp;							// instructs what msg to send if any; uint16 because it could get OBJECT_1_RESPONSE = 1024
uint16 DNP_time_delay;						// sets to 9000, and then shows in case
//---- DNP variables
uint8  object_string[30];					// hold dnp object info
uint8  obj_ptr;								// it is index of byte in the object_string[obj_ptr]


/********************************************************************/
/*                      ERASE  EVENT                                */
/********************************************************************/
/* Description: This function pushes down the event array by the number
				of records it wants pushed off the bottom. The bottom holds
				the oldest events.
   Inputs: num of events to erase
   Outputs:  None
   Notes:

   Revision:    02/26/16      REC     Created.
*********************************************************************/

void Erase_Events(uint8 events_to_erase)
{
	uint8 record;
	if ((events_to_erase <= events_record_num - 1) && (events_record_num != 0))	// limit checks
	{
		if ((events_to_erase < MAX_EVENT_NUMBER) && (events_to_erase > 0))		// limit checks
		{
			while (events_to_erase != 0)										// num of events confirmed read
			{
				for (record = 2; record <= MAX_EVENT_NUMBER; record++)			// move elements of record back to previous record
				{
					//debug = ((record - 1) * 8) + 1;
					event_array[((record - 1) * 2) + 0] = event_array[(record * 2) + 0];
					event_array[((record - 1) * 2) + 1] = event_array[(record * 2) + 1];
				}
				events_to_erase--;
				events_record_num--;											// move back ptr to next vacant spot
				event_cntr--;
				if (events_record_num == 1)
					events_record_num = 0;										// all done
			}//End While
		}//End IF
		else //no need to save any or push any down
		{
			events_record_num = 0;
			event_cntr = 0;
			events_to_erase = 0;
		}
	}
}

/********************************************************************/
/*                      S T O R E    E V E N T                      */
/********************************************************************/
/* Description: This function is used to write an event to the EEPROM
			Maintains the last ten events
//-!- IK20240108 correction: now events are kept in RAM

	WAS:Event is coded by a bit number (only 8 event types),
//-!- IK20240709 correction
	Inputs: Event is coded by a number (255 event types),
			point state = ON, OFF == 1 or 0
   Outputs: Stored event in event array
   Notes:
	//IK20250709  BUG: all event were bits are shifted up, bit 0 is not used.
	as the result, HI_BAT_Event which supposed to be Bit_0 is saved as LOW_BAT_EVENT_INDEX (into Bit_1)
	WAS: EventType is bit number (only 8 event types)
	NOW: EventType is a number (0-254) of event type, 255 is reserved for NO_EVENT

	PointState is ON (1) or OFF (0)
	Revision:    02/26/16      REC     Created.
*/
/********************************************************************/
void Store_Event(uint8 EventType, uint8 PointState)
{
	//called from main
	uint8 tmp_point_state;

	if (PointState == 1)
		tmp_point_state = DNP_STATE_ON_PT_ONLINE;		// 0x81 DNP state on and point online
	else
		tmp_point_state = DNP_STATE_OFF_PT_ONLINE;		// 0x01 DNP state went off and is online

	iien1 |= 0x02;										// set have class 1 data
	// //IK20250709  event increment WAS HERE. BUG: record 0 is never used
	if (events_record_num == 0)							// if no events then start at 1
	{
		event_cntr = 0;
		events_record_num++;
	}
	if (events_record_num > MAX_EVENT_NUMBER)			// just do 10 records
	{
		Erase_Events(1);								// erase oldest event
		events_record_num = MAX_EVENT_NUMBER;			// put ptr back to ten
		//IK20231214 not checked //ten_events = true;
		ovr_flow = true;								// had an overflow
	}

	event_array[(events_record_num * 2)] = EventType;	//IK20250805 FIXING BUG event indexes not power of 2, same same as Class0. I redefined events
	event_array[(events_record_num * 2) + 1] = tmp_point_state;

	// IK20250709 moved event increment after first record is done -
	// This requires checking / testing Read_Event() and Erase_Events()
	//if (events_record_num == 0)                            // if no events then start at 1
	//{
	//	event_cntr = 0;
	//	events_record_num++;
	//}

	event_cntr++;
	events_record_num++;
	// IK20240110 nobody checks!	event_type = NO_EVENT;
}


/********************************************************************/
/*                      R E A D     E V E N T                       */
/********************************************************************/
/* Description: This function is used to read the stored events from EEPROM
//-!- IK20240108 correction: now events are in RAM

   Inputs: record number
   Outputs: Event, state
   Notes: it updates global variables 'event_type' and 'point_state'

   Revision:    02/26/16      REC     Created.
*/
/********************************************************************/

void Read_Event(uint8 record)
{
	event_type = event_array[(record * 2)];			// even address is event type
	point_state = event_array[(record * 2) + 1];	// odd address is point state, 0 or 1
}

//-!- IK20250714 function is not used
int AddBinaryFloatToWorkString(float value, int start_pos)
{
	float tFloat = value;
	Uint32 Qfloat = *((Uint32*)(&tFloat));	// Quasi-float: 4 bytes representing float but addressed as long
	uint8 aByte = (uint8)Qfloat;			// lower byte
	wrk_str[start_pos] = aByte;				// write 1st byte
	start_pos++;							// 2nd byte address
	aByte = (uint8)(Qfloat >> 8);			// 2nd byte
	wrk_str[start_pos] = aByte;				// write 2nd byte
	start_pos++;							// 3rd byte address
	aByte = (uint8)(Qfloat >> 16);			// 3rd byte
	wrk_str[start_pos] = aByte;				// write 3rd byte
	start_pos++;							// 4th byte address
	aByte = (uint8)(Qfloat >> 24);			// 4th upper byte
	wrk_str[start_pos] = aByte;				// write upper byte
	return start_pos;
}

// DNP3 is Little - endian(LSB first).
//-!- IK20250814 function is not used
// // returns last written position in wrk_str
int Add_DNP_BinaryLongToWorkString(long value, int start_pos)
{
	long tLong = value;
	uint8 aByte = (uint8)tLong;				// lower byte
	wrk_str[start_pos] = aByte;				// write 1st byte
	start_pos++;							// 2nd byte address
	aByte = (uint8)(tLong >> 8);			// 2nd byte
	wrk_str[start_pos] = aByte;				// write 2nd byte
	start_pos++;							// 3rd byte address
	aByte = (uint8)(tLong >> 16);			// 3rd byte
	wrk_str[start_pos] = aByte;				// write 3rd byte
	start_pos++;							// 4th byte address
	aByte = (uint8)(tLong >> 24);			// 4th upper byte
	wrk_str[start_pos] = aByte;				// write upper byte
	return start_pos;
}

/********************************************************************/
/*                      S E N D   D N P   M S G                     */
/********************************************************************/
/*  DESCRIPTION: This routine builds the various messages to be sent.
				and then sets the "send" variable to send nothing so
				the message is only sent once.

   Inputs:  send,type
   Outputs: output message string (wrk_str)
   Notes:   Length includes the control, destination, and source fields
			CRC fields are not included. Rec_num is always 1 more than
			the actual number of records.
			send bit description

   Revision:    02/22/16      REC     Created.
*/

/********************************************************************/
void Build_DNP_DLL_Header(uint8* buf, uint8 length, uint8 control)
{
	uint16 address = (uint16)SysData.NV_UI.meter_address;

	buf[0] = 0x05;									//	start byte 1
	buf[1] = 0x64;									//	start byte 2
	buf[2] = length;								//	length of msg, including control, destination, source, and CRC
	buf[3] = control;								//	cntrl octet
	buf[4] = SysData.NV_UI.host_address & 0x00FF;	//	low order destination
	buf[5] = SysData.NV_UI.host_address >> 8;		//	high order destination
	buf[6] = address & 0x00FF;	//	address & 0x00FF;//low order source
	buf[7] = address >> 8;		//	address >> 8;//high order source
	Calc_DNPCRC(buf, 8);							//	Calc the Dll CRC on 8 bytes
	buf[8] = crc & 0xFF;							//	append DLL crc low byte first
	buf[9] = crc >> 8;								//	append DLL high byte of crc
}


void Send_DNP_Msg(uint16 type)
{
	Int32 battery_milliVolts = (Int32)(rt.OutData.measured.battery_voltage_f * V_to_mV);		// for DNP and Modbus transmission, 1 mV = 1 count
	Int32 fault_millivolts = (Int32)(rt.OutData.measured.G_fault_voltage_f * V_to_mV);		// convert float to int for fault milli voltage
	Int32 minus_gnd_millivolts = (Int32)(rt.OutData.measured.minus_gnd_volts_f * V_to_mV);	// convert float to int for minus gnd milli voltage
	Int32 ripple_milliAmpers = (Int32)(rt.OutData.measured.ripple_mA_f);		// convert float to int for ripple current
	Int32 ripple_milliVolts = (Int32)(rt.OutData.measured.ripple_mV_f);		// convert float to int for ripple voltage
	uint8 i;
	uint8 temp_length, events_to_send, rec_ptr, bytes_to_send, j;
	uint8 events_sent = 0;
	uint8 str_ptr = 0, crc_cntr = 0, out_data = 0;

	iien2 &= 0x0F;							// clear unimplemented
	iien1 &= 0x83;							// clear unimplemented
	if ((type & CLASS_1) && (events_record_num == 0))
	{
		type = type & 0xFFDF;				// CLear Class 1 reply
		if (type == 0)
			type = ERROR_RESPONSE;			// Changed to Null cause nothing to report
	}

	if (ovr_flow == true)					// if overflowed buffer
		iien2 |= 0x08;						// set overflow
	else
		iien2 &= 0xF7;						// clear overflow


	if ((type == ALL_DATA) && (events_requested == 0)
		|| (events_record_num == 0))		//if all data and no events change
		type = CLASS_0;						// to class 0


	// ------------- DATA Link Layer Common to all msgs --
	//-------------  Build Data Link Layer ---------------
	bytes_to_send = 10;						// num of bytes in DLL (min num)
	temp_length = 5;						// length of msg at start

	switch (type)
	{
	case SEND_NOTHING:
		break;
	case DLL_ACK_CONFIRM:						// data not ovfl, B to A, From Sec
	{
		Build_DNP_DLL_Header(wrk_str, 5, 0x00);	// 5 - length, 0x00 - DLL ACK Confirm
		//bytes_to_send = 10;					// num of bytes in DLL(min num)
		//temp_length = 5;						// length of msg at start
		Send_232(bytes_to_send);
		break;
	}
	case LINK_STATUS:
	{
		Build_DNP_DLL_Header(wrk_str, 5, 0x0B);	// 5 - length, 0x0B - LINK_STATUS
		//bytes_to_send = 10;					// num of bytes in DLL(min num)	//temp_length = 5;						// length of msg at start
		Send_232(bytes_to_send);
		break;
	}
	case CLASS_0:									// returns all the points that are active
	{
		Build_DNP_DLL_Header(wrk_str, 0x17, 0x44);	// 0x17 (23D) - length, 0x44 - CLASS 0
		wrk_str[10] = 0xC0;							// TH Final frame & frame
		wrk_str[11] = 0xC0 | sequence_num;			// Appl cntrl 1st, final,
		if (all_stations_msg == true)
		{
			wrk_str[11] = 0xE0 | sequence_num;		//ask for confirm
			pending_confirm = CONFIRM_PENDING;
		}
		wrk_str[12] = 0x81;							// Appl. Response
		wrk_str[13] = iien1;						// IIEN 1
		wrk_str[14] = iien2;						// 10th item
		/************ Add Object One - Status  ***************/
		wrk_str[15] = 0x01;							// object 1
		wrk_str[16] = 0x01;							// variation
		wrk_str[17] = 0x00;							// qualifier
		wrk_str[18] = 0x00;							// start
		wrk_str[19] = 0x07 + EXTRA_DNP_BYTE;		// quantity 8 points
		wrk_str[20] = (uint8)Display_Info.alarm_status;
#ifdef LAST_GASP
		wrk_str[21] = Display_Info.alarm_status >> 8;	// last gasp
		/************  Add Object 30 - analog (Voltages) *****/
		wrk_str[21 + EXTRA_DNP_BYTE] = 30;				// object 30
		wrk_str[22 + EXTRA_DNP_BYTE] = 3;				// variation 3
		wrk_str[23 + EXTRA_DNP_BYTE] = 0x00;			// qualifier
		wrk_str[24 + EXTRA_DNP_BYTE] = 0;				// start
		Calc_DNPCRC(&wrk_str[10], 16);					// need CRC here because of each 16 bytes need CRC
		wrk_str[26] = crc & 0x00FF;						// append crc low byte first
		wrk_str[27] = crc >> 8;							// append high byte of crc
		wrk_str[28] = SysData.analog_points - 1;		// stop(number of active points)
#else
		/************  Add Object 30 - analog (Voltages) *****/
		wrk_str[21 + EXTRA_DNP_BYTE] = 30;							// object 30
		wrk_str[22 + EXTRA_DNP_BYTE] = 3;							// variation 3
		wrk_str[23 + EXTRA_DNP_BYTE] = 0x00;						// qualifier
		wrk_str[24 + EXTRA_DNP_BYTE] = 0;							//start
		wrk_str[25 + EXTRA_DNP_BYTE] = SysData.analog_points - 1;	// stop(number of active points)
		Calc_DNPCRC(&wrk_str[10], 16);								// need CRC here cause 16 bytes
		wrk_str[26 + EXTRA_DNP_BYTE] = crc & 0x00FF;				// append crc low byte first
		wrk_str[27 + EXTRA_DNP_BYTE] = crc >> 8;					// append high byte of crc

#endif //#ifdef LAST_GASP

		//length = 21 here
		if (SysData.analog_points == 0)
			SysData.analog_points = 1;								// just in case it is screwed up
		if (SysData.analog_points > 5)
			SysData.analog_points = 5;								// just in case it is screwed up
		//>>>>>>>>>> ASSEMBLING DNP MESSAGE for Bat volts, fault, etc >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
		//for (i = 1; i <= SysData.analog_points; i++)
		{
			if (SysData.analog_points >= 1) // if (i == 1)				// place Battery millivolts into DNP que
			{
				str_ptr = 27 + EXTRA_DNP_BYTE;						// point to last point written
				Add_DNP_BinaryLongToWorkString(battery_milliVolts, str_ptr + 1);
				bytes_to_send = 32 + EXTRA_DNP_BYTE;
				if (SysData.analog_points == 1)
				{
					Calc_DNPCRC(&wrk_str[28], 4 + EXTRA_DNP_BYTE);	// calc the CRC on 4 bytes if analog pts = 1
					wrk_str[str_ptr + 5] = crc & 0x00FF;			// append crc low byte first
					wrk_str[str_ptr + 6] = crc >> 8;				// append high byte of crc
					bytes_to_send += 2;
				}
				wrk_str[2] = 25 + EXTRA_DNP_BYTE;					// length 25 bytes for 1 analog point
			}
			if (SysData.analog_points >= 2) // if (i == 2)				// fault voltage active? if so place fault millivolts into DNP que
			{
				wrk_str[2] = 29 + EXTRA_DNP_BYTE;					// length - 29 bytes for 2 analog points
				Add_DNP_BinaryLongToWorkString(fault_millivolts, str_ptr + 5);
				bytes_to_send = 36 + EXTRA_DNP_BYTE;
				if (SysData.analog_points == 2)
				{
					Calc_DNPCRC(&wrk_str[28], 8 + EXTRA_DNP_BYTE);	// calc the CRC on 8 bytes if analog pts = 2
					wrk_str[str_ptr + 9] = crc & 0x00FF;			// append crc low byte first
					wrk_str[str_ptr + 10] = crc >> 8;				// append high byte of crc
					bytes_to_send += 2;
				}
			}
			if (SysData.analog_points >= 3) // if (i == 3)				// minus ground voltage active?
			{
				Add_DNP_BinaryLongToWorkString(minus_gnd_millivolts, str_ptr + 9);
				bytes_to_send = 40 + EXTRA_DNP_BYTE;
				if (SysData.analog_points == 3)
				{
					Calc_DNPCRC(&wrk_str[28], 12 + EXTRA_DNP_BYTE);	// calc CRC on 12 bytes if analog pts = 3
					wrk_str[str_ptr + 13] = crc & 0x00FF;			// append crc low byte first
					wrk_str[str_ptr + 14] = crc >> 8;				// append high byte of crc
					bytes_to_send += 2;
				}
				wrk_str[2] = 33 + EXTRA_DNP_BYTE;					// length - 33 bytes now
			}
			if (SysData.analog_points >= 4) // if (i == 4)				// Send ripple voltage
			{
				wrk_str[str_ptr + 13] = (uint8)ripple_milliVolts;
				wrk_str[str_ptr + 14] = (uint8)(ripple_milliVolts >> 8);
				wrk_str[str_ptr + 15] = (uint8)(ripple_milliVolts >> 16);
#ifdef LAST_GASP
				Calc_DNPCRC(&wrk_str[28], 16);						// need CRC on 16 bytes here
				wrk_str[str_ptr + 16] = crc & 0x00FF;				// append crc low byte first
				wrk_str[str_ptr + 17] = crc >> 8;					// append high byte of crc
				wrk_str[str_ptr + 18] = (uint8)(rt.ripple_milliVolts >> 24);	// wrk_str[str_ptr+18] = data5.ripple_volts_byte[3];
#else
				wrk_str[str_ptr + 16] = (uint8)(ripple_milliVolts >> 24);	// wrk_str[str_ptr+16] = data5.ripple_volts_byte[3];
				Calc_DNPCRC(&wrk_str[28], 16);							// need CRC on 16 bytes here
				wrk_str[str_ptr + 17] = crc & 0x00FF;					// append crc low byte first
				wrk_str[str_ptr + 18] = crc >> 8;						// append high byte of crc
#endif //#ifdef LAST_GASP

				bytes_to_send = 46 + EXTRA_DNP_BYTE;
				wrk_str[2] = 37 + EXTRA_DNP_BYTE;
			}
			if (SysData.analog_points >= 5) // if (i == 5)				// Send ripple current
			{
				Add_DNP_BinaryLongToWorkString((int)(rt.OutData.measured.ripple_mA_f), str_ptr + 19);
				Calc_DNPCRC(&wrk_str[46 + EXTRA_DNP_BYTE], 4);			// end of msg so CRC needed on 4 bytes
				wrk_str[str_ptr + 23] = crc & 0x00FF;					// append crc low byte first
				wrk_str[str_ptr + 24] = crc >> 8;						// append high byte of crc
				bytes_to_send = 52 + EXTRA_DNP_BYTE;					// 52
				wrk_str[2] = 41 + EXTRA_DNP_BYTE;						// 41
			}
		}
		Calc_DNPCRC(&wrk_str[0], 8);									// Calc the Dll CRC on 8 bytes since length changed
		wrk_str[8] = crc & 0x00FF;										// append DLL crc low byte first
		wrk_str[9] = crc >> 8;											// append DLL high byte of crc
		Send_232(bytes_to_send);
		break;
	}
	case CLASS_1:
	{
		events_to_send = 0;												// init events to send
		Build_DNP_DLL_Header(wrk_str, 0x17, 0x44);						// 0x17 (23D) - length, 0x44 - CLASS 0
		wrk_str[10] = 0xC0;												//TH Final frame & frame
		wrk_str[11] = 0xE0 | sequence_num;								// Appl cntrl 1st, final & confirm,
		wrk_str[12] = 0x81;												// Appl. Response
		wrk_str[13] = iien1;											// IIEN 1
		wrk_str[14] = iien2;											// 10th item
		// ***** APP Layer  *******************
		str_ptr = 15;													// last + 1
		crc_cntr = 5;													// 5 bytes since last crc
		bytes_to_send = 15;
		temp_length = 10;
		// ****** OK if there are events then build reply ************
		if ((events_requested != 0) && (events_record_num > 0))			// have object 2 events?
		{
			events_to_send = events_requested;							// only send what have
			temp_length += 4;											// add in object and qualifier
			wrk_str[str_ptr] = 02;										// frozen cntr without time
			wrk_str[str_ptr + 1] = 0x01;								// variation 1
			wrk_str[str_ptr + 2] = 0x17;								// qualifier 17
			wrk_str[str_ptr + 3] = events_to_send;						// num of points..14th item
			str_ptr += 4;												// last + 1 (point to 19)
			crc_cntr += 4;												// adds first 4 bytes of object.
			bytes_to_send += 4;
			temp_length += (events_to_send * 2);						// add in according to num of records
			rec_ptr = 1;												// oldest first
			for (i = 1; i <= events_to_send; i++)						// Get all records
			{
				Read_Event(rec_ptr);									// get a history record, updates global 'event_type' and 'point_state'
				for (j = 0; j <= 1; j++)								// get record data
				{
					if (crc_cntr != 16)
					{
						if (j == 0)
							out_data = event_type;
						if (j == 1)
							out_data = point_state;						// flag = Online
						wrk_str[str_ptr] = out_data;
						crc_cntr++;
						str_ptr++;
						bytes_to_send++;								// increment for each byte of record
					}// End insert data
					else
					{
						Calc_DNPCRC(&wrk_str[str_ptr - crc_cntr], crc_cntr);
						wrk_str[str_ptr] = crc & 0x00FF;
						str_ptr++;
						wrk_str[str_ptr] = crc >> 8;
						str_ptr++;
						bytes_to_send += 2;								// account for crc bytes
						crc_cntr = 0;									// reset for 16 more bytes
						j--;											// correct j for not getting out_data
					}	// End insert CRC
				}	// End get Record Data
				events_sent++;
				rec_ptr++;												// point to next record to send
				if (rec_ptr > MAX_EVENT_NUMBER)							// however if at bottom and more to get
					rec_ptr = 1;										// start at top of queue
			} // end getting events
		} // end object 2 events
		if (crc_cntr != 0)												// need an ending CRC?
		{
			Calc_DNPCRC(&wrk_str[str_ptr - crc_cntr], crc_cntr);
			wrk_str[str_ptr] = crc & 0x00FF;
			str_ptr++;
			wrk_str[str_ptr] = crc >> 8;
			str_ptr++;
			bytes_to_send += 2;
		}//End last CRC
		wrk_str[2] = temp_length;
		Calc_DNPCRC(&wrk_str[0], 8);									// Calc Dll CRC on 8 bytes length changed
		wrk_str[8] = crc & 0x00FF;										// append DLL crc low byte first
		wrk_str[9] = crc >> 8;											// append DLL high byte of crc
		Send_232(bytes_to_send);										// send it
		pending_confirm = CONFIRM_PENDING;
		break;
	}
	case OBJECT_1_RESPONSE:
	{
		Build_DNP_DLL_Header(wrk_str, 16 + EXTRA_DNP_BYTE, 0x44);		// 0x17 (23D) - length, 0x44 - CLASS 0
		wrk_str[10] = 0xC0;												// TH Final frame & frame
		wrk_str[11] = 0xC0 | sequence_num;								// Appl cntrl 1st, final,
		if (all_stations_msg == true)
		{
			wrk_str[11] = 0xE0 | sequence_num;							// ask for confirm
			pending_confirm = CONFIRM_PENDING;
		}
		wrk_str[12] = 0x81;												// Appl. Response
		wrk_str[13] = iien1;											// IIEN 1
		wrk_str[14] = iien2;											// 10th item
		// ************ Add Object One - Status  ***************
		wrk_str[15] = 0x01;												// object 1
		wrk_str[16] = 0x01;												// variation
		wrk_str[17] = 0x00;												// qualifier
		wrk_str[18] = 0x00;												// start
		wrk_str[19] = 0x07 + EXTRA_DNP_BYTE;							// quantity 8 points
		wrk_str[20] = Display_Info.alarm_status;
#ifdef LAST_GASP
		wrk_str[21] = Display_Info.alarm_status >> 8;
#endif //#ifdef LAST_GASP
		Calc_DNPCRC(&wrk_str[10], 11 + EXTRA_DNP_BYTE);					// Calc the Dll CRC on 11 bytes
		wrk_str[21 + EXTRA_DNP_BYTE] = crc & 0x00FF;					// append DLL crc low byte first
		wrk_str[22 + EXTRA_DNP_BYTE] = crc >> 8;						// append DLL high byte of crc
		Send_232(23 + EXTRA_DNP_BYTE);									// send it
		break;
	}
	case OBJECT_30_RESPONSE:
	{
		Build_DNP_DLL_Header(wrk_str, 0x11, 0x44);						// 0x11 (17D) - length, 0x44 - CLASS 0
		wrk_str[10] = 0xC0;												// TH Final frame & frame
		wrk_str[11] = 0xC0 | sequence_num;								// Appl cntrl 1st, final,
		if (all_stations_msg == true)
		{
			wrk_str[11] = 0xE0 | sequence_num;							// ask for confirm
			pending_confirm = CONFIRM_PENDING;
		}
		wrk_str[12] = 0x81;												// Appl. Response
		wrk_str[13] = iien1;											// IIEN 1
		wrk_str[14] = iien2;											// 10th item

		// ************  Add Object 30 - analog (Voltages) *****
		// ************  variation 0 or 1 - 32 bit analog with flag ***
		if ((variation == 0) || (variation == 1))
		{
			wrk_str[15] = 30;											// object 30
			wrk_str[16] = 1;											// variation 1 32 bit with flag
			wrk_str[17] = 0x00;											// qualifier
			if ((DNPqualifier == 6) || (DNPqualifier == 0))
			{
				wrk_str[18] = DNPstart - 1;								// start
				wrk_str[19] = DNPstop - 1;								// stop
				str_ptr = 20;											// next position in array
				DNPqualifier = 0;										// change qual 6 to 0
				bytes_to_send = 20;
				temp_length = 15;
			}
			if (DNPqualifier == 1)
			{
				wrk_str[17] = 0x01;										// change to qualifier 1
				wrk_str[18] = DNPstart - 1;								// start
				wrk_str[19] = 0;										// 16 bit
				wrk_str[20] = DNPstop - 1;								// stop
				wrk_str[21] = 0;										// 16 bit
				str_ptr = 22;											// next position in array
				bytes_to_send = 22;
				temp_length = 17;
			}
			for (i = DNPstart; i <= DNPstop; i++)
			{
				if (i == 1)								// Battery voltage
				{
					wrk_str[str_ptr] = 0x01;									// Flag always on line
					wrk_str[str_ptr + 1] = (uint8)(battery_milliVolts);		// data1.voltage_byte[0];
					wrk_str[str_ptr + 2] = (uint8)(battery_milliVolts >> 8);	// data1.voltage_byte[1];
					wrk_str[str_ptr + 3] = (uint8)(battery_milliVolts >> 16);	// data1.voltage_byte[2];
					if (DNPqualifier == 1)										//-!- IK20231220 long is NOT written in consequitive bytes, upper byte at pos [6]
					{
						Calc_DNPCRC(&wrk_str[10], 16);							// calc the CRC on 16 bytes
						wrk_str[str_ptr + 4] = crc & 0x00FF;					// append crc low byte first
						wrk_str[str_ptr + 5] = crc >> 8;						// append high byte of crc
						wrk_str[str_ptr + 6] = (uint8)(battery_milliVolts >> 24);	//-!- IK20231220 long is NOT written in consequitive bytes, upper byte at pos [6]
						Calc_DNPCRC(&wrk_str[str_ptr + 6], 1);					// calc the CRC on 1 bytes
						wrk_str[str_ptr + 7] = crc & 0x00FF;					// append crc low byte first
						wrk_str[str_ptr + 8] = crc >> 8;						// append high byte of crc
						bytes_to_send += 9;										// including crc
					}
					if (DNPqualifier == 0)
					{
						wrk_str[str_ptr + 4] = (uint8)(battery_milliVolts >> 24);	// wrk_str[str_ptr + 4] = data1.voltage_byte[3]; //-!- IK20231220 long is written in consequitive bytes
						Calc_DNPCRC(&wrk_str[10], 15);							// calc the CRC on 15 bytes
						wrk_str[str_ptr + 5] = crc & 0x00FF;					// append crc low byte first
						wrk_str[str_ptr + 6] = crc >> 8;						// append high byte of crc
						bytes_to_send += 7;										// including crc
					}
					temp_length += 5;
				}
				if (i == 2)					// fault voltage active? if so do it
				{
					if (DNPqualifier == 0)
					{
						wrk_str[str_ptr + 5] = 0x01;							// Flag - on line
						Calc_DNPCRC(&wrk_str[10], 16);							// calc the CRC on 16 bytes (needed)
						wrk_str[str_ptr + 6] = crc & 0x00FF;					// append crc low byte first
						wrk_str[str_ptr + 7] = crc >> 8;						// append high byte of crc
						Add_DNP_BinaryLongToWorkString(fault_millivolts, str_ptr + 8);
						Calc_DNPCRC(&wrk_str[str_ptr + 8], 4);					// calc the CRC on 4 bytes (maybe needed)
						wrk_str[str_ptr + 12] = crc & 0x00FF;					// append crc low byte first
						wrk_str[str_ptr + 13] = crc >> 8;						// append high byte of crc
						bytes_to_send += 7;										// 9 minus 2 from last crc
					}
					if (DNPqualifier == 1)
					{
						wrk_str[str_ptr + 7] = 0x01;							// Flag - on line
						Add_DNP_BinaryLongToWorkString(fault_millivolts, str_ptr + 8);
						Calc_DNPCRC(&wrk_str[str_ptr + 6], 6);					// calc the CRC on 6 bytes
						wrk_str[str_ptr + 12] = crc & 0x00FF;					// append crc low byte first
						wrk_str[str_ptr + 13] = crc >> 8;						// append high byte of crc
						bytes_to_send += 5;										// 7 minus 2 from last crc
					}
					temp_length += 5;
				}
				if (i == 3)					// minus ground voltage active?
				{
					if (DNPqualifier == 0)
					{
						wrk_str[str_ptr + 12] = 0x01;							// Flag on line (overwrite last crc)
						Add_DNP_BinaryLongToWorkString(minus_gnd_millivolts, str_ptr + 13);	// 32 bit -gnd V
						Calc_DNPCRC(&wrk_str[str_ptr + 8], 9);					// calc the CRC on 9 bytes since last crc
						wrk_str[str_ptr + 17] = crc & 0x00FF;					// append crc low byte first
						wrk_str[str_ptr + 18] = crc >> 8;						// append high byte of crc
					}
					if (DNPqualifier == 1)
					{
						wrk_str[str_ptr + 12] = 0x01;							// Flag on line (overwrite last crc)
						Add_DNP_BinaryLongToWorkString(minus_gnd_millivolts, str_ptr + 13);//32 bit -gnd V
						Calc_DNPCRC(&wrk_str[str_ptr + 6], 11);					// calc the CRC on 9 bytes since last crc
						wrk_str[str_ptr + 17] = crc & 0x00FF;					// append crc low byte first
						wrk_str[str_ptr + 18] = crc >> 8;						// append high byte of crc
					}
					temp_length += 5;
					bytes_to_send += 5;											// 7 - 2 = 5
				}
				if (i == 4)														// ripple voltage
				{
					if (DNPqualifier == 0)
					{
						wrk_str[str_ptr + 17] = 0x01;							// Flag on line
						Add_DNP_BinaryLongToWorkString(ripple_milliVolts, str_ptr + 18);	// 32 bit ripple I
						Calc_DNPCRC(&wrk_str[str_ptr + 8], 14);					// calc the CRC on 14 bytes
						wrk_str[str_ptr + 22] = crc & 0x00FF;					// append crc low byte first
						wrk_str[str_ptr + 23] = crc >> 8;						// append high byte of crc
					}
					if (DNPqualifier == 1)
					{
						wrk_str[str_ptr + 17] = 0x01;							// Flag on line
						Add_DNP_BinaryLongToWorkString(ripple_milliVolts, str_ptr + 18); // 32 bit ripple I
						Calc_DNPCRC(&wrk_str[str_ptr + 6], 16);					// calc the CRC on 16 bytes needed
						wrk_str[str_ptr + 22] = crc & 0x00FF;					// append crc low byte first
						wrk_str[str_ptr + 23] = crc >> 8;						// append high byte of crc
					}
					temp_length += 5;
					bytes_to_send += 5;											// 7-2=5
				}
				if (i == 5)					// ripple I
				{
					if (DNPqualifier == 0)
					{
						wrk_str[str_ptr + 22] = 0x01;							// Flag on line
						wrk_str[str_ptr + 23] = (uint8)(ripple_milliAmpers);		// data4.ripple_i_byte[0];//32 bit ripple Current mA
						Calc_DNPCRC(&wrk_str[str_ptr + 8], 16);					// need CRC on 16 bytes here
						wrk_str[str_ptr + 24] = crc & 0x00FF;					// append crc low byte first
						wrk_str[str_ptr + 25] = crc >> 8;						// append high byte of crc
						wrk_str[str_ptr + 26] = (uint8)(ripple_milliAmpers >> 8);	// data4.ripple_i_byte[1];
						wrk_str[str_ptr + 27] = (uint8)(ripple_milliAmpers >> 16);	// data4.ripple_i_byte[2];
						wrk_str[str_ptr + 28] = (uint8)(ripple_milliAmpers >> 24);	// data4.ripple_i_byte[3];
						Calc_DNPCRC(&wrk_str[str_ptr + 26], 3);					// calc the CRC on 3 bytes
						wrk_str[str_ptr + 29] = crc & 0x00FF;					// append crc low byte first
						wrk_str[str_ptr + 30] = crc >> 8;						// append high byte of crc
						bytes_to_send += 7;										// 9-2=7
					}
					if (DNPqualifier == 1)
					{
						wrk_str[str_ptr + 24] = 0x01;							// Flag on line
						Add_DNP_BinaryLongToWorkString(ripple_milliAmpers, str_ptr + 25);
						Calc_DNPCRC(&wrk_str[str_ptr + 24], 5);					// calc the CRC on 5 bytes
						wrk_str[str_ptr + 29] = crc & 0x00FF;					// append crc low byte first
						wrk_str[str_ptr + 30] = crc >> 8;						// append high byte of crc
						bytes_to_send += 7;
					}
					temp_length += 5;
				}
			}
			wrk_str[2] = temp_length;
		}// end variation 0 & 1 32 bit with flag
		// ****** Variation 3 - 32 bit without flag

		if (variation == 3)
		{
			wrk_str[15] = 30;													// object 30 (6th since crc)
			wrk_str[16] = 3;													// variation 1 32 bit without flag
			wrk_str[17] = 0x00;													// qualifier
			if ((DNPqualifier == 6) || (DNPqualifier == 0))
			{
				wrk_str[18] = DNPstart - 1;										// start
				wrk_str[19] = DNPstop - 1;										// stop
				str_ptr = 20;													// next position in array
				DNPqualifier = 0;												// change qual 6 to 0
				bytes_to_send = 20;
				temp_length = 15;
			}
			if (DNPqualifier == 1)
			{
				wrk_str[17] = 0x01;												//change to qualifier 1
				wrk_str[18] = DNPstart - 1;										// start
				wrk_str[19] = 0;												// 16 bit
				wrk_str[20] = DNPstop - 1;										// stop
				wrk_str[21] = 0;												// 16 bit
				str_ptr = 22;													// next position in array
				bytes_to_send = 22;
				temp_length = 17;
			}
			for (i = DNPstart; i <= DNPstop; i++)
			{
				if (i == 1)               //Battery voltage
				{
					wrk_str[str_ptr] = (uint8)(battery_milliVolts);		// data1.voltage_byte[0];
					wrk_str[str_ptr + 1] = (uint8)(battery_milliVolts >> 8);	// data1.voltage_byte[1];
					wrk_str[str_ptr + 2] = (uint8)(battery_milliVolts >> 16);// data1.voltage_byte[2];
					wrk_str[str_ptr + 3] = (uint8)(battery_milliVolts >> 24);// data1.voltage_byte[3];
					if (DNPqualifier == 1)
					{
						Calc_DNPCRC(&wrk_str[10], 16);							// calc the CRC on 16 bytes
						wrk_str[str_ptr + 4] = crc & 0x00FF;					// append crc low byte first
						wrk_str[str_ptr + 5] = crc >> 8;						// append high byte of crc
						bytes_to_send += 6;										// including crc
					}
					if (DNPqualifier == 0)
					{
						Calc_DNPCRC(&wrk_str[10], 14);							// calc the CRC on 14 bytes
						wrk_str[str_ptr + 4] = crc & 0x00FF;					// append crc low byte first
						wrk_str[str_ptr + 5] = crc >> 8;						// append high byte of crc
						bytes_to_send += 6;										// including crc
					}
					temp_length += 4;
				}
				if (i == 2)               //fault voltage active? if so do it
				{
					if (DNPqualifier == 0)
					{
						wrk_str[str_ptr + 4] = (uint8)(fault_millivolts);	// data3.fault_voltage_byte[0];
						wrk_str[str_ptr + 5] = (uint8)(fault_millivolts >> 8);// data3.fault_voltage_byte[1];
						Calc_DNPCRC(&wrk_str[10], 16);							// calc the CRC on 16 bytes
						wrk_str[str_ptr + 6] = crc & 0x00FF;					// crc low byte first
						wrk_str[str_ptr + 7] = crc >> 8;						// append high byte of crc
						wrk_str[str_ptr + 8] = (uint8)(fault_millivolts >> 16);	// data3.fault_voltage_byte[2];
						wrk_str[str_ptr + 9] = (uint8)(fault_millivolts >> 24);	// data3.fault_voltage_byte[3];
						Calc_DNPCRC(&wrk_str[str_ptr + 8], 2);					// calc the CRC on 2 bytes
						wrk_str[str_ptr + 10] = crc & 0x00FF;					// crc low byte first
						wrk_str[str_ptr + 11] = crc >> 8;						// append high byte of crc
						bytes_to_send += 6;										// 8 minus 2 from last crc
					}
					if (DNPqualifier == 1)
					{
						Add_DNP_BinaryLongToWorkString(fault_millivolts, str_ptr + 6);
						Calc_DNPCRC(&wrk_str[str_ptr + 6], 4);					// calc the CRC on 4 bytes (maybe needed)
						wrk_str[str_ptr + 10] = crc & 0x00FF;					// append crc low byte first
						wrk_str[str_ptr + 11] = crc >> 8;						// append high byte of crc
						bytes_to_send += 6;										// last crc needed
					}
					temp_length += 4;
				}
				if (i == 3)					// minus ground voltage active?
				{
					if (DNPqualifier == 0)
					{
						Add_DNP_BinaryLongToWorkString(minus_gnd_millivolts, str_ptr + 10);
						Calc_DNPCRC(&wrk_str[str_ptr + 8], 6);					// calc the CRC on 6 bytes since last crc
						wrk_str[str_ptr + 14] = crc & 0x00FF;					// append crc low byte first
						wrk_str[str_ptr + 15] = crc >> 8;						// append high byte of crc
					}
					if (DNPqualifier == 1)
					{
						Add_DNP_BinaryLongToWorkString(minus_gnd_millivolts, str_ptr + 10);
						Calc_DNPCRC(&wrk_str[str_ptr + 6], 8);					// calc the CRC on 8 bytes since last crc
						wrk_str[str_ptr + 14] = crc & 0x00FF;					// append crc low byte first
						wrk_str[str_ptr + 15] = crc >> 8;						// append high byte of crc
					}
					temp_length += 4;
					bytes_to_send += 4;											// 6 - 2 = 4
				}
				if (i == 4)					// ripple current
				{
					if (DNPqualifier == 0)
					{
						Add_DNP_BinaryLongToWorkString(ripple_milliVolts, str_ptr + 14);
						Calc_DNPCRC(&wrk_str[str_ptr + 8], 10);					// calc the CRC on 10 bytes
						wrk_str[str_ptr + 18] = crc & 0x00FF;					// append crc low byte first
						wrk_str[str_ptr + 19] = crc >> 8;						// append high byte of crc
					}
					if (DNPqualifier == 1)
					{
						Add_DNP_BinaryLongToWorkString(ripple_milliVolts, str_ptr + 14);
						Calc_DNPCRC(&wrk_str[str_ptr + 6], 12);					// calc the CRC on 12 bytes needed
						wrk_str[str_ptr + 18] = crc & 0x00FF;					// append crc low byte first
						wrk_str[str_ptr + 19] = crc >> 8;						// append high byte of crc
					}
					temp_length += 4;
					bytes_to_send += 4;											// 6-2=4
				}
				if (i == 5)
				{
					if (DNPqualifier == 0)
					{
						Add_DNP_BinaryLongToWorkString(ripple_milliAmpers, str_ptr + 18);
						Calc_DNPCRC(&wrk_str[str_ptr + 8], 14);					// need CRC on 14 bytes here
						wrk_str[str_ptr + 22] = crc & 0x00FF;					// append crc low byte first
						wrk_str[str_ptr + 23] = crc >> 8;						// append high byte of crc
						bytes_to_send += 4; //
					}
					if (DNPqualifier == 1)
					{
						Add_DNP_BinaryLongToWorkString(ripple_milliAmpers, str_ptr + 18);
						Calc_DNPCRC(&wrk_str[str_ptr + 6], 16);					// calc the CRC on 16 bytes
						wrk_str[str_ptr + 22] = crc & 0x00FF;					// append crc low byte first
						wrk_str[str_ptr + 23] = crc >> 8;						// append high byte of crc
						bytes_to_send += 4;
					}
					temp_length += 4;
				} // end i = 5
			} // end variation 2 - 32 bit without flag
			wrk_str[2] = temp_length;
		} // end variation 3 & not qualifier 6
		Calc_DNPCRC(&wrk_str[0], 8);											// Calc the Dll CRC on 8 bytes
		wrk_str[8] = crc & 0x00FF;												// append DLL crc low byte first
		wrk_str[9] = crc >> 8;													// append DLL high byte of crc
		Send_232(bytes_to_send);												// send it
		break;
	}
	case ALL_DATA:
	{
		events_to_send = 0;														// init events to send
		events_sent = 0;														// incase no binary will cause battery
		Build_DNP_DLL_Header(wrk_str, 0x11 + EXTRA_DNP_BYTE, 0x44);				// 0x11 (17D) - length, 0x44 - CLASS 0

		wrk_str[10] = 0xC0;														// TH Final frame & frame
		wrk_str[11] = 0xC0 | sequence_num;										// Appl cntrl 1st, final,
		if (all_stations_msg == true)
		{
			wrk_str[11] = 0xE0 | sequence_num;									// ask for confirm
			pending_confirm = CONFIRM_PENDING;
		}
		wrk_str[12] = 0x81;														// Appl. Response
		wrk_str[13] = iien1;													// IIEN 1
		wrk_str[14] = iien2;													// 10th item
		if (events_requested != 0)
			wrk_str[11] = 0xE0 | sequence_num;									// Appl cntrl 1st, final, confirm
		// ************ APP Layer  *******************
		// ************ Add Object One - Status  ***************
		wrk_str[15] = 0x01;														// object 1
		wrk_str[16] = 0x01;														// variation
		wrk_str[17] = 0x00;														// qualifier
		wrk_str[18] = 0x00;														// start
		wrk_str[19] = 0x07 + EXTRA_DNP_BYTE;									// quantity 8 or 9 points
		wrk_str[20] = Display_Info.alarm_status;
#ifdef LAST_GASP
		wrk_str[21] = Display_Info.alarm_status >> 8;;
#endif //#ifdef LAST_GASP
		// ************  Add Object 30 - analog (Voltages) *****
		wrk_str[21 + EXTRA_DNP_BYTE] = 30;										// object 30
		wrk_str[22 + EXTRA_DNP_BYTE] = 1;										// variation 1: 32 bit with flag
		wrk_str[23 + EXTRA_DNP_BYTE] = 0x00;									// qualifier
		wrk_str[24 + EXTRA_DNP_BYTE] = 0;										// start
#ifndef LAST_GASP
		wrk_str[25] = SysData.analog_points - 1;								// stop(number of active points)
#endif //#ifdef LAST_GASP
		Calc_DNPCRC(&wrk_str[10], 16);											// calc the CRC on 16 bytes
		wrk_str[26] = crc & 0x00FF;												// append crc low byte first
		wrk_str[27] = crc >> 8;													// append high byte of crc
#ifdef LAST_GASP
		wrk_str[28] = SysData.analog_points - 1;//stop(number of active points)
#endif //#ifdef LAST_GASP
		temp_length = 21 + EXTRA_DNP_BYTE;
		if (SysData.analog_points == 0)
			SysData.analog_points = 1;											// just incase it is screwed up
		if (SysData.analog_points > 5)
			SysData.analog_points = 5;
		for (i = 1; i <= SysData.analog_points; i++)
		{
			if (i == 1)
			{
				str_ptr = 27 + EXTRA_DNP_BYTE;									// point to last point written
				wrk_str[str_ptr + 1] = 0x01;									// Flag always on line
				Add_DNP_BinaryLongToWorkString(battery_milliVolts, str_ptr + 2);
				str_ptr += 5;													// account for last (5)
				//-!- IK20240222 bug in Rev.J: CRC assigned in wrong place: wrk_str[str_ptr+6]=crc & 0x00FF; but because of increment by 5, it should be wrk_str[str_ptr+1] to account for last (5)
				//-!- IK20240222>> FIX: str_ptr += 5;							// disable increment
				bytes_to_send = 33 + EXTRA_DNP_BYTE;
				crc_cntr = 5 + EXTRA_DNP_BYTE;
				temp_length += 5;
				if ((SysData.analog_points == 1) && (events_requested == 0))
				{
					Calc_DNPCRC(&wrk_str[28], 5 + EXTRA_DNP_BYTE);				// calc the CRC on 5 bytes
					wrk_str[str_ptr + 6] = crc & 0x00FF;						// append crc low byte first
					wrk_str[str_ptr + 7] = crc >> 8;							// append high byte of crc
					bytes_to_send += 2;
					crc_cntr = 0;
				}
				wrk_str[2] = temp_length;										// length for 1 analog point
			}
			if (i == 2)															// fault voltage active? if so do it
			{
				str_ptr = 32 + EXTRA_DNP_BYTE;
				wrk_str[str_ptr + 1] = 0x01;									// Flag - on line
				Add_DNP_BinaryLongToWorkString(fault_millivolts, str_ptr + 2);
				//-!- IK20240222 bug in Rev.J: CRC assigned in wrong place: wrk_str[str_ptr+6]=crc & 0x00FF; but because of increment by 5, it should be wrk_str[str_ptr+1] to account for last (5)
				//-!- IK20240222>> FIX: str_ptr += 5; // disable increment
				bytes_to_send = 38 + EXTRA_DNP_BYTE;
				crc_cntr = 10 + EXTRA_DNP_BYTE;
				temp_length += 5;
				if ((SysData.analog_points == 2) && (events_requested == 0))
				{
					Calc_DNPCRC(&wrk_str[28], 10);								// calc the CRC on 10 bytes
					wrk_str[str_ptr + 6] = crc & 0x00FF;						// append crc low byte first
					wrk_str[str_ptr + 7] = crc >> 8;							// append high byte of crc
					bytes_to_send += 2;
					crc_cntr = 0;
				}
				wrk_str[2] = temp_length;
			}
			if (i == 3)															// minus ground voltage active?
			{
				str_ptr = 37 + EXTRA_DNP_BYTE;
				wrk_str[str_ptr + 1] = 0x01;  //Flag on line
				Add_DNP_BinaryLongToWorkString(minus_gnd_millivolts, str_ptr + 2);
				//-!- IK20240222 bug in Rev.J: CRC assigned in wrong place: wrk_str[str_ptr+6]=crc & 0x00FF; but because of increment by 5, it should be wrk_str[str_ptr+1] to account for last (5)
				//-!- IK20240222>> FIX: str_ptr += 5; // disable increment
				bytes_to_send = 43 + EXTRA_DNP_BYTE;
				crc_cntr = 15 + EXTRA_DNP_BYTE;
				temp_length += 5;
				if ((SysData.analog_points == 3) && (events_requested == 0))
				{
					Calc_DNPCRC(&wrk_str[28], 15);								// calc the CRC on 15 bytes
					wrk_str[str_ptr + 6] = crc & 0x00FF;						// append crc low byte first
					wrk_str[str_ptr + 7] = crc >> 8;							// append high byte of crc
					bytes_to_send += 2;
					crc_cntr = 0;
				}
				wrk_str[2] = temp_length;
			}
			if (i == 4)
			{
				str_ptr = 42 + EXTRA_DNP_BYTE;
#ifdef LAST_GASP
				Calc_DNPCRC(&wrk_str[28], 16);//calc the CRC on 16 bytes
				wrk_str[str_ptr + 1] = crc & 0x00FF; //append crc low byte first
				wrk_str[str_ptr + 2] = crc >> 8; //append high byte of crc
				wrk_str[str_ptr + 3] = 0x01;   //Flag on line
#else
				wrk_str[str_ptr + 1] = 0x01;									// Flag on line
				Calc_DNPCRC(&wrk_str[28], 16);									// calc the CRC on 16 bytes
				wrk_str[str_ptr + 2] = crc & 0x00FF;							// append crc low byte first
				wrk_str[str_ptr + 3] = crc >> 8;								// append high byte of crc
#endif //#ifdef LAST_GASP
				//wrk_str[str_ptr + 4] = data5.ripple_volts_byte[0];//32 bit ripple I
				//wrk_str[str_ptr + 5] = data5.ripple_volts_byte[1];
				//wrk_str[str_ptr + 6] = data5.ripple_volts_byte[2];
				//wrk_str[str_ptr + 7] = data5.ripple_volts_byte[3];
				Add_DNP_BinaryLongToWorkString((int)(rt.OutData.measured.ripple_mV_f), str_ptr + 4);
#ifdef LAST_GASP
				str_ptr += 7;                //account for last (7)
#endif //#ifdef LAST_GASP
				//-!- IK20240222>> FIX: str_ptr += 7;							// disable increment
				bytes_to_send = 50 + EXTRA_DNP_BYTE;
				crc_cntr = 4 + EXTRA_DNP_BYTE;
				temp_length += 5;
				if ((SysData.analog_points == 4) && (events_requested == 0))
				{
					Calc_DNPCRC(&wrk_str[str_ptr + 4], 4);						// calc the CRC on 4 bytes
#ifdef LAST_GASP
					wrk_str[str_ptr + 1] = crc & 0x00FF;						// append crc low byte first
					wrk_str[str_ptr + 2] = crc >> 8;							// append high byte of crc
#else
					wrk_str[str_ptr + 8] = crc & 0x00FF;						// append crc low byte first
					wrk_str[str_ptr + 9] = crc >> 8;							// append high byte of crc
#endif //#ifdef LAST_GASP
					bytes_to_send += 2;
					crc_cntr = 0;
				}
				wrk_str[2] = temp_length;
			}
			if (i == 5)
			{
				str_ptr = 49 + EXTRA_DNP_BYTE;
				wrk_str[str_ptr + 1] = 0x01;									// Flag on line
				Add_DNP_BinaryLongToWorkString(ripple_milliAmpers, str_ptr + 2);
				str_ptr += 5;													// account for last (5) for use with events
				if (events_requested == 0)										// no events so put the ending crc on
				{
					Calc_DNPCRC(&wrk_str[46], 9);								// calc the CRC on 9 bytes
					wrk_str[str_ptr + 6] = crc & 0x00FF;						// append crc low byte first
					wrk_str[str_ptr + 7] = crc >> 8;							// append high byte of crc
					bytes_to_send = 57;
				}
				else
				{
					bytes_to_send = 55 + EXTRA_DNP_BYTE;
				}
				temp_length += 5;
				wrk_str[2] = temp_length;
				crc_cntr = 9 + EXTRA_DNP_BYTE;
			}
		}

		// ****************** Add in class 1 ***********************
		// ****** OK if there are events then build reply ************
		if (events_requested != 0)      //have object 2 events?
		{
			events_to_send = events_requested;									// used in confirmation to erase
#ifdef LAST_GASP
			if (crc_cntr == 16) //only happens when analogs = 3
			{
				Calc_DNPCRC(&wrk_str[28], 16);									// calc the CRC on 16 bytes
				wrk_str[str_ptr + 1] = crc & 0x00FF;							// append crc low byte first
				wrk_str[str_ptr + 2] = crc >> 8;								// append high byte of crc
#else
			wrk_str[str_ptr + 1] = 01;											// frozen cntr without time
			crc_cntr++;
			if (crc_cntr == DNP_FRAME_SIZE)										// only happens when analogs = 3
			{
				Calc_DNPCRC(&wrk_str[28], 16);									// calc the CRC on 16 bytes
				wrk_str[str_ptr + 2] = crc & 0x00FF;							// append crc low byte first
				wrk_str[str_ptr + 3] = crc >> 8;								// append high byte of crc
#endif //#ifdef LAST_GASP
				str_ptr += 2;
				bytes_to_send += 2;
				crc_cntr = 0;
			}
#ifdef LAST_GASP
			wrk_str[str_ptr + 1] = 01;											// frozen cntr without time
#endif //#ifdef LAST_GASP
			wrk_str[str_ptr + 2] = 0x02;										// variation 2
			wrk_str[str_ptr + 3] = 0x17;										// qualifier 17
			wrk_str[str_ptr + 4] = events_to_send;								// num of points..14th item
			crc_cntr += (3 + EXTRA_DNP_BYTE);
			str_ptr += 4;														// last + 1
			temp_length += 4;													// add in object and qualifier
			bytes_to_send += 4;
			temp_length += (events_to_send * 2);								// add according to num of records
			rec_ptr = 1;														// oldest first
			for (i = 1; i <= events_to_send; i++)								// Get all records
			{
				Read_Event(rec_ptr);											// get a history record Oldest first
				for (j = 0; j <= 1; j++)										// get record data
				{
					if (crc_cntr != DNP_FRAME_SIZE)
					{
						if (j == 0)
							out_data = event_type;
						if (j == 1)
							out_data = point_state;								// flag = Online

						wrk_str[str_ptr + 1] = out_data;
						crc_cntr++;
						str_ptr++;
						bytes_to_send++;										// increment for each byte of record
					} // End insert data
					else
					{
						Calc_DNPCRC(&wrk_str[(str_ptr + 1) - crc_cntr], crc_cntr);
						wrk_str[str_ptr + 1] = crc & 0x00FF;
						str_ptr++;
						wrk_str[str_ptr + 1] = crc >> 8;
						str_ptr++;
						bytes_to_send += 2;										// account for crc bytes
						crc_cntr = 0;											// reset for 16 more bytes
						j--;													// correct j for not getting out_data
					} // End insert CRC
				} // End get Record Data
				rec_ptr++;														// point to next record to send
				events_sent++;
				if (rec_ptr > MAX_EVENT_NUMBER)									// however if at bottom and more to get
					rec_ptr = 1;												// start at top of queue
			}//end getting events
			if (crc_cntr != 0)													// need an ending CRC?
			{
				Calc_DNPCRC(&wrk_str[(str_ptr + 1) - crc_cntr], crc_cntr);
				wrk_str[str_ptr + 1] = crc & 0x00FF;
				str_ptr++;
				wrk_str[str_ptr + 1] = crc >> 8;
				str_ptr++;
				bytes_to_send += 2;
			} // End last CRC
			} // end object 2 events

		wrk_str[2] = temp_length;
		Calc_DNPCRC(&wrk_str[0], 8);											// Calc the Dll CRC on 8 bytes
		wrk_str[8] = crc & 0x00FF;												// append DLL crc low byte first
		wrk_str[9] = crc >> 8;													// append DLL high byte of crc
		Send_232(bytes_to_send);
		pending_confirm = CONFIRM_PENDING;
		break;
		}
	case ERROR_RESPONSE:
	{
		Build_DNP_DLL_Header(wrk_str, 0x0A, 0x44);								// 0x0A (10D) - length, 0x44 - CLASS 0

		wrk_str[10] = 0xC0;														// TH Final frame & frame
		wrk_str[11] = 0xC0 | sequence_num;										// Appl cntrl 1st, final,
		if (all_stations_msg == true)
		{
			wrk_str[11] = 0xE0 | sequence_num;
			pending_confirm = CONFIRM_PENDING;
		}
		wrk_str[12] = 0x81;														// Appl. Response
		wrk_str[13] = iien1;													// IIEN 1
		wrk_str[14] = iien2;													// 10th item
		Calc_DNPCRC(&wrk_str[10], 5);											// calc the CRC on 5 bytes
		wrk_str[15] = crc & 0x00FF;												// append crc low byte first
		wrk_str[16] = crc >> 8;													// append high byte of crc
		Send_232(17);															// send it
		break;
	}
	case TIME_DELAY_RESPONSE:
	{	// ************ APP Layer  ***************
		Build_DNP_DLL_Header(wrk_str, 16, 0x44);								// 0x10 (16D) - length, 0x44 - CLASS 0
		wrk_str[10] = 0xC0;														// TH Final frame & frame
		wrk_str[11] = 0xC0 | sequence_num;										//Appl cntrl 1st, final,
		wrk_str[12] = 0x81;														// Appl. Response
		wrk_str[13] = iien1;													// IIEN 1
		wrk_str[14] = iien2;													// 10th item
		wrk_str[15] = 52;														// object 52 - time delay fine
		wrk_str[16] = 2;														// variation 2
		wrk_str[17] = 0x07;														// qualifier
		wrk_str[18] = 0x01;														// start
		wrk_str[19] = DNP_time_delay & 0x00FF;
		wrk_str[20] = DNP_time_delay >> 8;
		Calc_DNPCRC(&wrk_str[10], 11);											// calc the CRC on 11 bytes
		wrk_str[21] = crc & 0x00FF;												// append crc low byte first
		wrk_str[22] = crc >> 8;													// append high byte of crc
		Send_232(23);															// send it
		break;
	}
	default:																	// none of the above
		break;																	// so do nothing
	}//end switch

	send_dnp = 0;
	type = 0;
	iien1 = iien1 & 0xBF;														// clear device trouble bit
	if (all_stations_msg == false)
		iien1 = iien1 & 0xFE;													// clear all stations
	comm_state = RCVANDXMT;													// in enum UART_Events
	}
/********************************************************************/
// IK20251029 - strange behavior in Visual Studio: - without fake code below (need extra '}') the function Send_DNP_Msg() is not detected by Intellisense and not appear in the list of functions
#if (0)
}//end switch
#endif //if (0)

/*                 D N P   A p p                                    */
/********************************************************************/
/*  DESCRIPTION: This routine parses the DNP msg for action.

   Inputs: rt.HostRxBuff,type
   Outputs: object_string, send, TWI output msg
   Notes: None.

   Revision:    11/22/15      REC     Created.
				11/05/18      REC     Added points for PWM calibration
*/
/********************************************************************/
void DNP_App(void)
{
	uint8 i, j;
	long tmp_cal;
	Uchar DNP_point_CmdCode;

	send_dnp = 0;   //init to send nothing
	if (obj_ptr == 0)
		length--;												// forget the start of APP Layer

	sequence_num = rt.HostRxBuff[11] & 0x0F;					// get sequence number
	for (j = 0; j <= 20; j++)
	{
		object_string[j] = 0xFF;								// clear out last stuff
	}
	j = 0;
	for (i = 0; i <= 20; i++)									// make a copy of the
	{
		if (i == 13)											// jumps over crc was 12
			i += 2;
		object_string[j++] = rt.HostRxBuff[13 + i];				// object info
	}
	// ---------------------------  Start Parsing ----------------------
	iien2 = BAD_FUNCTION;						// FUNCTION not implemented
	//---------------------------  Confirm Reply  ---------------------
	DNP_point_CmdCode = object_string[obj_ptr + 4];
	if (rt.HostRxBuff[12] == CONFIRM)
	{
		length = 0;												// only do this once
		iien2 = GOOD_APDU;
		if (pending_confirm == CONFIRM_PENDING)
		{
			if (events_record_num > 0)							// if sent obj 2 events
			{
				Erase_Events(events_requested);					// erase them
			}
			events_requested = 0;
			ovr_flow = false;									// clear buffer overflow
			pending_confirm = NOTHING_PENDING;
			if (all_stations_msg == true)
			{
				all_stations_msg = false;
				iien1 &= 0xFE;
			}
			iien2 = GOOD_APDU;
			if (events_record_num == 0)   //new
				iien1 &= 0xFD;									// clear have class 1 data
		}//end pending confirm
	}//end confirm

	/*-----  Read   ---------------*/
	if (rt.HostRxBuff[12] == READ)								// Read stuff ?
	{
		iien2 = OBJECT_UNKNOWN;									// object unknown for now
		/*-------- Group 1 Read -----*/
		if (object_string[obj_ptr] == 1)
		{
			DNPqualifier = object_string[obj_ptr + 2];			// get qualifier
			variation = object_string[obj_ptr + 1];				// get variation
			if ((variation == 0) || (variation == 1))			// Check for allowed variations
			{
				iien2 = PARAMETER_ERROR;						// point out of range
				if (DNPqualifier == 6)							// Qualifier 6 ?
				{
					iien2 = GOOD_APDU;							// good function & object
					DNPstart = 1;								// first point
					DNPstop = SysData.analog_points;			// last point is last active point
					send_dnp = OBJECT_1_RESPONSE;
				}//end qual 6
				if (DNPqualifier == 0)							// variation 1 32 bit with flag
				{
					DNPstart = object_string[obj_ptr + 3] + 1;	// get start/change range to 1 - 6
					DNPstop = DNP_point_CmdCode + 1;			// get stop
					if ((DNPstart <= 8) && (DNPstop <= 8))		// 8 points(change to actual)
					{
						iien2 = GOOD_APDU;				// good function & object
						send_dnp = OBJECT_1_RESPONSE;
					}
				}//end qualifier 0   single point or range of static points
				if (DNPqualifier == 1)							// variation 3 32 bit with flag
				{
					DNPstart = (DNP_point_CmdCode * 256) + object_string[obj_ptr + 3];//get start
					DNPstart++;									// increment so that its base is 1
					DNPstop = (object_string[obj_ptr + 6] * 256) + object_string[obj_ptr + 5];  //get stop
					DNPstop++;									// increment so that its base is 1
					iien2 = PARAMETER_ERROR;					// point out of range
					if ((DNPstart <= 8) && (DNPstop <= 8))		// 8 points (change to actual)
					{
						iien2 = GOOD_APDU;						// good function & object
						send_dnp = OBJECT_1_RESPONSE;
					}
				} // end qualifier 1   single point or range of static points
			} // end accepted variations
		} // end binary Group 1

		/*------ Group 30 Read ----*/
		if (object_string[obj_ptr] == 30)						// Group 30 Analog input
		{
			DNPqualifier = object_string[obj_ptr + 2];			// get qualifier
			variation = object_string[obj_ptr + 1];				// get variation
			if ((variation == 0) || (variation == 1) ||			// Check for allowed
				(variation == 3))								// variations
			{
				iien2 = PARAMETER_ERROR;						// point out of range
				if (DNPqualifier == 6)							// Qualifier 6 ?
				{
					iien2 = GOOD_APDU;							// good function & object
					DNPstart = 1;								// first point
					DNPstop = SysData.analog_points;			// last point is last active point
					send_dnp = OBJECT_30_RESPONSE;
				}//end qual 6
				if (DNPqualifier == 0)							// variation 1: 32 bit with flag
				{
					DNPstart = object_string[obj_ptr + 3] + 1;	// get start/change range to 1 - 6
					DNPstop = DNP_point_CmdCode + 1;			// get stop
					if ((DNPstart <= SysData.analog_points + 1) && (DNPstop <= SysData.analog_points + 1))
					{
						iien2 = GOOD_APDU;						// good function & object
						send_dnp = OBJECT_30_RESPONSE;
					}
				} // end qualifier 0   single point or range of static points
				if (DNPqualifier == 1)							// variation 3 32 bit with flag
				{
					DNPstart = (DNP_point_CmdCode * 256) + object_string[obj_ptr + 3];	// get start
					DNPstart++;									// increment so that its base is 1
					DNPstop = (object_string[obj_ptr + 6] * 256) + object_string[obj_ptr + 5];	// get stop
					DNPstop++;									// increment so that its base is 1
					iien2 = PARAMETER_ERROR;        //point out of range
					if ((DNPstart <= SysData.analog_points + 1) && (DNPstop <= SysData.analog_points + 1))
					{
						iien2 = GOOD_APDU;						// good function & object
						send_dnp = OBJECT_30_RESPONSE;
					}
				} // end DNPqualifier 1   single point or range of static points
			} // end var 0,1,3
		} // end binary Group 30


		//------ Class Responses -------------
		if (object_string[obj_ptr] == 60)						// class query ?
		{														// Check for any request for static data
			iien2 = OBJECT_UNKNOWN;								// object unknown for now
			if ((object_string[obj_ptr + 1] == 0) || (object_string[obj_ptr + 1] == 1))
			{
				iien2 = PARAMETER_ERROR;						// point out of range
				if (object_string[obj_ptr + 2] == 6)			// Qualifier 6 ?
				{
					iien2 = GOOD_APDU;							// good function & object
					send_dnp = CLASS_0;
				}
			}
			if (((DNP_point_CmdCode == 0) || (DNP_point_CmdCode == 1)) &&
				(dnp_length > 11))								// request for default or class 0?
			{
				iien2 = PARAMETER_ERROR;						// point out of range
				if (object_string[obj_ptr + 5] == 6)			// Qualifier 6 ?
				{
					iien2 = GOOD_APDU;							// good function & object
					send_dnp = CLASS_0;
				}
			}
			if (((object_string[obj_ptr + 7] == 0) || (object_string[obj_ptr + 7] == 1)) &&
				((dnp_length > 13) && (dnp_length < 18)))		// request for default or class 0?
			{
				iien2 = PARAMETER_ERROR;						// point out of range
				if (object_string[obj_ptr + 8] == 6)			// Qualifier 6 ?
				{
					iien2 = GOOD_APDU;							// good function & object
					send_dnp = CLASS_0;
				}
			}
			if (((object_string[obj_ptr + 10] == 0) || (object_string[obj_ptr + 10] == 1)) &&	// request for default or class 0?
				((dnp_length > 17) && (dnp_length < 20)))
			{
				iien2 = PARAMETER_ERROR;						// point out of range
				if (object_string[obj_ptr + 11] == 6)			// Qualifier 6 ?
				{
					iien2 = GOOD_APDU;							// good function & object
					send_dnp = CLASS_0;
				}
			}
			if (((object_string[obj_ptr + 10] == 0) || (object_string[obj_ptr + 10] == 1)) &&	// request for default or class 0?
				(dnp_length > 19))
			{
				iien2 = PARAMETER_ERROR;						// point out of range
				if (object_string[obj_ptr + 11] == 6)			// Qualifier 6 ?
				{
					iien2 = GOOD_APDU;							// good function & object
					send_dnp = CLASS_0;
				}
			}
			// Now check for Class 1 anywhere in msg
			if ((object_string[obj_ptr + 1] == 2) || (DNP_point_CmdCode == 2) ||
				(object_string[obj_ptr + 7] == 2) || (object_string[obj_ptr + 10] == 2))
			{
				iien2 = PARAMETER_ERROR;						// point out of range
				if (object_string[obj_ptr + 2] == 6)			// Qualifier 6 ?
				{
					iien2 = GOOD_APDU;							// good function & object
					events_requested = events_record_num - 1;	// give em what there is
					if (send_dnp == CLASS_0)					// if didn't have a class 0 request
						send_dnp = ALL_DATA;					// change to all data
					else
						send_dnp = CLASS_1;						// no class 0 so just class 1
				}
				// *********** byte size num of events *****************
				if (object_string[obj_ptr + 2] == 0x07)
				{
					num_of_events = object_string[obj_ptr + 3];
					iien2 = GOOD_APDU;
					send_dnp |= CLASS_1;
					if (event_cntr >= num_of_events)			// if more events than requested
					{
						events_requested = num_of_events;		// give all thats requested
						if (events_requested > MAX_EVENT_NUMBER)
							events_requested = MAX_EVENT_NUMBER;// cap it at 10
					}
					else										// less events than requested
					{
						if (events_record_num > 0)				// if there are events
						{
							events_requested = events_record_num - 1;	// give em what there is
						}
						else
						{
							events_requested = 0;
						}
					} // end else

				} // End byte size request
				// *********** word size (2 bytes) num of events ***************
				if (object_string[obj_ptr + 2] == 0x08)
				{
					num_of_events = DNP_point_CmdCode;
					num_of_events = num_of_events << 8;
					num_of_events |= object_string[obj_ptr + 3];
					iien2 = GOOD_APDU;
					send_dnp |= CLASS_1;
					if (event_cntr >= num_of_events)
					{
						events_requested = num_of_events;
						if (events_requested > MAX_EVENT_NUMBER)
							events_requested = MAX_EVENT_NUMBER;		// cap it at 10
					}
					else
					{
						if (events_record_num > 0)
						{
							events_requested = events_record_num - 1;	// give all of them
						}
						else
						{
							events_requested = 0;
						}
					} // end else
				} // end word size request
				if (events_requested == 0)								// no events
					send_dnp = ERROR_RESPONSE;							// so send null
			}//end check for class 1 anywhere
		// Check for presence of request for class 2 or 3 only
		// ********** Redundant ? ************
			if (((object_string[obj_ptr + 1]) == 3) || ((DNP_point_CmdCode) == 3) ||			// Class 2
				((object_string[obj_ptr + 7]) == 3) || ((object_string[obj_ptr + 10]) == 3) ||
				((object_string[obj_ptr + 1]) == 4) || ((DNP_point_CmdCode) == 4) ||			// Class 3
				((object_string[obj_ptr + 7]) == 4) || ((object_string[obj_ptr + 10]) == 4))	// Class 3
			{
				// Check for previous response required else send error response
				if ((send_dnp != CLASS_1) && (send_dnp != CLASS_0) &&
					(send_dnp != ALL_DATA))
				{
					send_dnp = ERROR_RESPONSE;
				}
			} // end class 2 & 3 search
		} // end class 60
	} // End READ Function
	// ------ Write Function --------------
	if (rt.HostRxBuff[12] == WRITE)									// WRITE stuff ?
	{
		iien2 = OBJECT_UNKNOWN;										// object unknown for now
		/*------ Write IIEN ------------------*/
		if ((object_string[obj_ptr] == 80) && (object_string[obj_ptr + 1] == 0x01))
		{															// IIEN
			iien2 = PARAMETER_ERROR;								// point out of range
			if (object_string[obj_ptr + 2] == 0)					// Qualifier 0 ?
			{
				iien2 = PARAMETER_ERROR;							// point out of range
				if ((object_string[obj_ptr + 3] == 0x07) &&			// status point for iien1?
					(DNP_point_CmdCode == 0x07))
				{
					iien2 = GOOD_APDU;								// good function & object
					if ((object_string[obj_ptr + 5] & 0x80) == 0)	// clearing restart ?
						iien1 &= 0x7F;								// clear restart bit
					else
						iien1 |= 0x80;								// set restart bit
					send_dnp = ERROR_RESPONSE;						// confirm it
				} // end status message.
			} // End Qualifier										// end Qual = 0
			obj_ptr += 6;											// look at next object if needed
			length -= 6;
		}//End object & variation

		DNP_point_CmdCode = object_string[obj_ptr + 4];
		//-----  Write Calibration -----------
		if ((object_string[obj_ptr] == 40) && (object_string[obj_ptr + 1] == 0x01))
		{															// IIEN
			iien2 = PARAMETER_ERROR;								// point out of range
			if (object_string[obj_ptr + 2] == 0x17)					// Qualifier 17h ?
			{
				iien2 = PARAMETER_ERROR;							// point out of range
				// Check for calibration points:
				// Point 39 - Battery voltage  Point 40 - Fault volts    Point 41 - Minus Gnd
				// Point 42 - Ripple Current   Point 43 - Ripple Voltage Point 44 - V offset
				// Point 45 - PWM Command      Point 46 - PWM Value      Point 47 - Phase Selector
				// Point 48 (0x30) - Exit DNP SysData.NV_UI.StartUpProtocol, start ASCII_CMDS
				// Point 49 (0x31) - Exit DNP SysData.NV_UI.StartUpProtocol, start SETUP
				// Point 50 (0x32) - Exit DNP SysData.NV_UI.StartUpProtocol, start MODBUS
				//
				//DNP_point_CmdCode = object_string[obj_ptr + 4];
				if ((object_string[obj_ptr + 3] == 0x01) && //1 point
					(DNP_point_CmdCode >= CmdCalibrateVrange) &&
					(DNP_point_CmdCode <= DNPcmdSetModbusProtocol))
				{
					iien2 = GOOD_APDU;            //good function & object

					tmp_cal = ((long)object_string[obj_ptr + 9]) << 24;
					tmp_cal = tmp_cal + ((long)object_string[obj_ptr + 8] << 16);
					tmp_cal = tmp_cal + ((long)object_string[obj_ptr + 7] << 8);
					tmp_cal = tmp_cal + (long)object_string[obj_ptr + 6];

					//-!- change protocol by DNP command needs testing
					// IK20260212 Protocol switching from DNP commands
					// ASCII and SETUP are temporary protocols - only update rt.operating_protocol (never persist to EEPROM)
					// MODBUS is customer protocol - update both runtime and persistent storage
					if (DNP_point_CmdCode == ByteCmdSetAsciiProtocol) {
						rt.operating_protocol = ASCII_CMDS;		// ASCII is production/test only - runtime only
						// Do NOT modify SysData.NV_UI.StartUpProtocol
						display_mode = SELECT_PROTOCOL;			// Show change on display
					}
					if (DNP_point_CmdCode == DNPcmdSetSetupProtocol) {
						rt.operating_protocol = SETUP;			// SETUP is temporary - runtime only
						// Do NOT modify SysData.NV_UI.StartUpProtocol
						display_mode = SELECT_PROTOCOL;			// Show change on display
					}
					if (DNP_point_CmdCode == DNPcmdSetModbusProtocol) {
						rt.operating_protocol = MODBUS;			// MODBUS is customer protocol
						SysData.NV_UI.StartUpProtocol = MODBUS;	// Update EEPROM variable
						// Note: SaveToEE() should be called explicitly if persistence is desired
						display_mode = SELECT_PROTOCOL;			// Show change on display
					}

					if (DNP_point_CmdCode == CmdCalibrateVrange)			// 0x27 = 39D
						calibr_step = ADC_BATT_VOLTS;
					if (DNP_point_CmdCode == CmdCalibrateFaultV)			// 0x28 = 40D
						calibr_step = ADC_FAULT_VOLTS;
					if (DNP_point_CmdCode == CmdCalibrateMinusGround)		// 0x29 = 41D
						calibr_step = ADC_MINUS_GND_VOLTS;
					if (DNP_point_CmdCode == CmdCalibrateRipleCurrent)		// 0x2A = 42D
						calibr_step = ADC_RIPPLE_CURRENT;
					if (DNP_point_CmdCode == CmdCalibrateRippleVolt)		// 0x2B = 43D
						calibr_step = ADC_RIPPLE_VOLTAGE;
					if (DNP_point_CmdCode == CmdCalibrate1or3Phase)	// 0x2F = 47D
					{
						calibr_step = NOT_A_CALIBRATION;
						rt.ripple_calibration_phase = (uint8)tmp_cal;
					}

					if (DNP_point_CmdCode == CmdSetUnitVoltage)			// 0x2C = 44D
					{
						calibr_step = NOT_A_CALIBRATION;
						SysData.NV_UI.unit_type = (uint8)tmp_cal;			// value determines hi calibration pt
						SetCurrentLoopVoltagePoints();
						__disable_interrupt();

						SaveToEE(SysData.NV_UI.V4);
						SaveToEE(SysData.NV_UI.V20);
						SaveToEE(SysData.NV_UI.unit_type);
						__enable_interrupt();								// enable global interrupts
					}
					if (DNP_point_CmdCode == CmdSetPwmControl)			// 0x2D = 45D //PWM Command Instruction
					{
						calibr_step = NOT_A_CALIBRATION;					// added 8/14/19
						if (tmp_cal == 0x01)								// doing low current PWM
						{
							rt.i_cal_active = true;
							rt.cal_4mA = true;
							rt.cal_20mA = false;
						}
						if (tmp_cal == 0x02)								// doing high current PWM
						{
							rt.i_cal_active = true;
							rt.cal_20mA = true;
							rt.cal_4mA = false;
						}
						if (tmp_cal == 0x03)								// stop PWM adjustment all done
						{
							rt.i_cal_active = false;
							rt.cal_20mA = false;
							rt.cal_4mA = false;
							rt.i_cal_active = false;
							tmp_cal = 0;
							timer.PWM_calibration = 5000;					// to store values in 3 seconds. See: if ((timer.PWM_calibration < 2000) && (timer.PWM_calibration > 0)) ... EEPROM_Write_float... //after 3 sec of not cal mode
						}
					}
					if ((DNP_point_CmdCode == CmdSetPwmValue)			// 0x2E = 46D  If a PWM value
						&& (rt.i_cal_active == true)) //make it so
					{
						calibr_step = NOT_A_CALIBRATION;					// added 8/14/19
						if (rt.cal_20mA == true)
							SysData.CurrentOut_I420.Y2_highCalibrVal = (float)(tmp_cal * 0.00001f);	// IK20251110 it is duty cycle of PWM
						if (rt.cal_4mA == true)
							SysData.CurrentOut_I420.Y1_lowCalibrVal = (float)(tmp_cal * 0.00001f);		// IK20241205 replaced division / 100000 with multiplication);
					}

					// **** if tmp_cal value is greater than unit type volts -> converted to mV; example (48 V -> 48000 mV)
					// or for RV or RI tmp_cal is above 51 mV(or mA)
					// it is concidered  high point calibration, indicated by cal_status bit 2 is set
					if ((tmp_cal > ((long)SysData.NV_UI.unit_type * 1000)) // convert unit_type in Volts to mV
						|| ((tmp_cal > 51) && ((calibr_step == ADC_RIPPLE_VOLTAGE) || (calibr_step == ADC_RIPPLE_CURRENT))))	// calibr_step= 5 or 6
					{
						if (calibr_step == ADC_BATT_VOLTS)
							SysData.BatteryVolts.Y2_highCalibrVal = (float)tmp_cal;
						if (calibr_step == ADC_FAULT_VOLTS)
							SysData.FaultVolts.Y2_highCalibrVal = (float)tmp_cal;
						if (calibr_step == ADC_MINUS_GND_VOLTS)
							SysData.MinusGndVolts.Y2_highCalibrVal = (float)tmp_cal;

						if (calibr_step == ADC_RIPPLE_CURRENT)						// can be here
						{
							if (rt.ripple_calibration_phase == CalibrateSinglePhase) // 1 phase calibration == 127
								SysData.RippleCurr1ph.Y2_highCalibrVal = (float)tmp_cal;	// 1-ph, set test mA received with cal command, example 120 mA
							else           // can be not 127, means 0 or 255, where 0 means NOT_IN_CALIBRATION, and 255 means 3Phase_Calibration
								SysData.RippleCurr3ph.Y2_highCalibrVal = (float)tmp_cal;
						}
						if (calibr_step == ADC_RIPPLE_VOLTAGE)						// can be here
						{
							if (rt.ripple_calibration_phase == CalibrateSinglePhase)
								SysData.RippleVolts1ph.Y2_highCalibrVal = (float)tmp_cal * 0.001f;	// 1-ph, set test mV received with cal command coverted to Volts, example 500 mV
							else
								SysData.RippleVolts3ph.Y2_highCalibrVal = (float)tmp_cal * 0.001f;
						}
						cal_status |= RECEIVED_EXT_HI_VALUE;						// = setBit(cal_status, Bit_1);
						timer.Calibration = DEF_CALIBRATION_DELAY;					// give it 1.5s to get an ADC reading IK20250812 should be enough 1.5 sec.
					}
					else	/////////////////////////////////////////////////////// // is low cal
					{
						if (calibr_step == ADC_BATT_VOLTS)
							SysData.BatteryVolts.Y1_lowCalibrVal = (float)tmp_cal;
						if (calibr_step == ADC_FAULT_VOLTS)
							SysData.FaultVolts.Y1_lowCalibrVal = (float)tmp_cal;
						if (calibr_step == ADC_MINUS_GND_VOLTS)
							SysData.MinusGndVolts.Y1_lowCalibrVal = (float)tmp_cal;
						if (calibr_step == ADC_RIPPLE_CURRENT)
						{
							if (rt.ripple_calibration_phase == CalibrateSinglePhase) // 1-ph low cal
								SysData.RippleCurr1ph.Y1_lowCalibrVal = (float)tmp_cal;
							else
								SysData.RippleCurr3ph.Y1_lowCalibrVal = (float)tmp_cal;	// 3-ph
						}
						if (calibr_step == ADC_RIPPLE_VOLTAGE)
						{
							if (rt.ripple_calibration_phase == CalibrateSinglePhase)
								SysData.RippleVolts1ph.Y1_lowCalibrVal = (float)tmp_cal * 0.001f;	// tmp_cal == received with DNP command mV set on terminals, example: 49 [mV], saving as Volts
							else
								SysData.RippleVolts3ph.Y1_lowCalibrVal = (float)tmp_cal * 0.001f;
						}
						cal_status |= RECEIVED_EXT_LO_VALUE;						// Low point value calibration RECEIVED, = setBit(cal_status, Bit_0);
						// at this moment, cal_timer = 0;
						timer.Calibration = DEF_CALIBRATION_DELAY;					// give it 1.5s to get an ADC reading
					}
					if ((calibr_step == NOT_A_CALIBRATION)) // IK20260203 allow calibration in ASCII mode || (SysData.NV_UI.StartUpProtocol != DNP3))
					{
						calibr_step = NOT_A_CALIBRATION;
						cal_status = 0;
						timer.Calibration = 0;
					}
					send_dnp = ERROR_RESPONSE;										// confirm it
				} // status message.
			} // end Qual = 17
		} //end object 40 var 1
	}//End Write

	if (rt.HostRxBuff[12] == COLD_RESTART)
	{
		iien2 = GOOD_APDU;
		DNP_time_delay = 9000;														// 0.9 seconds
		send_dnp = TIME_DELAY_RESPONSE;												// send fine time
		restart_op = true;
	}
	if ((iien2 == BAD_FUNCTION) || (iien2 == OBJECT_UNKNOWN) || (iien2 == PARAMETER_ERROR))
		send_dnp = ERROR_RESPONSE;													// send null response

	if (length > 50)																// in case things wrapped
		length = 0;
}

void ClearRxBuffer(uint8 pattern) {
	__disable_interrupt();
	memset((void*)rt.HostRxBuff, pattern, sizeof(rt.HostRxBuff));	// clear out last msg
	rt.HostRxBuffPtr = rt.EchoRxBuffPtr = 0;						// ready for next msg
	num_of_inbytes = 0;	// IK20251219 'num_of_inbytes' is not used in SETUP protocol
	__enable_interrupt();
}

/********************************************************************/
/*                      P A R S E   D N P   M S G                   */
/********************************************************************/
/* Description: This function is called by the top level if the
				RS232 interrupt routine has built a complete message.
				this function will parse the incoming msg and perform
				the correct operation and response as described in the
				MODBUS RTU specification and the Electroswitch SDS
				document.
   Inputs:      pointer to scada_msg
   Outputs:     scada_op, send
   Notes:

   Revision:    09/25/99      REC   Created.
				2/13/03       REC   Fixed CROB bug. Increased safety
									timer to 30 ms. In the statement
									(num_of_inbytes <= length + 8) I
									added the = sign. These changes
									allowed the routine to wait for
									the last byte of the incomming
									msg before continuing.
*/
/********************************************************************/
#ifdef PC
extern long OCR2A;
extern long TIMSK2;
extern long SREG;
#define TWINT                       Bit_7 // 0x80 TWI Interrpt Flag. SECOND DEFINITION TO COMPILE IN VS. The real one in hardware.h
#define TIMING_INTERRUPT_SETTING    199 // IK20250925 changed from 124 to 199. 0x7C gives 1000 us TIMER 2 interrupt for all timing counters variables. SECOND DEFINITION TO COMPILE IN VS. The real one in hardware.h
#endif //PC
void Parse_DNP_Msg(void)
{
	uint16 received_crc;
	uint16 destination;
	uint8 control, wait_length;//, i;
	/******************************************************/
	comm_state = RCV; // in enum UART_Events
	timer.Generic = 100;	// ms
	WATCHDOG_RESET();
	if (Existing.baud_rate <= (enum Baud_Rate_Setting)Baud_600)	// IK20250206 redefined enum - now it is the real BR, not the UBRR setting; this reverses < > logic.BR is BELOW 600. default Baud Rate is 19200
		timer.Generic = 700;
	do {																// while unless something barfs
		if (OCR2A != TIMING_INTERRUPT_SETTING)
			timer.Generic = 0;
		if ((TIMSK2 & 0x02) != 0x02)
			timer.Generic = 0;
		if ((SREG & TWINT) == 0) // Bit_7
			timer.Generic = 0;
	} while ((num_of_inbytes < 10) && (timer.Generic != 0));			// get the header
	WATCHDOG_RESET();

	// --------- Get Length ------------------
	length = rt.HostRxBuff[2];
	dnp_length = length;
	if (length <= 50) // IK20250812 expected incoming transmission is not longer than 50 bytes
		//{
		//	if (length < 10) // IK20250812 if length is less than 10, it is not a valid DNP message
		//		return; // return from Parse_DNP_Msg, no valid DNP message
		//}
		//else
		//{
		//	length = 0; // reset length to zero, so that it will be ignored
		//	return; // return from Parse_DNP_Msg, no valid DNP message
		//}
	{
		//--------- Get Control Byte ------------
		control = rt.HostRxBuff[3];
		// --------- Get Destination Address -----
		destination = rt.HostRxBuff[5] << 8;
		destination = rt.HostRxBuff[4] | destination;
		// --------- Check address first ---------
		if (((destination <= 65519) && (destination == SysData.NV_UI.meter_address)) ||
			(destination == 0xFFFF) || (destination == 0xFFFE) ||
			(destination == 0xFFFD))
		{
			DNPbroadcast = false;
			if ((destination == 0xFFFF) || (destination == 0xFFFD))
			{
				DNPbroadcast = true;
				iien1 |= 0x01;											// set all stations bit
			}
			if (destination == 0xFFFE)
			{
				all_stations_msg = true;
				iien1 |= 0x01;											// set all stations bit
				DNPbroadcast = true;
			}

			// ----- Get Source Address ----------
			SysData.NV_UI.host_address = rt.HostRxBuff[7] << 8;
			SysData.NV_UI.host_address = rt.HostRxBuff[6] | SysData.NV_UI.host_address;
			// ----- Check Header CRC ------------
			received_crc = rt.HostRxBuff[9] << 8;
			received_crc = received_crc | rt.HostRxBuff[8];
			Calc_DNPCRC((uint8*)&rt.HostRxBuff[0], 8);

			if (received_crc == crc)
			{
				switch (control & 0x0F)									// get DLL function
				{
				case RESET_REMOTE_LINK:									// RESET REMOTE LINK
					if ((control & 0x10) == 0x00)						// FCV invalid
					{
						if (DNPbroadcast == false)
						{
							Send_DNP_Msg(DLL_ACK_CONFIRM);
						}
						dll_status |= 0x01;								// Link Reset
						dll_status |= 0x20;								// FCB = 1 for next msg
					}
					break;
				case RESET_USER_LINK:									// RESET USER LINK
					// if sent FCB doesn't match expected FCB send last Confirm msg
					// OBSOLETE in new DNP DLL SPECIFICATION
					break;
				case TEST_FUNCTION:										// TEST function for link
					//if sent FCB doesn't match expected FCB send last Confirm msg
					if (((dll_status & 0x01) == 0x01)					// DLL reset
						&& ((control & 0x10) == 0x10))					// FCV valid
					{
						if ((DNPbroadcast == false) &&
							((dll_status & 0x20) == (control & 0x20)))
						{
							Send_DNP_Msg(DLL_ACK_CONFIRM);
							dll_status ^= 0x20;							// toggle FCB for next msg.
						}
					}
					break;
				case USER_DATA:											// Send user data
					if (((dll_status & 0x01) == 0x01)					// DLL reset
						&& ((control & 0x10) == 0x10))					// FCV valid
					{
						// IK20231214 not checked //dlconfirm = true;
						if (Existing.baud_rate <= Baud_1200)		// IK20250206 redefined enum - now it is the real BR, not the UBRR setting; this reverses < > logic. set safety timer
							timer.Generic = 600;						// for 1200 baud
						else
							timer.Generic = 100;						//for 9600
						WATCHDOG_RESET();
						if (length >= 21)
							wait_length = length + 8;
						else
							wait_length = length + 6;

						while ((num_of_inbytes <= wait_length) &&
							(timer.Generic != 0))						// wait till rest of msg is available
						{
							if (OCR2A != TIMING_INTERRUPT_SETTING)
								timer.Generic = 0;
							if ((TIMSK2 & 0x02) != 0x02)
								timer.Generic = 0;
							if ((SREG & 0x80) != TWINT)
								timer.Generic = 0;
						}
						WATCHDOG_RESET();
						if (length > 5)
						{
							if (length >= 21)
							{
								received_crc = rt.HostRxBuff[27] << 8;
								received_crc = received_crc | rt.HostRxBuff[26];
								Calc_DNPCRC((uint8*)&rt.HostRxBuff[10], 16);
								if (received_crc == crc)
								{
									if (length > 21)
									{
										received_crc = rt.HostRxBuff[length + 8] << 8;
										received_crc |= rt.HostRxBuff[length + 7];
										Calc_DNPCRC((uint8*)&rt.HostRxBuff[28], length - 21);
										if (received_crc == crc)
										{
											if ((dll_status & 0x20) == (control & 0x20))
											{            //expected FCB
												if (DNPbroadcast == false)
												{
													Send_DNP_Msg(DLL_ACK_CONFIRM);
													dll_status ^= 0x20;		// toggle FCB for next msg.
												}
												obj_ptr = 0;				// set to point at first object
												length -= 7;				// forget the DLL,transport
												send_dnp = 0;
												DNP_App();					// go do app layer
											} // end good FCB
											else
											{
												if (DNPbroadcast == false)
												{
													Send_DNP_Msg(DLL_ACK_CONFIRM);
												}
											}
										}
									}
									else									// length is 21
									{
										if ((dll_status & 0x20) == (control & 0x20))
										{          //expected FCB
											if (DNPbroadcast == false)
											{
												Send_DNP_Msg(DLL_ACK_CONFIRM);
												dll_status ^= 0x20;			// toggle FCB for next msg.
											}
											obj_ptr = 0;					// set to point at first object
											length -= 7;					// forget the DLL,transport
											send_dnp = 0;
											DNP_App();						// go do app layer
										} // end good FCB
										else
										{
											if (DNPbroadcast == false)
											{
												Send_DNP_Msg(DLL_ACK_CONFIRM);
											}
										} // end bad FCB
									}//end length is 21
								} // end bad crc
							} // end length 21 or greater
							else // msg length between 6 and 20 bytes
							{
								received_crc = rt.HostRxBuff[length + 6] << 8;
								received_crc |= rt.HostRxBuff[length + 5];
								Calc_DNPCRC((uint8*)&rt.HostRxBuff[10], length - 5);
								if (received_crc == crc)
								{
									if ((dll_status & 0x20) == (control & 0x20))	// expected FCB
									{
										if (DNPbroadcast == false)
										{
											Send_DNP_Msg(DLL_ACK_CONFIRM);
											dll_status ^= 0x20;						// toggle FCB for next msg.
										}
										obj_ptr = 0;								// set to point at first object
										length -= 7;								// forget the DLL,transport
										send_dnp = 0;
										DNP_App();									// go do app layer
									} // rcv'd expected FCB
									else
										if (DNPbroadcast == false)
										{
											Send_DNP_Msg(DLL_ACK_CONFIRM);
										}
								} // good crc on msg between 6 and 20 octets
							} // length 6 to 20
						} // length was less than 5
					} // didn't process cause link wasn't reset
					break;
				case UNCONFIRMED_DATA:												// Send unconfirmed user data
					if ((control & 0x10) == 0x00)//FCV invalid
					{
						if (Existing.baud_rate <= Baud_1200)						// IK20250206 redefined enum - now it is the real BR, not the UBRR setting; this reverses < > logic.
							timer.Generic = 600;									// safety timer
						else
							timer.Generic = 100;
						WATCHDOG_RESET();
						if (length >= 21)
							wait_length = length + 8;
						else
							wait_length = length + 6;

						while ((num_of_inbytes <= (wait_length)) &&
							(timer.Generic != 0))									// wait here till rest of msg
						{															// is available
							if (OCR2A != TIMING_INTERRUPT_SETTING)
								timer.Generic = 0;
							if ((TIMSK2 & 0x02) != 0x02)
								timer.Generic = 0;
							if ((SREG & TWINT) == 0)
								timer.Generic = 0;
						}
						WATCHDOG_RESET();
						if (rt.HostRxBuff[12] == 0)									// CONFIRM
							timer.Generic = 1;

						if (length > 5)
						{
							if (length >= 21)
							{
								received_crc = rt.HostRxBuff[27] << 8;
								received_crc = received_crc | rt.HostRxBuff[26];
								Calc_DNPCRC((uint8*)&rt.HostRxBuff[10], 16);
								if (received_crc == crc)
								{
									if (length > 21)
									{
										received_crc = rt.HostRxBuff[length + 8] << 8;
										received_crc |= rt.HostRxBuff[length + 7];
										Calc_DNPCRC((uint8*)&rt.HostRxBuff[28], length - 21);
										if (received_crc == crc)
										{
											obj_ptr = 0;			// set to point at first object
											length -= 7;			// forget the DLL,transport
											send_dnp = 0;
											DNP_App();				// go do app layer
										}
									}//end length greater than 21
									else//length is 21
									{
										obj_ptr = 0;				// set to point at first object
										length -= 7;				// forget the DLL,transport
										send_dnp = 0;
										DNP_App();					// go do app layer
									}
								} // end good crc 21
							} // end greater than or equal to 21
							else
							{
								received_crc = rt.HostRxBuff[length + 6] << 8;
								received_crc |= rt.HostRxBuff[length + 5];
								Calc_DNPCRC((uint8*)&rt.HostRxBuff[10], length - 5);

								if (received_crc == crc)
								{
									//debug = 0;
									obj_ptr = 0;					// set to point at first object
									length -= 7;					// forget the DLL & Transport
									send_dnp = 0;
									DNP_App();						// go do app layer
								}
							}
						}
					} // end FCV invalid
					break;
				case REQUEST_LINK_STATUS:
					if ((control & 0x10) == 0x00)//FCV invalid
					{
						send_dnp = LINK_STATUS;
					}
					break;
				default:											// not any of above
					break;
				} // END SWITCH STATEMENT
			} // END CRC GOOD
		} // END ADDRESS MATCH
	} // END LENGTH CHECK
	// ---------- End of command checks -------
	rt.HostRxBuffPtr = 0;											// init rt.HostRxBuffPtr
	if (DNPbroadcast == true)										// if sent as Broadcast
		send_dnp = 0;												// do not reply
	ClearRxBuffer(0xFF);						// at this point all parsing is done so clear out the buffer with nonsense
} // end Parse_DNP_Msg()
