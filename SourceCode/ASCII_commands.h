//Remote Commands (first four bytes in command string), >>>lower case letters<<<<, pointer to function
const t_rci_commands rci[] =
{
#ifdef PC
	/*OK*/	"cls\r",	(void*)&ClearConsole,			// "cls" clears console in PC simulation
#endif
	// no parameters
	/*+*/	"////",		(void*)&IgnoreComment,			// "////" - comment line, IGNORES comment in downloading parameter files.
	/*+*/	"clea",		(void*)&clrscr,					// "clea" - clear screen
	/*+*/	"help",		(void*)&Print_Help,				// "help" - list of commands
	/*+*/	"?\0\0\0",	(void*)&Print_Help,				// "?" prints - list of commands
	/*+*/	"adch",		(void*)&Show_ADCcounts_and_Volts,	// "adch"  Report ADC readings in format: chan0, chan1, chan2, chan3, chan4, chan5, chan6, chan7
	/*+*/	"puls",		(void*)&Excitation_Pulse,		// Pulse set/get: 'pulse>on', 'pulse>off' - set right now; 'pulse>enab', IT DOES NOT SAVED into flash because it is test command.
	/*+*/	"dnp3",		(void*)&SwitchToDNP,			// exit ASCII RCI SysData.NV_UI.StartUpProtocol and swith to DNP immediately
	/*+*/	"modb",		(void*)&SwitchToModBus,			// exit ASCII RCI SysData.NV_UI.StartUpProtocol and swith to ModBus immediately
	/*+*/	"s\0\0\0",	(void*)&Print_Status,			// "s"  Print status BYTE (see "stat" command)
	/*+*/	"w\0\0\0",	(void*)&Print_System_Snapshot,	// "w": Print system information (same as "info" command)
	/*+*/	"b\0\0\0",	(void*)&Print_System_Brief,		// "b": Print brief information: measurements and alarms
	/*+*/	"brif",		(void*)&Print_System_Brief,		// "brif": Print brief information: measurements and alarms
	/*+*/	"d\0\0\0",	(void*)&SendRTdata,				// "d": Print measurements and alarm word
	/*+*/	"data",		(void*)&SendRTdata,				// "data": Print measurements and alarm word
	/*+*/	"disn",		(void*)&Send_to_NumericLEDs,	// "disN"  send string to upper Numeric LED display
	/*+*/	"disi",		(void*)&Send_to_ASCII_LEDs,		// "disI"  send string to lower Information ASCII LED display
	/*+*/	"disl",		(void*)&Send_to_STATUS_LEDs,	// "disL"  send LED byte. if upper bit of OverwriteLEDs is set, it overwrites (ON-OFF assigned by Display board firmware) with 3 lower bits of OverwriteLEDs
	/*+*/	"getb",		(void*)&Get_Buttons,			// "getB"  ask for uint16 button word and show it on screen.
	/*+*/	"rsto",		(void*)&Get_EEPROM_params,		// restore: GET SYSTEM PARAMETERS FROM FLASH if data is valid
	/*+*/	"dflt",		(void*)&SetDefaultsInRAM_by_command,// "dflt"(default)- sets default settings, but preserves serial port settings
	/*+*/	"rtmo",		(void*)&SetRealTimeMonitoring,	//  'rtmon>enab', 'rtmon>disa'.Enables / diasbles real time monitoring - send once in a few sec "brief" info to serial port

	//DO NOT MOVE "vers" command from here, it is used to start full parameter dump - "expo" command,
	// PUT ALL PARAMETER COMMANDS below before "expo" command
	/*+*/	"vers" ,	(void*)&Print_FW_Version,
	/*+*/	"hadr",		(void*)&SetGetHostAddress,		// DNP/ModBus Host Address set/get: 'hadr? returns Set command syntax 'hadr=NNNN', DNP range up to 65000, ModBus pange up to 255.  Setting in flash is updated after 'save' command.
	/*+*/	"madr",		(void*)&SetGetMeterAddress,		// DNP/ModBus Meter Address set/get: 'madr? returns Set command syntax 'madr=NNNN', DNP range up to 65000, ModBus pange up to 255.  Setting in flash is updated after 'save' command.
	/*+*/	"prot",		(void*)&SetGetProtocol,			// Protocol set/get: DNP or ModBus after startup
	/*+*/	"baud",		(void*)&SetGetBaudRate,			// Get 'baud[?]' returns Set command syntax 'baud>XXXXX'. Command accepts only std rates. Setting in flash is updated after 'save' command.
	/*+*/	"unit",		(void*)&SetGetVoltageRange,		// Get 'unit[?]' returns Set command syntax 'unit>XXXv'. Command accepts only 24v, 48v, 125v, 250v. Setting in flash is updated after 'save' command.
	/*+*/	"hbat",		(void*)&SetGetHighBatThreshold,	// Get 'hbat[?]' returns Set command syntax 'hbat=XXX'. Setting in flash is updated after 'save' command.
	/*+*/	"lbat",		(void*)&SetGet_LowBatThreshold,	// Get 'lbat[?]' returns Set command syntax 'lbat=XXX'. Setting in flash is updated after 'save' command.
	/*+*/	"vrip",		(void*)&SetGetRipVoltThreshold,	// Get 'vrip[?]' returns Set command syntax 'vrip=XXX'. Setting in flash is updated after 'save' command.
	/*+*/	"irip",		(void*)&SetGetRipCURRthreshold,	// Get 'irip[?]' returns Set command syntax 'irip=XXX'. Setting in flash is updated after 'save' command.
	/*+*/	"iloo",		(void*)&SetGetCurrentLoopRange,	// Current loop output Get 'iloo[p?]' returns Set command syntax 'iloop>i0-1', 'iloop>i420'. Setting in flash is updated after 'save' command.
	/*+*/	"phas",		(void*)&SetGetPhase,			// Get 'phas[e?]' returns Set command syntax 'phase>1ph', 'phase>3ph'. Setting in flash is updated after 'save' command.
														// Set / Get one-phase (120 Hz) or 3-phase (360 Hz) charger. Calibration parameters have 2 sets for 120 or 360 Hz
	/*+*/	"alar",		(void*)&SetGetAlarm,			// Alarm set/get: 'alarm rv>enab', 'alarm rv>disa'; 'alarm ri>enab', 'alarm ri>disa'; 'alarm ac>enab', 'alarm ac>disa'; 'alarm hz>enab', 'alarm hz>disa'
	/*+*/	"dela",		(void*)&SetGetDelay,			// Delay set/get: 'delay? returns Set command syntax 'delay=XX.X' in seconds. Setting in flash is updated after 'save' command.
	/*+*/	"buzz",		(void*)&SetGetBuzzer,			// Buzzer set/get: 'buzz>on', 'buzz>off' - set right now; 'buzz>enab', 'buzz>disa' - set/get setting in flash after 'save' command.
	/*+*/	"latc",		(void*)&SetGetLatch,			// Get 'latc[h?]' returns Set command syntax 'latch>enab', 'latch>disa'. Setting in flash is updated after 'save' command.

	/*+*/	"echo",		(void*)&Echo_Enab_Disab,		// "echo>enabled"  "echo>disabled" sets echo flag. "echo?" returns flag
	/*+*/	"expo",		(void*)&Export_settings,		// "expo" //creates play-back list of commands to restore settings
	///////"expo" ends auto param list

	/*+*/	"cpar",		(void*)&SetGetCalParam,			// "cpar#$=GGG.GGG" Set/get calibration parameter.
	/*+*/	"cali",		(void*)&SetCalibrationValue,	// "cali#$=GGG.GGG" Set Calibration value.
	/*+*/	"save",		(void*)&SaveCal,				// "save"  transfers data from RAM to FLASH. Use "SAVE" to permanently update it in FLASH
};
/*************************************************************/
