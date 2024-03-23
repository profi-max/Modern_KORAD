/*******************************************************************************
Created by profi-max (Oleg Linnik) 2024
https://profimaxblog.ru
https://github.com/profi-max

*******************************************************************************/
#include <Arduino.h>
#include <WiFi.h>
#include <EEPROM.h>
#include "modbus.h"
#include "common.h"
#include "korad.h"

// modbus defines
#define MB_PORT 502
#define MB_BUFFER_SIZE 256
#define RESPONSE_TIMEOUT_MS 1000

#define  MB_FC_NONE                     0
#define  MB_FC_READ_REGISTERS           3
#define  MB_FC_WRITE_REGISTER           6
#define  MB_FC_WRITE_MULTIPLE_REGISTERS 16
#define  MB_FC_ERROR_MASK 				128

//*************  USER DEFINES  *************
//  set define below to 1 ... 127
#define MB_DEVICE_ADDRESS 1

// uncomment the line below if you need debug messages
//#define MB_DEBUG
//#define KORAD_DEBUG_TX
//#define KORAD_DEBUG_RX
//*************  END of USER DEFINES  *************

#if (CORE_DEBUG_LEVEL > 0) || defined (MB_DEBUG) || defined (KORAD_DEBUG_TX) || defined (KORAD_DEBUG_RX) || defined (DEBUG_WITH_OSCILL)
	#define SERIAL_BUSY_BY_DEBUG
#endif


static const char* logTAG = "modbus.cpp";

//**********  KORAD UART MESSAGE TYPEDEF ************************
// https://sigrok.org/wiki/Atten_PPS3000_Series
typedef struct __attribute__((packed)) {
  uint16_t magicKey;
  uint16_t uValue;
  uint16_t iValue;
  uint32_t dummy1;
  uint32_t dummy2;
  uint8_t b01;
  uint8_t out;
  uint8_t alarm;
  uint8_t b00;
  uint8_t ocp;
  uint8_t mode;
  uint16_t dummy3;
  uint8_t dummy4;
  uint8_t checksum;
} korad_uart_msg_t;

//************   DPS PROTOCOL TYPES & DEFINES  *******************
typedef enum {
	protect_OK = 0,
	protect_OVP, 	// OVP overvoltage protection = 1 (if Uout > OVP)
	protect_OCP,	// OCP overcurrent protection = 2 (if Iout > OCP)
	protect_OPP,	// OPP overpower protection = 3 (if Pout > OPP)
	protect_OTP,  	// OTP overtime protection = 4 (timer stop)
	protect_OHP,	// OHP overheat protection = 5 (if T > Overheat)
	protect_BAT,	// BAT source battery low voltage protection = 6 ( Uin < Ubatmin - 10%)
	protect_BATLOWCURRENT,	// END charge battery low current = 7	(Iout < 50mA for 30 seconds)
	protect_EVP,	// EVP error voltage protection = 8 (if Uset > Umax)
	protect_ECP,	// ECP error current protection = 9	(if Iset > Imax)
} protect_t;

typedef struct {
	uint16_t USET; //  always format 00.00
	uint16_t ISET; // format 00.00 for DPS5020 // format 00.00 for DPS5015 // format 0.000 for DPS5005
	uint16_t SOVP; //  always format 00.00
	uint16_t SOCP; // format 00.00 for DPS5020 // format 00.00 for DPS5015 // format 0.000 for DPS5005
	uint16_t SOPP;  //  always format 0000.0
	uint16_t BLED;
	uint16_t PRFS; 	// Set of parameters      before ver 4.5 SOFT Soft Front //MPRE;	// Memory Preset Number
	uint16_t SINI;	// Power output switch
	uint16_t OTIM;	// overtime
	uint16_t d09;
	uint16_t d10;
	uint16_t d11;
	uint16_t d12;
	uint16_t d13;
	uint16_t d14;
	uint16_t d15;
} Profile_t;

//  Bitmask for mb_Data.STATE
#define STA_ANA_TEMP_SENSOR 0x01   // Analog temperature sensor in the model
#define STA_DIG_TEMP_SENSOR 0x02   // Digital temperature sensor LM75 found at startup
#define STA_GYRO_SENSOR 	0x04   // Optional MPU6050 gyro sensor found at startup
#define STA_CUR_DIV			0x08	// Current divider is 10
#define STA_MODEL_WUZHI		0x10	//This is WUZHI
#define STA_MODEL_KORAD		0x20	//This is KORAD

//  Bitmask for mb_Data.PARAM
#define PARAM_C_OR_F  0x01  		// hex for 0000 0001
#define PARAM_RESET_COUNTERS  0x02  // hex for 0000 0010
#define PARAM_SMART_DISPLAY   0x04  // hex for 0000 0100
#define PARAM_BATTERY_SOURCE  0x08  // hex for 0000 1000
#define PARAM_POWER_ON_START  0x10  // hex for 0001 0000
#define PARAM_LOCK_ON_START   0x20  // hex for 0010 0000
#define PARAM_SLEEP_ON_START  0x40	// hex for 0100 0000
#define PARAM_TURN_OFF_FILTER 0x80  //  turn off filter for Uout, Iout, Uin

// Bitmask for mb_Data.Profile[].PRFS
#define PROFILE_SOFT_FRONT 0x1
#define PROFILE_TRIGGER_OVP 0x2
#define PROFILE_TRIGGER_OCP 0x4

// CMD register
#define  CMD_SAVE_GLOBAL 1
#define  CMD_SAVE_PROFILE 2
#define  CMD_SAVE_SET_PROFILE 3
#define  CMD_CLR_COUNTERS 4


typedef struct {
	uint16_t USET; //  always format 00.00
	uint16_t ISET; // format 00.00 for DPS5020 // format 00.00 for DPS5015 // format 0.000 for DPS5005
	uint16_t UOUT; //  always format 00.00
	uint16_t IOUT; // format 00.00 for DPS5020 // format 00.00 for DPS5015 // format 0.000 for DPS5005
	uint16_t POWER; //  always format 0000.0 // PC apps  dont use it //  it is used for OPP
	uint16_t UIN; //  always format 00.00
	uint16_t LOCK;
	uint16_t PROTECT;
	uint16_t CVCC;
	uint16_t ONOFF;
	uint16_t BLED;   
	uint16_t MODEL;
	uint16_t VERSION;
	uint16_t TMP; // temperature
	uint16_t STATE; //Set of bites before ver 4.2 SPCN; // Source percent
	uint16_t DEBUG_DATA; //d16;

	uint16_t MGIC; // Magic key
	uint16_t DVID; // Model, device ID
	uint16_t COMM; // online-offline + Baud Rate + Modbus Address
	uint16_t GYRO; // 0xFFF0 - XYZ-rotaion / 0x000F bit 1 means Rotation on, bit 2 means Gyro sensor present only for Modbus !!!
	uint16_t MMAX; // Max memory number   //   Хранится в EEPROM вместe c BLED
	uint16_t PVER; // Protocol version //d22, 	// before ver 4.2 RCNT; // Reset counters
	uint16_t BCKL; 	// Backlight from BLED // before ver 4.2  C or F
	uint16_t OHP;  // overheat temperature
	uint16_t d25; 	// before ver 4.2 SMART; // Smart display
	uint16_t PARAM; // Set of parameters      before ver 4.2 BSRC; // battery source
	uint16_t MINS;	// minimum source battery 0%
	uint16_t MAXS;	// maximum source battary 100%
	uint16_t CLR1; //  PowerOff Color
	uint16_t CLR2;	// CV color
	uint16_t CLR3;	// CC color
	uint16_t BEEP;	

	uint16_t CMD;		// Execute Command register
	uint16_t TIME_L;	// Time counter low word
	uint16_t TIME_H;	// Time counter high word
	uint16_t MEM;  		// Profile index 
	uint16_t AHCNT_L;	// Amper/hour counter low word
	uint16_t AHCNT_H;	// Amper/hour counter high word
	uint16_t WHCNT_L;	// Watt/hour counter low word
	uint16_t WHCNT_H;	// Watt/hour counter high word
	uint16_t CLB_CMD;		//   Calibration Command  enumeration  calib_mode_t;
	uint16_t CLB_IDX;		// Calibration Index of operation for DoCalibration(uint8_t aIdx)
	uint16_t CLB_DATA_L;	// Calibration Data (uint32_t Value) Low Word for CalibRecord_t.Value
	uint16_t CLB_DATA_H;	// Calibration Data (uint32_t Value) High Word for CalibRecord_t.Value
	uint16_t IP_L;
	uint16_t IP_H;
	uint16_t d47;
	uint16_t d48;

	uint16_t Dummy[32];
	Profile_t Profile[20];
	uint16_t Dummy2[48]; //  Calibration data
} modbus_regs_t;

//******************  DPS MODEL DEFINES ***********************
#define DPS_VER 46U
#define DPS_PVER 46U // Protocol version
#define MAGIC_KEY 10024 //  XX Day - XX Month - X Year
#define SCLR1 0xFFFF  //  PowerOff Color   white
#define SCLR2 0x07E0	// CV color
#define SCLR3 0xB6FF	// CC color
#define MAX_TIM 5999
#define DPS_MOD 3005
#define MAX_U  3100
#define MAX_I  5100
#define MIN_I 10
#define OPP 1500	//  Max Power
#define NCR 2000  //  Typical current
#define DEF_STATE STA_CUR_DIV  | STA_MODEL_KORAD

//*****************  DPS MODEL TYPEDEFS  *************************
typedef struct {
	uint8_t func; // modbus function
	uint16_t nregs;	// number of registers
	uint16_t sreg;	// start register
}mb_last_func_t;

#define EEPROM_GLOBAL_DATA_ADDRESS sizeof(persistent_data_t)
#define CHARGE_LOW_CURRENT_OVERTIME 30
#define KORAD_POLL_PERIOD_MS 100

//*****************  DPS MODEL VARIABLES  *************************
mb_last_func_t gLastFunc;
int8_t changeModeCount = 0;
uint16_t softFrontVoltage = 0;
uint32_t gCurDiv; //  for KORAD3005 = 10,  for KORAD3010 = 1

modbus_regs_t mb_Data = {
500, 	//uint16_t USET;
100,	//uint16_t ISET;
0,		//uint16_t UOUT;
0,		//uint16_t IOUT;
0,		//uint16_t POWER;
3600,		//uint16_t UIN;
0,		//uint16_t LOCK;
0,		//uint16_t PROTECT;
0,		//uint16_t CVCC;
0,		//uint16_t ONOFF;
4,		//uint16_t BLED;
DPS_MOD,//uint16_t MODEL;
DPS_VER,//uint16_t VERSION;
20,		//uint16_t TMP; // temperature
DEF_STATE,		//uint16_t STATE; //Set of bites before ver 4.2 SPCN; // Source percent
0,		//uint16_t DEBUG_DATA;//d16;

MAGIC_KEY,	//uint16_t MGIC; // Magic number
DPS_MOD,//uint16_t DVID; // Model, device ID
0,	//uint16_t COMM; // online-offline(8=on) + Baud Rate(2=9600) + Modbus Address(0x0100 = 1)
0,	//uint16_t GYRO; // XYZ-axis
PROFILES_COUNT,//uint16_t MMAX; // Max Memory number
DPS_PVER, 		//uint16_t PVER; 	//  before ver 4.2 RCNT; // Reset counters
4,		//uint16_t BCKL; // Backlight from BLED // before ver 4.2  C or F
65,		//uint16_t OHP;  // overheat temperature
0,		//uint16_t d25; // before ver 4.2 SMART; // Smart display
PARAM_SMART_DISPLAY,	//uint16_t PARAM; // Set of parameters      before ver 4.2 BSRC; // battery source
3000,	//uint16_t MINS;	// minimum source battery 0%
4200,	//uint16_t MAXS;	// maximum source battary 100%
SCLR1,	//uint16_t CLR1; //  PowerOff Color   white
SCLR2,	//uint16_t CLR2;	// CV color
SCLR3,	//uint16_t CLR3;	// CC color
0,		//uint16_t BEEP;
0, 		//int16_t CMD;	// Command register
0,		//uint16_t TIME_L;	// Time counter low byte
0, 		//uint16_t TIME_H;	// Time counter high byte
0,		//uint16_t MEM;  // Memory  Number  for Chinese firmware
0,		//uint16_t AHCNT_L;	// Amper/hour counter low byte
0,		//uint16_t AHCNT_H;	// Amper/hour counter high byte
0,		//uint16_t WHCNT_L;	// Watt/hour counter low byte
0,		//uint16_t WHCNT_H;	// Watt/hour counter high byte
0,		//uint16_t CLB_CMD;		//   Calibration Command  enumeration  calib_mode_t;
0,		//uint16_t CLB_IDX;		// Calibration Index of operation for DoCalibration(uint8_t aIdx)
0,		//uint16_t CLB_DATA_L;	// Calibration Data (uint32_t Value) Low Word for CalibRecord_t.Value
0,		//uint16_t CLB_DATA_H;	// Calibration Data (uint32_t Value) High Word for CalibRecord_t.Value
0,		//uint16_t d12;
0,		//uint16_t d13;
0,		//uint16_t d14;
0,		//uint16_t d15;
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},		//uint16_t Dummy[32];
//		Profile_t Profile[20];
//	USET	ISET  SOVPSOCP	SOPP	BLED	PRFS		SINI	OTIM
{	{500,	NCR,	0,	0,	OPP,	5,		0,			0,		MAX_TIM,		0,0,0,0,0,0,0},
	{500,	NCR,	0,	0,	OPP,	5,		0,			0,		MAX_TIM,		0,0,0,0,0,0,0},
	{500,	NCR,	0,	0,	OPP,	5,		0,			0,		MAX_TIM,		0,0,0,0,0,0,0},
	{500,	NCR,	0,	0,	OPP,	5,		0,			0,		MAX_TIM,		0,0,0,0,0,0,0},
	{500,	NCR,	0,	0,	OPP,	5,		0,			0,		MAX_TIM,		0,0,0,0,0,0,0},
	{500,	NCR,	0,	0,	OPP,	5,		0,			0,		MAX_TIM,		0,0,0,0,0,0,0},
	{500,	NCR,	0,	0,	OPP,	5,		0,			0,		MAX_TIM,		0,0,0,0,0,0,0},
	{500,	NCR,	0,	0,	OPP,	5,		0,			0,		MAX_TIM,		0,0,0,0,0,0,0},
	{500,	NCR,	0,	0,	OPP,	5,		0,			0,		MAX_TIM,		0,0,0,0,0,0,0},
	{500,	NCR,	0,	0,	OPP,	5,		0,			0,		MAX_TIM,		0,0,0,0,0,0,0},
//  --------------  battery charge  --------------------
	{420,	NCR,	0,	0,	85,		5,		0,			0,		300,		0,0,0,0,0,0,0},
	{840,	NCR,	0,	0,	170,	5,		0,			0,		300,		0,0,0,0,0,0,0},
	{1260,	NCR,	0,	0,	255,	5,		0,			0,		300,		0,0,0,0,0,0,0},
	{1680,	NCR,	0,	0,	340,	5,		0,			0,		300,		0,0,0,0,0,0,0},
	{2100,	NCR,	0,	0,	425,	5,		0,			0,		300,		0,0,0,0,0,0,0},
	{2520,	NCR,	0,	0,	510,	5,		0,			0,		300,		0,0,0,0,0,0,0},
	{2940,	NCR,	0,	0,	595,	5,		0,			0,		300,		0,0,0,0,0,0,0},
	{3360,	NCR,	0,	0,	680,	5,		0,			0,		300,		0,0,0,0,0,0,0},
	{3780,	NCR,	0,	0,	765,	5,		0,			0,		300,		0,0,0,0,0,0,0},
	{4200,	NCR,	0,	0,	850,	5,		0,			0,		300,		0,0,0,0,0,0,0} },

	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}}; // uint16_t Dummy2[48]; //  Calibration data
//*******************   END OF DPS MODEL VARIABLES *****************************

WiFiServer mb_server(MB_PORT);
WiFiClient client;

uint8_t mb_buffer[MB_BUFFER_SIZE]; // send and recieve buffer
bool wait_for_korad_response = false;
uint32_t rtu_time_stamp = 0;
uint32_t tcp_time_stamp = 0;
uint32_t korad_time_stamp = 0;
uint8_t tx_out_onoff;
korad_uart_msg_t uart_buffer;

//===================================================================================================
void korad_debug(const char * str)
{
	uint8_t * p = (uint8_t *) &uart_buffer;
	debug_msg(str );
	debug_msg(": ");
	for (uint8_t i = 0; i < sizeof(korad_uart_msg_t) / 2; i++)
	{
		debug_msg(" %02X%02X", *p, *(p + 1));	
		p += 2;	
	}
	debug_msg("\n");	
}
//===================================================================================================
unsigned char checksum (unsigned char *ptr, size_t sz) {
    unsigned char chk = 0;
    while (sz-- != 0)
        chk -= *ptr++;
    return chk;
}
//===================================================================================================
void KoradTransmitMessage(void)
{
  	while (Serial1.available())
		Serial1.read();  // clear rx buffer
	
	uint16_t idx = mb_Data.MEM;
	uint16_t diff = mb_Data.USET / 20;
	if ((mb_Data.Profile[idx].PRFS & PROFILE_SOFT_FRONT) && (softFrontVoltage + diff  < mb_Data.USET))
		softFrontVoltage += diff;
	else softFrontVoltage = mb_Data.USET;
	uart_buffer.magicKey = 0x20AA;
	uart_buffer.uValue = (uint16_t)(softFrontVoltage >> 8 | softFrontVoltage << 8);
	uart_buffer.iValue = (uint16_t)(mb_Data.ISET >> 8 | mb_Data.ISET << 8);
	uart_buffer.dummy1 = 0;
	uart_buffer.dummy2 = 0;
	uart_buffer.b01 = 1;
	uart_buffer.out = (uint8_t)mb_Data.ONOFF;
	uart_buffer.alarm = 0;
	uart_buffer.b00 = 0;
	uart_buffer.ocp = 0;
	uart_buffer.mode = 0;
	uart_buffer.dummy3 = 0;
	uart_buffer.dummy4 = 0;
  	uart_buffer.checksum = checksum((unsigned char *) &uart_buffer, sizeof(korad_uart_msg_t) - 1);
	
	tx_out_onoff = uart_buffer.out;
  	Serial1.write((const char *) &uart_buffer, sizeof(korad_uart_msg_t));
	Serial1.flush();
	wait_for_korad_response = true;
	korad_time_stamp = millis();
#ifdef KORAD_DEBUG_TX
	korad_debug("TX");
#endif
}
//===================================================================================================
void KoradReceiveMessage(void)
{
	uint16_t bytesReady = Serial1.available();

	if (bytesReady >= sizeof(korad_uart_msg_t)) 
	{
		memset(&uart_buffer, 0, sizeof(korad_uart_msg_t));
		bool err = Serial1.readBytes((uint8_t*)&uart_buffer, sizeof(korad_uart_msg_t)) != sizeof(korad_uart_msg_t);	
//		unsigned char chs = checksum((unsigned char *) &uart_rx_buffer, sizeof(korad_uart_msg_t) - 1);
//		err |= chs != uart_rx_buffer.checksum;
		err |= uart_buffer.magicKey != 0x20AA;
		if (!err)
		{
#ifdef KORAD_DEBUG_RX
			korad_debug("RX");
#endif
			mb_Data.UOUT = uart_buffer.uValue >> 8 | uart_buffer.uValue << 8;
			mb_Data.IOUT = uart_buffer.iValue >> 8 | uart_buffer.iValue << 8;
			uint32_t watt = (uint32_t)(mb_Data.UOUT * mb_Data.IOUT ) / (100 * gCurDiv);
			mb_Data.POWER = watt / 10;
			if (tx_out_onoff != uart_buffer.out)
				mb_Data.ONOFF = uart_buffer.out;	
		}
		wait_for_korad_response = false;
	}
}

/*=================================================================================================*/
static const char aucCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40
};

static const char aucCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
    0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
    0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
    0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
    0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 
    0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
    0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
    0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
    0x41, 0x81, 0x80, 0x40
};

/*=================================================================================================*/
/* Calculate CRC16 */
uint16_t crc16( uint8_t * pucFrame, uint16_t usLen )
{
    uint8_t           ucCRCHi = 0xFF;
    uint8_t           ucCRCLo = 0xFF;
    int             iIndex;

    while( usLen-- )
    {
        iIndex = ucCRCLo ^ *( pucFrame++ );
        ucCRCLo = ( uint8_t )( ucCRCHi ^ aucCRCHi[iIndex] );
        ucCRCHi = aucCRCLo[iIndex];
    }
    return ( uint16_t )( ucCRCHi << 8 | ucCRCLo );
}

/*=================================================================================================*/
void mb_debug(const char * str, uint16_t start, uint16_t len)
{
#ifdef MB_DEBUG
	debug_msg(str);
	for (uint16_t i = 0; i < len; i++) 
	{
		debug_msg(" %02X", mb_buffer[i + start]);
	}
	debug_msg("\n");
#endif
}

/*=================================================================================================*/
void mb_client_write(const uint8_t * buf, size_t size)
{
	if (client && client.connected()) 
	{
		client.write(buf, size);
		client.flush();
		tcp_time_stamp = millis();
		if (buf == mb_buffer)
			mb_debug("TCP TX:", 0, size);
	}
}
/*=================================================================================================*/
/* TCP client poll */
void modbusTcpServerTask(void)
{
  	uint8_t mb_func = MB_FC_NONE;
  	uint16_t len = 0;
  	uint16_t * mbData = (uint16_t *)&mb_Data;

	gLastFunc.func = 0;
  
	if (mb_server.hasClient()) // Check if there is new client
	{
		// if client is free - connect
		if (!client || !client.connected()){
			if(client) client.stop();
			client = mb_server.accept(); //.available();
			
		// client is not free - reject
		} 
		else 
		{
			WiFiClient serverClient = mb_server.accept(); //.available();
			serverClient.stop();
		}
	}

  	//-------------------- Read from socket --------------------
  	if (client && client.connected() && client.available())
  	{
		delay(1);
		uint16_t bytesReady;
		while((bytesReady = client.available()) && (len < MB_BUFFER_SIZE))
		{
			len += client.readBytes(&mb_buffer[len], bytesReady);
		}
		if (len > 8)  mb_func = mb_buffer[7];  //Byte 7 of request is FC
		mb_debug("TCP RX:", 0, len);
	}
	else 
	{
		return;
	}	
        //-------------------- Read Registers (3) --------------------
	if (mb_func == MB_FC_READ_REGISTERS)
	{
		gLastFunc.func = mb_func;
		gLastFunc.sreg = word(mb_buffer[8], mb_buffer[9]);	
		gLastFunc.nregs = word(mb_buffer[10], mb_buffer[11]);
		uint16_t byteDataLength = gLastFunc.nregs * 2;
    	mb_buffer[5] = byteDataLength + 3; 
    	mb_buffer[8] = byteDataLength;   
        for(int i = 0; i < gLastFunc.nregs; i++)
        {
            mb_buffer[9 + i * 2] = highByte(mbData[gLastFunc.sreg + i]);
            mb_buffer[10 + i * 2] =  lowByte(mbData[gLastFunc.sreg + i]);
        }
		mb_client_write(mb_buffer, byteDataLength + 9);
	}
        //-------------------- Write Register (6) --------------------
    else if(mb_func == MB_FC_WRITE_REGISTER) 
	{
		gLastFunc.func = mb_func;
		gLastFunc.sreg = word(mb_buffer[8], mb_buffer[9]);
		gLastFunc.nregs = 1;
		mbData[gLastFunc.sreg] = word(mb_buffer[10], mb_buffer[11]);
		mb_buffer[5] = 6; 
		mb_client_write(mb_buffer, 12);
	}
		//-------------------- Write Multiple Registers (16) --------------------
	else if(mb_func == MB_FC_WRITE_MULTIPLE_REGISTERS) 
	{
		gLastFunc.func = mb_func;
		gLastFunc.sreg = word(mb_buffer[8], mb_buffer[9]);
		gLastFunc.nregs = word(mb_buffer[10], mb_buffer[11]);
		mb_buffer[5] = 6;
		for(int i = 0; i < gLastFunc.nregs; i++)
		{
			mbData[gLastFunc.sreg + i] =  word(mb_buffer[ 13 + i * 2], mb_buffer[14 + i * 2]);
		}
		mb_client_write(mb_buffer, 12);
	}
}

/*=================================================================================================*/
#ifndef SERIAL_BUSY_BY_DEBUG
void mb_serial_write(const uint8_t* buf, size_t size)
{
	mb_debug("RTU TX:", 6, size);
	while(Serial.available())
		Serial.read();
	Serial.write(buf, size);
	Serial.flush();
	rtu_time_stamp = millis();
}

/*=================================================================================================*/
void modbusRtuSlaveTask(void)
{
	uint16_t len = 0;
	bool crc_err = false;
	bool timeout = true;
  	uint8_t mb_func = MB_FC_NONE;
  	uint16_t * mbData = (uint16_t *)&mb_Data;
	uint32_t last_time = micros();
	uint16_t crc;
	gLastFunc.func = 0;

	if (Serial.available() > 0) 
	{
		while (micros() - last_time < 1750)	
		{
			uint16_t bytesReady = Serial.available();
			if (bytesReady > 0)
			{
				last_time = micros();
				len += Serial.readBytes(&mb_buffer[6 + len], bytesReady);
				mb_func = mb_buffer[7];
				if ((len >= 8) && (mb_func == MB_FC_WRITE_MULTIPLE_REGISTERS) && (len >= mb_buffer[12] + 9)) 
					{timeout = false; crc_err = crc16(&mb_buffer[6], mb_buffer[12] + 9) != 0; break;} 
				if ((len >= 8) && ((mb_func == MB_FC_READ_REGISTERS) || (mb_func == MB_FC_WRITE_REGISTER))) 
					{timeout = false; crc_err = crc16(&mb_buffer[6], 8) != 0; break;} 
			}
		}
	}

	if (timeout) return;
	if (mb_buffer[6] != MB_DEVICE_ADDRESS) return;

 	if (crc_err)
	{
		mb_debug("RTU RX:", 6, len);
		mb_buffer[7] |= MB_FC_ERROR_MASK;
		mb_buffer[8] = 0x08; // error code
		crc = crc16(&mb_buffer[6], 3);
		mb_buffer[9] = lowByte(crc);
		mb_buffer[10] = highByte(crc);
		mb_serial_write(&mb_buffer[6], 5);
	}
	else 
	{
		gLastFunc.sreg = word(mb_buffer[8], mb_buffer[9]);	
		if (mb_func == MB_FC_WRITE_REGISTER) gLastFunc.nregs = 1;
		else gLastFunc.nregs = word(mb_buffer[10], mb_buffer[11]);
		if ((gLastFunc.sreg +  gLastFunc.nregs) > (sizeof(mb_Data) / 2 ))
		{
			mb_debug("RTU RX:", 6, len);
			mb_buffer[7] |= MB_FC_ERROR_MASK;
			mb_buffer[8] = 0x02; // error code
			crc = crc16(&mb_buffer[6], 3);
			mb_buffer[9] = lowByte(crc);
			mb_buffer[10] = highByte(crc);
			mb_serial_write(&mb_buffer[6], 5);
		}
		else if (mb_func == MB_FC_READ_REGISTERS)
		{
			gLastFunc.func = mb_func;
			uint16_t byteDataLength = gLastFunc.nregs * 2;
			mb_buffer[8] = byteDataLength;   
			for(int i = 0; i < gLastFunc.nregs; i++)
			{
				mb_buffer[9 + i * 2] = highByte(mbData[gLastFunc.sreg + i]);
				mb_buffer[10 + i * 2] =  lowByte(mbData[gLastFunc.sreg + i]);
			}
			crc = crc16(&mb_buffer[6], byteDataLength + 3);
			mb_buffer[6 + byteDataLength + 3] = lowByte(crc);
			mb_buffer[7 + byteDataLength + 3] = highByte(crc);
			mb_serial_write(&mb_buffer[6], byteDataLength + 3 + 2);
		}
			//-------------------- Write Register (6) --------------------
		else if (mb_func == MB_FC_WRITE_REGISTER) 
		{
			gLastFunc.func = mb_func;
			mbData[gLastFunc.sreg] = word(mb_buffer[10], mb_buffer[11]);
			mb_serial_write(&mb_buffer[6], 8);
		}
			//-------------------- Write Multiple Registers (16) --------------------
		else if (mb_func == MB_FC_WRITE_MULTIPLE_REGISTERS) 
		{
			gLastFunc.func = mb_func;
			for(int i = 0; i < gLastFunc.nregs; i++)
			{
				mbData[gLastFunc.sreg + i] =  word(mb_buffer[ 13 + i * 2], mb_buffer[14 + i * 2]);
			}
			crc = crc16(&mb_buffer[6], 6);
			mb_buffer[12] = lowByte(crc);
			mb_buffer[13] = highByte(crc);
			mb_serial_write(&mb_buffer[6], 8);
		}
	}	
}
#endif
/*=================================================================================================*/
/*=================================================================================================*/
// called from main.MainDisplayTask(), so it is thread-save
void modbus_updateGlobalData(void)
{
	model_global_data_t model;	
	EEPROM.get(EEPROM_GLOBAL_DATA_ADDRESS, model);
	bool res = model.magicKey != mb_Data.MGIC;
	res |= model.profileIdx != mb_Data.MEM;
	res |= model.param != mb_Data.PARAM;
	res |= model.maxmem != mb_Data.MMAX;
	res |= model.color1 != mb_Data.CLR1;
	res |= model.color2 != mb_Data.CLR2;
	res |= model.color3 != mb_Data.CLR3;
	if (res)
	{
		model.magicKey = mb_Data.MGIC;
		model.profileIdx = mb_Data.MEM;
		model.param = mb_Data.PARAM;
		model.maxmem = mb_Data.MMAX;
		model.color1 = mb_Data.CLR1;
		model.color2 = mb_Data.CLR2;
		model.color3 = mb_Data.CLR3;
		EEPROM.put(EEPROM_GLOBAL_DATA_ADDRESS, model);
		EEPROM.commit();
	}
}

/*=================================================================================================*/
// called from main.MainDisplayTask(), so it is thread-save
void modbus_updateCurrentProfile(void)
{
	model_profile_t profile;
	uint16_t i = mb_Data.MEM;
	int addr = EEPROM_GLOBAL_DATA_ADDRESS + sizeof(model_global_data_t) + i * sizeof(model_profile_t);
	EEPROM.get(addr, profile);
	bool res = mb_Data.Profile[i].USET != profile.uset;
	res |= mb_Data.Profile[i].ISET != profile.iset;
	res |= mb_Data.Profile[i].SOPP != profile.sopp;
	res |= mb_Data.Profile[i].OTIM != profile.otim;
	res |= mb_Data.Profile[i].PRFS != profile.prfs;
	if (res)
	{
		profile.uset = mb_Data.Profile[i].USET;
		profile.iset = mb_Data.Profile[i].ISET;
		profile.sopp = mb_Data.Profile[i].SOPP;
		profile.otim = mb_Data.Profile[i].OTIM;
		profile.prfs = mb_Data.Profile[i].PRFS;
		EEPROM.put(addr, profile);	
		EEPROM.commit();
	}
}

/*=================================================================================================*/
// called from main.setup(), so it is thread-save
void modbus_initData(void)
{
	model_global_data_t model;
	model_profile_t profile;
	int addr;
	EEPROM.get(EEPROM_GLOBAL_DATA_ADDRESS, model);
	if (model.magicKey == mb_Data.MGIC)
	{
		mb_Data.MEM = model.profileIdx;
		mb_Data.PARAM = model.param;
		mb_Data.MMAX = model.maxmem;
		mb_Data.CLR1 = model.color1;
		mb_Data.CLR2 = model.color2;
		mb_Data.CLR3 = model.color3;
		addr = EEPROM_GLOBAL_DATA_ADDRESS + sizeof(model_global_data_t);
		for (uint8_t i = 0; i < PROFILES_COUNT; i++)
		{
			EEPROM.get(addr, profile);	
			addr += sizeof(model_profile_t);
			mb_Data.Profile[i].USET = profile.uset;
			mb_Data.Profile[i].ISET = profile.iset;
			mb_Data.Profile[i].SOPP = profile.sopp;
			mb_Data.Profile[i].OTIM = profile.otim;
			mb_Data.Profile[i].PRFS = profile.prfs;
		}
	}
	else // first start
	{
		mb_Data.MEM = 0;
		model.magicKey = mb_Data.MGIC;
		model.profileIdx = mb_Data.MEM;
		model.param = mb_Data.PARAM;
		model.maxmem = mb_Data.MMAX;
		model.color1 = mb_Data.CLR1;
		model.color2 = mb_Data.CLR2;
		model.color3 = mb_Data.CLR3;
		EEPROM.put(EEPROM_GLOBAL_DATA_ADDRESS, model);
		addr = EEPROM_GLOBAL_DATA_ADDRESS + sizeof(model_global_data_t);
		for (uint8_t i = 0; i < PROFILES_COUNT; i++)
		{
			profile.uset = mb_Data.Profile[i].USET;
			profile.iset = mb_Data.Profile[i].ISET;
			profile.sopp = mb_Data.Profile[i].SOPP;
			profile.otim = mb_Data.Profile[i].OTIM;
			profile.prfs = mb_Data.Profile[i].PRFS;
			EEPROM.put(addr, profile);	
			addr += sizeof(model_profile_t);
		}
		EEPROM.commit();
	}
	uint16_t idx = mb_Data.MEM;
	mb_Data.USET = mb_Data.Profile[idx].USET;
	mb_Data.ISET = mb_Data.Profile[idx].ISET;			
	(mb_Data.STATE & STA_CUR_DIV) ? gCurDiv = 10 : gCurDiv = 1;  //  for KORAD3005 gCurDiv = 10
}

/*=================================================================================================*/
// this function notifies main.MainDisplayTask() and wait until done
void updateEeprom(const EventBits_t uxBitsToSet)
{
	xEventGroupSetBits(gModbusEepromSaveEventGroup, uxBitsToSet); // notify main.MainDisplayTask()
	xEventGroupWaitBits(gModbusEepromSaveEventGroup, EEPROM_NEED_SAVE_DONE_EVENT, pdTRUE, pdFALSE,  100 / portTICK_PERIOD_MS);  // Wait a maximum of 100ms for either bit to be set.
}

/*=================================================================================================*/
void IncDecChangeModeCount(int8_t aVal)
{
	changeModeCount += aVal;
	if (changeModeCount > 5) changeModeCount = 5;
	if (changeModeCount < -5) changeModeCount = -5;
}

/*=================================================================================================*/
void modbus_loop(void)
{
	static uint32_t wifi_reset_btn_time = millis();
	static uint64_t ahCount64 = 0;
	static uint64_t whCount64 = 0;
	static uint32_t timeCounter = 0;
   	static uint32_t lastTime = 0;
	static uint8_t lowCurrentTimeCount = 0;
	uint32_t cur_time = millis();
	
	//--------------------- bridge ---------------------------
	if (millis() - rtu_time_stamp >  RESPONSE_TIMEOUT_MS * 2)
	{
		if ((gNetworkTask == NETWORK_TASK_MAIN) && (WiFi.status() == WL_CONNECTED)) 
		{
			modbusTcpServerTask();
		}
	}
	gModbusConnected = (millis() - tcp_time_stamp) < RESPONSE_TIMEOUT_MS * 2;

#ifndef SERIAL_BUSY_BY_DEBUG
	if (millis() - tcp_time_stamp >  RESPONSE_TIMEOUT_MS * 2)
	{
		modbusRtuSlaveTask();	
	}
	gModbusConnected |= (millis() - rtu_time_stamp) < RESPONSE_TIMEOUT_MS * 2;
#endif

	uint16_t idx = mb_Data.MEM;
	//--------------------- apply modbus changes ---------------------------
	if ((gLastFunc.func = MB_FC_WRITE_REGISTER) && (gLastFunc.sreg == 6)) // gData.LOCK
	{
		if (mb_Data.LOCK == 0) // end of session
		{
			mb_Data.Profile[idx].USET = mb_Data.USET;
			mb_Data.Profile[idx].ISET = mb_Data.ISET;			
			updateEeprom(EEPROM_NEED_SAVE_GLOBAL_EVENT | EEPROM_NEED_SAVE_PROFILE_EVENT); // notify main.MainDisplayTask()
		} 
	}
	else if ((gLastFunc.func = MB_FC_WRITE_REGISTER) && (gLastFunc.sreg == 9)) // mb_Data.ONOFF
	{
		if (mb_Data.ONOFF)
		{
			mb_Data.PROTECT = 0; // clear protection register
			lowCurrentTimeCount = 0;
			softFrontVoltage = 0;
			if (mb_Data.PARAM & PARAM_RESET_COUNTERS) 
			{
				timeCounter = 0;
				ahCount64 = 0;
				whCount64 = 0;	
			}
		}
	}
	else if ((gLastFunc.func = MB_FC_WRITE_REGISTER) && (gLastFunc.sreg == 20)) // mb_Data.MMAX
	{
		updateEeprom(EEPROM_NEED_SAVE_GLOBAL_EVENT); // notify main.MainDisplayTask()
	}
	else if ((gLastFunc.func = MB_FC_WRITE_REGISTER) && (gLastFunc.sreg == 32)) // mb_Data.CMD
	{
		if (mb_Data.CMD == CMD_SAVE_GLOBAL)
		{
			updateEeprom(EEPROM_NEED_SAVE_GLOBAL_EVENT); // notify main.MainDisplayTask()	
		}
		else if (mb_Data.CMD == CMD_SAVE_PROFILE)
		{
			updateEeprom(EEPROM_NEED_SAVE_GLOBAL_EVENT | EEPROM_NEED_SAVE_PROFILE_EVENT); // notify main.MainDisplayTask()

		}
		else if (mb_Data.CMD == CMD_SAVE_SET_PROFILE)
		{
			mb_Data.Profile[idx].USET = mb_Data.USET;
			mb_Data.Profile[idx].ISET = mb_Data.ISET;			
			updateEeprom(EEPROM_NEED_SAVE_GLOBAL_EVENT | EEPROM_NEED_SAVE_PROFILE_EVENT); // notify main.MainDisplayTask()
		}
		else if (mb_Data.CMD == CMD_CLR_COUNTERS) // Clear counters
		{
			timeCounter = 0;
			ahCount64 = 0;
			whCount64 = 0;	
			mb_Data.AHCNT_L = 0;
			mb_Data.AHCNT_H = 0;
			mb_Data.WHCNT_L = 0;
			mb_Data.WHCNT_H = 0;
			mb_Data.TIME_L = 0;
			mb_Data.TIME_H = 0;
		}
	}
	else if ((gLastFunc.func = MB_FC_WRITE_REGISTER) && (gLastFunc.sreg == 35)) // mb_Data.MEM
	{
		// changing profile
		mb_Data.USET = mb_Data.Profile[idx].USET;
		mb_Data.ISET = mb_Data.Profile[idx].ISET;			
	}

	//---------------------------------------------------------------
	cur_time = millis();
	if (mb_Data.LOCK && mb_Data.ONOFF)
	{
		//--------------------- CV or CC ---------------------------
		uint32_t aU = mb_Data.UOUT * 0x7FFF /  mb_Data.USET;
		uint32_t aI = mb_Data.IOUT * 0x7FFF /  mb_Data.ISET;

		if (aU >= aI) IncDecChangeModeCount(1);
		else IncDecChangeModeCount(-1);
		if (aI > 0x7E00) IncDecChangeModeCount(-4);
		if (aU > 0x7E00) IncDecChangeModeCount(4);

		if (changeModeCount >= 5) mb_Data.CVCC = 0;
		if (changeModeCount <= -5)
			mb_Data.CVCC = 1;

		//--------------------- time and energy counters ---------------------------
		if ( (cur_time - lastTime) >= 1000) // one second timer
		{
			timeCounter++;
			lastTime = cur_time;
			mb_Data.TIME_L = timeCounter & 0xFFFF;
			mb_Data.TIME_H = timeCounter >> 16;	
			ahCount64 = ahCount64 + ((uint64_t)mb_Data.IOUT << 32) / (3600 * gCurDiv);
			whCount64 = whCount64 + (((uint64_t)mb_Data.IOUT * (uint64_t)mb_Data.UOUT) << 32) / (360000 * gCurDiv);
			uint32_t ahCount = ahCount64 >> 32;
			uint32_t  whCount = whCount64 >> 32;
			mb_Data.AHCNT_L = ahCount;
			mb_Data.AHCNT_H = ahCount >> 16;
			mb_Data.WHCNT_L = whCount;
			mb_Data.WHCNT_H = whCount >> 16;
			if (mb_Data.Profile[idx].OTIM > 0)
			{
				if (timeCounter / 60 >= mb_Data.Profile[idx].OTIM) 
					{
					mb_Data.PROTECT = protect_OTP;
					}
			}
			if (idx >= 10) // Charge battery profiles
			{
				if (mb_Data.IOUT  < 5 * gCurDiv){	//  50 mA
					if (lowCurrentTimeCount < CHARGE_LOW_CURRENT_OVERTIME)
						lowCurrentTimeCount++;
					else mb_Data.PROTECT = protect_BATLOWCURRENT;
				}
				else lowCurrentTimeCount = 0;
			}
		}
		//--------------------- protection ---------------------------
		if (mb_Data.USET > MAX_U)
			mb_Data.PROTECT = protect_EVP;
		else if (mb_Data.ISET > MAX_I)
			mb_Data.PROTECT = protect_ECP;
		else if ((mb_Data.Profile[idx].PRFS & PROFILE_TRIGGER_OVP) && (mb_Data.UOUT >= mb_Data.USET))
			mb_Data.PROTECT = protect_OVP;
		else if (mb_Data.POWER > mb_Data.Profile[idx].SOPP) 
			mb_Data.PROTECT = protect_OPP;
		else if (mb_Data.Profile[idx].PRFS & PROFILE_TRIGGER_OCP)
		{	
			uint16_t percent = mb_Data.ISET / 100;
			if (((mb_Data.IOUT > mb_Data.ISET) && (mb_Data.IOUT - mb_Data.ISET <= percent)) ||
			((mb_Data.ISET >= mb_Data.IOUT) && (mb_Data.ISET - mb_Data.IOUT <= percent)))
				mb_Data.PROTECT = protect_OCP;
		}

		if (mb_Data.PROTECT != protect_OK)
			mb_Data.ONOFF = 0;	
	}
	else
	{
		if (mb_Data.USET > MAX_U)		mb_Data.USET = MAX_U;
		else if (mb_Data.ISET > MAX_I) 	mb_Data.ISET = MAX_I;
	}

	//--------------------- korad ---------------------------
	cur_time = millis();
	if (!wait_for_korad_response && (mb_Data.LOCK !=0))
	{
		if (((cur_time - tcp_time_stamp) < RESPONSE_TIMEOUT_MS)
		|| ((cur_time - rtu_time_stamp) < RESPONSE_TIMEOUT_MS))
			
		{
			if ((cur_time - korad_time_stamp) >= KORAD_POLL_PERIOD_MS)
			{
				KoradTransmitMessage();	
			}
		}
	}

	if (wait_for_korad_response)
	{
		if ((millis() - korad_time_stamp) < RESPONSE_TIMEOUT_MS)
		{
			KoradReceiveMessage();
		}
		else 
		{
			wait_for_korad_response = false;
#ifdef KORAD_DEBUG_RX
			debug_msg("KORAD NO ANSWER");
#endif
		}
	}	
}

/*=================================================================================================*/
void modbus_tcp_server_run(bool value)
{
	static bool running = false;
	if (value != running)
	{
		if (value ) mb_server.begin();
		else mb_server.stop();
		running = value;
	}
}
/*=================================================================================================*/
