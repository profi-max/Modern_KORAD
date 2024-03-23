/*******************************************************************************
Created by profi-max (Oleg Linnik) 2024
https://profimaxblog.ru
https://github.com/profi-max

*******************************************************************************/
#ifndef _KORAD_PROJECT_COMMON_H
#define _KORAD_PROJECT_COMMON_H 

#ifdef __cplusplus
extern "C" {
#endif

#include <Arduino.h>
#include "lvgl.h"
#include <esp_task_wdt.h>

/* Change to your screen resolution */
#define SCREEN_WIDTH 480
#define SCREEN_HEIGHT 320
#define BRIGHTNESS_STEP 10

/* Bits for gModbusEepromSaveEventGroup*/
#define EEPROM_NEED_SAVE_GLOBAL_EVENT BIT0
#define EEPROM_NEED_SAVE_PROFILE_EVENT BIT1
#define EEPROM_NEED_SAVE_DONE_EVENT BIT2

#define ASSERT(cond) int foo[(cond) ? 1 : -1]


// TYPES
typedef struct {    //  data stored in EEPROM
    uint32_t magicKey;
    lv_color16_t cvColor;
    lv_color16_t ccColor;
    lv_color16_t pwrColor;
    lv_color16_t resColor;
    lv_color16_t offColor;
    uint16_t bgColorIdx;
    uint8_t fontIdx;
    uint8_t brightness;
    uint8_t ScreensaverIdx;
    uint8_t beeperVolume;
    bool autoReset;
    bool useUART;
    bool blockUART;
    uint8_t prescaler;
    uint8_t shiftData;
    bool reversData;
    bool beforeEdge;
    bool wifi;
    bool reconnect;
}  persistent_data_t;

typedef struct {  //  for display oscill and power
    uint16_t uOut;
    uint16_t iOut;
    uint32_t power;
    uint32_t resist;
    bool outputOn;
    bool error;
} korad_data_t;

typedef struct {  //  for oscill SetRange, Energy calc and program via UART1 
    uint16_t uSet;
    uint16_t iSet;
    uint16_t uOut;
    uint16_t iOut;
    uint8_t uPointPos;
    uint8_t iPointPos;
    uint64_t ahCount64;
    uint64_t whCount64;
    bool ocp;
} verified_data_t;

typedef struct {  // data for WebSocket
    char uStr[6];
    char iStr[6];
    uint16_t leds;
} wsData_t;

typedef struct {
  const lv_font_t * big;
  const lv_font_t * small;
} fontSet_t;


typedef enum {
  NETWORK_DISCONNECTED,
  NETWORK_CONNECTING,
  NETWORK_WAIT_IP,
  NETWORK_CONNECT_FAILED,
  NETWORK_CONNECTED
} Network_Status_t;

typedef enum {
  NETWORK_TASK_NONE,
  NETWORK_TASK_STOP,
  NETWORK_TASK_MAIN,
  NETWORK_TASK_SCAN,
  NETWORK_TASK_CONNECT
} Network_Task_t;


// GLOBAL VARIABLES

extern EventGroupHandle_t gModbusEepromSaveEventGroup;

extern lv_color_t CLR_GRAY;
extern lv_color_t CLR_DARKGRAY;
extern lv_color_t CLR_WHITE;
extern lv_color_t CLR_BLACK;
extern lv_color_t CLR_CREEN;
extern lv_color_t CLR_RED;
extern lv_color_t CLR_SHADOW;
extern lv_color_t CLR_BORDER;


extern uint8_t gScreensaverArray[7];
extern lv_color16_t bgColorsArray[5];
extern fontSet_t fontsArray[3];


extern persistent_data_t gPersistData;
extern korad_data_t gData;
extern verified_data_t gVerifiedData; 
extern bool gFullUpdate;
extern uint32_t gTimeCount;
extern bool gDisplayBlinking;
extern bool gDrawOscill;
extern char * gLogoText;
extern uint16_t gLogoLength;
extern uint32_t gScreensaverTimeout;
extern uint32_t gPopupTimeout;
extern uint32_t gDataTimeout;
extern uint32_t gValueErrorCounter;
extern uint32_t gInterruptTimeoutErrorCounter;
extern uint32_t gVerificationCounter;
extern uint32_t gWifiClientsCounter;
extern bool gBeeperEnable;
extern bool gModbusConnected;


extern Network_Status_t gNetworkStatus;
extern Network_Task_t gNetworkTask;
extern char tmpSsidName[32];
extern char tmpSsidPassword[32];
extern char locSsidName[32];
extern uint32_t locIPaddress;

extern lv_obj_t * uLabel;
extern lv_obj_t * iLabel;
extern lv_obj_t * pLabel;
extern lv_obj_t * rLabel;
extern lv_obj_t * ovpLabel;
extern lv_obj_t * ocpLabel;
extern lv_obj_t * cvLabel;
extern lv_obj_t * ccLabel;
extern lv_obj_t * mLabel; // memory number label
extern lv_obj_t * outLabel;
extern lv_obj_t * uBar;
extern lv_obj_t * iBar;
extern lv_obj_t * ahLabel; 
extern lv_obj_t * whLabel; 
extern lv_obj_t * tLabel; 
extern lv_obj_t * wifiLabel; 
extern lv_obj_t * logoLabel; 
extern lv_obj_t * ui_WifiList;
extern lv_obj_t * ui_WifiSpinner;
extern lv_timer_t * wifiTimer;


// function from main.cpp
int debug_msg(const char *format, ...);
void setBrightness(uint8_t value);
void SavePersistentData(void);
void uartBeginEnd(bool state);
void uartTransmitBuffer(const char * buffer, size_t size);
void updateWifiList();
void directDraw (int32_t x, int32_t y, int32_t w, int32_t h, const uint16_t* data);
void changeNetworkTask(Network_Task_t newTask);

// function from ui_events.h
void ui_AfterInit(void);
void LoadPreviousScreen(void);
void DebugScreenUpdateErrors(void);

// functions
void initCommonData(void);
void updateStateLabel(lv_obj_t * aLabel, bool active);
void updateOutLabel(bool active);
void updateUIndicator(char* str, uint16_t aValue, uint8_t pointPos, int8_t blinkPos);
void updateIIndicator(char* str, uint16_t aValue, uint8_t pointPos, int8_t blinkPos);
void recolorUIIndicator(lv_color_t color);
void updatePIndicator(bool fullUpdate);
void updateRIndicator(bool fullUpdate);
void UpdateEnergyLabels(void);
void ResetEnergyCounters(void);
uint32_t strToUint(const char * text, uint8_t dpPos);
void uint16toStr(uint16_t aValue, char* str, uint8_t dpPos, int8_t blinkPos, bool firstZero);
void beepInit(void);
void beepLoop(void);
void doBeep(void);


#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif