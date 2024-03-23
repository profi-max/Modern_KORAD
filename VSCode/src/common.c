/*******************************************************************************
Created by profi-max (Oleg Linnik) 2024
https://profimaxblog.ru
https://github.com/profi-max

*******************************************************************************/
#include "common.h"
#include <lvgl.h>
#include "lv_conf.h"
#include "korad.h"
#include <driver/ledc.h>


EventGroupHandle_t gModbusEepromSaveEventGroup;

lv_color_t CLR_GRAY;
lv_color_t CLR_DARKGRAY;
lv_color_t CLR_WHITE;
lv_color_t CLR_BLACK;
lv_color_t CLR_CREEN;
lv_color_t CLR_RED;
lv_color_t CLR_SHADOW;
lv_color_t CLR_BORDER;
lv_color16_t bgColorsArray[5];
uint8_t gScreensaverArray[7] = {0, 5, 10, 15, 20, 25, 30};

static char logoText[] = "  KORAD      KA3005D       DIGITAL CONTROL DC  POWER SUPPLY     30V 5A  ";

persistent_data_t gPersistData = {
  .magicKey = 10012024,   // day month year when compiled  
  .autoReset = true,
  .useUART = false,
  .blockUART = true,
  .brightness = 230,
  .ScreensaverIdx = 0,
  .beeperVolume = 3,
  .bgColorIdx = 2,
  .fontIdx = 0,
  .prescaler = 5,
  .shiftData = 0,
  .reversData = false,
  .beforeEdge = false,
  .wifi = false,
  .reconnect = false
};

korad_data_t gData = {
  .uOut = 0,
  .iOut = 0,
  .power = 0,
  .resist = 0,
  .outputOn = false,
};

verified_data_t gVerifiedData;

bool gFullUpdate = false;
uint32_t gTimeCount = 0;
bool gDisplayBlinking = false; // hide or show Power and Resistance labels data, and stop-run oscill
bool gDrawOscill = false;
char * gLogoText;
uint16_t gLogoLength;
uint32_t gScreensaverTimeout = 0;
uint32_t gPopupTimeout = 0;
uint32_t gDataTimeout = 0;
uint32_t gValueErrorCounter = 0;
uint32_t gInterruptTimeoutErrorCounter = 0;
uint32_t gVerificationCounter = 0;
uint32_t gWifiClientsCounter = 0;
bool gBeeperEnable = true;
bool gModbusConnected = false;

Network_Status_t gNetworkStatus = NETWORK_DISCONNECTED;
Network_Task_t gNetworkTask = NETWORK_TASK_STOP;

char tmpSsidName[32];
char tmpSsidPassword[32];
char locSsidName[32];
uint32_t locIPaddress;
unsigned long beepEndTime = 0;

lv_obj_t * uLabel = NULL;
lv_obj_t * iLabel = NULL;
lv_obj_t * pLabel = NULL;
lv_obj_t * rLabel = NULL;
lv_obj_t * ovpLabel = NULL;
lv_obj_t * ocpLabel = NULL;
lv_obj_t * cvLabel = NULL;
lv_obj_t * ccLabel = NULL;
lv_obj_t * mLabel = NULL; // memory number label
lv_obj_t * outLabel = NULL;
lv_obj_t * uBar = NULL;
lv_obj_t * iBar = NULL;
lv_obj_t * ahLabel = NULL; 
lv_obj_t * whLabel = NULL; 
lv_obj_t * tLabel = NULL; 
lv_obj_t * wifiLabel = NULL; 
lv_obj_t * logoLabel = NULL; 
lv_obj_t * ui_WifiList;
lv_obj_t * ui_WifiSpinner;

//===================================================================================================
void initCommonData(void)
{
  CLR_GRAY = lv_color_hex(0x8f8f8f);
  CLR_DARKGRAY = lv_color_hex(0x6f6f6f);
  CLR_WHITE = lv_color_white();
  CLR_BLACK = lv_color_black();
  CLR_CREEN = lv_palette_main(LV_PALETTE_GREEN);
  CLR_RED = lv_palette_main(LV_PALETTE_RED);
  CLR_SHADOW = lv_color_hex(0x60BEFF);
  CLR_BORDER = lv_color_hex(0x878787);

  bgColorsArray[4] = lv_palette_main(LV_PALETTE_BLUE_GREY);
  bgColorsArray[4] = lv_color_darken(bgColorsArray[4], LV_OPA_60);
  bgColorsArray[3] = lv_color_darken(bgColorsArray[4], LV_OPA_70);
  bgColorsArray[2] = lv_color_darken(bgColorsArray[4], LV_OPA_80);
  bgColorsArray[1] = lv_color_darken(bgColorsArray[4], LV_OPA_90);
  bgColorsArray[0] = CLR_BLACK;

  gPersistData.cvColor = lv_palette_main(LV_PALETTE_YELLOW);
  gPersistData.ccColor = lv_palette_main(LV_PALETTE_LIGHT_BLUE);
  gPersistData.pwrColor = lv_palette_main(LV_PALETTE_ORANGE);
  gPersistData.resColor = lv_palette_main(LV_PALETTE_GREEN);
  gPersistData.offColor = lv_color_white();

  gLogoText = logoText;
  gLogoLength = sizeof(logoText);
}
//===================================================================================================
void updateStateLabel(lv_obj_t * aLabel, bool active)
{
  if (active) {
    lv_obj_set_style_bg_color(aLabel, CLR_WHITE, LV_PART_MAIN); 
    lv_obj_set_style_text_color(aLabel, CLR_BLACK, LV_PART_MAIN);
  }
  else {
    lv_obj_set_style_bg_color(aLabel, CLR_GRAY, LV_PART_MAIN); 
    lv_obj_set_style_text_color(aLabel, CLR_DARKGRAY, LV_PART_MAIN);
  }
}
//===================================================================================================
void updateOutLabel(bool active)
{
  if (outLabel != NULL) {
    if (active) {
      lv_obj_set_style_bg_color(outLabel, CLR_CREEN, LV_PART_MAIN); 
      lv_label_set_text(outLabel, "ON");
      lv_obj_set_style_shadow_color(outLabel, CLR_SHADOW, LV_PART_MAIN);
      lv_obj_set_style_border_color(outLabel, CLR_SHADOW, LV_PART_MAIN);
    }
    else {
      lv_obj_set_style_bg_color(outLabel, CLR_RED, LV_PART_MAIN); 
      lv_label_set_text(outLabel, "OFF");
      lv_obj_set_style_shadow_color(outLabel, bgColorsArray[gPersistData.bgColorIdx], LV_PART_MAIN);
      lv_obj_set_style_border_color(outLabel, CLR_BORDER, LV_PART_MAIN);
    }
  }
  if ((uBar != NULL) && (iBar != NULL)) {
    if (active) {
      lv_obj_set_style_bg_color(uBar, gPersistData.cvColor, LV_PART_INDICATOR);
      lv_obj_set_style_bg_color(iBar, gPersistData.ccColor, LV_PART_INDICATOR);
    }
    else {
      lv_obj_set_style_bg_color(uBar, CLR_GRAY, LV_PART_INDICATOR);
      lv_obj_set_style_bg_color(iBar, CLR_GRAY, LV_PART_INDICATOR);
    }
  }
}
//===================================================================================================
void uint32toStr(uint32_t value, char * str, uint8_t len, uint8_t dpPos)
{
	const static DRAM_ATTR uint32_t Mul32[8] = {10000000, 1000000, 100000, 10000, 1000, 100,  10, 1};
  bool nobit = true;
  if (len > 7) return;
  dpPos = 7 - dpPos;
  uint8_t i = 8 - len;
  bool err = value >= Mul32[i - 1];
  for (; i < 8; i++) {
    uint32_t aBit = value / Mul32[i];
    if (err) *str = '-';
    else if ((aBit == 0) && nobit && (i < dpPos)) continue; //*str = ' ';
    else {*str = 0x30 + aBit; nobit = false;}
    value -= aBit * Mul32[i];
    str++;
    if (i == dpPos) {
      *str = '.'; str++;
    }
  }
  *str = 0;
}
/*****************************************************************************************/
void uint16toStr(uint16_t aValue, char* str, uint8_t dpPos, int8_t blinkPos, bool firstZero)
{
  const static DRAM_ATTR uint16_t Mul[4] = {1000, 100, 10, 1};
  uint8_t i;
  bool error = aValue >= VALUE_ERR_CODE;
  for (i=0; i < 4; i++) {
    uint16_t aVal = aValue / Mul[i];
    if (error) *str ='-';
    else if (i == blinkPos)  *str = ' ';
    else  *str = (char) aVal + 0x30;
    aValue -= aVal * Mul[i];
    str++;
    if (i == dpPos) {
      *str = '.';
      str++;
    }
    else if ((i == 0) && !firstZero && (aVal == 0))
      str--;
  }
  *str = 0;
}

//===================================================================================================
void updateUIndicator(char* str, uint16_t aValue, uint8_t pointPos, int8_t blinkPos)
{
  static uint16_t oldValue = 0xFFFF;
  static int8_t oldBlinkPos = -1;
  static int8_t prevBlinkPos = -1;
  if (uBar != NULL)
    if (gFullUpdate || ((oldValue != aValue) && (blinkPos < 0)) ){
      if (aValue < VALUE_ERR_CODE) {
        uint16_t pos = aValue * 100 / MAX_U;
        lv_bar_set_value(uBar, pos, LV_ANIM_OFF);  
      }
    }
  if (uLabel != NULL)
    if (gFullUpdate || (oldValue != aValue) || (oldBlinkPos != blinkPos)) {
        oldValue = aValue;
        oldBlinkPos = blinkPos;  
        uint16toStr(aValue, str, pointPos, blinkPos, true);
        if ((str[0] == '0') && (prevBlinkPos != 0))
        str[0] = ' ';
        prevBlinkPos = blinkPos;
        lv_label_set_text(uLabel, str);
    }
}
//===================================================================================================
void updateIIndicator(char* str, uint16_t aValue, uint8_t pointPos, int8_t blinkPos)
{
  static uint16_t oldValue = 0xFFFF;
  static int8_t oldBlinkPos = -1;
  if (iBar != NULL)
    if (gFullUpdate || ((oldValue != aValue) && (blinkPos < 0))) {
      if (aValue < VALUE_ERR_CODE) {
        uint16_t pos = aValue * 100 / MAX_I;
        lv_bar_set_value(iBar, pos, LV_ANIM_OFF);  
      }
    }
  if (iLabel != NULL)
    if (gFullUpdate || (oldValue != aValue) || (oldBlinkPos != blinkPos)) {
        oldValue = aValue;
        oldBlinkPos = blinkPos;  
        uint16toStr(aValue, str, pointPos, blinkPos, true);
        lv_label_set_text(iLabel, str);
    }
}
//===================================================================================================
void recolorUIIndicator(lv_color_t color)
{
//  static lv_color_t oldColor = CLR_WHITE;
  if ((uLabel != NULL) && (iLabel != NULL)) {
    lv_obj_set_style_text_color(uLabel, color, LV_PART_MAIN);
    lv_obj_set_style_text_color(iLabel, color, LV_PART_MAIN); 
  }
}
//===================================================================================================
void updatePIndicator(bool fullUpdate)
{
  static uint32_t oldValue = ENERGY_ERR_CODE;
  uint32_t value = gData.power;

  if (pLabel != NULL) {
    lv_color_t color;
    if (gDisplayBlinking || (value == ENERGY_ERR_CODE)) {
      color = CLR_GRAY;
      value = ENERGY_ERR_CODE;
    }
    else color = gData.outputOn ? gPersistData.pwrColor : gPersistData.offColor; 
    if (lv_obj_get_style_text_color(pLabel, LV_PART_MAIN).full != color.full) {
      lv_obj_set_style_text_color(pLabel, color, LV_PART_MAIN);
    }
  
    if (fullUpdate || (oldValue != value)) {
        oldValue = value;
        char str[8];
        uint32toStr(value, str, 4, 1);
        lv_label_set_text(pLabel, str);
    }
  }
}
//===================================================================================================
void updateRIndicator(bool fullUpdate)
{
  static uint32_t oldValue = ENERGY_ERR_CODE;
  uint32_t value = gData.resist;

  if (rLabel != NULL) {
    lv_color_t color;
    if (gDisplayBlinking || (value == ENERGY_ERR_CODE)) {
      color = CLR_GRAY;
      value = ENERGY_ERR_CODE;
    }
    else color = gData.outputOn ? gPersistData.resColor : gPersistData.offColor; 
    if (lv_obj_get_style_text_color(rLabel, LV_PART_MAIN).full != color.full) {
      lv_obj_set_style_text_color(rLabel, color, LV_PART_MAIN);
    }

    if (fullUpdate || (oldValue != value)) {
        oldValue = value;
        char str[8];
        uint32toStr(value, str, 5, 1);
        lv_label_set_text(rLabel, str);
    }
  }
}
//===================================================================================================
void UpdateEnergyLabels(void)
{
  char str[12];
	if (tLabel != NULL) {
    const static DRAM_ATTR uint32_t aMul[7] = {360000, 36000, 3600, 600, 60, 10, 1};
    char * pstr = str;
    bool nobit = true;
    uint32_t aValue = gTimeCount;
    for (uint8_t i = 0; i < 7; i++) {
      uint32_t abit = aValue / aMul[i];
      if ((abit == 0) && nobit && (i <4))  continue; //*pstr = ' ';
      else {
        *pstr = abit + 0x30;  nobit = false; 
        if ((i == 2) || (i == 4)) {pstr++; *pstr = ':';}
        }
        pstr++;
      aValue -= abit * aMul[i];
    }
    *pstr = 0;
    lv_label_set_text(tLabel, str);
  }
  if (ahLabel != NULL) {
    uint32toStr(gVerifiedData.ahCount64 >> 32, str, 7, 3);
    lv_label_set_text(ahLabel, str);
  }
  if (whLabel != NULL) {
    uint32toStr(gVerifiedData.whCount64 >> 32, str, 7, 2);
    lv_label_set_text(whLabel, str);
  }
}
//===================================================================================================
void ResetEnergyCounters(void)
{
  gTimeCount = 0;
  gVerifiedData.ahCount64 = 0;
  gVerifiedData.whCount64 = 0;
  UpdateEnergyLabels();
}
//===================================================================================================
uint32_t strToUint(const char * text, uint8_t dpPos) 
{
	const static DRAM_ATTR uint16_t Mul16[6] = {1000, 100, 10, 1, 0, 0};
	uint8_t len = strlen(text);
	uint32_t value = 0;
	bool dp = false;
	for (uint8_t i=0; i < len; i++) {
		uint8_t aBit = text[i] - 0x30;
		if (text[i] == '.') { dp = true; continue;}
		if (dp) {
			dpPos++;
			value += aBit * Mul16[dpPos];
		}
		else value = value * 10 + aBit * Mul16[dpPos];
	}
  return value;
}
//===================================================================================================
void beepInit(void)
{
#ifndef DEBUG_WITH_OSCILL
  ledcSetup(BEEPER_PWM_CHANNEL, 2000, 8);
#endif
}
//===================================================================================================
void beepLoop(void)
{
#ifndef DEBUG_WITH_OSCILL
  if (beepEndTime) {
    if (millis() >= beepEndTime) {
      beepEndTime = 0;
      ledcDetachPin(BEEPER_OUTPUT_PIN);
    }
  }
#endif
}
//===================================================================================================
void doBeep(void)
{
#ifndef DEBUG_WITH_OSCILL
  if (gPersistData.beeperVolume) {
    if (gBeeperEnable && (beepEndTime == 0)) {
      beepEndTime = millis() + 50;
      ledcAttachPin(BEEPER_OUTPUT_PIN, BEEPER_PWM_CHANNEL);
      ledcWrite(BEEPER_PWM_CHANNEL, 25 * gPersistData.beeperVolume);  
    }
  }
#endif
}
//===================================================================================================



