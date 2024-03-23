/*
 * oscill.c
 *
 *  Created on: 27 янв. 2022 г.
 *      Author: profi-max
 *
 */

#include <string.h>
#include <lvgl.h>
#include "common.h"
#include "korad.h"
#include "oscill.h"
#include "ui.h"

#define OSC_X 40
#define OSC_Y 40
#define OSC_HEIGHT 180
#define OSC_WIDTH 320
#define OSC_VERT_DIVISION 5
#define OSC_GRADIENT_CNT 20  // for Antialiasing lines drawing

typedef struct{
	int16_t Volt;
	int16_t Amper;
} OscDataRec_t;
typedef struct{
	uint8_t Vp;
	uint8_t Ap;
} OscPointRec_t;

const uint16_t RangeArray[8] = {20,50,100,200,500,1000,2000,5000}; // 100 means 1 volt and 100 means 1 amper when iFactor = 10 (1000 means 1 amper if iFactor = 1)

OscDataRec_t OscData[OSC_WIDTH];
OscPointRec_t OscPoints[OSC_WIDTH];
uint16_t lineDotsTemplate[OSC_HEIGHT + 1];
uint16_t lineGridTemplate[OSC_HEIGHT + 1];
uint16_t uGradient[OSC_GRADIENT_CNT];
uint16_t iGradient[OSC_GRADIENT_CNT];
uint16_t uRangeIdx = 5, iRangeIdx = 5;
uint8_t gOscMode = 3; //  1bit - V ray; 2bit - A ray; 3bit - A ray in front
uint32_t OscIdx = 0;
uint16_t iFactor = 1; // = 10 for 10.00 Amper

//=============================================================================================
void OscUpdateRange_Volt(void)
{
	int32_t y;
	for (uint16_t i = 0; i < OSC_WIDTH; i++){
		y = OSC_HEIGHT - ((OscData[i].Volt * OSC_HEIGHT / OSC_VERT_DIVISION) / RangeArray[uRangeIdx]) - 1;
		if (y < 0) y = 0;
		else if (y >= OSC_HEIGHT) y = OSC_HEIGHT - 1;
		OscPoints[i].Vp = y;
	}
	char str[12];
	uint16toStr(RangeArray[uRangeIdx], str, gVerifiedData.uPointPos, -1, false);
	strcat(str, " V/d");
	lv_label_set_text(ui_OscVoltLabel, str);
}
//=============================================================================================
void OscUpdateRange_Amper(void)
{
	int32_t y;
	for (uint16_t i = 0; i < OSC_WIDTH; i++){
		y = OSC_HEIGHT - ((OscData[i].Amper * OSC_HEIGHT / OSC_VERT_DIVISION ) / RangeArray[iRangeIdx]) - 1;
		if (y < 0) y = 0;
		else if (y >= OSC_HEIGHT) y = OSC_HEIGHT - 1;
		OscPoints[i].Ap = y;
	}
	char str[12];
	uint16toStr(RangeArray[iRangeIdx] * iFactor, str, gVerifiedData.iPointPos, -1, false);
	strcat(str, " A/d");
	lv_label_set_text(ui_OscAmperLabel, str);
}
//=============================================================================================
void oscSetRange(void)
{
	uint16_t i;
	uint8_t oldRangeIdxV = uRangeIdx;
	uint8_t oldRangeIdxA = iRangeIdx;

	for (i = 0; i < 7; i++) {
		if (gVerifiedData.uSet <  (RangeArray[i] * OSC_VERT_DIVISION)) break;
	}
	uRangeIdx = i;
	for (i = 0; i < 8; i++) {
		if (gVerifiedData.iSet <  (RangeArray[i] * OSC_VERT_DIVISION)) break;
	}
	iRangeIdx = i;

	if (oldRangeIdxV != uRangeIdx)
		OscUpdateRange_Volt();
	if (oldRangeIdxA != iRangeIdx)
		OscUpdateRange_Amper();
}
//=============================================================================================
void IRAM_ATTR OscUpdateData(void)
{
	if (!gDisplayBlinking && !gData.error) 
	{
		uint16_t aIdx = OscIdx % OSC_WIDTH;
		if (gVerifiedData.iPointPos == 1) iFactor = 10;
		else iFactor = 1;
		OscData[aIdx].Volt = gData.uOut;
		OscData[aIdx].Amper = gData.iOut;
		OscIdx++;
		// Update Volt point
		int32_t y = OSC_HEIGHT - ((gData.uOut * OSC_HEIGHT / OSC_VERT_DIVISION) / RangeArray[uRangeIdx]) - 1;
		if (y < 0) y = 0;
		else if (y >= OSC_HEIGHT) y = OSC_HEIGHT - 1;
		OscPoints[aIdx].Vp = y;
		// Update Amper point
		y = OSC_HEIGHT - ((gData.iOut * OSC_HEIGHT / OSC_VERT_DIVISION) / RangeArray[iRangeIdx]) - 1;
		if (y < 0) y = 0;
		else if (y >= OSC_HEIGHT) y = OSC_HEIGHT - 1;
		OscPoints[aIdx].Ap = y;
	}
}
//=============================================================================================

	//---------------------------------------------------------------------
/*	void DrawVerticalLine__(uint16_t * buf, uint8_t Prev, uint8_t Next, uint16_t Color)
	{
		uint8_t dif;
		if (Prev > Next){
			dif = (Prev - Next) / 2;
			while (dif > 0) {
				buf[Next + dif] = Color;
				dif--;
			}
		}
		else if (Next > Prev){
			dif = (Next - Prev) / 2;
			while (dif > 0) {
				buf[Next - dif] = Color;
				dif--;
			}
		}
	}*/
	//---------------------------------------------------------------------
	void DrawVerticalLine__(uint16_t * buf, uint8_t Prev, uint8_t Next, uint16_t * gradient)
	{
		uint8_t dif, max;
		uint16_t i;
		if (Prev > Next){
			dif = (Prev - Next); 
			max = dif;
			while (dif > 0) {
				i = OSC_GRADIENT_CNT * dif / max;
				buf[Next + dif] = gradient[i];
				dif--;
			}
		}
		else if (Next > Prev){
			dif = (Next - Prev); 
			max = dif;
			while (dif > 0) {
				i = OSC_GRADIENT_CNT * dif / max;
				buf[Next - dif] = gradient[i];
				dif--;
			}
		}
	}
	//---------------------------------------------------------------------
	void DrawVoltageLine(uint16_t * buf, uint32_t x, int32_t aIdx, int32_t nIdx1, int32_t nIdx2)
	{
		// Draw Voltage point
		if ((gOscMode & 1) != 0){
			uint32_t yV = OscPoints[aIdx].Vp;
			buf[yV] = uGradient[0];  
			// Draw Voltage vertical line
			if (x > 0)
				DrawVerticalLine__(buf, OscPoints[nIdx1].Vp, yV, uGradient);
			if (x < OSC_WIDTH - 1)
				DrawVerticalLine__(buf, OscPoints[nIdx2].Vp, yV, uGradient);
		}
	}
	//---------------------------------------------------------------------
	void DrawAmperageLine(uint16_t * buf, uint32_t x, int32_t aIdx, int32_t nIdx1, int32_t nIdx2)
	{
		// Draw Amperage point
		if ((gOscMode & 2) != 0){
			uint32_t yA = OscPoints[aIdx].Ap;
			buf[yA] = iGradient[0];  
			// Draw Amperage vertical line
			if (x > 0)
				DrawVerticalLine__(buf, OscPoints[nIdx1].Ap, yA, iGradient);
			if (x < OSC_WIDTH - 1)
				DrawVerticalLine__(buf, OscPoints[nIdx2].Ap, yA, iGradient);
		}
	}
	//---------------------------------------------------------------------

//=============================================================================================

void DrawOscill(void)
{
	static uint16_t line1[OSC_HEIGHT + 1]; //  first buffer
	static uint16_t line2[OSC_HEIGHT + 1];	// second buffer
	uint16_t * buf;
	int32_t aIdx, nIdx1, nIdx2;
	uint32_t c = 0x1F - (OscIdx & 0x1F);
	for (uint32_t x = 0; x < OSC_WIDTH; x++){
		if (x & 1)  buf = line1;
		else buf = line2;

		// Draw grid
		if ((x & 0x1F) == c) memcpy(buf, lineGridTemplate, OSC_HEIGHT * 2 + 2);
		else memcpy(buf, lineDotsTemplate, OSC_HEIGHT * 2 + 2);
		aIdx = (OscIdx + x) % OSC_WIDTH;
		// Previous point
		nIdx1 = aIdx - 1;
		if (nIdx1 < 0) nIdx1 = OSC_WIDTH - 1;
		// Next point
		nIdx2 = aIdx + 1;
		if (nIdx2 >= OSC_WIDTH) nIdx2 = nIdx2 - OSC_WIDTH;

		if ((gOscMode & 4) == 0){
			DrawAmperageLine(buf, x, aIdx, nIdx1, nIdx2);
			DrawVoltageLine(buf, x, aIdx, nIdx1, nIdx2);  // Voltage line in front
		}
		else{
			DrawVoltageLine(buf, x, aIdx, nIdx1, nIdx2);
			DrawAmperageLine(buf, x, aIdx, nIdx1, nIdx2); // Amperage line in front
		}

		directDraw(OSC_X  + x, OSC_Y, 1, OSC_HEIGHT + 1, buf);
	}
}
//==============================================================================================================
void oscLoop(void)
{
	OscUpdateData();
	if (gDrawOscill) DrawOscill();
}
//==============================================================================================================
void OscClearData(void)
{
	for (uint16_t i=0; i < OSC_WIDTH; i++){
		OscData[i].Volt = 0;
		OscData[i].Amper = 0;
		OscPoints[i].Vp = OSC_HEIGHT - 1;
		OscPoints[i].Ap = OSC_HEIGHT - 1;
	}
}
//=============================================================================================
void oscInitColors(void)
{
	lv_color16_t uColor = gPersistData.cvColor;  //(COLOR_VOLT)
	lv_color16_t iColor = gPersistData.ccColor;  //(COLOR_AMPER);
	lv_color16_t bgColor = lv_color_black();
	uint16_t y;
	for (y = 0; y < OSC_GRADIENT_CNT; y++) {
		uint16_t mix = 255 * y / OSC_GRADIENT_CNT;
		uGradient[y] = lv_color_mix(bgColor, uColor, mix).full;  //  from color to black
		iGradient[y] = lv_color_mix(bgColor, iColor, mix).full;
	}
	uint16_t GridColor = lv_color_hex(0x6f6f6f).full;   //DARKGRAY);
	for (y=0; y <= OSC_HEIGHT; y++)
		lineGridTemplate[y] = GridColor;
	for (y=0; y < OSC_HEIGHT; y++)
		lineDotsTemplate[y] = bgColorsArray[gPersistData.bgColorIdx].full; //or CLR_BLACK;
//	for (y=0; y < OSC_HEIGHT; y = y + OSC_HEIGHT / OSC_VERT_DIVISION)
//		lineDotsTemplate[y] = GridColor;
	for (y=0; y <= OSC_VERT_DIVISION; y++)
		lineDotsTemplate[y * (OSC_HEIGHT / OSC_VERT_DIVISION)] = GridColor;
}
//=============================================================================================
void oscInit(void)
{
	OscClearData();
	oscInitColors();
}
//=============================================================================================

