/*******************************************************************************
Created by profi-max (Oleg Linnik) 2024
https://profimaxblog.ru
https://github.com/profi-max

*******************************************************************************/
#ifndef _KORAD_PROJECT_KORAD_H
#define _KORAD_PROJECT_KORAD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <Arduino.h>

/*
    gVariable means global variable
    sVariable or staticVariable means static variable
    dp  means decimal point
    dpPos means decimal point position: (1.234  dpPos = 0) (12.34 dpPos = 1)
    pin DB means decimal bit
    pin DB0 means LEDs
    pin DB1 means (1)234
    pin DB2 means 1(2)34 and so on
    gRowData[0] is LEDs
*/

 // WARNING! The define below (DEBUG_WITH_OSCILL) might crash the application in white screen!!! I do`nt know why
 //#define DEBUG_WITH_OSCILL // for debug with external oscilloscope


#ifdef WT32SC01PLUS
    #define KORAD_DB_PIN   GPIO_NUM_10  // Decimal bit
    #define KORAD_CLK_PIN  GPIO_NUM_11  // clock
    #define KORAD_DATA_PIN GPIO_NUM_12  // data
    #define KORAD_UART_TX GPIO_NUM_13
    #define KORAD_UART_RX GPIO_NUM_21  
    #define BEEPER_OUTPUT_PIN GPIO_NUM_14
    #define BEEPER_PWM_CHANNEL 2 // check LCD backlight PWM channel !!!

    #define KORAD_GPIO_INPUT GPIO_IN_REG
    #define CLK_BIT_MASK  1 << KORAD_CLK_PIN
    #define DATA_BIT_MASK  1 << KORAD_DATA_PIN  

    #define KORAD_TIMER TIMER_0
    #define KORAD_TIMER_GROUP TIMER_GROUP_0
#else
    /* ESP-IDF Programming Guide: GPIO34-39 can only be set as input mode and do not have software-enabled pullup or pulldown functions.*/

    #define KORAD_DB_PIN   GPIO_NUM_35  // Decimal bit
    #define KORAD_CLK_PIN  GPIO_NUM_32  // clock
    #define KORAD_DATA_PIN GPIO_NUM_33  // data
    #define KORAD_UART_TX GPIO_NUM_27
    #define KORAD_UART_RX GPIO_NUM_34  
    #define BEEPER_OUTPUT_PIN GPIO_NUM_26
    #define BEEPER_PWM_CHANNEL 0 // check LCD backlight PWM channel !!!

    /* Use REG_READ(GPIO_IN_REG) for GPIO0~31 input 
    #define KORAD_GPIO_INPUT GPIO_IN_REG
    #define CLK_BIT_MASK  1 << KORAD_CLK_PIN
    #define DATA_BIT_MASK  1 << KORAD_DATA_PIN  */

    /* Use REG_READ(GPIO_IN1_REG) for GPIO32~39 input */
    #define KORAD_GPIO_INPUT GPIO_IN1_REG
    #define CLK_BIT_MASK  1 << (KORAD_CLK_PIN - 32)
    #define DATA_BIT_MASK  1 << (KORAD_DATA_PIN - 32)

    #define KORAD_TIMER TIMER_0
    #define KORAD_TIMER_GROUP TIMER_GROUP_0
#endif

#define BIT_MASK_M1 0x8000
#define BIT_MASK_M2 0x0080
#define BIT_MASK_M3 0x0200
#define BIT_MASK_M4 0x0004
#define BIT_MASK_M5 0x0002
#define BIT_MASK_OVP 0x2000
#define BIT_MASK_OCP 0x0800
#define BIT_MASK_CC  0x0100
#define BIT_MASK_CV  0x0010
#define BIT_MASK_OUT 0x0001
#define BIT_MASK_MODBUS 0x4000 // no LED
#define BIT_MASK_MEMORIES (BIT_MASK_M1 | BIT_MASK_M2 | BIT_MASK_M3 | BIT_MASK_M4 | BIT_MASK_M5 | BIT_MASK_MODBUS)
#define VALUE_ERR_CODE 10000
#define ENERGY_ERR_CODE 0xFFFFFFFF
#define MAX_U 3100
#define MAX_I 5100


// Global variables
extern bool gDataChanged; // to avoid the data corruption with interrupt
extern uint16_t gTempData[5]; // for transfer the data from interrupt
extern TaskHandle_t gInterruptTaskHandler;

// functions
void InterruptTask(void *pvParameter);
uint8_t GetVoltBitValue(uint8_t aBitValue, bool* dp); // dp means decimal point
uint8_t GetAmperBitValue(uint8_t aBitValue, bool* dp); // dp means decimal point
void transmitKoradCommand(uint16_t uSet, uint16_t iSet, bool onOff);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif