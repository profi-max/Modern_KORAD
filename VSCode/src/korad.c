/*******************************************************************************
Created by profi-max (Oleg Linnik) 2024
https://profimaxblog.ru
https://github.com/profi-max

*******************************************************************************/
#include <Arduino.h>
#include "driver/timer.h"
#include "korad.h"
#include "common.h"

typedef struct __attribute__((packed)) {
  uint16_t magicKey;
  uint16_t uSet;
  uint16_t iSet;
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
} uart_request_t;



bool gDataChanged; // to avoid the data corruption with interrupt
uint16_t gTempData[5]; // for transfer the data from interrupt
static unsigned long staticPrevDB_Time = 0xFFFFFFF; //  prev decimal bit time
static uint8_t staticDecimalBitIndex = 0;  // Decimal bit index
static uint32_t staticAlarmValue = 5000;
static const char* logTAG = "korad.c";


TaskHandle_t gInterruptTaskHandler;
void IRAM_ATTR DB_pin_interrupt_cb(void *args);
bool IRAM_ATTR timer_isr_cb(void *args);

//===================================================================================================
//===================================================================================================
void InterruptTask(void *pvParameter)
{
  ESP_LOGI(logTAG, "Interrupt task running on core %d", xPortGetCoreID()); 
  
  gpio_pad_select_gpio(KORAD_CLK_PIN);
  gpio_set_direction(KORAD_CLK_PIN, GPIO_MODE_INPUT);
  gpio_pullup_dis(KORAD_CLK_PIN);
  gpio_pulldown_en(KORAD_CLK_PIN);

  gpio_pad_select_gpio(KORAD_DATA_PIN);
  gpio_set_direction(KORAD_DATA_PIN, GPIO_MODE_INPUT);
  gpio_pullup_dis(KORAD_DATA_PIN);
  gpio_pulldown_en(KORAD_DATA_PIN);
  
  // Set interrupt mode to pin KORAD_DB_PIN
  gpio_pad_select_gpio(KORAD_DB_PIN);
  gpio_set_direction(KORAD_DB_PIN, GPIO_MODE_INPUT);
  gpio_pullup_dis(KORAD_DB_PIN);
  gpio_pulldown_en(KORAD_DB_PIN);
  gpio_set_intr_type(KORAD_DB_PIN, GPIO_INTR_POSEDGE);
  gpio_install_isr_service(ESP_INTR_FLAG_IRAM |  ESP_INTR_FLAG_EDGE | ESP_INTR_FLAG_LEVEL3);
  gpio_isr_handler_add(KORAD_DB_PIN, DB_pin_interrupt_cb, (void *)KORAD_DB_PIN);

  // timer for synchronize with KORAD 
   timer_config_t config = {
    .divider = 80,  //  tacting of the counter 1us
    .alarm_en = TIMER_ALARM_EN,
    .auto_reload = TIMER_AUTORELOAD_EN,
    .counter_dir = TIMER_COUNT_UP,
    .counter_en = TIMER_PAUSE,
    .intr_type = TIMER_INTR_MAX //interrupt level
  };
  timer_init(KORAD_TIMER_GROUP, KORAD_TIMER , &config); 
  timer_enable_intr(KORAD_TIMER_GROUP, KORAD_TIMER);
  timer_isr_callback_add(KORAD_TIMER_GROUP, KORAD_TIMER, timer_isr_cb, NULL, ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3);
  timer_set_counter_value(KORAD_TIMER_GROUP, KORAD_TIMER, 0);
  
  while(1)  vTaskSuspend(NULL); // suspend itself 
}
//===================================================================================================
//===================================================================================================
void IRAM_ATTR DB_pin_interrupt_cb(void *args) 
{
  static uint8_t counter = 0;
  if (counter == gPersistData.prescaler) { 
    unsigned long DB_Time = micros();
    if (DB_Time > staticPrevDB_Time) {
      uint32_t decimalBitPeriod = DB_Time - staticPrevDB_Time;
      if (decimalBitPeriod > 1000) {
        staticAlarmValue = decimalBitPeriod / (gPersistData.prescaler * 5) ;
        uint32_t alarmValue;
        if (gPersistData.beforeEdge) alarmValue = staticAlarmValue - 40;
        else alarmValue = staticAlarmValue - 200;  //9
        timer_group_set_alarm_value_in_isr(KORAD_TIMER_GROUP, KORAD_TIMER, alarmValue );
        timer_group_set_counter_enable_in_isr(KORAD_TIMER_GROUP, KORAD_TIMER, true); //  start timer
        staticDecimalBitIndex = 0;
#ifdef DEBUG_WITH_OSCILL
        REG_SET_BIT(GPIO_OUT_REG, 1 << BEEPER_OUTPUT_PIN); // set BEEPER_OUTPUT_PIN level 1 for debug with oscilloscope
#endif
      }
    }
    counter = 0;
    staticPrevDB_Time = DB_Time; 
  }
  counter++;
}
//===================================================================================================
bool IRAM_ATTR timer_isr_cb(void *args)
{
  static uint16_t staticData[5];
  register uint16_t aData = 0;
  register uint32_t portData, level, oldLevel;
  register uint16_t timeout = 5000; // (2000 for KA3005D only) 5000 => 500us 
  register uint16_t bitCount = 0;

#ifdef DEBUG_WITH_OSCILL
  REG_SET_BIT(GPIO_OUT_REG, 1 << KORAD_UART_TX);  // set KORAD_UART_TX level 1 for debug with oscilloscope
  REG_CLR_BIT(GPIO_OUT_REG, 1 << BEEPER_OUTPUT_PIN); // set BEEPER_OUTPUT_PIN level 0 for debug with oscilloscope
#endif
  
  // Use REG_READ(GPIO_IN_REG) for GPIO0~31 input or use REG_READ(GPIO_IN1_REG) for GPIO32~39 input
  portData = REG_READ(KORAD_GPIO_INPUT);  
  level = portData & CLK_BIT_MASK;
  oldLevel = level;
  while(timeout--) {
    portData = REG_READ(KORAD_GPIO_INPUT);
    level = portData & CLK_BIT_MASK;
    if (level != oldLevel) {
      if (level) { // raising edge
        aData <<= 1;
        if (portData & DATA_BIT_MASK)  aData |= 1; 
        bitCount++;
        timeout = 240; // // (100 for KA3005D only) 240 => 30us
      }
      oldLevel = level;
    }  
  }

  if (bitCount > 15) {
    staticData[staticDecimalBitIndex] = aData;
    staticDecimalBitIndex++;
    if (staticDecimalBitIndex >= 5) {
      timer_group_set_counter_enable_in_isr(KORAD_TIMER_GROUP, KORAD_TIMER, false);   // stop timer
      timer_set_counter_value(KORAD_TIMER_GROUP, KORAD_TIMER, 0);
      memcpy(gTempData, staticData, sizeof(staticData)); // transfer the data
      staticDecimalBitIndex = 0;
      gVerificationCounter++;
      gDataTimeout = 0;
      gDataChanged = true; 
    }
#ifdef DEBUG_WITH_OSCILL
    REG_CLR_BIT(GPIO_OUT_REG, 1 << KORAD_UART_TX); // set KORAD_UART_TX level 0 for debug with oscilloscope
#endif
    timer_group_set_alarm_value_in_isr(KORAD_TIMER_GROUP, KORAD_TIMER, staticAlarmValue); 
  }
  else {
    gInterruptTimeoutErrorCounter++;
#ifdef DEBUG_WITH_OSCILL
    REG_CLR_BIT(GPIO_OUT_REG, 1 << KORAD_UART_TX); // set KORAD_UART_TX level 0 for debug with oscilloscope
#endif
    timer_group_set_counter_enable_in_isr(KORAD_TIMER_GROUP, KORAD_TIMER, false);   // stop timer
    timer_set_counter_value(KORAD_TIMER_GROUP, KORAD_TIMER, 0);
  }
  return false;
}
//===================================================================================================
//===================================================================================================
uint8_t GetVoltBitValue(uint8_t aBitValue, bool* dp)
{
  *dp = (aBitValue & 4) != 0;  // check decimal point
  aBitValue &= 0b11111011; // exclude decimal point
  switch (aBitValue) {
    case 0:  return 10;  // this decimal bit is blinking
    case 0xFA: return 0;
    case 0x22: return 1;
    case 0xB9: return 2;
    case 0xAB: return 3;
    case 0x63: return 4;
    case 0xCB: return 5;
    case 0xDB: return 6;
    case 0xA2: return 7;
    case 0xFB: return 8;
    case 0xEB: return 9;
    default: return 11;  // error data code
  }
}
//===================================================================================================
uint8_t GetAmperBitValue(uint8_t aBitValue, bool* dp)
{
  *dp = (aBitValue & 4) != 0;  // check decimal point
  aBitValue &= 0b11111011; // exclude decimal point
  switch (aBitValue) {
    case 0:  return 10;  // this decimal bit is blinking
    case 0xFA: return 0;
    case 0x82: return 1;
    case 0xB9: return 2;
    case 0xAB: return 3;
    case 0xC3: return 4;
    case 0x6B: return 5;
    case 0x7B: return 6;
    case 0xA2: return 7;
    case 0xFB: return 8;
    case 0xEB: return 9;
    default: return 11;  // error data code
  }
}
//===================================================================================================
unsigned char checksum (unsigned char *ptr, size_t sz) {
    unsigned char chk = 0;
    while (sz-- != 0)
        chk -= *ptr++;
    return chk;
}
//===================================================================================================
void transmitKoradCommand(uint16_t uSet, uint16_t iSet, bool onOff, bool block)
{
  if (block && gData.outputOn) return;
  uart_request_t uart_tx_buffer = {
    .magicKey = 0x20AA,
    .uSet = (uSet >> 8) | (uSet << 8),
    .iSet = (iSet >> 8) | (iSet << 8),
    .dummy1 = 0,
    .dummy2 = 0,
    .b01 = 1,
    .out = 0,
    .alarm = 0,
    .b00 = 0,
    .ocp = 0,
    .mode = 0,
    .dummy3 = 0,
    .dummy4 = 0
  };
  uart_tx_buffer.out = onOff;
  if (gVerifiedData.ocp) uart_tx_buffer.ocp = 1;
  uart_tx_buffer.checksum = checksum((unsigned char *) &uart_tx_buffer, sizeof(uart_request_t) - 1);
  uartTransmitBuffer((const char *) &uart_tx_buffer, sizeof(uart_request_t));
}
//===================================================================================================
