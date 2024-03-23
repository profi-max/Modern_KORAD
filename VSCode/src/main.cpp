/*******************************************************************************
Created by profi-max (Oleg Linnik) 2024
https://profimaxblog.ru
https://github.com/profi-max

*******************************************************************************/
#define LGFX_USE_V1

#include <Arduino.h>
#include <lvgl.h>
#include <LovyanGFX.hpp>
#include <esp_task_wdt.h>
#include <HardwareSerial.h>
#include <EEPROM.h>
#include <vector>
#include <WiFi.h> // needed to connect to WiFi
#include <ArduinoJson.h> // needed for JSON encapsulation (send multiple variables with one string)
#include <WiFiClientSecure.h>
#include <WebSocketsServer.h>
#include <WebServer.h>

#include "ui.h"
#include "korad.h"
#include "common.h"
#include "oscill.h"
#include "modbus.h"


#define NETWORK_TIMEOUT 10000
#define EEPROM_SIZE sizeof(persistent_data_t) + sizeof(model_global_data_t) + PROFILES_COUNT * sizeof(model_profile_t)	

const char* hostName = "KORAD_3005D";
static const char* logTAG = "main.cpp";

#ifdef WT32SC01PLUS
#define SCR 32
class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_ST7796 _panel_instance;
  lgfx::Bus_Parallel8 _bus_instance;
  lgfx::Light_PWM _light_instance;
  lgfx::Touch_FT5x06 _touch_instance;

public:
  LGFX(void)
  {
    {
      auto cfg = _bus_instance.config();

      cfg.port = 0;
      cfg.freq_write = 40000000;
      cfg.pin_wr = 47; // pin number connecting WR
      cfg.pin_rd = -1; // pin number connecting RD
      cfg.pin_rs = 0;  // Pin number connecting RS(D/C)
      cfg.pin_d0 = 9;  // pin number connecting D0
      cfg.pin_d1 = 46; // pin number connecting D1
      cfg.pin_d2 = 3;  // pin number connecting D2
      cfg.pin_d3 = 8;  // pin number connecting D3
      cfg.pin_d4 = 18; // pin number connecting D4
      cfg.pin_d5 = 17; // pin number connecting D5
      cfg.pin_d6 = 16; // pin number connecting D6
      cfg.pin_d7 = 15; // pin number connecting D7

      _bus_instance.config(cfg);              // Apply the settings to the bus.
      _panel_instance.setBus(&_bus_instance); // Sets the bus to the panel.
    }

    {                                      // Set display panel control.
      auto cfg = _panel_instance.config(); // Get the structure for display panel settings.

      cfg.pin_cs = -1;   // Pin number to which CS is connected (-1 = disable)
      cfg.pin_rst = 4;   // pin number where RST is connected (-1 = disable)
      cfg.pin_busy = -1; // pin number to which BUSY is connected (-1 = disable)

      // * The following setting values ​​are set to general default values ​​for each panel, and the pin number (-1 = disable) to which BUSY is connected, so please try commenting out any unknown items.

      cfg.memory_width = 320;  // Maximum width supported by driver IC
      cfg.memory_height = 480; // Maximum height supported by driver IC
      cfg.panel_width = 320;   // actual displayable width
      cfg.panel_height = 480;  // actual displayable height
      cfg.offset_x = 0;        // Panel offset in X direction
      cfg.offset_y = 0;        // Panel offset in Y direction
      cfg.offset_rotation = 0;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits = 1;
      cfg.readable = false;
      cfg.invert = true;
      cfg.rgb_order = false;
      cfg.dlen_16bit = false;
      cfg.bus_shared = true;

      _panel_instance.config(cfg);
    }

    {                                      // Set backlight control. (delete if not necessary)
      auto cfg = _light_instance.config(); // Get the structure for backlight configuration.

      cfg.pin_bl = 45;     // pin number to which the backlight is connected
      cfg.invert = false;  // true to invert backlight brightness
      cfg.freq = 44100;    // backlight PWM frequency
      cfg.pwm_channel = 7; // PWM channel number to use

      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance); // Sets the backlight to the panel.
    }

    { // Configure settings for touch screen control. (delete if not necessary)
      auto cfg = _touch_instance.config();

      cfg.x_min = 0;   // Minimum X value (raw value) obtained from the touchscreen
      cfg.x_max = 319; // Maximum X value (raw value) obtained from the touchscreen
      cfg.y_min = 0;   // Minimum Y value obtained from touchscreen (raw value)
      cfg.y_max = 479; // Maximum Y value (raw value) obtained from the touchscreen
      cfg.pin_int = 7; // pin number to which INT is connected
      cfg.bus_shared = false;
      cfg.offset_rotation = 0;

      // For I2C connection
      cfg.i2c_port = 0;    // Select I2C to use (0 or 1)
      cfg.i2c_addr = 0x38; // I2C device address number
      cfg.pin_sda = 6;     // pin number where SDA is connected
      cfg.pin_scl = 5;     // pin number to which SCL is connected
      cfg.freq = 400000;   // set I2C clock

      _touch_instance.config(cfg);
      _panel_instance.setTouch(&_touch_instance); // Set the touchscreen to the panel.
    }

    setPanel(&_panel_instance); // Sets the panel to use.
  }
};

#else
#define SCR 32
class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_ST7796 _panel_instance;
  lgfx::Bus_SPI _bus_instance;
  lgfx::Light_PWM _light_instance;
  lgfx::Touch_FT5x06 _touch_instance;

public:
  LGFX(void)
  {
    {
      auto cfg = _bus_instance.config(); // Get the structure for bus configuration.

      // SPI bus settings
      cfg.spi_host = VSPI_HOST; // Select the SPI to use ESP32-S2,C3 : SPI2_HOST or SPI3_HOST / ESP32 : VSPI_HOST or HSPI_HOST
      // * With the ESP-IDF version upgrade, VSPI_HOST and HSPI_HOST descriptions are deprecated, so if an error occurs, use SPI2_HOST and SPI3_HOST instead.
      cfg.spi_mode = 3;                  // Set SPI communication mode (0 ~ 3)
      cfg.freq_write = 27000000;         // SPI clock when sending (up to 80MHz, rounded to 80MHz divided by an integer)
      cfg.freq_read = 6000000;           // SPI clock when receiving
      cfg.spi_3wire = false;             // set to true if receiving on MOSI pin
      cfg.use_lock = true;               // set to true to use transaction lock
      cfg.dma_channel = SPI_DMA_CH_AUTO; // Set the DMA channel to use (0=not use DMA / 1=1ch / 2=ch / SPI_DMA_CH_AUTO=auto setting)
      // * With the ESP-IDF version upgrade, SPI_DMA_CH_AUTO (automatic setting) is recommended for the DMA channel. Specifying 1ch and 2ch is deprecated.
      cfg.pin_sclk = 14; // set SPI SCLK pin number
      cfg.pin_mosi = 13; // Set MOSI pin number for SPI
      cfg.pin_miso = -1; // set SPI MISO pin number (-1 = disable)
      cfg.pin_dc = 21;   // Set SPI D/C pin number (-1 = disable)

      _bus_instance.config(cfg);              // Apply the settings to the bus.
      _panel_instance.setBus(&_bus_instance); // Sets the bus to the panel.
    }

    {                                      // Set display panel control.
      auto cfg = _panel_instance.config(); // Get the structure for display panel settings.

      cfg.pin_cs = 15;   // Pin number to which CS is connected (-1 = disable)
      cfg.pin_rst = 22;  // pin number where RST is connected (-1 = disable)
      cfg.pin_busy = -1; // pin number to which BUSY is connected (-1 = disable)

      // * The following setting values ​​are set to general default values ​​for each panel, and the pin number (-1 = disable) to which BUSY is connected, so please try commenting out any unknown items.

      cfg.memory_width = SCREEN_HEIGHT;  // Maximum width supported by driver IC
      cfg.memory_height = SCREEN_WIDTH; // Maximum height supported by driver IC
      cfg.panel_width = SCREEN_HEIGHT;   // actual displayable width
      cfg.panel_height = SCREEN_WIDTH;  // actual displayable height
      cfg.offset_x = 0;        // Panel offset in X direction
      cfg.offset_y = 0;        // Panel offset in Y direction
      cfg.offset_rotation = 1;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits = 1;
      cfg.readable = false;
      cfg.invert = false;
      cfg.rgb_order = false;
      cfg.dlen_16bit = false;
      cfg.bus_shared = false;

      _panel_instance.config(cfg);
    }

    {                                      // Set backlight control. (delete if not necessary)
      auto cfg = _light_instance.config(); // Get the structure for backlight configuration.

      cfg.pin_bl = 23;     // pin number to which the backlight is connected
      cfg.invert = false;  // true to invert backlight brightness
      cfg.freq = 44100;    // backlight PWM frequency
      cfg.pwm_channel = 1; // PWM channel number to use

      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance); // Sets the backlight to the panel.
    }

    { // Configure settings for touch screen control. (delete if not necessary)
      auto cfg = _touch_instance.config();

      cfg.x_min = 0;    // Minimum X value (raw value) obtained from the touchscreen
      cfg.x_max = SCREEN_HEIGHT - 1;  // Maximum X value (raw value) obtained from the touchscreen
      cfg.y_min = 0;    // Minimum Y value obtained from touchscreen (raw value)
      cfg.y_max = SCREEN_WIDTH - 1;  // Maximum Y value (raw value) obtained from the touchscreen
      cfg.pin_int = 39; // pin number to which INT is connected
      cfg.bus_shared = false;
      cfg.offset_rotation = 0;

      // For I2C connection
      cfg.i2c_port = 1;    // Select I2C to use (0 or 1)
      cfg.i2c_addr = 0x38; // I2C device address number
      cfg.pin_sda = 18;    // pin number where SDA is connected
      cfg.pin_scl = 19;    // pin number to which SCL is connected
      cfg.freq = 400000;   // set I2C clock

      _touch_instance.config(cfg);
      _panel_instance.setTouch(&_touch_instance); // Set the touchscreen to the panel.
    }

    setPanel(&_panel_instance); // Sets the panel to use.
  }
};
#endif

// Create an instance of the prepared class.
LGFX tft;

static uint16_t gRowData[5];  // the  main data
static wsData_t wsTempData;   // data for websocket

//HardwareSerial SerialPort(1); // use UART1
TimerHandle_t energyProgTimer, blinkProgTimer;

std::vector<String> foundWifiList;

xSemaphoreHandle wifiListMutex, websocketMutex;
bool foundWifiListReady = false;
static EventGroupHandle_t dataChangedEventGroup;
static EventGroupHandle_t networkChangeTaskEventGroup;
#define DATA_CHANGED_EVT BIT0
#define NETWORK_CHANGE_TASK_RQST_EVT BIT0
#define NETWORK_CHANGE_TASK_DONE_EVT BIT1

Network_Task_t newNetworkTask;
TaskHandle_t gMainDisplayTaskHandler, gNetworkTaskHandler, gIdleTaskHandler;

WebServer server(80);                                 // the server uses port 80 (standard port for websites
WebSocketsServer webSocket = WebSocketsServer(81);    // the websocket uses port 81 (standard port for websockets


extern const char index_html_start[] asm("_binary_src_index_html_start");
extern const char index_html_end[] asm("_binary_src_index_html_end");
extern const char favicon_ico_start[] asm("_binary_src_favicon_ico_start");
extern const char favicon_ico_end[] asm("_binary_src_favicon_ico_end");

void MainDisplayTask(void *pvParameter);
void IdleTask(void *pvParameter);
void NetworkTask(void *pvParameter);
void updateDebugData(void);
void energyProgTimer_cb( TimerHandle_t xTimer );
void blinkProgTimer_cb( TimerHandle_t xTimer );
void onHeartBeat(lv_timer_t *timer);

//=====================================================================================================================
void IRAM_ATTR my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  if (tft.getStartCount() == 0) // Processing if not yet started
  {
    tft.startWrite();  // prepare SPI for transaction
  }
  // Yes, yes. ESP32-S3 has 8/16-bit parallel LCD intrface with DMA
  tft.waitDMA();  
  tft.pushImageDMA(area->x1, area->y1, area->x2 - area->x1 + 1, area->y2 - area->y1 + 1, &color_p->full);
    // tft.pushImageDMA(area->x1, area->y1, area->x2 - area->x1 + 1, area->y2 - area->y1 + 1, (lgfx::swap565_t *)&color_p->full);
  
  lv_disp_flush_ready(disp); /* tell lvgl that flushing is done */
}
//=====================================================================================================================
void IRAM_ATTR directDraw (int32_t x, int32_t y, int32_t w, int32_t h, const uint16_t* data)
{
  if (tft.getStartCount() == 0)
  {
    tft.startWrite();
  }
  tft.waitDMA();
  tft.pushImageDMA(x, y, w, h, data);
}
//=====================================================================================================================
void IRAM_ATTR my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  uint16_t touchX, touchY;
  if( tft.getTouch( &touchX, &touchY ) )
  {
      data->state = LV_INDEV_STATE_PR;
      /*Set the coordinates*/
      data->point.x = touchX;
      data->point.y = touchY;
  }
  else data->state = LV_INDEV_STATE_REL;
}
//=====================================================================================================================
void IRAM_ATTR my_touchpad_feedback(lv_indev_drv_t *indev_driver, uint8_t data)
{
  gScreensaverTimeout = 0;
  gPopupTimeout = 0;
}
//=====================================================================================================================
void SavePersistentData(void)
{
  EEPROM.writeBytes(0, &gPersistData, sizeof(gPersistData));
  EEPROM.commit();
}
//=====================================================================================================================
void setBrightness(uint8_t value)
{
  tft.setBrightness(value);
}
//=====================================================================================================================
void uartBeginEnd(bool state)
{
  if (state) Serial1.begin(9600, SERIAL_8N1, KORAD_UART_RX, KORAD_UART_TX);
  else Serial1.end();
}
//=====================================================================================================================
void uartTransmitBuffer(const char * buffer, size_t size)
{
#ifndef DEBUG_WITH_OSCILL
	if (gPersistData.useUART && !gModbusConnected) Serial1.write(buffer, size);
#endif
}
//=====================================================================================================================
/*********************************************************************************************************************
 *                                    SETUP & LOOP
**********************************************************************************************************************/
int debug_msg(const char *format, ...)
{

      int len;
    va_list arg;
    va_start(arg, format);
    len = Serial.printf(format, arg);
    va_end(arg);
    return len;
}

//=====================================================================================================================
int redirect_debug_msg(const char *format, va_list args)
{
  return debug_msg(format, args);
}

//=====================================================================================================================
void setup()
{

//  uint8_t *disp_draw_buf = (uint8_t *)heap_caps_malloc(SCREEN_WIDTH * SCR, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
//  uint8_t *disp_draw_buf2 = (uint8_t *)heap_caps_malloc(SCREEN_WIDTH * SCR, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);

  static lv_disp_draw_buf_t draw_buf;
  static lv_disp_drv_t disp_drv;
  static lv_color_t disp_draw_buf[SCREEN_WIDTH * SCR];
  static lv_color_t disp_draw_buf2[SCREEN_WIDTH * SCR];

  // Serial for debug messages and Modbus RTU :
  // WT32SC01PLUS RX0=GPIO44 TX0=GPIO43
  // WT32SC01 Serial is over USB
  // see -DCORE_DEBUG_LEVEL=3  in the platformio.ini
  // see #define SERIAL_BUSY_BY_DEBUG in the modbus.cpp
  Serial.begin(115200);
  //esp_log_set_vprintf(redirect_debug_msg); // redirect debug messages

  
  tft.init(); // Initialize LovyanGFX
  tft.initDMA(); // Yes, yes. ESP32-S3 has 8/16-bit parallel LCD intrface with DMA
  tft.startWrite();
  setBrightness(0);

  lv_init(); // Initialize lvgl

  // Setting display to landscape
  if (tft.width() < tft.height())  tft.setRotation(tft.getRotation() ^ 1);

  if (!disp_draw_buf)  ESP_LOGE(logTAG, "LVGL disp_draw_buf allocate failed!"); 
  else {
    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, disp_draw_buf2, SCREEN_WIDTH * SCR);

    /* Initialize the display */
    lv_disp_drv_init(&disp_drv);
    /* Change the following line to your display resolution */
    disp_drv.hor_res = SCREEN_WIDTH;
    disp_drv.ver_res = SCREEN_HEIGHT;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    /* Initialize the input device driver */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    indev_drv.feedback_cb = my_touchpad_feedback;
    lv_indev_drv_register(&indev_drv);


    initCommonData();
    if (EEPROM.begin(EEPROM_SIZE)) {
      if (EEPROM.readULong(0) == gPersistData.magicKey) {
        EEPROM.readBytes(0, &gPersistData, sizeof(gPersistData));
      }
      else {
        EEPROM.writeBytes(0, &gPersistData, sizeof(gPersistData));
        EEPROM.commit();
      }
      modbus_initData();
    }

    gPersistData.wifi ? gNetworkTask = NETWORK_TASK_MAIN : gNetworkTask = NETWORK_TASK_STOP;

#ifdef DEBUG_WITH_OSCILL
      gpio_pad_select_gpio(KORAD_UART_TX);
      gpio_set_direction(KORAD_UART_TX, GPIO_MODE_OUTPUT);
      gpio_set_level(KORAD_UART_TX, 0);
      gpio_pad_select_gpio(BEEPER_OUTPUT_PIN);
      gpio_set_direction(BEEPER_OUTPUT_PIN, GPIO_MODE_OUTPUT);
      gpio_set_level(BEEPER_OUTPUT_PIN, 0);
#else
    if (gPersistData.useUART)   uartBeginEnd(true);
#endif

  wifiListMutex = xSemaphoreCreateMutex(); // Wifi AP found list - search AP asynch 
  websocketMutex = xSemaphoreCreateMutex(); // websocket data transfer
  dataChangedEventGroup = xEventGroupCreate(); // websocket data changed
  networkChangeTaskEventGroup = xEventGroupCreate(); // request to change network task
  gModbusEepromSaveEventGroup = xEventGroupCreate(); // The events raise when modbus need to save the EEPROM data

  xTaskCreatePinnedToCore(MainDisplayTask, "MainDisplayTask", 4096,  NULL, 1, &gMainDisplayTaskHandler, 0);
  xTaskCreatePinnedToCore(InterruptTask, "InterruptTask", 2048,  NULL, 1, &gInterruptTaskHandler, 1);
  xTaskCreatePinnedToCore(IdleTask, "IdleTask", 2048,  NULL, tskIDLE_PRIORITY, &gIdleTaskHandler, 0);
  xTaskCreatePinnedToCore(NetworkTask, "NetworkTask", 4096,  NULL, 1, &gNetworkTaskHandler, 1);
  ESP_LOGI(logTAG, "Setup done");  
  }
}
//=====================================================================================================================
void loop()
{
  // let arduino task sleep
  vTaskSuspend(NULL); // suspend itself 
}
//===================================================================================================
/*********************************************************************************************************************
 *                                    NETWORK TASKs
**********************************************************************************************************************/
void webServerNotFound(void) {
  ESP_LOGE(logTAG, "Web server not found"); 
}
/*********************************************************************************************************************/
void webSocketSendData(bool aFullData)
{
  StaticJsonDocument<220> jsonDocTx;
  static char output[220];
  static wsData_t oldData;
  wsData_t wsData;
  uint32_t changes;
  const uint32_t VOLT_CHANGED = 0x00010000;
  const uint32_t AMPER_CHANGED = 0x00020000;
  
  xSemaphoreTake(websocketMutex, portMAX_DELAY);
  memcpy(&wsData, &wsTempData, sizeof(wsData));
  xSemaphoreGive(websocketMutex);

  if (aFullData) changes = 0xFFFFFFFF;
  else {
    changes = oldData.leds ^ wsData.leds;
    if (strcmp(oldData.uStr, wsData.uStr) != 0) changes |= VOLT_CHANGED;
    if (strcmp(oldData.iStr, wsData.iStr) != 0) changes |= AMPER_CHANGED;
  }

  if ((webSocket.connectedClients() > 0) && (changes != 0)) {
    jsonDocTx.clear();
    if (changes & BIT_MASK_OVP) jsonDocTx["ovp_led"] = (wsData.leds & BIT_MASK_OVP) != 0;
    if (changes & BIT_MASK_OCP) jsonDocTx["ocp_led"] = (wsData.leds & BIT_MASK_OCP) != 0;
    if (changes & BIT_MASK_CC) jsonDocTx["cc_led"] = (wsData.leds & BIT_MASK_CC) != 0;
    if (changes & BIT_MASK_CV) jsonDocTx["cv_led"] = (wsData.leds & BIT_MASK_CV) != 0;
    if (changes & BIT_MASK_OUT) jsonDocTx["out_led"] = (wsData.leds & BIT_MASK_OUT) != 0;
    if (changes & BIT_MASK_M1) jsonDocTx["m1_led"] = (wsData.leds & BIT_MASK_M1) != 0;
    if (changes & BIT_MASK_M2) jsonDocTx["m2_led"] = (wsData.leds & BIT_MASK_M2) != 0;
    if (changes & BIT_MASK_M3) jsonDocTx["m3_led"] = (wsData.leds & BIT_MASK_M3) != 0;
    if (changes & BIT_MASK_M4) jsonDocTx["m4_led"] = (wsData.leds & BIT_MASK_M4) != 0;
    if (changes & BIT_MASK_M5) jsonDocTx["m5_led"] = (wsData.leds & BIT_MASK_M5) != 0; 
    if (changes & VOLT_CHANGED) jsonDocTx["u_str"] = String(wsData.uStr); //"12.34";
    if (changes & AMPER_CHANGED) jsonDocTx["i_str"] = String(wsData.iStr); //"5.678";

    serializeJson(jsonDocTx, output, 200);
//    ESP_LOGI(logTAG, "Sending: %s", output); 
    webSocket.broadcastTXT(output);
    memcpy(&oldData, &wsData, sizeof(wsData));
  } 
}
/**********************************************************************************************************************/
void webSocketEvent(byte num, WStype_t type, uint8_t * payload, size_t length) { 
  switch (type) { 
    case WStype_PING:
      // pong will be send automatically
      ESP_LOGI(logTAG, "We get ping"); 
      break;
    case WStype_PONG:   
      ESP_LOGI(logTAG, "Answer to a ping we send"); 
      break;  
    case WStype_DISCONNECTED:  
      ESP_LOGI(logTAG, "Client %d disconnected", num); 
      gWifiClientsCounter = webSocket.connectedClients(); 
      break;
    case WStype_CONNECTED: 
      ESP_LOGI(logTAG, "Client %d connected", num); 
      webSocketSendData(true);
      gWifiClientsCounter = webSocket.connectedClients(); 
      break;
    case WStype_TEXT: 
      StaticJsonDocument<32> jsonDocRx; 
      DeserializationError error = deserializeJson(jsonDocRx, payload);
      if (error) {
        ESP_LOGE(logTAG, "deserializeJson() failed!") ;
        return;
      }
      else {
        // JSON string was received correctly, so information can be retrieved:
        if (jsonDocRx["offonclick"]) transmitKoradCommand(gVerifiedData.uSet, gVerifiedData.iSet, !gData.outputOn, false);
        else ESP_LOGE(logTAG, "Received  undefined event"); 
      }
      break;
}
}
/**********************************************************************************************************************/
void WifiListClickEvent_cb(lv_event_t *e) {
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *obj = lv_event_get_target(e);

  if (code == LV_EVENT_CLICKED) {
    doBeep();
    String selectedItem = String(lv_list_get_btn_text(ui_WifiList, obj));
    for (int i = 0; i < selectedItem.length() - 1; i++) {
      if (selectedItem.substring(i, i + 2) == " (") {
        lv_timer_pause(wifiTimer);
        gNetworkTask = NETWORK_TASK_NONE;
        String ssidName = "Enter password for " + selectedItem.substring(0, i);
        memset(tmpSsidName, 0, sizeof(tmpSsidName)); 
        strncpy(tmpSsidName, selectedItem.substring(0, i).c_str(), selectedItem.substring(0, i).length());
        lv_label_set_text(ui_PasswordLabel, ssidName.c_str());
        lv_obj_clear_state(ui_PasswordCloseButton, LV_STATE_DISABLED);
		    lv_obj_clear_state(ui_PasswordTextArea, LV_STATE_DISABLED);
		    lv_obj_add_state(ui_PasswordConnectButton, LV_STATE_DISABLED);
        lv_textarea_set_text(ui_PasswordTextArea, "");
        lv_scr_load_anim(ui_PasswordScreen, LV_SCR_LOAD_ANIM_NONE, 1, 0, false);
        break;
      }
    }
  }
}
/**********************************************************************************************************************/
void updateWifiList() 
{
  static int FoundNetworks = 0;
  if (foundWifiListReady && (foundWifiList.size() > 0) && (FoundNetworks != foundWifiList.size())) {
    xSemaphoreTake( wifiListMutex, portMAX_DELAY );
    lv_obj_clean(ui_WifiList);
    for (std::vector<String>::iterator item = foundWifiList.begin(); item != foundWifiList.end(); ++item) {
      lv_obj_t *btn = lv_list_add_btn(ui_WifiList, LV_SYMBOL_WIFI, (*item).c_str());
      lv_obj_add_event_cb(btn, WifiListClickEvent_cb, LV_EVENT_CLICKED, NULL);
      delay(1);
    }
    FoundNetworks = foundWifiList.size();
    foundWifiListReady = false;
    xSemaphoreGive(wifiListMutex);
    lv_obj_add_flag(ui_WifiSpinner, LV_OBJ_FLAG_HIDDEN);
  }
}
/**********************************************************************************************************************/
void onWiFiEvent(WiFiEvent_t event) 
{
  switch (event) {
    case SYSTEM_EVENT_STA_DISCONNECTED:
      locIPaddress = 0;
      memset(locSsidName, 0, sizeof(locSsidName));
      gNetworkStatus = NETWORK_DISCONNECTED;
      break;

    case SYSTEM_EVENT_STA_CONNECTED:
      memset(locSsidName, 0, sizeof(locSsidName));
      strncpy(locSsidName, WiFi.SSID().c_str(), WiFi.SSID().length());
      gNetworkStatus = NETWORK_WAIT_IP;
      break;

    case SYSTEM_EVENT_STA_GOT_IP:
      locIPaddress = WiFi.localIP();
      gNetworkStatus = NETWORK_CONNECTED;
      break;

    case SYSTEM_EVENT_STA_LOST_IP:
      locIPaddress = 0;
      gNetworkStatus = NETWORK_WAIT_IP;
      break;

    case SYSTEM_EVENT_SCAN_DONE:
          int n = WiFi.scanComplete();
          xSemaphoreTake(wifiListMutex, portMAX_DELAY);
          foundWifiList.clear();
          for (int i = 0; i < n; ++i) {
            String item = WiFi.SSID(i) + " (" + WiFi.RSSI(i) + ") " + ((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " " : "*");
            foundWifiList.push_back(item);
          }
          foundWifiListReady |= foundWifiList.size() > 0;
          xSemaphoreGive(wifiListMutex);
          WiFi.scanDelete();
      break;
  }
}
/**********************************************************************************************************************/
void NetworkTask(void *pvParameter) 
{
  const unsigned long RECONNECT_PERIOD = 5000; // ms
  const unsigned long RESCAN_PERIOD = 6000; // ms
  static unsigned long nextReconnectTime = 2000; //  delay after power up for autoReconnect
  static unsigned long nextScanTime = 0;
  static bool stopped = true;
  unsigned long curTime;

  ESP_LOGI(logTAG, "Network task running on core %d", xPortGetCoreID()); 
  WiFi.mode(WIFI_STA);
	WiFi.setTxPower(WIFI_POWER_11dBm);    // Set WiFi RF power output  ???  Does it work?
  WiFi.hostname("KORAD TCP");
  WiFi.onEvent(onWiFiEvent);
 
  // Start web server and web socket server 
  server.on("/", []() { server.send_P(200, "text/html", index_html_start, index_html_end - index_html_start); });
//  server.on("/favicon.ico", []() { server.send_P(200, "image/vnd.microsoft.icon", favicon_ico_start, favicon_ico_end - favicon_ico_start); });
  server.on("/favicon.ico", []() { server.send_P(200, "iimage/x-icon", favicon_ico_start, favicon_ico_end - favicon_ico_start); });
  server.onNotFound(webServerNotFound);
  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);  
//  webSocket.enableHeartbeat(5000, 5000, 2);

  

  while (1) {
    //----------------------- Events from MainDisplayTask --------------------------
    EventBits_t bits = xEventGroupClearBits(networkChangeTaskEventGroup, NETWORK_CHANGE_TASK_RQST_EVT);
    if (bits & NETWORK_CHANGE_TASK_RQST_EVT) 
    {
      gNetworkTask = newNetworkTask;
      xEventGroupSetBits(gModbusEepromSaveEventGroup, NETWORK_CHANGE_TASK_DONE_EVT); // notify MainDisplayRask loop
    }

    //-------------------------------------------------------------------------------
    curTime = millis();
    switch(gNetworkTask) {
      case NETWORK_TASK_NONE:
        modbus_loop();
        vTaskDelay(10);
        break;

      case NETWORK_TASK_STOP:
        if (WiFi.isConnected()) {
          stopped = true;
          webSocket.disconnect();
          modbus_tcp_server_run(false);
          WiFi.disconnect();
          vTaskDelay(100);
        }
        else gNetworkTask = NETWORK_TASK_NONE;
        break;

      case NETWORK_TASK_MAIN:
        if (WiFi.isConnected()) {
          if (gPersistData.useUART)
          {
            modbus_tcp_server_run(true);
            modbus_loop();
          }
          server.handleClient(); // Needed for the webserver to handle all clients
          webSocket.loop(); // Update function for the webSockets 
          EventBits_t bits = xEventGroupClearBits(dataChangedEventGroup, DATA_CHANGED_EVT);
          if (bits & DATA_CHANGED_EVT) {    
            if (locIPaddress != 0) webSocketSendData(false);
          }
        }
        else if (curTime > nextReconnectTime) {
          if (!WiFi.getAutoReconnect()) {
            WiFi.setAutoReconnect(true);
            nextReconnectTime = curTime + RECONNECT_PERIOD / 2;
          }
          else {
            if (stopped) {
              WiFi.begin();
              ESP_LOGI(logTAG, "wifi begin"); 
              nextReconnectTime = curTime + RECONNECT_PERIOD;
            }
            else {
              modbus_tcp_server_run(false);
              WiFi.disconnect();
              ESP_LOGI(logTAG, "wifi disconnect"); 
              nextReconnectTime = curTime + RECONNECT_PERIOD / 2;
            }
            stopped = !stopped;
          }
          vTaskDelay(100);
          gNetworkStatus = NETWORK_CONNECTING;
        }
        break;

      case NETWORK_TASK_CONNECT:
        WiFi.begin(tmpSsidName, tmpSsidPassword);
        gNetworkStatus = NETWORK_CONNECTING;
        if (WiFi.waitForConnectResult(10000) == WL_CONNECTED) {
          WiFi.setAutoReconnect(true);
          stopped = false;
          ESP_LOGI(logTAG, "Connected to  %s Password %s", tmpSsidName, tmpSsidPassword); 
        }
        else {
          gNetworkStatus = NETWORK_CONNECT_FAILED;
        }
        gNetworkTask = NETWORK_TASK_NONE;
        break;

      case NETWORK_TASK_SCAN:
        if (curTime > nextScanTime) {
          if (!WiFi.isConnected())  WiFi.setAutoReconnect(false); // stop higher priority reconnect task
          nextScanTime = curTime + RESCAN_PERIOD;
          WiFi.scanNetworks(true);
        }
        else vTaskDelay(100);
        break;
    }
  }
}
/*********************************************************************************************************************
 *                                    COMMON TASKs
**********************************************************************************************************************/
void MainDisplayTask(void *pvParameter) 
{
  ESP_LOGI(logTAG, "MainDisplay task running on core %d", xPortGetCoreID()); 
  energyProgTimer = xTimerCreate("Timer1", 1000, pdTRUE, (void*)0, energyProgTimer_cb);
  blinkProgTimer = xTimerCreate("Timer2", 1000, pdFALSE, (void*)0, blinkProgTimer_cb);
  ui_init();
  ui_AfterInit();
  oscInit();
  beepInit();
  lv_timer_create(onHeartBeat, 100, NULL);
  
  static uint16_t gOldRowData[5]; //  for check if the data has been changed
  static bool staticOldOutputOn = false;
  uint16_t changes;
  wsData_t wsData; // data for websocket
  bool verifiedData = false; 
  uint8_t i;
  const uint8_t DATA_TIMEOUT = 5; //  10 means 1 second

  while(1) {

    gDataChanged = false;
      //---------------------  Check timeout ----------------------------------------------
    if (gDataTimeout >= DATA_TIMEOUT) {  //  No data been recieved last 1 second
      if (gDataTimeout == DATA_TIMEOUT) {
        gRowData[0] = 0; // turn off LEDs
        gRowData[1] = 0x0101; // force VALUE_ERR_CODE
        gOldRowData[1] = 0; // force result = true
        gDataTimeout++;
      }
    }
    //---------------------  Transfer Data from Interrupt -------------------------------
    else if (gPersistData.reversData) {
      for (i = 0; i < 5; i++) {  // transfer the data from interrupt with revers and shift
        uint8_t j = (i + gPersistData.shiftData) % 5;
        gRowData[i] = gTempData[4 - j];
      }
    }
    else {
      for (i = 0; i < 5; i++) {  // transfer the data from interrupt with shift
        uint8_t j = (i + gPersistData.shiftData) % 5;
        gRowData[i] = gTempData[j];
      }
    }
    if (gDataChanged) continue; // to avoid the data corruption
    
    //---------------------  Check changes in data ----------------------------------------------
    bool result = false;  //  result = true if data has been changed
    for (i = 0; i < 5; i++) {
      result |= gRowData[i] != gOldRowData[i];
    }

    //---------------------  Check verification 0.3 sec -----------------------------------------
    if (result) gVerificationCounter = 0;
    else  if (gRowData[0] != 0) {
      if (gPersistData.beforeEdge) {
        if (gVerificationCounter == (60 / gPersistData.prescaler)) verifiedData = true;  // verification period 0.3 sec
      }
      else if (gPersistData.prescaler <= 5) {
        if (gVerificationCounter == (15 / gPersistData.prescaler)) verifiedData = true;  // verification period 0.3 sec
      }
      else if (gVerificationCounter == 3) verifiedData = true;  // verification period  more then 0.3 sec
    }
    
    //---------------------  Apply changes ------------------------------------------------------
    // gFullUpdate when screen changes
    if (result || gFullUpdate || verifiedData) {
      if (gFullUpdate) changes = 0xFFFF;
      else changes = gOldRowData[0] ^ gRowData[0];  //  changes in LEDs 
      memcpy(gOldRowData, gRowData, 10);
      
      uint8_t aBit;
      uint32_t uValue = 0, iValue = 0;
      const uint16_t Mul[5] = {0, 1, 10, 100, 1000};
      int8_t uBlinkPos = -1, iBlinkPos = -1;
      bool dp;
      uint8_t uPointPos = 0xFF, iPointPos = 0xFF;
       
      //---------------------  Decode voltage ----------------------------------------------
      for (i = 1; i < 5; i++) {
        aBit =  GetVoltBitValue(gRowData[i] >> 8, &dp); 
        if (aBit > 10) { uValue = VALUE_ERR_CODE; gValueErrorCounter++; break; }
        if (aBit == 10) { aBit = 0; uBlinkPos = 4 - i; } // blinking
        if (dp) { uPointPos = 4 - i; }
        uValue += aBit * Mul[i];
      }

      //---------------------  Decode amperage ----------------------------------------------
      for (i = 1; i < 5; i++) {
        aBit =  GetAmperBitValue(gRowData[i] & 0xFF, &dp); 
        if (aBit > 10) { iValue = VALUE_ERR_CODE; gValueErrorCounter++; break; }
        if (dp) {iPointPos = 4 - i; }
        if (aBit == 10) { aBit = 0;  iBlinkPos = 4 - i; }  // blinking  
        iValue += aBit * Mul[i];
      }

      //---------------------  Data state  ---------------------------------------------------
      bool blinking = (uBlinkPos >= 0) || (iBlinkPos >= 0);
      gData.error = (uValue >= VALUE_ERR_CODE) || (iValue >= VALUE_ERR_CODE);
      //---------------------  Verification --------------------------------------------------
      if (verifiedData) {
        if (!blinking && !gData.error) {
          gVerifiedData.uPointPos = uPointPos;
          gVerifiedData.iPointPos = iPointPos;
          gVerifiedData.ocp = gRowData[0] & BIT_MASK_OCP;
          if (gData.outputOn) { gVerifiedData.uOut = uValue; gVerifiedData.iOut = iValue; } 
          else {
            gVerifiedData.uSet = uValue;  gVerifiedData.iSet = iValue; 
            gVerifiedData.uOut = 0; gVerifiedData.iOut = 0;
            oscSetRange();
            }
        }
        verifiedData = false;
        gVerificationCounter++;
        ESP_LOGI(logTAG, "Verified %d", xTaskGetTickCount()); 
        continue;
      }

      //---------------------  Draw new data --------------------------------------------------
      updateUIndicator(wsData.uStr, uValue, uPointPos, uBlinkPos);
      updateIIndicator(wsData.iStr, iValue, iPointPos, iBlinkPos);

      bool outputOn = (gRowData[0] & BIT_MASK_OUT) != 0;
      if (staticOldOutputOn != outputOn) {
          gData.outputOn = outputOn; 
          staticOldOutputOn = outputOn;
          if (outputOn) {
            xTimerStart(energyProgTimer, 0);
            if (gPersistData.autoReset) ResetEnergyCounters();
          }
          else xTimerStop(energyProgTimer, 0);
      }
      
      if (changes & BIT_MASK_OUT) {
        updateOutLabel(gData.outputOn);
      }

      if (blinking) {
        xTimerStart(blinkProgTimer, 0);
        gDisplayBlinking = true;
      }
      
      if (!gData.error && !blinking) {
        if (iValue == 0) gData.resist = ENERGY_ERR_CODE;
        else gData.resist = (uValue * 100) / iValue;
        gData.power = uValue * iValue / 10000;
      } 
      else gData.power = gData.resist = ENERGY_ERR_CODE;
      updatePIndicator(gFullUpdate);
      updateRIndicator(gFullUpdate);
      
      if (changes & (BIT_MASK_CV | BIT_MASK_CC)) {
        uint16_t cVcc = gRowData[0] & (BIT_MASK_CV | BIT_MASK_CC);
        updateStateLabel(cvLabel, (gRowData[0] & BIT_MASK_CV));
        updateStateLabel(ccLabel, (gRowData[0] & BIT_MASK_CC));
        if (cVcc == 0) recolorUIIndicator(gPersistData.offColor);
        else if (cVcc == BIT_MASK_CV) recolorUIIndicator(gPersistData.cvColor);
        else recolorUIIndicator(gPersistData.ccColor);
      } 

      if (changes & BIT_MASK_OVP) updateStateLabel(ovpLabel, gRowData[0] & BIT_MASK_OVP);
      if (changes & BIT_MASK_OCP) updateStateLabel(ocpLabel, gRowData[0] & BIT_MASK_OCP);
      
      if (changes & (BIT_MASK_M1 | BIT_MASK_M2 | BIT_MASK_M3 | BIT_MASK_M4 | BIT_MASK_M5)) {
        uint8_t memIdx = 0;
        if (gRowData[0] & BIT_MASK_M1) memIdx = 1;
        else if (gRowData[0] & BIT_MASK_M2) memIdx = 2;
        else if (gRowData[0] & BIT_MASK_M3) memIdx = 3;
        else if (gRowData[0] & BIT_MASK_M4) memIdx = 4;
        else if (gRowData[0] & BIT_MASK_M5) memIdx = 5;
        lv_label_set_text_fmt(mLabel, "M%d", memIdx);
      }

      //---------------------  Data for oscill -------------------------------------------------
      if (gData.outputOn) {
        if ((uValue != gVerifiedData.uSet) || (gRowData[0] & BIT_MASK_CV)) gData.uOut = uValue; 
        if ((iValue != gVerifiedData.iSet) || (gRowData[0] & BIT_MASK_CC)) gData.iOut = iValue;
        }
      else {gData.uOut = gData.iOut = 0;}
      //---------------------  Send data to web socket ------------------------------------------
      wsData.leds = gRowData[0];
      xSemaphoreTake(websocketMutex, portMAX_DELAY);
      memcpy(&wsTempData, &wsData, sizeof(wsData));
      xSemaphoreGive(websocketMutex);
      xEventGroupSetBits(dataChangedEventGroup, DATA_CHANGED_EVT); // notify web socket
      //---------------------  Finish -----------------------------------------------------------
      ESP_LOGI(logTAG, "Data changed %d", xTaskGetTickCount()); 
      gFullUpdate = false;
      gScreensaverTimeout = 0;
    }
    
    lv_timer_handler(); /* let the GUI do its work */
    beepLoop();

    //----------------------- Events from modbus (network task)  --------------------------
    EventBits_t bits = xEventGroupClearBits(gModbusEepromSaveEventGroup, EEPROM_NEED_SAVE_GLOBAL_EVENT | EEPROM_NEED_SAVE_PROFILE_EVENT);
    if (bits & EEPROM_NEED_SAVE_GLOBAL_EVENT) 
      modbus_updateGlobalData();
    if (bits & EEPROM_NEED_SAVE_PROFILE_EVENT)
      modbus_updateCurrentProfile();
    if (bits)
      xEventGroupSetBits(gModbusEepromSaveEventGroup, EEPROM_NEED_SAVE_DONE_EVENT); // notify modbus loop

    vTaskDelay(5);
  }
}
//===================================================================================================
//===================================================================================================
void IdleTask(void *pvParameter)
{
  static uint16_t scrollIdx = 0;
  ESP_LOGI(logTAG, "Idle task running on core %d", xPortGetCoreID()); 
  while(1) {

    lv_obj_t * actScreen = lv_scr_act();
    static uint32_t SettingsScreenIP = 0xFFFFFFFF;
    lv_color_t color;
    locIPaddress == 0 ? color = CLR_DARKGRAY : color = CLR_WHITE; 
    
    if (logoLabel != NULL) {
      uint16_t len = gLogoLength;
      char text[len--];
      uint16_t partLen = len - scrollIdx;
      memcpy(text, gLogoText + scrollIdx, partLen);
      memcpy(text + partLen, gLogoText, scrollIdx);
      text[len] = 0;
      scrollIdx++;
      scrollIdx %= len;  // if (scrollIdx == len) scrollIdx = 0;
      lv_label_set_text(logoLabel, text);
    }
    
    if (wifiLabel != NULL) {
      if (lv_obj_get_style_text_color(wifiLabel, LV_PART_MAIN).full != color.full)
        lv_obj_set_style_text_color(wifiLabel, color, LV_PART_MAIN); 
    }
    
    if (actScreen == ui_SettingsScreen) {
      if (locIPaddress == 0) {
        if (gNetworkStatus == NETWORK_CONNECTING) lv_label_set_text(ui_WifiIpLabel, "Connecting ...");
        else if (gNetworkStatus == NETWORK_WAIT_IP) lv_label_set_text(ui_WifiIpLabel, "Waiting for IP");
        else lv_label_set_text(ui_WifiIpLabel, "Not connected");
      } else if (SettingsScreenIP != locIPaddress) {
		    uint8_t * ip  = (uint8_t *) &locIPaddress;
		    lv_label_set_text_fmt(ui_WifiIpLabel, "IP %3d.%3d.%3d.%3d", ip[0], ip[1], ip[2], ip[3]);
      }
      SettingsScreenIP = locIPaddress;
    }

    else if (actScreen == ui_DebugScreen)  updateDebugData();
    vTaskDelay(200);

    /*   //*****  for debug stack free space
      static uint32_t counter = 0;
      counter++;
      if (counter == 100) {
        counter = 0;
        ESP_LOGD(logTAG, "MainDisplay task stack free space %d", uxTaskGetStackHighWaterMark(gMainDisplayTaskHandler));
        ESP_LOGD(logTAG, "Network task stack free space %d", uxTaskGetStackHighWaterMark(gNetworkTaskHandler));
        ESP_LOGD(logTAG, "Interrupt stack free space %d", uxTaskGetStackHighWaterMark(gInterruptTaskHandler));
        ESP_LOGD(logTAG, "Idle task stack free space %d", uxTaskGetStackHighWaterMark(gIdleTaskHandler));
      } */ 
  }
}
//===================================================================================================
//===================================================================================================
void updateDebugData(void)
{
  lv_label_set_text_fmt(ui_InterErrLabel, "Timeouts %d", gInterruptTimeoutErrorCounter);	
  lv_label_set_text_fmt(ui_ValueErrLabel, "Errors %d", gValueErrorCounter);	
  lv_label_set_text_fmt(ui_ClientsLabel, "Wifi Clients %d", gWifiClientsCounter);	
  if (lv_scr_act() == ui_DebugScreen) {
    char  str1[40];
    sprintf(str1, "%.2X %.2X %.2X %.2X",
      gRowData[4] >> 8,
      gRowData[3] >> 8,
      gRowData[2] >> 8,
      gRowData[1] >> 8);
    lv_label_set_text(ui_uuDebugLabel, str1);

    sprintf(str1, "%.2X %.2X %.2X %.2X %.4X",
      gRowData[4] & 0xFF,
      gRowData[3] & 0xFF,
      gRowData[2] & 0xFF,
      gRowData[1] & 0xFF,
      gRowData[0] );
    lv_label_set_text(ui_iiDebugLabel, str1);
  }
}
//===================================================================================================
void energyProgTimer_cb( TimerHandle_t xTimer )
{
  gTimeCount++;
  if (!gDisplayBlinking) {
    gVerifiedData.ahCount64 += ((uint64_t)gVerifiedData.iOut << 32) / 3600;
    gVerifiedData.whCount64 += (((uint64_t)gVerifiedData.iOut * (uint64_t)gVerifiedData.uOut) << 32) / 3600000;
 }
  UpdateEnergyLabels();
}
//===================================================================================================
void blinkProgTimer_cb( TimerHandle_t xTimer )
{
  gDisplayBlinking = false;
  updatePIndicator(true);
  updateRIndicator(true);
}
//===================================================================================================
void onHeartBeat(lv_timer_t *timer)
{
	static uint8_t oldBrightness = 0;
	static bool oldOutputOn = false;
  uint32_t ssTimeout;
  
  if (gPersistData.ScreensaverIdx == 0) ssTimeout = 0xFFFFFFFF;
  else ssTimeout  = gScreensaverArray[gPersistData.ScreensaverIdx] * 600;
	
	oscLoop();

	gDataTimeout++;
	if (!gData.outputOn && (gScreensaverArray[gPersistData.ScreensaverIdx] > 0))
		gScreensaverTimeout++;
	else gScreensaverTimeout = 0;

	lv_obj_t * scr = lv_scr_act();
	if ((scr != ui_Screen1) && (scr != ui_Screen2) && (scr != ui_Screen3) && (scr != ui_DebugScreen)) {
		gPopupTimeout++;
		if (oldOutputOn != gData.outputOn) {
			oldOutputOn = gData.outputOn;
			if (oldOutputOn) LoadPreviousScreen();  //  hide popup screens when user turns output on
		}
	}
	else {
    gPopupTimeout = 0;
    oldOutputOn = gData.outputOn;
  }
	if (gPopupTimeout > 300) // timeout 30 sec
		LoadPreviousScreen();
	else 
	if (gScreensaverTimeout >= ssTimeout) {
		if (oldBrightness - BRIGHTNESS_STEP > 20) setBrightness(oldBrightness -= BRIGHTNESS_STEP);
	}
	else {
		if(oldBrightness > gPersistData.brightness)  setBrightness(oldBrightness = gPersistData.brightness);
    else if (oldBrightness + BRIGHTNESS_STEP <= gPersistData.brightness) setBrightness(oldBrightness += BRIGHTNESS_STEP);
  }
}
//===================================================================================================
void changeNetworkTask(Network_Task_t newTask)
{
	newNetworkTask = newTask;
  xEventGroupSetBits(networkChangeTaskEventGroup, NETWORK_CHANGE_TASK_RQST_EVT); // notify main.NetworkTask()
	xEventGroupWaitBits(networkChangeTaskEventGroup, NETWORK_CHANGE_TASK_DONE_EVT, pdTRUE, pdFALSE,  100 / portTICK_PERIOD_MS);  // Wait a maximum of 100ms for either bit to be set.
}
//===================================================================================================
