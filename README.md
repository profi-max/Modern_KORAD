# Modern KORAD
### Update your KORAD using the LCD display and the touch panel.
https://youtu.be/W0K4ZL6MwYY

 [![Watch the video](https://img.youtube.com/vi/W0K4ZL6MwYY/hqdefault.jpg)](https://www.youtube.com/embed/W0K4ZL6MwYY)
 
# How to make modern KORAD
You actually need to replace the display board with a new LCD. The project is based on the low cost ESP32 development board. The board WT32-SC01 with 3.5 inch 320x480 capacitive multi-touch LCD Screen and built-in Wifi. 
Also you need an external wifi antenna and a DC-DC converter (12v -> 5v, 1A).
![WT32-SC01](/Pictures/wt32sc01.jpg)
![WIFI Antenna](/Pictures/wifi_antenna.jpg)
![DC-DC converter](/Pictures/dc_dc.jpg)
![IPX](/Pictures/IPX.jpg)
![IDC](/Pictures/IDC.jpg)

There are two options for the board:
1) WT32-SC01 - recommended
2) WT32-SC01 PLUS - is not recommended due to its size.
<details>
<summary>Click to view where to buy</summary>
Aliexpress pages: 
+ https://aliexpress.ru/item/1005003745843708.html
+ https://aliexpress.ru/item/1005004267336768.html
+ https://aliexpress.ru/item/1005003297175908.html
+ https://aliexpress.ru/item/1005003880321464.html
</details>

## How to flash the board (2 ways)
1. Use finished firmware files from the folder **Binary**. Upload Flash Download Tools  from official page: https://www.espressif.com/en/support/download/other-tools
Connect USB cable to your development board, run the application, select COM port and bin-file. When download is completed push "RST" (Reset) button to restart your dev board.
2. Use Visual Studio Code + Platformio. Open project from **VSCode** folder. Build and upload the project.
<details>
<summary>Click to view Flash Download Tools</summary>
<image src="/Pictures/WT32SC01_Flash_start.jpg" alt="Flasher">
</details>

## How it works
The project is suitable for all models KA3005D and KA3005P. Models without programmable control (KA3005D) will work, but you will not be able to control the current values and voltage.

Although KORAD3005D is not intended for programmable control, nevertheless, some of the devices can work like KORAD3005P. Under the cover on the printed circuit board there is a connector for communication via UART. Let's call such devices that can be controlled via UART KORAD3005DP.
To see the difference between KORAD3005D and KORAD3005DP, just listen to it. Watch this short video and you will hear the difference.

https://youtu.be/TDWt2rKNsIM

## Schematics
All schematics can be found in the **Schematics** folder.
+ **J4** (korad main board) connecting display board
+ **J9** (korad main board) connecting interface board 
![KORAD main board](/Pictures/korad_inner.jpg)
![My main board](/Pictures/my_board.JPG)
![Connection](/Schematics/Schematic_WT32_SC01.jpg)

## Programming
You can change the user inteface using [SquareLine Studio](https://squareline.io/).
SquareLine Studio is a wonderful product. But if you do not plan to change the user interface, you can do without SquareLine Studio. All you need is Visual Studio Code with Platformio.

The UI project is in the **SquareLine** folder. Change File->Project Settings, FILE EXPORT, Project Export Root and UI File Export Path to your correct folders.
<details>
<summary>Click to view SquareLine Studio Project Settings</summary>
<image src="/Pictures/SLStudio.jpg" alt="Project Settings">
</details>

## User manual
Yellow is highlighted the clickable areas. On the first screen you can change between (W) Watts and (R) Resistance.
![Screen 1](/Pictures/Screen1.jpg) ![Screen 2](/Pictures/Screen2.jpg)
![Screen 3](/Pictures/Screen3.jpg)

On the third screen you can change the oscilloscope mode: 
-> Single voltage beam -> Single current beam -> Both beams. Voltage ahead -> Both beams. Current ahead ->

### Settings screen
![Settings Screen](/Pictures/SettingsScreen.jpg)
+ Wi-Fi. ON or OFF. If ON you can enter wifi settings.
+ Energy auto reset. If ON it resets the energy and time counters every time when the output is on.
+ Programmable KA3005P/DP.  If ON Lets programmable interface.
+ Block when output is ON.  If OFF Lets programmable interface when the output is on.
+ Exterior. Enters the exterior screen.
+ Screensaver. If ON dim the backlight in some minutes after the last usage.
+ Beeper. The buzzer volume.

### Exterior Screen
![Exterior Screen](/Pictures/ExteriorScreen.jpg)

At the left pane you can change the colors for:  CC mode, CV mode and so on.
At the right pane you can change the fonts: Segment, Electro, Roboro.
Default button restores default exterior.

### Debug Screen
![Debug Screen 1](/Pictures/DebugScreenD.jpg)
![Debug Screen 1](/Pictures/DebugScreenP.jpg)


## DPSmaster - Windows PC application for managing KORAD3005
1. WiFi: In the DPSmaster application select "TCP" and enter correct KORAD's local IP address.
2. USB:  In the DPSmaster application select correct COM-port. Baud rate 115200, slave address 1.

DPSmaster officail page: https://profimaxblog.ru/dpsmaster/

DPSmaster full review: https://youtu.be/1nZQ2FA08Fg

![DPSmaster](/Pictures/DPSmaster.jpg)

## How to use WT32-SC01 PLUS (not recommended)
You need an additional UART-USB converter for debug and modbus communication with DPSmaster.
<details>
<summary>Click to view schematics</summary>
<image src="/Schematics/Schematic_WT32_SC01_PLUS.jpg" alt="WT32_SC01_PLUS">
</details>

## Fix compilation problem
![Error](/Pictures/SLS_error.jpg)

Add #include <math.h> in the automaticaly generated ui_DebugScreen.c file.
![Error fix](/Pictures/SLS_error_fix.jpg)
