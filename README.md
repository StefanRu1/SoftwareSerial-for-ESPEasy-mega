# SoftwareSerial-for-ESPEasy-mega
SoftwareSerial Plugin for ESPEasy mega

 * This plug in is based on a plugin written by immi/amunra/chons
 * Parts of the plugin are derived from _P149_MHZ19.ino
 * numbering from https://github.com/letscontrolit/ESPEasyPluginPlayground (132 is not used,
 * therefore changed from 130 to 132)
 *
 * The Code was changed on 12.12.2020 to use ESPSerial to reduce footprint in flash
 * Logging was introduced with DEBUG and DEBUG_DEV level
 * With Debug you see initialization and data flowing
 * With Debug_Dev you see also connection from telnet or no connection
 * 
 * This plugin enables a software serial port on ESP8266 to a WiFi network 
 * (similar to _P020_Ser2Net, but uses different pins and therefore softwareserial).
 *
 * devicepin1 - is RX for ESP8266
 * devicepin2 - is TX for ESP8266
 *
 * version: 0.2 04/04/2018 PeMue
 * version: 0.5 12/12/2020 Stefanru -> FHEM Forum
 
 See: https://forum.fhem.de/index.php/topic,86592.msg1109877.html#msg1109877
 
 Build with VCode and Platformio on Version Release mega-20201130 without issues.
 Tested with FotoLED to get digital current meter output.
 
 Two small adaptions in _P020_Ser2Net.ino where necessary.
 Line 254: delay(1) -> delay(0)
 Line 14: #define P020_BUFFER_SIZE 128 -> #define P020_BUFFER_SIZE 256

