/*
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
 */

#include "_Plugin_Helper.h"
#include "src/Helpers/_CPlugin_Helper.h"
//#include <ESPeasySerial.h>

// uncomment in case of use as official plugin
//#ifdef USES_P132

// chose one of the stati
//build all the normal stable plugins 
//#define PLUGIN_BUILD_NORMAL
//build all plugins that are in test stadium
//#define PLUGIN_BUILD_TESTING
//build all plugins that still are being developed and are broken or incomplete
//#define PLUGIN_BUILD_DEV

// plugin definitions
#define PLUGIN_132
#define PLUGIN_ID_132         132
#define PLUGIN_NAME_132       "Software Serial Server"
#define PLUGIN_VALUENAME1_132 "SwSerSrv"
 
//#define BUFFER_SIZE_SW 512
boolean Plugin_132_init = false;
 
WiFiServer *ser2netServerSW;
WiFiClient ser2netClientSW;

// old starts from immi/amunra/chons, will be replaced after chosing GPIO pins
ESPeasySerial *Plugin_132_SS; 

boolean Plugin_132(byte function, struct EventStruct *event, String& string)
{
  boolean success = false;

  switch (function) 
  {
    case PLUGIN_DEVICE_ADD:
    {
      Device[++deviceCount].Number = PLUGIN_ID_132;
      //Device[deviceCount].Type = DEVICE_TYPE_SINGLE; // single has only one input pin
      Device[deviceCount].Type = DEVICE_TYPE_DUAL;    // two input pins
      Device[deviceCount].VType = Sensor_VType::SENSOR_TYPE_SINGLE; // one output
      //Device[deviceCount].Ports = 0; // No Ports available in Structure
      Device[deviceCount].PullUpOption       = false; // added
      Device[deviceCount].InverseLogicOption = false; // added
      Device[deviceCount].Custom = true;
      Device[deviceCount].TimerOption = false;
      break;
    }

    case PLUGIN_GET_DEVICENAME:
    {
      string = F(PLUGIN_NAME_132);
      break;
    }

    case PLUGIN_GET_DEVICEVALUENAMES:
    {
      strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[0], PSTR(PLUGIN_VALUENAME1_132));
      break;
    }

    case PLUGIN_SET_DEFAULTS: {
      ExtraTaskSettings.TaskDevicePluginConfigLong[0] = 24;     // Port 
      ExtraTaskSettings.TaskDevicePluginConfigLong[1] = 9600;   // Baud 
      ExtraTaskSettings.TaskDevicePluginConfigLong[2] = 512;    // SerialBuffer
      ExtraTaskSettings.TaskDevicePluginConfigLong[3] = 512;    // NetBuffer

      success = true;
      break;
    }

    case PLUGIN_WEBFORM_LOAD:
    {
      addFormNumericBox(F("TCP Port"),  F("p132_port"), ExtraTaskSettings.TaskDevicePluginConfigLong[0], 1, 1000);
      addFormNumericBox(F("Baud Rate"), F("p132_baud"), ExtraTaskSettings.TaskDevicePluginConfigLong[1], 1, 56800);
      addFormNumericBox(F("Serial Buffer Size"),  F("p132_sbuf"), ExtraTaskSettings.TaskDevicePluginConfigLong[2], 1, 1024);
      addFormNumericBox(F("Net Buffer Size"),  F("p132_nbuf"), ExtraTaskSettings.TaskDevicePluginConfigLong[3], 1, 1024);
      addFormPinSelect(F("Select ESP8266 Rx pin"), F("taskdevicepin1"), Settings.TaskDevicePin1[event->TaskIndex]);
      addFormPinSelect(F("Select ESP8266 Tx pin"), F("taskdevicepin2"), Settings.TaskDevicePin2[event->TaskIndex]);
      
      //char tmpString[128];
      //sprintf_P(tmpString, PSTR("<TR><TD>TCP Port:<TD><input type='text' name='plugin_132_port' value='%u'>"), ExtraTaskSettings.TaskDevicePluginConfigLong[0]);
      //addHtml(tmpString);
      //sprintf_P(tmpString, PSTR("<TR><TD>Baud Rate (max. 57600):<TD><input type='text' name='plugin_132_baud' value='%u'>"), ExtraTaskSettings.TaskDevicePluginConfigLong[1]);
      //addHtml(tmpString);   

      //addHtml(F("<TR><TD>Select ESP8266 Rx pin:<TD>"));
      //addPinSelect(false, "taskdevicepin1", Settings.TaskDevicePin1[event->TaskIndex]);
      //addHtml(F("<TR><TD>Select ESP8266 Tx pin:<TD>"));
      //addPinSelect(false, "taskdevicepin2", Settings.TaskDevicePin2[event->TaskIndex]);

      success = true;
      break;
    }

    case PLUGIN_WEBFORM_SAVE:
    {
      ExtraTaskSettings.TaskDevicePluginConfigLong[0]     = getFormItemInt(F("p132_port"));
      ExtraTaskSettings.TaskDevicePluginConfigLong[1]     = getFormItemInt(F("p132_baud"));
      ExtraTaskSettings.TaskDevicePluginConfigLong[2]     = getFormItemInt(F("p132_sbuf"));
      ExtraTaskSettings.TaskDevicePluginConfigLong[3]     = getFormItemInt(F("p132_nbuf"));
      // save values (port, baud rate, TX GPIO, RX GPIO)
      //String plugin1 = web_server.arg(F("plugin_132_port"));
      //ExtraTaskSettings.TaskDevicePluginConfigLong[0] = plugin1.toInt();
      //String plugin2 = web_server.arg(F("plugin_132_baud"));
      //ExtraTaskSettings.TaskDevicePluginConfigLong[1] = plugin2.toInt();
      //String plugin3 = web_server.arg(F("taskdevicepin1"));
      //Settings.TaskDevicePluginConfig[event->TaskIndex][0] = plugin3.toInt(); // from P142
      //String plugin4 = web_server.arg(F("taskdevicepin2"));
      //Settings.TaskDevicePluginConfig[event->TaskIndex][1] = plugin4.toInt();

      //SaveTaskSettings(event->TaskIndex);

      success = true;
      break;
    }

    case PLUGIN_INIT:
    {
      LoadTaskSettings(event->TaskIndex);
      
      if ((ExtraTaskSettings.TaskDevicePluginConfigLong[0] != 0) && (ExtraTaskSettings.TaskDevicePluginConfigLong[1] != 0) && (ExtraTaskSettings.TaskDevicePluginConfigLong[2] != 0) && (ExtraTaskSettings.TaskDevicePluginConfigLong[3] != 0) && (Settings.TaskDevicePin1[event->TaskIndex] != -1) && (Settings.TaskDevicePin2[event->TaskIndex] != -1))
      {
        // Use ESPeasySerial Software Option, port = 6 SW, Pin1, Pin2, inverse = false, buffer = 1024, softwareserail = true
        Plugin_132_SS = new ESPeasySerial(ESPEasySerialPort::software, Settings.TaskDevicePin1[event->TaskIndex], Settings.TaskDevicePin2[event->TaskIndex], false, ExtraTaskSettings.TaskDevicePluginConfigLong[2], true);
        Plugin_132_SS->begin(ExtraTaskSettings.TaskDevicePluginConfigLong[1]);
        
        // Wifi Server
        ser2netServerSW = new WiFiServer(ExtraTaskSettings.TaskDevicePluginConfigLong[0]);           
        ser2netServerSW->begin();

        String log =String((ExtraTaskSettings.TaskDevicePluginConfigLong[1]),DEC);
        addLog(LOG_LEVEL_DEBUG,"Init done");
        addLog(LOG_LEVEL_DEBUG,log);
        Plugin_132_init = true;
      }
      success = true;
      break;
    }

    // repeat ten times per second (main loop)
    case PLUGIN_TEN_PER_SECOND:
    {      
      if (Plugin_132_init)
      {        
        size_t bytes_read;

        if (!ser2netClientSW)
        {          
          while (Plugin_132_SS->available()) // ESPEasySerial
          {
            Plugin_132_SS->read();
            addLog(LOG_LEVEL_DEBUG_DEV,"SoftSer: ESPserial available");
          }
          ser2netClientSW = ser2netServerSW->available();     
          addLog(LOG_LEVEL_DEBUG_DEV,"SoftSer: Net available");
        }

       
        if (ser2netClientSW.connected())
        {     
           // Net IN     
          uint8_t net_buf[ExtraTaskSettings.TaskDevicePluginConfigLong[3]];
          int count = ser2netClientSW.available();

          if (count > 0) {
            if (count > ExtraTaskSettings.TaskDevicePluginConfigLong[3])
              count = ExtraTaskSettings.TaskDevicePluginConfigLong[3];
            bytes_read = ser2netClientSW.read(net_buf, count);            
            Plugin_132_SS->write(net_buf, bytes_read);
            Plugin_132_SS->flush();
            if (count == ExtraTaskSettings.TaskDevicePluginConfigLong[3])                                  // if we have a full buffer, drop the last position to stuff
                                                                          // with string end marker
            {
              count--;
              addLog(LOG_LEVEL_ERROR, F("SoftSer: Network buffer full!"));
            }
            net_buf[count]=0;            
            addLog(LOG_LEVEL_DEBUG,(char*)net_buf);
          }

          // Serial IN
          uint8_t serial_buf[ExtraTaskSettings.TaskDevicePluginConfigLong[3]];
          size_t bytes_read = 0;
          PrepareSend();
          while (Plugin_132_SS->available() && bytes_read < ExtraTaskSettings.TaskDevicePluginConfigLong[3]) {
            serial_buf[bytes_read] = Plugin_132_SS->read();
            bytes_read++;            
          }
          delay(0);

          if (bytes_read != ExtraTaskSettings.TaskDevicePluginConfigLong[3])
          {
            if (bytes_read > 0) 
            {
              ser2netClientSW.write((const uint8_t*)serial_buf, bytes_read);
              ser2netClientSW.flush();            
            }
          }
          else // if we have a full buffer, drop the last position to stuff with string end marker
          {
            while (Plugin_132_SS->available()) { // read possible remaining data to avoid sending rubbish...
              Plugin_132_SS->read();
            }
            bytes_read--;
            addLog(LOG_LEVEL_ERROR, F("SoftSer: Serial buffer full!"));
          }

          serial_buf[bytes_read]=0;          
          addLog(LOG_LEVEL_DEBUG,(char*)serial_buf);
        }
        else 
        {
          ser2netClientSW.stop();
          Plugin_132_SS->flush();
        }
        success = true;
      }
      break;
    }
  }
  return success;
}

//#endif // USES_P132
//#endif // PLUGIN_BUILD_NORMAL or PLUGIN_BUILD_TESTING or PLUGIN_BUILD_DEV