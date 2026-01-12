/*
  HeatPump.cpp - Mitsubishi Heat Pump control library for Arduino
  Copyright (c) 2017 Al Betschart.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include "HeatPump.h"

#include "Globals.h"

static const char* TAG = "HeatPump"; // Logging tag

// Constructor /////////////////////////////////////////////////////////////////

HeatPump::HeatPump(devicestate::IIODevice* io_device, RequestScheduler scheduler) :
    io_device_{io_device},
    scheduler{scheduler},
    hpProtocol{} {

  lastWanted = CUSTOM_MILLIS;
  lastSend = 0;
  infoMode = 0;
  lastRecv = CUSTOM_MILLIS - (PACKET_SENT_INTERVAL_MS * 10);
  autoUpdate = false;
  firstRun = true;
  tempMode = false;
  waitForRead = false;
  externalUpdate = false;
  wideVaneAdj = false;
  functions = heatpumpFunctions();
}

// Public Methods //////////////////////////////////////////////////////////////
bool HeatPump::connect() {
  io_device_->begin();

  if(onConnectCallback) {
    onConnectCallback();
  }

  // send the CONNECT packet twice - need to copy the CONNECT packet locally
  byte packet[CONNECT_LEN];
  memcpy(packet, CONNECT, CONNECT_LEN);
  //for(int count = 0; count < 2; count++) {
  writePacket(packet, CONNECT_LEN);
  while(!canRead()) { CUSTOM_DELAY(10); }
  int packetType = readPacket();
  connected = (packetType == RCVD_PKT_CONNECT_SUCCESS);
  return connected;
  //}
}

uint8_t HeatPump::translatePacket(int packetType) {
    // Legacy path kept temporarily if some callsites still pass packetType indices.
    // Map legacy indices to real codes and delegate to buildAndSendInfoPacket.
    uint8_t code = 0x02; // default to settings
    switch (packetType) {
    case 0: code = 0x02; break; // RQST_PKT_SETTINGS
    case 1: code = 0x03; break; // RQST_PKT_ROOM_TEMP
    case 2: code = 0x06; break; // RQST_PKT_STATUS
    case 3: code = 0x04; break; // RQST_PKT_UNKNOWN
    case 4: code = 0x05; break; // RQST_PKT_TIMERS
    case 5: code = 0x09; break; // RQST_PKT_STANDBY
    default: code = 0x02; break;
    }
    return code;
}

/*
void HeatPump::buildAndSendRequestPacket(int packetType) {
    // Legacy path kept temporarily if some callsites still pass packetType indices.
    // Map legacy indices to real codes and delegate to buildAndSendInfoPacket.
    uint8_t code = 0x02; // default to settings
    switch (packetType) {
    case 0: code = 0x02; break; // RQST_PKT_SETTINGS
    case 1: code = 0x03; break; // RQST_PKT_ROOM_TEMP
    case 2: code = 0x04; break; // RQST_PKT_UNKNOWN
    case 3: code = 0x05; break; // RQST_PKT_TIMERS
    case 4: code = 0x06; break; // RQST_PKT_STATUS
    case 5: code = 0x09; break; // RQST_PKT_STANDBY
    case 6: code = 0x42; break; // RQST_PKT_HVAC_OPTIONS
    default: code = 0x02; break;
    }
    this->buildAndSendInfoPacket(code);
}
*/

void HeatPump::buildAndSendInfoPacket(uint8_t code) {
    uint8_t packet[PACKET_LEN] = {};
    hpProtocol.createInfoPacket(packet, code);
    this->writePacket(packet, PACKET_LEN);
}

bool HeatPump::update() {
  while(!canSend(false)) { CUSTOM_DELAY(10); }

  // Flush the serial buffer before updating settings to clear out
  // any remaining responses that would prevent us from receiving
  // RCVD_PKT_UPDATE_SUCCESS
  readAllPackets();

  byte packet[PACKET_LEN] = {};
  createPacket(packet, wantedSettings);
  writePacket(packet, PACKET_LEN);

  while(!canRead()) { CUSTOM_DELAY(10); }
  int packetType = readPacket();

  if(packetType == RCVD_PKT_UPDATE_SUCCESS) {
    // call sync() to get the latest settings from the heatpump for autoUpdate, which should now have the updated settings
    if(autoUpdate) { //this sync will happen regardless, but autoUpdate needs it sooner than later.
	    while(!canSend(true)) {
		    CUSTOM_DELAY(10);
	    }
	    sync(RQST_PKT_SETTINGS);
    } else {
      // No auto update, but the next time we sync, fetch the updated settings first
      infoMode = 0;
    }

    return true;
  } else {
    return false;
  }
}

void HeatPump::sync(byte packetType) {
  if((!connected) || (CUSTOM_MILLIS - lastRecv > (PACKET_SENT_INTERVAL_MS * 10))) {
    connect();
  }
  else if(canRead()) {
    readAllPackets();
  }
  else if(autoUpdate && !firstRun && wantedSettings != currentSettings && packetType == PACKET_TYPE_DEFAULT) {
    update();
  }
  else if(canSend(true)) {
    if (packetType == PACKET_TYPE_DEFAULT) {
      const uint8_t code = this->translatePacket(infoMode);
      this->buildAndSendInfoPacket(code);
      if (infoMode == (fastSync ? 2 : 5)) {
        infoMode = 0;
      } else {
        infoMode++;
      }
    } else {
      const uint8_t code = this->translatePacket(packetType);
      this->buildAndSendInfoPacket(code);
    }
  }
}

void HeatPump::enableExternalUpdate() {
  autoUpdate = true;
  externalUpdate = true;
}

void HeatPump::disableExternalUpdate() {
  externalUpdate = false;
}

void HeatPump::enableAutoUpdate() {
  autoUpdate = true;
}

void HeatPump::disableAutoUpdate() {
  autoUpdate = false;
}

heatpumpSettings HeatPump::getSettings() {
  return currentSettings;
}

unsigned long HeatPump::getLastWanted() {
  return lastWanted;
}

void HeatPump::setFastSync(bool setting) {
  fastSync = setting;
}

bool HeatPump::isConnected() {
  return connected;
}

void HeatPump::setSettings(heatpumpSettings settings) {
  setPowerSetting(settings.power);
  setModeSetting(settings.mode);
  setTemperature(settings.temperature);
  setFanSpeed(settings.fan);
  setVaneSetting(settings.vane);
  setWideVaneSetting(settings.wideVane);
}

bool HeatPump::getPowerSettingBool() {
  return currentSettings.power == POWER_MAP[1] ? true : false;
}

void HeatPump::setPowerSetting(bool setting) {
  wantedSettings.power = lookupByteMapIndex(POWER_MAP, 2, POWER_MAP[setting ? 1 : 0]) > -1 ? POWER_MAP[setting ? 1 : 0] : POWER_MAP[0];
  lastWanted = CUSTOM_MILLIS;
}

const char* HeatPump::getPowerSetting() {
  return currentSettings.power;
}

void HeatPump::setPowerSetting(const char* setting) {
  int index = lookupByteMapIndex(POWER_MAP, 2, setting);
  if (index > -1) {
    wantedSettings.power = POWER_MAP[index];
  } else {
    wantedSettings.power = POWER_MAP[0];
  }
  lastWanted = CUSTOM_MILLIS;
}

const char* HeatPump::getModeSetting() {
  return currentSettings.mode;
}

void HeatPump::setModeSetting(const char* setting) {
  int index = lookupByteMapIndex(MODE_MAP, 5, setting);
  if (index > -1) {
    wantedSettings.mode = MODE_MAP[index];
  } else {
    wantedSettings.mode = MODE_MAP[0];
  }
  lastWanted = CUSTOM_MILLIS;
}

float HeatPump::getTemperature() {
  return currentSettings.temperature;
}

void HeatPump::setTemperature(float setting) {
  if(!tempMode){
    wantedSettings.temperature = lookupByteMapIndex(TEMP_MAP, 16, (int)(setting + 0.5)) > -1 ? setting : TEMP_MAP[0];
  }
  else {
    setting = setting * 2;
    setting = round(setting);
    setting = setting / 2;
    wantedSettings.temperature = setting < 10 ? 10 : (setting > 31 ? 31 : setting);
  }
  lastWanted = CUSTOM_MILLIS;
}

void HeatPump::setRemoteTemperature(float setting) {
  byte packet[PACKET_LEN] = {};
  
  hpProtocol.prepareSetPacket(packet, PACKET_LEN);

  packet[5] = 0x07;
  if(setting > 0) {
    packet[6] = 0x01;
    setting = setting * 2;
    setting = round(setting);
    packet[7] = (byte)(setting - 16);
    packet[8] = (byte)(setting + 128);
  }
  else {
    packet[6] = 0x00;
    packet[8] = 0x80; //MHK1 send 80, even though it could be 00, since ControlByte is 00
  } 
  // add the checksum
  byte chkSum = checkSum(packet, 21);
  packet[21] = chkSum;
  while(!canSend(false)) { CUSTOM_DELAY(10); }
  writePacket(packet, PACKET_LEN);
}

const char* HeatPump::getFanSpeed() {
  return currentSettings.fan;
}


void HeatPump::setFanSpeed(const char* setting) {
  int index = lookupByteMapIndex(FAN_MAP, 6, setting);
  if (index > -1) {
    wantedSettings.fan = FAN_MAP[index];
  } else {
    wantedSettings.fan = FAN_MAP[0];
  }
  lastWanted = CUSTOM_MILLIS;
}

const char* HeatPump::getVaneSetting() {
  return currentSettings.vane;
}

void HeatPump::setVaneSetting(const char* setting) {
  int index = lookupByteMapIndex(VANE_MAP, 7, setting);
  if (index > -1) {
    wantedSettings.vane = VANE_MAP[index];
  } else {
    wantedSettings.vane = VANE_MAP[0];
  }
  lastWanted = CUSTOM_MILLIS;
}

const char* HeatPump::getWideVaneSetting() {
  return currentSettings.wideVane;
}

void HeatPump::setWideVaneSetting(const char* setting) {
  int index = lookupByteMapIndex(WIDEVANE_MAP, 7, setting);
  if (index > -1) {
    wantedSettings.wideVane = WIDEVANE_MAP[index];
  } else {
    wantedSettings.wideVane = WIDEVANE_MAP[0];
  }
  lastWanted = CUSTOM_MILLIS;
}

bool HeatPump::getIseeBool() { //no setter yet
  return currentSettings.iSee;
}

heatpumpStatus HeatPump::getStatus() {
  return currentStatus;
}

float HeatPump::getRoomTemperature() {
  return currentStatus.roomTemperature;
}

bool HeatPump::getOperating() {
  return currentStatus.operating;
}

float HeatPump::FahrenheitToCelsius(int tempF) {
  float temp = (tempF - 32) / 1.8;                
  return ((float)round(temp*2))/2;                 //Round to nearest 0.5C
}

int HeatPump::CelsiusToFahrenheit(float tempC) {
  float temp = (tempC * 1.8) + 32;                //round up if heat, down if cool or any other mode
  return (int)(temp + 0.5);
}

void HeatPump::setOnConnectCallback(ON_CONNECT_CALLBACK_SIGNATURE) {
  this->onConnectCallback = onConnectCallback;
}

void HeatPump::setSettingsChangedCallback(SETTINGS_CHANGED_CALLBACK_SIGNATURE) {
  this->settingsChangedCallback = settingsChangedCallback;
}

void HeatPump::setStatusChangedCallback(STATUS_CHANGED_CALLBACK_SIGNATURE) {
  this->statusChangedCallback = statusChangedCallback;
}

void HeatPump::setPacketCallback(PACKET_CALLBACK_SIGNATURE) {
  this->packetCallback = packetCallback;
}

//#### WARNING, THE FOLLOWING METHOD CAN F--K YOUR HP UP, USE WISELY ####
void HeatPump::sendCustomPacket(byte data[], int packetLength) {
  while(!canSend(false)) { CUSTOM_DELAY(10); }

  packetLength += 2; // +2 for first header byte and checksum
  packetLength = (packetLength > PACKET_LEN) ? PACKET_LEN : packetLength; // ensure we are not exceeding PACKET_LEN
  byte packet[packetLength];
  packet[0] = HEADER[0]; // add first header byte

  // add data
  for (int i = 0; i < packetLength; i++) {
    packet[(i+1)] = data[i]; 
  }

  // add checksum
  byte chkSum = checkSum(packet, (packetLength-1));
  packet[(packetLength-1)] = chkSum;

  writePacket(packet, packetLength);
}

// Private Methods //////////////////////////////////////////////////////////////

bool HeatPump::canSend(bool isInfo) {
  return (CUSTOM_MILLIS - (isInfo ? PACKET_INFO_INTERVAL_MS : PACKET_SENT_INTERVAL_MS)) > lastSend;
}  

bool HeatPump::canRead() {
  return (waitForRead && (CUSTOM_MILLIS - PACKET_SENT_INTERVAL_MS) > lastSend);
}

void HeatPump::createPacket(byte *packet, heatpumpSettings settings) {
  hpProtocol.prepareSetPacket(packet, PACKET_LEN);
  
  if(settings.power != currentSettings.power) {
    packet[8]  = POWER[lookupByteMapIndex(POWER_MAP, 2, settings.power)];
    packet[6] += CONTROL_PACKET_1[0];
  }
  if(settings.mode!= currentSettings.mode) {
    packet[9]  = MODE[lookupByteMapIndex(MODE_MAP, 5, settings.mode)];
    packet[6] += CONTROL_PACKET_1[1];
  }
  if(!tempMode && settings.temperature!= currentSettings.temperature) {
    packet[10] = TEMP[lookupByteMapIndex(TEMP_MAP, 16, settings.temperature)];
    packet[6] += CONTROL_PACKET_1[2];
  }
  else if(tempMode && settings.temperature!= currentSettings.temperature) {
    float temp = (settings.temperature * 2) + 128;
    packet[19] = (int)temp;
    packet[6] += CONTROL_PACKET_1[2];
  }
  if(settings.fan!= currentSettings.fan) {
    packet[11] = FAN[lookupByteMapIndex(FAN_MAP, 6, settings.fan)];
    packet[6] += CONTROL_PACKET_1[3];
  }
  if(settings.vane!= currentSettings.vane) {
    packet[12] = VANE[lookupByteMapIndex(VANE_MAP, 7, settings.vane)];
    packet[6] += CONTROL_PACKET_1[4];
  }
  if(settings.wideVane!= currentSettings.wideVane) {
    packet[18] = WIDEVANE[lookupByteMapIndex(WIDEVANE_MAP, 7, settings.wideVane)] | (wideVaneAdj ? 0x80 : 0x00);
    packet[7] += CONTROL_PACKET_2[0];
  }
  // add the checksum
  byte chkSum = checkSum(packet, 21);
  packet[21] = chkSum;
}

void HeatPump::writePacket(byte *packet, int length) {
  for (int i = 0; i < length; i++) {
      io_device_->write((uint8_t)packet[i]);
  }

  if(packetCallback) {
    packetCallback(packet, length, (char*)"packetSent");
  }
  waitForRead = true;
  lastSend = CUSTOM_MILLIS;
}

int HeatPump::readByte() {
  uint8_t inputData;
  io_device_->read(&inputData);
  return inputData;
}

// Additional references
//  https://github.com/echavet/MitsubishiCN105ESPHome/blob/98a7c603972acfd1c435d2dc1c4414784c3dad38/components/cn105/hp_readings.cpp#L353
int HeatPump::readPacket() {
  byte header[INFOHEADER_LEN] = {};
  byte data[PACKET_LEN] = {};
  bool foundStart = false;
  int dataSum = 0;
  byte checksum = 0;
  byte dataLength = 0;
  
  waitForRead = false;

  if(io_device_->available() > 0) {
    // read until we get start byte 0xfc
    while(io_device_->available() > 0 && !foundStart) {
      header[0] = readByte();
      if(header[0] == HEADER[0]) {
        foundStart = true;
        CUSTOM_DELAY(100); // found that this delay increases accuracy when reading, might not be needed though
      }
    }

    if(!foundStart) {
      return RCVD_PKT_FAIL;
    }
    
    //read header
    for(int i=1;i<5;i++) {
      header[i] = readByte();
    }
    
    //check header
    if(header[0] == HEADER[0] && header[2] == HEADER[2] && header[3] == HEADER[3]) {
      dataLength = header[4];
      
      for(int i=0;i<dataLength;i++) {
        data[i] = readByte();
      }
  
      // read checksum byte
      data[dataLength] = readByte();
  
      // sum up the header bytes...
      for (int i = 0; i < INFOHEADER_LEN; i++) {
        dataSum += header[i];
      }

      // ...and add to that the sum of the data bytes
      for (int i = 0; i < dataLength; i++) {
        dataSum += data[i];
      }
  
      // calculate checksum
      checksum = (0xfc - dataSum) & 0xff;

      if(data[dataLength] == checksum) {
        lastRecv = CUSTOM_MILLIS;
        if(packetCallback) {
          byte packet[37]; // we are going to put header[5] and data[32] into this, so the whole packet is sent to the callback
          for(int i=0; i<INFOHEADER_LEN; i++) {
            packet[i] = header[i];
          }
          for(int i=0; i<(dataLength+1); i++) { //must be dataLength+1 to pick up checksum byte
            packet[(i+5)] = data[i];
          }
          packetCallback(packet, PACKET_LEN, (char*)"packetRecv");
        }

        if(header[1] == 0x62) {
          switch(data[0]) {
            case 0x02: { // setting information
              heatpumpSettings receivedSettings;
              receivedSettings.power       = lookupByteMapValue(POWER_MAP, POWER, 2, data[3]);
              receivedSettings.iSee = data[4] > 0x08 ? true : false;
              receivedSettings.mode = lookupByteMapValue(MODE_MAP, MODE, 5, receivedSettings.iSee  ? (data[4] - 0x08) : data[4]);

              if(data[11] != 0x00) {
                int temp = data[11];
                temp -= 128;
                receivedSettings.temperature = (float)temp / 2;
                tempMode =  true;
              } else {
                receivedSettings.temperature = lookupByteMapValue(TEMP_MAP, TEMP, 16, data[5]);
              }

              receivedSettings.fan         = lookupByteMapValue(FAN_MAP, FAN, 6, data[6]);
              receivedSettings.vane        = lookupByteMapValue(VANE_MAP, VANE, 7, data[7]);
              receivedSettings.wideVane    = lookupByteMapValue(WIDEVANE_MAP, WIDEVANE, 7, data[10] & 0x0F);
              wideVaneAdj = (data[10] & 0xF0) == 0x80 ? true : false;
              
              if(settingsChangedCallback && receivedSettings != currentSettings) {
                currentSettings = receivedSettings;
                settingsChangedCallback();
              } else {
                currentSettings = receivedSettings;
              }

              // if this is the first time we have synced with the heatpump, set wantedSettings to receivedSettings
              // hack: add grace period of a few seconds before respecting external changes
              if(firstRun || (autoUpdate && externalUpdate && CUSTOM_MILLIS - lastWanted > AUTOUPDATE_GRACE_PERIOD_IGNORE_EXTERNAL_UPDATES_MS)) {
                wantedSettings = currentSettings;
                firstRun = false;
              }

              return RCVD_PKT_SETTINGS;
            }

            case 0x03: { //Room temperature reading
              //ESP_LOGD("Decoder", "[0x03 room temperature]");
              //this->last_received_packet_sensor->publish_state("0x62-> 0x03: Data -> Room temperature");
              //                 0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
              // FC 62 01 30 10 03 00 00 0E 00 94 B0 B0 FE 42 00 01 0A 64 00 00 A9
              //                         RT    OT RT SP ?? ?? ?? RM RM RM
              // FC 62 01 30 10 03 00 00 0B 00 94 AB 00 00 00 00 00 00 00 00 00 10 
              // RT = room temperature (in old format and in new format)
              // OT = outside air temperature
              // SP = room setpoint temperature?
              // RM = indoor unit operating time in minutes

              heatpumpStatus receivedStatus;
              if(data[6] != 0x00) {
                int temp = data[6];
                temp -= 128;
                receivedStatus.roomTemperature = (float)temp / 2;
              } else {
                receivedStatus.roomTemperature = lookupByteMapValue(ROOM_TEMP_MAP, ROOM_TEMP, 32, data[3]);
              }

              receivedStatus.runtimeHours = float((data[11] << 16) | (data[12] << 8) | data[13]) / 60;

              currentStatus.roomTemperature = receivedStatus.roomTemperature;
              currentStatus.runtimeHours = receivedStatus.runtimeHours;
              if((statusChangedCallback)) {
                  statusChangedCallback(currentStatus);
              }

              return RCVD_PKT_ROOM_TEMP;
            }

            case 0x04: { // unknown
                break; 
            }

            case 0x05: { // timer packet
              heatpumpTimers receivedTimers;
              receivedTimers.mode                = lookupByteMapValue(TIMER_MODE_MAP, TIMER_MODE, 4, data[3]);
              receivedTimers.onMinutesSet        = data[4] * TIMER_INCREMENT_MINUTES;
              receivedTimers.onMinutesRemaining  = data[6] * TIMER_INCREMENT_MINUTES;
              receivedTimers.offMinutesSet       = data[5] * TIMER_INCREMENT_MINUTES;
              receivedTimers.offMinutesRemaining = data[7] * TIMER_INCREMENT_MINUTES;

              // callback for status change
              if(statusChangedCallback && currentStatus.timers != receivedTimers) {
                currentStatus.timers = receivedTimers;
                statusChangedCallback(currentStatus);
              } else {
                currentStatus.timers = receivedTimers;
              }

              return RCVD_PKT_TIMER;
            }

            case 0x06: { // status
              //FC 62 01 30 10 06 00 00 1A 01 00 00 00 00 00 00 00 00 00 00 00 3C
              //MSZ-RW25VGHZ-SC1 / MUZ-RW25VGHZ-SC1
              //FC 62 01 30 10 06 00 00 00 01 00 08 05 50 00 00 42 00 00 00 00 B7
              //                           OP IP IP EU EU       ??
              //FC 62 01 30 10 06 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 57
              // OP = operating status (1 = compressor running, 0 = standby)
              // IP = Current input power in Watts (16-bit decimal)
              // EU = energy usage
              //      (used energy in kWh = value/10)
              //      TODO: Currently the maximum size of the counter is not known and
              //            if the counter extends to other bytes.
              // ?? = unknown bytes that appear to have a fixed/constant value

              heatpumpStatus receivedStatus;
              receivedStatus.operating = data[4];
              receivedStatus.compressorFrequency = data[3];
              receivedStatus.inputPower = (data[5] << 8) | data[6];
              receivedStatus.kWh = float((data[7] << 8) | data[8]) / 10.0;

              currentStatus.operating = receivedStatus.operating;
              currentStatus.compressorFrequency = receivedStatus.compressorFrequency;
              currentStatus.inputPower = receivedStatus.inputPower;
              currentStatus.kWh = receivedStatus.kWh;
              // callback for status change -- not triggered for compressor frequency at the moment
              if(statusChangedCallback) {
                statusChangedCallback(currentStatus);
              }

              return RCVD_PKT_STATUS;
            }

            case 0x09: { // standby mode maybe?
              /* Power */
              //ESP_LOGD(LOG_CYCLE_TAG, "5b: Receiving Power/Standby response");
              //this->getPowerFromResponsePacket();
              //FC 62 01 30 10 09 00 00 00 02 02 00 00 00 00 00 00 00 00 00 00 50
              break;
            }
            
            case 0x20:
            case 0x22: {
              if (dataLength == 0x10) {
                if (data[0] == 0x20) {
                  functions.setData1(&data[1]);
                } else {
                  functions.setData2(&data[1]);
                }
                  
                return RCVD_PKT_FUNCTIONS;
              }
              break;
            }
          } 
        } 
        
        if(header[1] == 0x61) { //Last update was successful 
          return RCVD_PKT_UPDATE_SUCCESS;
        } else if(header[1] == 0x7a) { //Last update was successful 
          connected = true;
          return RCVD_PKT_CONNECT_SUCCESS;
        }
      }
    }
  }

  return RCVD_PKT_FAIL;
}

void HeatPump::readAllPackets() {
  while (io_device_->available() > 0) {
    readPacket();
  }
}

heatpumpFunctions HeatPump::getFunctions() {
  functions.clear();
  
  byte packet1[PACKET_LEN] = {};
  byte packet2[PACKET_LEN] = {};

  prepareInfoPacket(packet1, PACKET_LEN);
  packet1[5] = FUNCTIONS_GET_PART1;
  packet1[21] = checkSum(packet1, 21);

  prepareInfoPacket(packet2, PACKET_LEN);
  packet2[5] = FUNCTIONS_GET_PART2;
  packet2[21] = checkSum(packet2, 21);
  
  while(!canSend(false)) { CUSTOM_DELAY(10); }
  writePacket(packet1, PACKET_LEN);
  readPacket();

  while(!canSend(false)) { CUSTOM_DELAY(10); }
  writePacket(packet2, PACKET_LEN);
  readPacket();

  // retry reading a few times in case responses were related
  // to other requests
  for (int i = 0; i < 5 && !functions.isValid(); ++i) {
    CUSTOM_DELAY(100);
    readPacket();
  }

  return functions;
}

bool HeatPump::setFunctions(heatpumpFunctions const& functions) {
  if (!functions.isValid()) {
    return false;
  }

  byte packet1[PACKET_LEN] = {};
  byte packet2[PACKET_LEN] = {};

  hpProtocol.prepareSetPacket(packet1, PACKET_LEN);
  packet1[5] = FUNCTIONS_SET_PART1;
  
  hpProtocol.prepareSetPacket(packet2, PACKET_LEN);
  packet2[5] = FUNCTIONS_SET_PART2;
  
  functions.getData1(&packet1[6]);
  functions.getData2(&packet2[6]);

  // sanity check, we expect data byte 15 (index 20) to be 0
  if (packet1[20] != 0 || packet2[20] != 0)
    return false;
    
  // make sure all the other data bytes are set
  for (int i = 6; i < 20; ++i) {
    if (packet1[i] == 0 || packet2[i] == 0)
      return false;
  }

  packet1[21] = checkSum(packet1, 21);
  packet2[21] = checkSum(packet2, 21);

  while(!canSend(false)) { CUSTOM_DELAY(10); }
  writePacket(packet1, PACKET_LEN);
  readPacket();

  while(!canSend(false)) { CUSTOM_DELAY(10); }
  writePacket(packet2, PACKET_LEN);
  readPacket();

  return true;
}
