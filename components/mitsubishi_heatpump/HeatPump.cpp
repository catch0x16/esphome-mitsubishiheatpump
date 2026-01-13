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

HeatPump::HeatPump(devicestate::IIODevice* io_device) :
    io_device_{io_device},
    hpState{},
    hpProtocol{} {

  lastWanted = CUSTOM_MILLIS;
  lastSend = 0;
  infoMode = 0;
  lastRecv = CUSTOM_MILLIS - (PACKET_SENT_INTERVAL_MS * 10);
  firstRun = true;
  waitForRead = false;
  autoUpdate = true;
  externalUpdate = true;
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
  hpProtocol.createPacket(packet, hpState);
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
  else if(autoUpdate && !firstRun && hpState.getWantedSettings().hasChanged && packetType == PACKET_TYPE_DEFAULT) {
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

heatpumpSettings HeatPump::getSettings() {
  return hpState.getCurrentSettings();
}

bool HeatPump::isConnected() {
  return connected;
}

void HeatPump::setPowerSetting(const char* setting) {
  hpState.setPowerSetting(setting);
  lastWanted = CUSTOM_MILLIS;
}

void HeatPump::setModeSetting(const char* setting) {
  hpState.setModeSetting(setting);
  lastWanted = CUSTOM_MILLIS;
}

void HeatPump::setTemperature(float setting) {
  hpState.setTemperatureSetting(setting);
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

void HeatPump::setFanSpeed(const char* setting) {
  hpState.setFanSpeed(setting);
  lastWanted = CUSTOM_MILLIS;
}

void HeatPump::setVaneSetting(const char* setting) {
  hpState.setVaneSetting(setting);
  lastWanted = CUSTOM_MILLIS;
}

void HeatPump::setWideVaneSetting(const char* setting) {
  hpState.setWideVaneSetting(setting);
  lastWanted = CUSTOM_MILLIS;
}

heatpumpStatus HeatPump::getStatus() {
  return currentStatus;
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

// Private Methods //////////////////////////////////////////////////////////////

bool HeatPump::canSend(bool isInfo) {
  return (CUSTOM_MILLIS - (isInfo ? PACKET_INFO_INTERVAL_MS : PACKET_SENT_INTERVAL_MS)) > lastSend;
}  

bool HeatPump::canRead() {
  return (waitForRead && (CUSTOM_MILLIS - PACKET_SENT_INTERVAL_MS) > lastSend);
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
              heatpumpSettings oldSettings = hpState.getCurrentSettings();

              hpProtocol.parseSettings0x02(data, hpState);
              
              if(settingsChangedCallback && oldSettings != hpState.getCurrentSettings()) {
                settingsChangedCallback();
              }

              // if this is the first time we have synced with the heatpump, set wantedSettings to receivedSettings
              // hack: add grace period of a few seconds before respecting external changes
              if(firstRun || (autoUpdate && externalUpdate && CUSTOM_MILLIS - lastWanted > AUTOUPDATE_GRACE_PERIOD_IGNORE_EXTERNAL_UPDATES_MS)) {
                hpState.getWantedSettings().resetSettings();
                firstRun = false;
              }

              return RCVD_PKT_SETTINGS;
            }

            case 0x03: { //Room temperature reading
              hpProtocol.parseStatus0x03(data, hpState);
              if((statusChangedCallback)) {
                  statusChangedCallback(hpState.getCurrentStatus());
              }

              return RCVD_PKT_ROOM_TEMP;
            }

            case 0x04: { // unknown
                break; 
            }

            case 0x05: { // timer packet
              heatpumpTimers oldTimers = hpState.getTimers();

              hpProtocol.parseTimers0x05(data, hpState);

              // callback for status change
              if(statusChangedCallback && oldTimers != hpState.getTimers()) {
                statusChangedCallback(hpState.getCurrentStatus());
              }

              return RCVD_PKT_TIMER;
            }

            case 0x06: { // status
              hpProtocol.parseStatus0x06(data, hpState);
              // callback for status change -- not triggered for compressor frequency at the moment
              if(statusChangedCallback) {
                statusChangedCallback(hpState.getCurrentStatus());
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
            
            case 0x20: {
              if (dataLength == 0x10) {
                hpProtocol.parseFunctions0x20(data, hpState);
                return RCVD_PKT_FUNCTIONS;
              }
              break;
            }
            case 0x22: {
              if (dataLength == 0x10) {
                hpProtocol.parseFunctions0x22(data, hpState);
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
  hpState.getFunctions().clear();
  
  byte packet1[PACKET_LEN] = {};
  byte packet2[PACKET_LEN] = {};

  hpProtocol.prepareInfoPacket(packet1, PACKET_LEN);
  packet1[5] = FUNCTIONS_GET_PART1;
  packet1[21] = checkSum(packet1, 21);

  hpProtocol.prepareInfoPacket(packet2, PACKET_LEN);
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
  for (int i = 0; i < 5 && !hpState.getFunctions().isValid(); ++i) {
    CUSTOM_DELAY(100);
    readPacket();
  }

  return hpState.getFunctions();
}
