/*
  HeatPump.h - Mitsubishi Heat Pump control library for Arduino
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
#ifndef __HeatPump_H__
#define __HeatPump_H__
#include <cstdint>
#include <cmath>
#include <cstring>
#include <string>

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else

#endif

#include "cn105_types.h"
#include "cn105_protocol.h"
#include "cn105_state.h"

#include "info_request.h"
#include "request_scheduler.h"

#include "io_device.h"

#include "heatpumpFunctions.h"
using namespace devicestate;

#include <functional>
#define ON_CONNECT_CALLBACK_SIGNATURE std::function<void()> onConnectCallback
#define SETTINGS_CHANGED_CALLBACK_SIGNATURE std::function<void()> settingsChangedCallback
#define STATUS_CHANGED_CALLBACK_SIGNATURE std::function<void(heatpumpStatus newStatus)> statusChangedCallback
#define PACKET_CALLBACK_SIGNATURE std::function<void(byte* packet, unsigned int length, char* packetDirection)> packetCallback

typedef uint8_t byte;

class HeatPump
{
  private:
    CN105Protocol hpProtocol;

    static const int PACKET_SENT_INTERVAL_MS = 1000;
    static const int PACKET_INFO_INTERVAL_MS = 2000;
    static const int AUTOUPDATE_GRACE_PERIOD_IGNORE_EXTERNAL_UPDATES_MS = 30000;

    CN105State hpState;

    // Hacks
    unsigned long lastWanted;

    // initialise to all off, then it will update shortly after connect;
    heatpumpStatus currentStatus{ 0, 0, false, {TIMER_MODE_MAP[0], 0, 0, 0, 0}, 0, 0, 0, 0 };

    unsigned long lastSend;
    bool waitForRead;
    int infoMode;
    unsigned long lastRecv;
    bool connected = false;
    bool autoUpdate;
    bool firstRun;
    //bool tempMode;
    bool externalUpdate;
    //bool wideVaneAdj;
    bool fastSync = false;

    bool canSend(bool isInfo);
    bool canRead();

    void createPacket(byte *packet);

    int readByte();
    int readPacket();
    void readAllPackets();
    void writePacket(byte *packet, int length);

    // callbacks
    ON_CONNECT_CALLBACK_SIGNATURE {nullptr};
    SETTINGS_CHANGED_CALLBACK_SIGNATURE {nullptr};
    STATUS_CHANGED_CALLBACK_SIGNATURE {nullptr};
    PACKET_CALLBACK_SIGNATURE {nullptr};

  public:
    devicestate::IIODevice* io_device_;

    // general
    HeatPump(devicestate::IIODevice* io_device);

    bool connect();
    bool update();
    uint8_t translatePacket(int packetType);
    void buildAndSendInfoPacket(uint8_t code);
    void sync(byte packetType = PACKET_TYPE_DEFAULT);

    // settings
    heatpumpSettings getSettings();

    void setPowerSetting(const char* setting);
    
    void setModeSetting(const char* setting);
    void setTemperature(float setting);
    void setRemoteTemperature(float setting);
    void setFanSpeed(const char* setting);
    void setVaneSetting(const char* setting);
    void setWideVaneSetting(const char* setting);
    

    // status
    heatpumpStatus getStatus();
    bool isConnected();

    // functions
    // NOTE: These methods have been tested with a PVA (P-series air handler) unit and has not been tested with anything else. Use at your own risk.
    heatpumpFunctions getFunctions();

    // callbacks
    void setOnConnectCallback(ON_CONNECT_CALLBACK_SIGNATURE);
    void setSettingsChangedCallback(SETTINGS_CHANGED_CALLBACK_SIGNATURE);
    void setStatusChangedCallback(STATUS_CHANGED_CALLBACK_SIGNATURE);
    void setPacketCallback(PACKET_CALLBACK_SIGNATURE);

};
#endif