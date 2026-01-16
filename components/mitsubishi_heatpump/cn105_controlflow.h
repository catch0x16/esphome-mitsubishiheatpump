#pragma once

#ifdef USE_ESP32
#include <mutex>
#endif

#include "io_device.h"

#include "cn105_types.h"
#include "cn105_state.h"
#include "cn105_connection.h"
#include "cn105_protocol.h"
#include "info_request.h"

#include "cycle_management.h"
#include "request_scheduler.h"

#include "esphome.h"

namespace devicestate {

    class CN105ControlFlow {
        public:
            using RetryCallback = std::function<void(const std::string&, uint32_t, uint8_t, std::function<esphome::RetryResult(uint8_t)>)>;
            using AcquireCallback = std::function<void()>;

            CN105ControlFlow(
                CN105Connection* connection,
                CN105State* hpState,
                RequestScheduler::TimeoutCallback timeoutCallback,
                RequestScheduler::TerminateCallback terminateCallback,
                RetryCallback retryCallback
            );

            void set_debounce_delay(uint32_t delay);

            void loop(cycleManagement& loopCycle);
            void registerInfoRequests();

            void setRemoteTemperature(const float current);
            void pingExternalTemperature();
            void completeCycle();

            void acquireWantedSettingsLock(AcquireCallback callback);

        private:
            CN105Connection* connection_;
            CN105State* hpState_;
            RequestScheduler::TimeoutCallback timeoutCallback_;
            RetryCallback retryCallback_;
            RequestScheduler scheduler_;
            CN105Protocol hpProtocol;

            uint32_t debounce_delay_;
            uint32_t remote_temp_timeout_;

            bool shouldSendExternalTemperature_ = false;
            float remoteTemperature_ = 0;

#ifdef USE_ESP32
            std::mutex wantedSettingsMutex;
#else
            void emulateMutex(const char* retryName, std::function<void()>&& f);
            volatile bool wantedSettingsMutex = false;
#endif

            bool processInput(CN105State& hpState);
            void buildAndSendInfoPacket(uint8_t code);
            void buildAndSendRequestsInfoPackets(cycleManagement& loopCycle);
            void buildAndSendRequestPacket(int packetType);

            void sendWantedSettingsDelegate();
            bool sendWantedSettings();
            void checkPendingWantedSettings(cycleManagement& loopCycle);

            bool sendWantedRunStates();
            void checkPendingWantedRunStates(cycleManagement& loopCycle);

            bool getOffsetDirection();

            void sendRemoteTemperature();
    };

}
