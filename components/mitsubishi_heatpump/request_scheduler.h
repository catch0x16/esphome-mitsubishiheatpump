#pragma once

#include "info_request.h"
#include "cn105_state.h"
#include <vector>
#include <functional>
#include <string>

namespace devicestate {

    class CN105State; // forward declaration

    /**
     * @class RequestScheduler
     * @brief Manages the cyclic request queue and their chaining.
     *
     * This class extracts the INFO request management logic from the CN105State component
     * to respect the single responsibility principle (SRP).
     */
    class RequestScheduler {
    public:
        /**
         * @brief Callback type for sending a packet
         * @param code The request code to send
         */
        using SendCallback = std::function<void(uint8_t)>;

        /**
         * @brief Callback type for timeout management
         * @param name Unique timeout name
         * @param timeout_ms Timeout duration in milliseconds
         * @param callback Function to call when timeout expires
         */
        using TimeoutCallback = std::function<void(const std::string&, uint32_t, std::function<void()>)>;

        /**
         * @brief Callback type for terminating a cycle
         */
        using TerminateCallback = std::function<void()>;

        /**
         * @brief Callback type for obtaining the CN105State context (for canSend and onResponse)
         * @return Pointer to CN105State (can be nullptr)
         */
        using ContextCallback = std::function<CN105State* ()>;

        /**
         * @brief Constructor
         * @param send_callback Callback to send a packet
         * @param timeout_callback Callback for timeout management (can be nullptr if not used)
         * @param terminate_callback Callback to terminate a cycle
         * @param context_callback Callback to obtain the CN105State context (for canSend and onResponse)
         */
        RequestScheduler(
            SendCallback send_callback,
            TimeoutCallback timeout_callback = nullptr,
            TerminateCallback terminate_callback = nullptr,
            ContextCallback context_callback = nullptr
        );

        /**
         * @brief Registers a request in the queue
         * @param req The request to register (non-const reference to allow modification)
         */
        void register_request(InfoRequest& req);

        /**
         * @brief Clears the request list
         */
        void clear_requests();

        /**
         * @brief Disables a request by its code
         * @param code The code of the request to disable
         */
        void disable_request(uint8_t code);

        /**
         * @brief Checks if the queue is empty
         * @return true if empty, false otherwise
         */
        bool is_empty() const;

        /**
         * @brief Sends the next request after the one with the specified code
         * @param previous_code The code of the previous request (0x00 to start)
         * @param context CN105State context to check canSend (can be nullptr, uses context_callback_ if provided)
         */
        void send_next_after(uint8_t previous_code, CN105State* context = nullptr);

        /**
         * @brief Marks a response as received for a given code and calls the onResponse callback if present
         * @param code The code of the request whose response was received
         * @param context CN105State context to call onResponse (can be nullptr)
         */
        void mark_response_seen(uint8_t code, CN105State* context = nullptr);

        /**
         * @brief Processes a received response
         * @param code The code of the received response
         * @param context CN105State context to call onResponse (can be nullptr, uses context_callback_ if provided)
         * @return true if the response was processed, false otherwise
         */
        bool process_response(uint8_t code, CN105State* context = nullptr);

        /**
         * @brief Method to call in the main loop to manage timeouts
         * Note: Timeout management is currently handled via callbacks,
         * this method is intended for future management if needed.
         */
        void loop();

    private:
        std::vector<InfoRequest> requests_;          // Request queue
        int current_request_index_;                  // Current request index
        SendCallback send_callback_;                  // Callback to send a packet
        TimeoutCallback timeout_callback_;            // Callback for timeout management
        TerminateCallback terminate_callback_;        // Callback to terminate a cycle
        ContextCallback context_callback_;            // Callback to obtain the CN105State context

        /**
         * @brief Sends a specific request by its code
         * @param code The code of the request to send
         * @param context CN105State context to check canSend (can be nullptr)
         */
        void send_request(uint8_t code, CN105State* context = nullptr);
    };

}

