# Bug Analysis: esphome-mitsubishiheatpump

## Executive Summary

This document provides a comprehensive analysis of bugs discovered in the esphome-mitsubishiheatpump project that could cause panics or crashes in an ESPHome environment.

**Total Bugs Identified:** 21 across 7 categories

| Severity | Count | Description |
|----------|-------|-------------|
| CRITICAL | 4 | Immediate crash potential |
| HIGH | 6 | Significant functional issues |
| MEDIUM | 11 | Edge cases and code quality |

---

## Severity Classification

- **CRITICAL**: Bugs that can cause immediate crashes, memory corruption, or complete system failure
- **HIGH**: Bugs that cause significant functional issues or unpredictable behavior
- **MEDIUM**: Bugs that cause minor issues or edge-case failures unlikely in normal operation

---

## CRITICAL Severity Bugs

### C1. MovingSlopeAverage Vector Access When Empty

**Location:** `components/mitsubishi_heatpump/floats.h` lines 97-109

```cpp
float increment(float next) {
    const int n = this->data.size();

    if (n == window) {
        double lastDelta = calculateDelta(this->data[0], this->data[1]);
        deltaSum -= lastDelta;
        this->data.erase(this->data.begin());
    }

    double delta = calculateDelta(this->data[n - 1], next);  // BUG: when n=0, accesses data[-1]
    this->deltaSum += delta;
    this->data.push_back(next);
    return delta;
}
```

**Reasoning:** When `increment()` is called on an empty vector (n=0), `this->data[n - 1]` becomes `this->data[-1]`, causing undefined behavior and likely a crash.

**Impact:** Crash on first temperature reading when PID is enabled.

---

### C2. Checksum Validation Buffer Over-read

**Location:** `components/mitsubishi_heatpump/cn105_connection.cpp` lines 127-154

```cpp
bool CN105Connection::checkSum() {
    uint8_t packetCheckSum = storedInputData[this->bytesRead];  // No bounds check
    uint8_t processedCS = 0;

    for (int i = 0; i < this->dataLength + 5; i++) {
        processedCS += this->storedInputData[i];  // BUG: no bounds check against MAX_DATA_BYTES
    }
    // ...
}
```

**Reasoning:** If a malformed packet declares a large `dataLength`, the loop reads beyond the 64-byte `storedInputData` buffer, causing memory corruption or crash.

**Impact:** Memory corruption or crash when processing malformed packets.

---

### C3. ESP8266 Mutex Emulation is Fundamentally Unsafe

**Location:** `components/mitsubishi_heatpump/cn105_controlflow.h` line 64

```cpp
#ifdef USE_ESP32
    std::mutex wantedSettingsMutex;
#else
    volatile bool wantedSettingsMutex = false;  // BUG: volatile bool is NOT atomic
#endif
```

**Location:** `components/mitsubishi_heatpump/cn105_controlflow.cpp` lines 45-66

```cpp
void CN105ControlFlow::emulateMutex(const char* retryName, std::function<void()>&& f) {
    retryCallback_(retryName, 100, 10, [this, f, retryName](uint8_t retry_count) {
        if (this->wantedSettingsMutex) {
            if (retry_count < 1) {
                ESP_LOGW(retryName, "10 retry calls failed..., forcing unlock...");
                this->wantedSettingsMutex = true;  // BUG: Should be false!
                f();
                this->wantedSettingsMutex = false;
                return esphome::RetryResult::DONE;
            }
            // ...
        }
        // ...
    });
}
```

**Reasoning:**
1. `volatile bool` does NOT provide atomicity - TOCTOU race condition exists
2. Line with "forcing unlock" sets mutex to `true` instead of `false` (typo)

**Impact:** Race conditions causing corrupted settings, potentially damaging equipment.

---

### C4. Uninitialized Timing Variables

**Location:** `components/mitsubishi_heatpump/cn105_connection.h` lines 68-71

```cpp
unsigned long lastSend;              // UNINITIALIZED
unsigned long lastConnectRqTimeMs;   // UNINITIALIZED
unsigned long lastReconnectTimeMs;   // UNINITIALIZED
unsigned long lastResponseMs;        // UNINITIALIZED
```

**Reasoning:** These timing variables contain garbage values on startup. Comparisons like `CUSTOM_MILLIS - this->lastSend > 300` produce incorrect results.

**Impact:** Connection may fail to establish or flood UART with packets.

---

## HIGH Severity Bugs

### H1. DeviceStateManager::publish() Null Pointer Dereferences

**Location:** `components/mitsubishi_heatpump/devicestatemanager.cpp` lines 396-407

```cpp
void DeviceStateManager::publish() {
    this->device_status_operating->publish_state(this->deviceStatus.operating);
    this->device_status_current_temperature->publish_state(this->deviceStatus.currentTemperature);
    this->device_status_compressor_frequency->publish_state(this->deviceStatus.compressorFrequency);
    // ... more sensor pointers dereferenced without null checks
}
```

**Reasoning:** Sensor pointers are dereferenced without null checks. If any sensor is not configured in YAML, the pointer is null.

**Impact:** Crash when configuration omits optional sensors.

---

### H2. Workflow Steps Null Parameter Usage

**Location:** `components/mitsubishi_heatpump/hysterisis_workflowstep.cpp` lines 21-79
**Location:** `components/mitsubishi_heatpump/pid_workflowstep.cpp` lines 43-82

```cpp
void HysterisisWorkflowStep::run(const float currentTemperature,
                                  devicestate::IDeviceStateManager* deviceManager) {
    // deviceManager used throughout without null check
    HysterisisResult result = this->getHysterisisResult(currentTemperature, deviceManager);
    // ...
}
```

**Reasoning:** The `deviceManager` parameter is used without null validation throughout these workflow steps.

**Impact:** Crash if deviceManager is null due to initialization order issues.

---

### H3. Memory Allocation Without Null Checks

**Location:** `components/mitsubishi_heatpump/espmhp.cpp` lines 675-739

```cpp
this->hysterisisWorkflowStep = new HysterisisWorkflowStep(...);
this->pidWorkflowStep = new PidWorkflowStep(...);
IIODevice* io_device = new UARTIODevice(...);
this->hpState_ = new CN105State();
CN105Connection* hpConnection = new CN105Connection(...);
this->hpControlFlow_ = new CN105ControlFlow(...);
this->dsm = new devicestate::DeviceStateManager(...);
// None check for nullptr
```

**Reasoning:** On memory-constrained embedded systems, `new` can fail. These allocations don't check for failure.

**Impact:** Null pointer dereference crash if memory allocation fails during setup.

---

### H4. CircularBuffer Allocation Without Error Check

**Location:** `components/mitsubishi_heatpump/floats.h` lines 44-50

```cpp
CircularBuffer(int capacity) {
    this->capacity = capacity;
    this->count = 0;
    this->items = new float[capacity];  // No null check
    this->head = 0;
    this->tail = 0;
}
```

**Reasoning:** Array allocation could fail on memory-constrained devices.

**Impact:** Crash when accessing items array if allocation failed.

---

### H5. Debounce Logic is Inverted

**Location:** `components/mitsubishi_heatpump/cn105_controlflow.cpp` lines 108-110

```cpp
if (!(now - this->hpState_->getWantedSettings().lastChange < this->debounce_delay_)) {
    ESP_LOGI(LOG_ACTION_EVT_TAG, "Skipping checkPendingWantedSettings due to lastChange");
    return;
}
```

**Reasoning:** The condition `!(time < delay)` means "skip when time >= delay" (enough time passed). Correct debounce should skip when NOT enough time has passed.

**Current behavior:** Skips sending settings when enough time HAS passed
**Correct behavior:** Should skip when NOT enough time has passed

**Impact:** Settings changes are never sent to the heat pump after the debounce delay expires.

---

### H6. CUSTOM_MILLIS Wraparound Handling is Incomplete

**Location:** `components/mitsubishi_heatpump/cycle_management.cpp` lines 63-70

```cpp
bool cycleManagement::hasUpdateIntervalPassed() {
    if (CUSTOM_MILLIS < lastCompleteCycleMs) return false;  // BUG: returns false forever after wrap
    return (CUSTOM_MILLIS - lastCompleteCycleMs) > update_interval;
}

bool cycleManagement::doesCycleTimeOut() {
    if (CUSTOM_MILLIS < lastCycleStartMs) return false;  // Same issue
    return (CUSTOM_MILLIS - lastCycleStartMs) > (2 * update_interval) + 1000;
}
```

**Reasoning:** When `CUSTOM_MILLIS` wraps around after ~49.7 days, `CUSTOM_MILLIS < lastCompleteCycleMs` becomes permanently true, causing the function to always return `false`.

**Impact:** After ~49.7 days uptime, the component stops polling the heat pump.

---

## MEDIUM Severity Bugs

### M1. Uninitialized data Pointer

**Location:** `components/mitsubishi_heatpump/cn105_connection.h` line 45

```cpp
uint8_t* data;  // Never initialized
```

**Impact:** Potential crash if getData() is called before a packet is processed.

---

### M2. lookupByteMapValue Misleading Defaults

**Location:** `components/mitsubishi_heatpump/cn105_utils.cpp` lines 43-57

```cpp
const char* lookupByteMapValue(...) {
    // ...
    if (defaultValue != nullptr) {
        return defaultValue;
    } else {
        return valuesMap[0];  // May not be appropriate default
    }
}
```

**Impact:** Incorrect mode/fan/vane display for unknown byte values.

---

### M3. getWideVaneSetting Returns Potentially Null Value

**Location:** `components/mitsubishi_heatpump/cn105_state.cpp` lines 106-114

```cpp
const char* CN105State::getWideVaneSetting() {
    if (this->wantedSettings.wideVane) {
        return this->wantedSettings.wideVane;
    } else {
        return this->currentSettings.wideVane;  // Could be null
    }
}
```

**Impact:** Crash if result used in strcmp without null check.

---

### M4. Unused nb_deffered_requests Field

**Location:** `components/mitsubishi_heatpump/cn105_types.h` line 173

```cpp
struct wantedHeatpumpSettings : heatpumpSettings {
    bool hasChanged;
    bool hasBeenSent;
    uint8_t nb_deffered_requests;  // Declared but NEVER used
    long lastChange;
    // ...
};
```

**Reasoning:** The `nb_deffered_requests` field suggests there was intended logic to track and queue deferred setting requests, but this was never implemented. Currently, if settings change while a previous change is pending, the previous change is silently overwritten.

**Impact:** User settings may be lost when rapid successive changes occur. No queuing or counting of deferred requests exists.

---

### M5. Connection Bootstrap Can Hang for 120 Seconds

**Location:** `components/mitsubishi_heatpump/cn105_connection.cpp` lines 36-80

**Impact:** Poor user experience during initial boot.

---

### M6. remote_temp_timeout_ Set to uint32_t Max

**Location:** `components/mitsubishi_heatpump/cn105_controlflow.cpp` line 31

```cpp
this->remote_temp_timeout_ = 4294967295;  // ~49 days
```

**Impact:** Remote temperature readings may never expire (stale data persists).

---

### M7. Timeout Calculation Overflow Risk

**Location:** `components/mitsubishi_heatpump/cycle_management.cpp` line 70

```cpp
return (CUSTOM_MILLIS - lastCycleStartMs) > (2 * update_interval) + 1000;
```

**Impact:** Premature timeouts with very large update intervals.

---

### M8. Buffer Overflow Prevention Has Off-by-One

**Location:** `components/mitsubishi_heatpump/cn105_connection.cpp` lines 367-371

```cpp
if (this->bytesRead >= (MAX_DATA_BYTES - 1)) {
    this->initBytePointer();
}
storedInputData[this->bytesRead] = inputData;  // Written after check
```

**Impact:** Potential off-by-one buffer write.

---

### M9. Missing Break Statements in Switch Cases

**Location:** `components/mitsubishi_heatpump/espmhp.cpp` lines 567-616

```cpp
switch (deviceState.swingMode) {
    case SwingMode::SwingMode_Both:
        this->swing_mode = climate::CLIMATE_SWING_BOTH;
    case SwingMode::SwingMode_Vertical:      // BUG: Fallthrough!
        this->swing_mode = climate::CLIMATE_SWING_VERTICAL;
    case SwingMode::SwingMode_Horizontal:    // BUG: Fallthrough!
        this->swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
    case SwingMode::SwingMode_Off:           // BUG: Fallthrough!
        this->swing_mode = climate::CLIMATE_SWING_OFF;
}

// Same pattern for vertical swing (lines 579-596):
switch(deviceState.verticalSwingMode) {
    case VerticalSwingMode::VerticalSwingMode_Swing:
        this->update_swing_vertical("swing");
    case VerticalSwingMode::VerticalSwingMode_Auto:  // Fallthrough!
        this->update_swing_vertical("auto");
    // ... continues without breaks
}

// Same pattern for horizontal swing (lines 599-616)
```

**Reasoning:** Missing `break` statements cause fall-through, resulting in all cases executing sequentially. The swing mode is set correctly initially, then immediately overwritten by subsequent cases.

**Impact:** Swing modes always show as the last value (OFF/down/right) regardless of actual state.

**Fix:** Add `break;` after each case body.

---

### M10. Status Update Logic Inverted

**Location:** `components/mitsubishi_heatpump/cn105_state.cpp` lines 349-351

```cpp
void CN105State::updateCurrentStatus(heatpumpStatus& status) {
    if (status != this->currentStatus) {  // BUG: Should be == not !=
        return;
    }
    this->statusInitialized = true;

    // ... update fields
    this->currentStatus.operating = status.operating;
    this->currentStatus.compressorFrequency = status.compressorFrequency;
    this->currentStatus.inputPower = status.inputPower;
    this->currentStatus.kWh = status.kWh;
    this->currentStatus.runtimeHours = status.runtimeHours;
    this->currentStatus.roomTemperature = status.roomTemperature;
    this->currentStatus.outsideAirTemperature = status.outsideAirTemperature;
}
```

**Reasoning:** The condition `if (status != this->currentStatus) { return; }` means:
- Returns early (does nothing) when status HAS changed
- Continues to update when status is THE SAME

This is the opposite of intended behavior. The code should skip updating when nothing changed, and apply updates when there ARE changes.

**Impact:** Status updates are never applied - the display always shows stale/initial data.

**Fix:** Change to `if (status == this->currentStatus) { return; }`

---

### M11. Recently Fixed: getOffsetDirection nullptr guard

**Location:** `components/mitsubishi_heatpump/cn105_controlflow.cpp` lines 317-320

**Status:** FIXED in commit ccd3315 - shows the pattern for proper null checking.

```cpp
bool CN105ControlFlow::getOffsetDirection() {
    if (this->hpState_->getCurrentSettings().mode == nullptr) {
        return false;
    }
    // ...
}
```

---

## Fix Options by Category

### Category 1: Null Pointer Dereferences

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| A (Recommended) | Individual null checks before each dereference | Simple, explicit, easy to understand | Verbose, repetitive |
| B | Macro-based null check (`SAFE_PUBLISH(sensor, value)`) | Concise, reduces duplication | Harder to debug |
| C | Optional/smart pointer wrapper | Modern C++, compile-time safety | Significant refactoring |

---

### Category 2: Buffer Overflow / Array Out-of-Bounds

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| A (Recommended) | Add bounds checks before array access | Minimal change, handles edge case | Slightly changes return semantics |
| B | Require minimum elements in constructor | Eliminates edge case | Requires meaningful initial value |
| C | Use sentinel value (NaN) for no-data | Explicit no-data indication | Callers must check |

**MovingSlopeAverage fix:**
```cpp
float increment(float next) {
    const int n = this->data.size();
    if (n == 0) {
        this->data.push_back(next);
        return 0;  // No delta for first element
    }
    // ... rest of code
}
```

---

### Category 3: Uninitialized Variables

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| A (Recommended) | In-class initialization (`unsigned long lastSend = 0;`) | Clear, guaranteed, modern C++ | None |
| B | Constructor initialization list | Traditional C++ | Must update constructor |

---

### Category 4: Memory Allocation Failures

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| A (Recommended) | Check for nullptr and call `mark_failed()` | Graceful failure, diagnostics | Adds code to each allocation |
| B | Use `std::unique_ptr` | Modern C++, automatic cleanup | Requires ownership refactoring |

---

### Category 5: Race Conditions (ESP8266)

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| A (Recommended) | Use `std::atomic<bool>` | Proper atomicity, minimal change | Still not a true blocking mutex |
| B | Disable interrupts during critical section | Guaranteed atomicity | Affects all timing |
| C | FreeRTOS semaphore | Proper mutex semantics | Platform-dependent |

**Immediate fix for typo:**
```cpp
// Change line in cn105_controlflow.cpp:
this->wantedSettingsMutex = true;   // WRONG
this->wantedSettingsMutex = false;  // CORRECT
```

---

### Category 6: Timing Issues

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| A (Recommended) | Use signed arithmetic for wraparound | Correctly handles 32-bit wrap | Requires careful casting |
| B | Use 64-bit timestamps | Wraparound never happens | Increased memory, less efficient on 32-bit |

**Wraparound-safe check:**
```cpp
bool hasUpdateIntervalPassed() {
    int32_t elapsed = (int32_t)(CUSTOM_MILLIS - lastCompleteCycleMs);
    return elapsed > (int32_t)update_interval;
}
```

---

### Category 7: Logic Errors

**Debounce fix (direct):**
```cpp
// BEFORE (incorrect - skips when time HAS passed)
if (!(now - lastChange < debounce_delay_)) { return; }

// AFTER (correct - skips when time has NOT passed)
if ((now - lastChange) < debounce_delay_) { return; }
```

---

## Recommended Priority Order

### Phase 1: Critical Fixes (Immediate)
1. **C1** - MovingSlopeAverage empty vector access
2. **C3** - ESP8266 mutex typo (`true` -> `false`)
3. **C4** - Initialize timing variables
4. **H5** - Fix inverted debounce logic
5. **M9** - Fix missing break statements in switch cases
6. **M10** - Fix inverted status update logic

### Phase 2: High Priority (Next Sprint)
7. **C2** - Add bounds check in checkSum loop
8. **H1** - Add null checks in DeviceStateManager::publish()
9. **H2** - Add null parameter validation in workflow steps
10. **H6** - Fix CUSTOM_MILLIS wraparound handling

### Phase 3: Stability Improvements
11. **H3/H4** - Add memory allocation checks
12. **M1** - Initialize data pointer
13. **M3** - Handle null return from getWideVaneSetting
14. **M4** - Add logging when settings are discarded

### Phase 4: Code Quality
15. **M2** - Improve lookupByteMapValue default handling
16. **M5** - Consider reducing bootstrap timeout
17. **M6** - Use named constant for disabled timeout
18. **M7** - Use safe arithmetic for timeout calculation

---

## Critical Files Summary

| File | Bugs | Priority |
|------|------|----------|
| `floats.h` | C1, H4 | CRITICAL |
| `cn105_controlflow.cpp` | C3, H5, M4, M6, M11 | CRITICAL |
| `cn105_controlflow.h` | C3 | CRITICAL |
| `cn105_connection.h` | C4, M1 | CRITICAL |
| `cn105_connection.cpp` | C2, M5, M8 | HIGH |
| `cycle_management.cpp` | H6, M7 | HIGH |
| `devicestatemanager.cpp` | H1 | HIGH |
| `hysterisis_workflowstep.cpp` | H2 | HIGH |
| `pid_workflowstep.cpp` | H2 | HIGH |
| `espmhp.cpp` | H3, M9 | HIGH |
| `cn105_state.cpp` | M3, M10 | MEDIUM |
| `cn105_utils.cpp` | M2 | MEDIUM |
