#pragma once

struct cycleManagement {

    unsigned int update_interval;

    bool cycleRunning = false;
    unsigned long lastCycleStartMs = 0;
    unsigned long lastCompleteCycleMs = 0;

    void setUpdateInterval(unsigned int update_interval);

    void init();
    void cycleStarted();
    void cycleEnded(bool timedOut = false);
    bool hasUpdateIntervalPassed();
    bool doesCycleTimeOut();
    bool isCycleRunning();
    void deferCycle();
    void checkTimeout();

};