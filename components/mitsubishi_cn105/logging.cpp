#include "logging.h"

void log_info_uint32(const char* tag, const char* msg, uint32_t value, const char* suffix) {
#if __GNUC__ >= 11
    ESP_LOGI(tag, "%s %lu %s", msg, (unsigned long)value, suffix);
#else
    ESP_LOGI(tag, "%s %u %s", msg, (unsigned int)value, suffix);
#endif
}

void log_debug_uint32(const char* tag, const char* msg, uint32_t value, const char* suffix) {
#if __GNUC__ >= 11
    ESP_LOGD(tag, "%s %lu %s", msg, (unsigned long)value, suffix);
#else
    ESP_LOGD(tag, "%s %u %s", msg, (unsigned int)value, suffix);
#endif
}
