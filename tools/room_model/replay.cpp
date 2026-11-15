// Replay recorded operation through the firmware's own response-rate estimator
// and print the PI gains it arrives at. Use it to pick starting values for a
// room, or to check a change to the estimator against real data.
//
//   c++ -std=c++17 -O2 -Icomponents/workflow_climate -o /tmp/replay tools/room_model/replay.cpp
//   /tmp/replay hailee=hailee.csv primary=primary.csv main=main.csv
//
// Each CSV has a header row and one row per minute with at least the columns
// `temperature` (the room temperature the controller uses), `power` (1 while
// the workflow has the device switched on), `operating` (1 while the device is
// actively heating) and `valid` (True/False). Recordings are treated as heating.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "response_rate_estimator.h"

using workflow::pid::Gains;
using workflow::pid::ResponseRateEstimator;
using workflow::pid::TuningConfig;

static std::vector<std::string> split(const std::string &line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) fields.push_back(field);
  if (!line.empty() && line.back() == ',') fields.emplace_back();
  return fields;
}

static float number(const std::string &text) { return text.empty() ? NAN : std::strtof(text.c_str(), nullptr); }

int main(int argc, char **argv) {
  const TuningConfig config;
  for (int arg = 1; arg < argc; arg++) {
    const std::string spec = argv[arg];
    const size_t eq = spec.find('=');
    if (eq == std::string::npos) {
      std::fprintf(stderr, "expected ROOM=file.csv, got %s\n", argv[arg]);
      return EXIT_FAILURE;
    }
    std::ifstream file(spec.substr(eq + 1));
    std::string line;
    if (!std::getline(file, line)) {
      std::fprintf(stderr, "cannot read %s\n", spec.substr(eq + 1).c_str());
      return EXIT_FAILURE;
    }
    int temperature_col = -1, power_col = -1, operating_col = -1, valid_col = -1;
    const std::vector<std::string> header = split(line);
    for (size_t i = 0; i < header.size(); i++) {
      if (header[i] == "temperature") temperature_col = i;
      if (header[i] == "power") power_col = i;
      if (header[i] == "operating") operating_col = i;
      if (header[i] == "valid") valid_col = i;
    }
    if (temperature_col < 0 || power_col < 0 || operating_col < 0 || valid_col < 0) {
      std::fprintf(stderr, "%s: missing a required column\n", spec.c_str());
      return EXIT_FAILURE;
    }

    ResponseRateEstimator estimator;
    uint32_t now = 0;
    uint32_t minutes = 0, operating_minutes = 0, last_count = 0;
    float lowest = INFINITY, highest = 0.0f;
    std::printf("== %s\n", spec.substr(0, eq).c_str());
    while (std::getline(file, line)) {
      const std::vector<std::string> row = split(line);
      now += 60000;
      if (row.size() <= static_cast<size_t>(valid_col) || row[valid_col] != "True") {
        estimator.update(NAN, false, true, now);  // a hole in the recording
        continue;
      }
      const float temperature = number(row[temperature_col]);
      const bool operating = number(row[power_col]) > 0.5f && number(row[operating_col]) > 0.5f;
      minutes++;
      operating_minutes += operating ? 1 : 0;
      estimator.update(temperature, operating, true, now);
      if (estimator.measurements() != last_count) {
        last_count = estimator.measurements();
        const float rate = estimator.rate(true) * 3600.0f;
        if (last_count > 10) {  // after the starting value has washed out
          lowest = std::fmin(lowest, rate);
          highest = std::fmax(highest, rate);
        }
        if (last_count <= 3 || last_count % 20 == 0) {
          const Gains gains = workflow::pid::gains_from_rate(estimator.rate(true), config);
          std::printf("   day %5.1f  measurement %3u  rate %.2f C/h  kp %.2f  ki %.5f\n", now / 86400000.0f, last_count,
                      rate, gains.kp, gains.ki);
        }
      }
    }
    const Gains gains = workflow::pid::gains_from_rate(estimator.rate(true), config);
    std::printf("   %u valid minutes, %u operating, %u measurements\n", minutes, operating_minutes,
                estimator.measurements());
    std::printf("   learned rate ranged %.2f..%.2f C/h after the first 10 measurements\n", lowest, highest);
    std::printf("   final: rate %.2f C/h  ->  kp: %.2f  ki: %.5f  kd: 0.0\n", estimator.rate(true) * 3600.0f, gains.kp,
                gains.ki);
  }
  return EXIT_SUCCESS;
}
