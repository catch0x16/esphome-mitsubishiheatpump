#include <iostream>
#include <vector>
#include <stdexcept>

using namespace std;

#include "../floats.h"

void roundToDecimals_tests() {
    std::cout<<devicestate::roundToDecimals(0, 0)<<"==0\n";
    std::cout<<devicestate::roundToDecimals(2, 0)<<"==2\n";

    std::cout<<devicestate::roundToDecimals(1.555, 2)<<"==1.56\n";
    std::cout<<devicestate::roundToDecimals(1.5550, 2)<<"==1.56\n";
    std::cout<<devicestate::roundToDecimals(1.5551, 2)<<"==1.56\n";
    std::cout<<devicestate::roundToDecimals(1.3555, 3)<<"==1.356\n";
}

void testSameFloat() {
  std::cout<<devicestate::same_float(21.100000, 21.308987, 0.001f)<<"==1";
}

double calculateDelta(double y1, double y2) {
  return y2 - y1;
}

void testSlope() {
  vector<pair<double, double>> dataPoints = {
    {1, 2},
    {2, 6},
    {3, 7},
    {4, 7},
    {5, 7},
    {6, 6},
    {7, 6},
    {8, 8},
    {9, 7},
    {10, 5},
    {11, 5},
    {12, 6}
  };
  
  int window = 6;

  int n = 0;
  float deltaSum = 0;
  // Calculate and print slopes between consecutive points
  for (size_t i = 0; i < dataPoints.size() - 1; ++i) {
    double delta = calculateDelta(dataPoints[i].second, dataPoints[i + 1].second);

    if (n < window) {
      n++;
    } else {
      int lastI = i - n;
      double lastDelta = calculateDelta(dataPoints[lastI].second, dataPoints[lastI + 1].second);
      deltaSum -= lastDelta;
    }
    deltaSum += delta;

    cout << "Slope between (" << dataPoints[i].first << ", " << dataPoints[i].second << ") and ("
            << dataPoints[i + 1].first << ", " << dataPoints[i + 1].second << ") is: " << delta << " with avg: " << deltaSum / n << endl;
  }
}

void testMovingAverageSlope() {
  vector<pair<double, double>> dataPoints = {
    {1, 2},
    {2, 6},
    {3, 7},
    {4, 7},
    {5, 7},
    {6, 6},
    {7, 6},
    {8, 8},
    {9, 7},
    {10, 5},
    {11, 5},
    {12, 6}
  };

  devicestate::MovingSlopeAverage* msa = new devicestate::MovingSlopeAverage(6);

  for (size_t i = 0; i < dataPoints.size() - 1; ++i) {
    const float delta = msa->increment(dataPoints[i].second);
    const float average = msa->getAverage();

    cout << "Slope between (" << dataPoints[i].first << ", " << dataPoints[i].second << ") and ("
      << dataPoints[i + 1].first << ", " << dataPoints[i + 1].second << ") is: " << delta << " with avg: " << average << endl;
  }

}

int main() {
  testMovingAverageSlope();
}