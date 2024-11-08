#include <iostream>

#include "../pidcontroller.h"

void roundToDecimals_tests() {
    std::cout<<roundToDecimals(0, 0)<<"==0\n";
    std::cout<<roundToDecimals(2, 0)<<"==2\n";

    std::cout<<roundToDecimals(1.555, 2)<<"==1.56\n";
    std::cout<<roundToDecimals(1.5550, 2)<<"==1.56\n";
    std::cout<<roundToDecimals(1.5551, 2)<<"==1.56\n";
    std::cout<<roundToDecimals(1.3555, 3)<<"==1.356\n";
}

void handlesDerivative() {
    PIDController *pid = new PIDController(
      0, 0, 2, 1000, 10, -255, 100
    );

    std::cout<<pid->update(0)<<"==0\n";
    std::cout<<pid->update(5)<<"==-10\n";
}

void handlesIntegral() {
    PIDController *pid = new PIDController(
      0, 2, 0, 1000, 10, -255, 100
    );

    std::cout<<pid->update(0)<<"==20\n";
    std::cout<<pid->update(0)<<"==40\n";
    std::cout<<pid->update(5)<<"==50\n";
}

void handlesNegativeIntegral() {
    PIDController *pid = new PIDController(
      0, 2, 0, 1000, -10, -255, 100
    );

    std::cout<<pid->update(0)<<"==-20\n";
    std::cout<<pid->update(0)<<"==-40\n";
}

void calculateMultipleUpdates() {
    PIDController *pid = new PIDController(
      1, 0.1, 1, 1000, 10, -255, 100
    );

    std::cout<<pid->update(0)<<"==11\n";
    std::cout<<pid->update(5)<<"==1.5\n";
    std::cout<<pid->update(11)<<"==-5.6\n";
    std::cout<<pid->update(10)<<"==2.4\n";
}

void simulate() {
    float p = 0.5;
    float i = 0.02;
    float d = 0.01;
    PIDController *pid = new PIDController(
      p,
      i,
      d,
      2000,
      20,
      16,
      31
    );

    float epsilon = 0.051;
    float current = 17;
    for (int i = 0; i<100; i++) {
      std::cout<<i<<": "<<current<<" "<<pid->update(current)<<"\n";
      current += epsilon;
    }
}

int main() {
  /*
    roundToDecimals_tests();
    std::cout<<"\n";
    handlesDerivative();
    std::cout<<"\n";
    handlesIntegral();
    std::cout<<"\n";
    handlesNegativeIntegral();
    std::cout<<"\n";
    calculateMultipleUpdates();
  */
 simulate();
}