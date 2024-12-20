#include <iostream>

#include "../pidcontroller.h"
#include "../floats.h"

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
    float p = 0.1;
    float i = 0.0006;
    float d = 0.0;
    float target = 21;
    PIDController *pid = new PIDController(
      p,
      i,
      d,
      2000,
      target,
      target - 2.0,
      target + 2.0
    );

    float epsilon = 0.017;
    float current = 20;
    for (int i = 0; i<1; i++) {
      const float adjustment = pid->update(current);
      std::cout<<i<<": current={"<<current<<"} target={"<<target<<"} adjusted={"<<adjustment<<"}\n";
      if ((current < (target + 1)) && ((i >= 66 ||  i <= 100) || (i >= 155 || i <= 275) || (i >= 400 || i <= 489))) {
        current += epsilon;
      }

      if ((current > target) && ((i >= 666 || i <= 700) || (i >= 755 || i <= 805) || (i >= 900 || i <= 976))) {
        current -= epsilon;
      }
    }
}

void testSameFloat() {
  std::cout<<devicestate::same_float(21.100000, 21.308987, 0.001f)<<"==1";
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
    simulate();
  */
  testSameFloat();
}