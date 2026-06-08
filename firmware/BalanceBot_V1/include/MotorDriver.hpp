#include <Arduino.h>

class MotorDriver
{
public:
    void init(); 
    void setLeftPWM(int);
    void setRightPWM(int);

private:

    int enA = 5;
    int motor1_input1 = 6;
    int motor1_input2 = 7;

    const int motor2_input1 = 4;
    const int motor2_input2 = 10;
    const int enB = 11;


};