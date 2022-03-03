#include <Arduino.h>
#include <Romi32U4.h>
#include <servo32u4.h>

class JawServo
{
private:
    Servo32U4 servo;
    const int linearPotPin = A0;
    const int stopOpen = 1600;
    const int stopClose = 1100;
    const int close = 1800;
    const int open = 1000;
    // const int delay = 540;  // ms
    const int openVADC = 220;
    const int closedVADC = 280;

    int prevVADC = 0;
    const int TOLERANCE = 0;

public:
    JawServo();

    float getPotVADC();

    void openNoPot();
    void closeNoPot();
    boolean openJaw();
    boolean closeJaw();
    boolean getMoving();
    void stopServoOpen();
    void stopServoClose();
};