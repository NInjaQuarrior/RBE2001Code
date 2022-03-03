#pragma once

#ifndef BLUEMOTOR_H
#define BLUEMOTOR_H
class BlueMotor
{
public:
    BlueMotor();
    void resetEncoder();
    void setEffort(int effort);

    void setEffortNoD(int effort);
    boolean moveTo(int position);
    long getPosition();
    void holdTo(long position);
    void setup();

    const int Side25Place = -9000;
    const int Side25Prep = -8000;
    const int Side25Travel = 0;

    const int Side45Place = -5000;
    const int Side45Prep = -8500;
    const int Side45Travel = 0;

    const int platPlace = 150;

    const int platPlaceLeft = -600;

    void setEffort(int effort, bool clockwise);

public:
    static void isr();
    void setCount(float countNew);

    const int PWMOutPin = 11;
    const int AIN2 = 4;
    const int AIN1 = 13;
    const int ENCA = 0;
    const int ENCB = 1;
    const long DEADBAND = 30;
    const int DEAD_ZONE = 100; // TODO
    const float kP = 3.5f;
    const float kPDown = 1.5f;
};
#endif