#include <Arduino.h>
#include <BlueMotor.h>
#include <Romi32U4.h>

int oldValue = 0;
int newValue;
int count = 0;
int errorCount = 0;
unsigned time = 0;

const char error = 5;

// array for using quad encoding
char encoderArray[4][4] = {
    {0, -1, 1, error},
    {1, 0, error, -1},
    {-1, error, 0, 1},
    {error, 1, -1, 0}};

BlueMotor::BlueMotor()
{
}

/**
 * @brief sets up the encoder. MUST run before using the blue motor
 *
 */
void BlueMotor::setup()
{
    pinMode(PWMOutPin, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);
    TCCR1A = 0xA8; // 0b10101000; //gcl: added OCR1C for adding a third PWM on pin 11
    TCCR1B = 0x11; // 0b00010001;
    ICR1 = 400;
    OCR1C = 0;

    attachInterrupt(digitalPinToInterrupt(ENCA), isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCB), isr, CHANGE);
    resetEncoder();
}

/**
 * @brief resets encoder value to 0
 *
 */
void BlueMotor::resetEncoder()
{
    count = 0;
}

void BlueMotor::setCount(float countNew)
{
    count = countNew;
}

/**
 * @brief gets current encoder valuie
 *
 * @return long encoder count
 */
long BlueMotor::getPosition()
{
    return count;
}

/**
 * @brief Updates the encoder values
 *
 */
void BlueMotor::isr()
{

    newValue = (digitalRead(1) << 1) | digitalRead(0);
    char value = encoderArray[oldValue][newValue];
    if (value == error)
    {
        errorCount++;
    }
    else
    {
        count -= value;
    }
    oldValue = newValue;
}

/**
 * @briefsets blue motor effort
 *
 * @param effort NOTE: not out of 1. values below 100 dont really move the motor. Possible out of 540ish
 * use negative to go backwards
 */
void BlueMotor::setEffort(int effort)
{
    if (effort < 0)
    {
        setEffort(-effort, true);
    }
    else
    {
        setEffort(effort, false);
    }
}

void BlueMotor::setEffortNoD(int effort)
{
    if (effort < 0)
    {
        effort = -effort;
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    }
    else
    {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    }
    effort = ((effort / 400) * (400 - DEAD_ZONE)) + DEAD_ZONE;
    OCR1C = constrain(effort, 0, 400);
}

/**
 * @brief Sets blue motor effort
 *
 * @param effort NOTE: not out of 1. values below 100 dont really move the motor.
 * @param clockwise direction of the motor
 */
void BlueMotor::setEffort(int effort, bool clockwise)
{
    if (clockwise)
    {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    }
    else
    {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    }

    OCR1C = constrain(effort, 0, 400);
}

/**
 * @brief gives current spinning rpm of the motor
 *
 * @return float rpm
 */
float getRPM()
{
    return ((newValue - oldValue) / 270) / (time / 1000 / 60);
}

/**
 * @brief moves the blue motor to a specific encoder distance then stops running the motor
 *
 * @param target idk rly know yet. maybe 360 for full rotation //TODO
 * @return true if reached with a deadband of the target
 */
boolean BlueMotor::moveTo(int target)
{

  //  Serial.println(getPosition());
    if (target < 0)
    {
        setEffortNoD((getPosition() - target) * kP);
    }
    else
    {
        setEffortNoD((getPosition() - target) * kPDown);
    }
    if ((getPosition() - target) > -DEADBAND && (getPosition() - target) < DEADBAND)
    {
        setEffort(0);
        return true;
    }
    return false;
}

/**
 * @brief moves the motor to a specific encoder distance and continues supplying voltage to hold it there
 *
 * @param target maybe 360 for one rotation //TODO
 */
void BlueMotor::holdTo(long target)
{

    setEffort((getPosition() - target) * kP);
}
