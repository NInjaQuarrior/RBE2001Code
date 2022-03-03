
#include <MyDrive.h>

//#define PI 3.14159265358979323846
Rangefinder rangefinder(17, 12);
MyDrive::MyDrive()
{
}

void MyDrive::init()
{
    rangefinder.init();
}

/**
 * turns a certain amount of degrees
 * @param degrees degrees to turn, negative to turn counter-clockwise
 * @param speed degrees per second to move
 * @return true when complete
 */
boolean MyDrive::turn(float degrees, float speed)
{

    chassis.turnFor(degrees, speed, true); // change block based on requirments

    return true;
}

/**
 * turn until not called
 * @param direct -1 for left, 1 for right
 * @param speed in cm/s
 */
void MyDrive::turnContinuous(int direct, float speed)
{
    if (direct <= 0)
    {
        chassis.setMotorEfforts(-speed, speed);
        // left.setMotorEffort(-effort);
        //  right.setMotorEffort(effort);
    }
    else if (direct > 0)
    {
        chassis.setMotorEfforts(speed, -speed);
        // left.setMotorEffort(effort);
        //   right.setMotorEffort(-effort);
    }
}

/**
 * drive straight a certain amount of inches
 * @param inches inches to move, negative to go backwars
 * @param speed degrees per second to move
 * @return true when complete
 */
boolean MyDrive::driveInches(float inches, float speed)
{
    // TODO
    // degrees for each wheel to move
    // float moveDegrees = (inches / (2 * PI * (WHEEL_DIAMETER / 2))) * 360;

    chassis.driveFor(inches * CENTI_CONV, speed, true);
    // move
    //  left.startMoveFor(moveDegrees, speed);
    //  right.moveFor(moveDegrees, speed);

    return true;
}

/**
 * drive straight a certain amount of inches
 * @param centi centimeters to move, negative to go backwars
 * @param speed degrees per second to move
 * @return true when complete
 */
boolean MyDrive::driveCentimeters(float centi, float speed)
{
    // degrees for each wheel to move
    // float moveDegrees = (centi / (2 * PI * ((WHEEL_DIAMETER / CENTI_CONV) / 2))) * 360;

    chassis.driveFor(centi, speed, true);
    // move
    // left.startMoveFor(moveDegrees, speed);
    //  right.moveFor(moveDegrees, speed);

    return true;
}

/**
 * drive based on effort
 * @param effort -1  to 1
 */
void MyDrive::setEffort(float effort)
{

    chassis.setMotorEfforts(effort, effort);
    //  left.setEffort(effort);
    //  right.setEffort(effort);
}

/**
 * drive based on effort
 * @param speed in degrees per second, negative goes backwards
 * TODO
 */
void MyDrive::setSpeed(float speed)
{

    chassis.setMotorEfforts(speed, speed);
    // left.setSpeed(speed);
    //  right.setSpeed(speed);
}

/**
 * drives to a set distance away from a target using the ultrasonic
 * @param targetDist distance to move to
 * @param curDist the ultrasonic same unit as targetDist
 * @return true when at proper distance
 */
boolean MyDrive::driveTo(float targetDist, float curDist)
{
    // if the robot is at the distance within a deadband
    if (curDist > targetDist - ULTRA_DEAD && curDist < targetDist + ULTRA_DEAD)
    {
        // stop
        setEffort(0);
        return true;
    }

    // move the robot to right distance
    setSpeed(ULTRA_DRIVE);

    return false;
}

/**
 * follows the black line using p control
 * @param error the currect difference between the two line sensors getDifference()
 * @param leftSense the current value of left Sensor
 * @param rightSense current value of the right sensor
 */
void MyDrive::followLine(float error)
{

    float leftEffort = LINE_BASE_SPEED - (error * LINE_PROP);
    float rightEffort = LINE_BASE_SPEED + (error * LINE_PROP);

    chassis.setMotorEfforts(leftEffort, rightEffort);
    // left.setEffort(LINE_BASE_SPEED + (error * LINE_PROP));
    // right.setEffort(LINE_BASE_SPEED - (error * LINE_PROP));
}

/**
 * drive straight forward until find a line
 * @param speed the speed in degrees per second
 * @param leftSense the current value of left Sensor
 * @param rightSense current value of the right sensor
 * @return true when hits an line
 */
boolean MyDrive::driveTillLine(float speed, float leftSense, float rightSense)
{
    // if either light sensors og above the dark value
    if (leftSense > LINE_SENSE_BLACK || rightSense > LINE_SENSE_BLACK /*&& error < lineFollowTurnDead*/)
    {
        setEffort(0);
        return true;
    }
    // keep driving

    chassis.setTwist(10, 0);
    return false;
}

/**
 * line follow until find a t intersection
 * @param speed the speed in degrees per second
 * @param leftSense the current value of left Sensor
 * @param rightSense current value of the right sensor
 * @return true when hits an line
 */
boolean MyDrive::lineFollowTillLine(float leftSense, float rightSense, float error)
{
    // if either light sensors og above the dark value
    if (leftSense > LINE_SENSE_BLACK && rightSense > LINE_SENSE_BLACK /*&& error < lineFollowTurnDead*/)
    {
        return true;
    }
    // keep following line
    followLine(error);
    return false;
}

/**
 * line follow until ultra reaches target distance, only goes forward
 * @param leftSense the current value of left Sensor
 * @param rightSense current value of the right sensor
 * @param error difference between left and right sensors
 * @param curDist current ultrasonic distance
 * @param targetDist target distance, same unit as curDist
 * @return true when at target distance
 */
boolean MyDrive::lineFollowToTargetDistance(float error, float curDist, float targetDist)
{
    // if not in target distance
    // Serial.println(curDist);
    // Serial.println(rangefinder.getDistance());
    if (rangefinder.getDistance() <= targetDist)
    {
        setEffort(0);
        return true;
    }
    else
    {
        followLine(error);
        return false;
    }
}

boolean turned = false;
/**
 * turn until find a line
 *
 * @param direct -1 for left, 1 for right
 * @param leftSense the current value of left Sensor
 * @param rightSense current value of the right sensor
 * @return true found a line
 */
boolean MyDrive::alignToLine(int direct, float leftSense, float rightSense)
{
    // if (turn(direct * PREP_ALIGN_ANGLE, TURN_SPEED_MED))
    // {
    //     turned = true;
    // }

    // turn left
    if (direct < 0)
    {

        if (rightSense > LINE_SENSE_BLACK)
        {
            chassis.setMotorEfforts(0, 0);
            // left.setSpeed(0);
            // right.setSpeed(0);
            return true;
        }
    }
    // turn right
    else if (direct >= 0)
    {
        // left line sensor found line

        if (leftSense > LINE_SENSE_BLACK)
        {
            chassis.setMotorEfforts(0, 0);
            return true;
        }
    }
    // keep turning
    turnContinuous(direct, TURN_SPEED_SLOW);
    return false;
}
/**
 * @brief
 *
 * @param side true if right
 * @return boolean moved to pickup the panel
 */
boolean MyDrive::movePanelPickUp(boolean side, float curDist, float leftSense, float rightSense, float error)
{
    switch (movePanelState)
    {
    case INIT_TURN:
        if (side == true)
        {
            turn(90, TURN_SPEED_MED);
            movePanelState = GO_ROOF;
            rangefinder.getDistance();
            delay(400); // TODO test speds
        }
        else
        {
            turn(-90, TURN_SPEED_MED);
            movePanelState = GO_ROOF;
            rangefinder.getDistance();
            delay(400);
        }

        break;
    case GO_ROOF:
        if (side == 1)
        {
            if (lineFollowToTargetDistance(error, NULL, DIST_FROM_ROOF_RIGHT))
            {
                movePanelState = MOVE_PANEL;
            }
        }
        else
        {
            if (lineFollowToTargetDistance(error, NULL, DIST_FROM_ROOF_LEFT))
            {
                movePanelState = MOVE_PANEL;
            }
        }
        //  delay(100);
        break;
    case MOVE_PANEL:
        setEffort(0);
        // movePanelState = INIT_TURN;
        return true;

        //  delay(100);
        break;
    }

    return false;
}
