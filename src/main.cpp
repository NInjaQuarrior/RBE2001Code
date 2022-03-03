#include <Arduino.h>
#include <wpi-32u4-lib.h>
#include <RemoteConstants.h>
#include <IRdecoder.h>
#include <MyDrive.h>
#include <LineSensor.h>
#include <BlueMotor.h>

#include <JawServo.h>

// start Object declarations ++++++++++++++++++++++++++++++++++++++++++++++++
Chassis chassis;

MyDrive drive;

LineSensor lSense;

BlueMotor motor;

IRDecoder decoder(14);

// MyUltraSonic ultra;

JawServo servo;

Romi32U4ButtonB buttonB;

// end Object declarations ===================================================================

// START sensor value variables +++++++++++++++++++++++++++++++++++++++++++++

uint16_t keyPress;
float leftSense;
float rightSense;
float error;

// ultrasonic
// float curDistIN;
// float curDistCM;

// end sensor values ===================================================================

// Start function variables +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++==
boolean allowRun = false;

// end function variables ===================================================================

// START constants +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
const uint16_t ESTOP = remoteUp;           // TODO
const uint16_t ESTOP_RESTART = remoteDown; // TODO
const uint16_t STOP_BUTTON = remotePlayPause;
// TODO
const uint16_t S_RIGHT_FIRST = remote1;
const uint16_t S_RIGHT_SECOND = remote2;
const uint16_t S_LEFT_FIRST = remote4;
const uint16_t S_LEFT_SECOND = remote5;

// end constants ===================================================================

// START enums +++++++++++++++++++++++++++++++++++++++++++++++++++++

// TODO check sides
/**
 * @brief side of the field
 * Right = 25 degrees
 * Left = 45 degrees
 *
 */
enum RobotSide
{
  LEFT,
  RIGHT,
  BRUH_WHY_CANT_I_SET_ENUM_TO_NULL_THE_FIRST
};

RobotSide robotSide = BRUH_WHY_CANT_I_SET_ENUM_TO_NULL_THE_FIRST;

enum RobotRun
{
  FIRST_ROBOT,
  SECOND_ROBOT,
  BRUH_WHY_CANT_I_SET_ENUM_TO_NULL
};

RobotRun robotRun = BRUH_WHY_CANT_I_SET_ENUM_TO_NULL;

enum RunState
{
  HOW_PICKUP,
  PICKUP_ROOF,
  MOVE_ROOF,
  PLACE_PLAT,
  PICKUP_PLAT,
  PLACE_ROOF,
  CROSS_SIDE

};

RunState runState = HOW_PICKUP;

enum PickRoofState
{
  WAIT,
  CLOSE_GRIP,
  REMOVE_PANEL,
  BACKUP,
  TURN_AROUND,
  DRIVE_INTER

};

PickRoofState pickRoofState = WAIT;

enum PlacePlatState
{
  CENTER,
  TURN_1,
  MOVE_PLAT,
  MOVE_PLAT_CHECK,
  MOVE_GRIP,
  OPEN_JAW
};

PlacePlatState placePlatState = CENTER;

enum PickPlatState
{
  WAIT_AND_CLOSE_PP,
  CLOSE_GRIP_PP,
  BACK_UP_PP,
  TURN_AROUND_PP,
  MOVE_TO_SECT_PP,
  CENTER_ROBOT_PP,
  TURN_ROOF_PP,
  MOVE_START_PP,
  MOVE_FORWARD_PP,
  FINISH_GRIP_MOVE_PP
};
PickPlatState pickPlatState = WAIT_AND_CLOSE_PP;

enum PlaceRoofState
{
  PREP_PANEL_PR,
  PLACE_PANEL_PR,
  PLACE_PANEL_PR_DOS,
  RELEASE_PR,
  BACKUP_PR_DOS,
  BACKUP_PR_ONE,
  TURN_PR
};

PlaceRoofState placeRoofState = PREP_PANEL_PR;

enum CrossSideState
{
  DRIVE_ONE_CC,
  TURN_ONE_CC,
  DRIVE_TAPE_CC,
  CENTER_CC,
  TURN_TWOP_CC,
  DRIVE_SECT_CC,
  CENTER_DOS_CC
};
CrossSideState crossSideState = DRIVE_ONE_CC;
// end enums ===================================================================

/**
 * @brief setup code for initilizing stuffs
 *
 */
void setup()
{
  drive.init();
  chassis.init();
  motor.setup();
  decoder.init();
  // ultra.initI();

  Serial.begin(9600);
}

/**
 * @brief updates sensor values to local variables, should be constantly called in the loop() function
 *
 */
void updateValues()
{
  //  curDistCM = ultra.getDistanceCM();
  keyPress = decoder.getKeyCode();
  leftSense = lSense.getLeft();
  rightSense = lSense.getRight();
  error = lSense.getDifference();

  // ultrasonic
  // curDistIN = ultra.getDistanceIN();
}

/**
 * @brief starting with the gripper opon on the panel pick up the panel
 * and drive to the intersection
 *
 * @param RobotSide
 * @return true if ran throught the state machine
 */
boolean pickUpPanelRoof(RobotSide s)
{
  switch (pickRoofState)
  {
  case WAIT:
    // flash the led //TODO
    // pickRoofState = CLOSE_GRIP;  // TODO
    if (keyPress == STOP_BUTTON) // TODO
    {
      pickRoofState = CLOSE_GRIP;
    }

    break;
  case CLOSE_GRIP:
    if (servo.closeJaw())
    {
      pickRoofState = REMOVE_PANEL;
      if (s == RIGHT)
      {
        motor.setCount(-9000);
        // Serial.println(motor.getPosition());
      }
      else
      {
        motor.setCount(motor.Side45Place);
      }
    }
    break;
  case REMOVE_PANEL:
    if (s == RIGHT)
    {
      // motor.setCount(motor.Side25Place);
      if (motor.moveTo(motor.Side25Prep))
      {
        // delay(100);
        pickRoofState = BACKUP;
      }
    }
    else
    {
      if (motor.moveTo(motor.Side45Prep))
      {
        // delay(100);
        pickRoofState = BACKUP;
      }
    }
    break;
  case BACKUP:
    if (drive.driveInches(-2.5, 5))
    {
      // delay(100);
      pickRoofState = TURN_AROUND;
    }
    break;
  case TURN_AROUND:
    if (drive.alignToLine(-1, leftSense, rightSense))
    {
      delay(100);
      pickRoofState = DRIVE_INTER;
    }
    break;
  case DRIVE_INTER:
    if (drive.lineFollowTillLine(leftSense, rightSense, error))
    {
      pickRoofState = WAIT;
      return true;
    }
    break;
  }
  return false;
}

boolean placePlat(RobotSide s)
{
  switch (placePlatState)
  {
  case CENTER:
    if (drive.driveInches(drive.CENTER_ROBOT_DIST, drive.DRIVE_SPEED_MED))
    {
      // delay(100);
      placePlatState = TURN_1;
    }
    break;
  case TURN_1:
    if (s == RIGHT)
    {
      if (drive.turn(-80, drive.TURN_SPEED_SLOW))
      {
        // delay(100);
        placePlatState = MOVE_PLAT;
        delay(100);
        // ultra.getDistanceCM();
      }
    }
    else
    {
      if (drive.turn(80, drive.TURN_SPEED_SLOW))
      {
        // delay(100);
        placePlatState = MOVE_PLAT;
        delay(100);
        // ultra.getDistanceCM();
      }
    }

    break;
  case MOVE_PLAT:
    // Serial.println("moving plat");
    if (drive.lineFollowToTargetDistance(error, NULL, 5)) // TODO make constant once tested
    {
      // delay(100);
      placePlatState = MOVE_PLAT_CHECK;
      delay(300);
    }
    break;
  case MOVE_PLAT_CHECK:
    // Serial.println("moving plat");
    if (drive.lineFollowToTargetDistance(error, NULL, 5)) // TODO make constant once tested
    {
      // delay(100);
      placePlatState = MOVE_GRIP;
    }
    break;
  case MOVE_GRIP:

    if (s == RIGHT)
    {
      if (motor.moveTo(motor.platPlace))
      {
        // delay(100);
        placePlatState = OPEN_JAW;
      }
    }
    else
    {
      if (motor.moveTo(motor.platPlaceLeft))
      {
        // delay(100);
        placePlatState = OPEN_JAW;
      }
    }
    break;
  case OPEN_JAW:
    if (servo.openJaw())
    {
      return true;
    }
    break;
  }
  return false;
}

boolean pickUpPlat(RobotSide s)
{
  switch (pickPlatState)
  {
  case WAIT_AND_CLOSE_PP:
    // TODO LED
    if (keyPress == STOP_BUTTON) // TODO
    {
      pickPlatState = CLOSE_GRIP_PP;
    }
    // pickPlatState = CLOSE_GRIP_PP;
    break;
  case CLOSE_GRIP_PP:
    if (servo.closeJaw())
    {

      pickPlatState = MOVE_START_PP;
    }
    break;
  case MOVE_START_PP:
    if (s == RIGHT)
    {
      if (motor.moveTo(motor.Side25Prep))
      {
        // delay(100);
        pickPlatState = BACK_UP_PP;
      }
    }
    else
    {
      if (motor.moveTo(motor.Side45Prep))
      {
        // delay(100);
        pickPlatState = BACK_UP_PP;
      }
    }
    break;
  case BACK_UP_PP:
    if (drive.driveInches(-2.5, drive.DRIVE_SPEED_MED))
    {
      // delay(100);
      pickPlatState = TURN_AROUND_PP;
    }
    break;
  case TURN_AROUND_PP:
    if (drive.alignToLine(-1, leftSense, rightSense))
    {
      // delay(100);
      pickPlatState = MOVE_TO_SECT_PP;
    }
    break;

  case MOVE_TO_SECT_PP:
    if (drive.lineFollowTillLine(leftSense, rightSense, error))
    {
      //  delay(100);
      pickPlatState = CENTER_ROBOT_PP;
    }
    break;
  case CENTER_ROBOT_PP:
    if (drive.driveInches(drive.CENTER_ROBOT_DIST, drive.DRIVE_SPEED_MED))
    {
      // delay(100);
      pickPlatState = TURN_ROOF_PP;
    }

    break;
  case TURN_ROOF_PP:
    if (s == RIGHT)
    {
      if (drive.turn(85, drive.TURN_SPEED_MED))
      {
        // delay(100);
        pickPlatState = MOVE_FORWARD_PP;
        delay(100);
      }
    }
    else
    {
      if (drive.turn(-85, drive.TURN_SPEED_MED))
      {
        // delay(500);
        pickPlatState = MOVE_FORWARD_PP;
        // ultra.getDistanceCM();
        delay(100);
      }
    }
    break;

  case MOVE_FORWARD_PP:
    if (drive.lineFollowToTargetDistance(error, NULL, 11))
    // TODO distance
    {
      pickPlatState = FINISH_GRIP_MOVE_PP;
      delay(300);
    }
    break;
  case FINISH_GRIP_MOVE_PP:
    return true;
    break;
  }
  return false;
}

boolean placeRoof(RobotSide s)
{

  switch (placeRoofState)
  {
  case PREP_PANEL_PR:

    if (s == RIGHT)
    {
      if (drive.lineFollowToTargetDistance(error, NULL, drive.DIST_FROM_ROOF_RIGHT))
      {
        placeRoofState = RELEASE_PR;
      }
    }
    else
    {
      if (drive.lineFollowToTargetDistance(error, NULL, 6))
      {
        placeRoofState = RELEASE_PR;
      }
    }
    break;
  case RELEASE_PR:
    if (servo.openJaw())
    {

      placeRoofState = PLACE_PANEL_PR;
    }
    break;
  case PLACE_PANEL_PR:
    if (s == RIGHT)
    {
      if (motor.moveTo(motor.Side25Place))
      {

        placeRoofState = BACKUP_PR_ONE;
      }
    }
    else
    {
      if (motor.moveTo(motor.Side45Place))
      {

        placeRoofState = BACKUP_PR_ONE;
      }
    }
    break;

  case BACKUP_PR_ONE:

    if (drive.driveInches(-4, drive.DRIVE_SPEED_MED))

    {

      placeRoofState = PLACE_PANEL_PR_DOS;
    }
    break;
  case PLACE_PANEL_PR_DOS:
    if (s == RIGHT)
    {
      if (motor.moveTo(motor.Side25Prep))
      {

        placeRoofState = BACKUP_PR_DOS;
      }
    }
    else
    {
      if (motor.moveTo(motor.Side45Prep))
      {

        placeRoofState = BACKUP_PR_DOS;
      }
    }
    break;
  case BACKUP_PR_DOS:

    if (drive.driveInches(-4, drive.DRIVE_SPEED_MED))

    {

      placeRoofState = TURN_PR;
    }
    break;
  case TURN_PR:
    if (s == RIGHT)
    {
      if (drive.turn(90, drive.TURN_SPEED_MED))
      {

        return true;
      }
    }
    else
    {
      if (drive.turn(-90, drive.TURN_SPEED_MED))
      {

        return true;
      }
    }

    break;
  }
  return false;
}

boolean crossSide(RobotSide s)
{

  switch (crossSideState)
  {
  case DRIVE_ONE_CC:
    if (drive.driveInches(8, drive.DRIVE_SPEED_MED))
    {
      crossSideState = TURN_ONE_CC;
    }
    break;
  case TURN_ONE_CC:
    if (s == RIGHT)
    {
      if (drive.turn(-90, drive.TURN_SPEED_MED))
      {
        crossSideState = DRIVE_TAPE_CC;
      }
    }
    else
    {
      if (drive.turn(90, drive.TURN_SPEED_MED))
      {
        crossSideState = DRIVE_TAPE_CC;
      }
    }
    break;
  case DRIVE_TAPE_CC:
    if (drive.driveTillLine(drive.DRIVE_SPEED_FAST, leftSense, rightSense))
    {
      crossSideState = CENTER_CC;
    }
    break;
  case CENTER_CC:
    if (drive.driveInches(drive.CENTER_ROBOT_DIST, drive.DRIVE_SPEED_MED))
    {
      crossSideState = TURN_TWOP_CC;
    }

    break;
  case TURN_TWOP_CC:
    if (s == RIGHT)
    {
      if (drive.turn(-90, drive.TURN_SPEED_MED))
      {
        crossSideState = DRIVE_SECT_CC;
      }
    }
    else
    {
      if (drive.turn(90, drive.TURN_SPEED_MED))
      {
        crossSideState = DRIVE_SECT_CC;
      }
    }

    break;
  case DRIVE_SECT_CC:
    if (drive.lineFollowTillLine(leftSense, rightSense, error))
    {
      crossSideState = CENTER_DOS_CC;
    }

    break;
  case CENTER_DOS_CC:
    if (drive.driveInches(drive.CENTER_ROBOT_DIST, drive.DRIVE_SPEED_MED))
    {
      return true;
    }

    break;
  }
  return false;
}

bool movedOnce = false;
boolean run()
{

  // starting near the intersection facing the roof
  switch (runState)
  {
  case HOW_PICKUP:
    // return to intersection facing away from roof

    if (robotRun == FIRST_ROBOT)
    {
      runState = PICKUP_ROOF;
    }
    else
    {
      runState = MOVE_ROOF;
    }
    break;
  case MOVE_ROOF:
    // if (robotSide == RIGHT && movedOnce == false)
    // {
    //   while (!motor.moveTo(-5000))
    //   {
    //     Serial.println(motor.getPosition());
    //   }
    //   motor.setCount(0);
    //   while (!motor.moveTo(-5000))
    //   {
    //     Serial.println(motor.getPosition());
    //   }
    //   movedOnce = true;
    // }
    // else
    // {
    //   while (!motor.moveTo(motor.Side45Place))
    //   {
    //   }
    //   movedOnce = true;
    //   // motor.moveTo();
    // }
    if (drive.movePanelPickUp(robotSide, NULL, leftSense, rightSense, error))
    {
      runState = PICKUP_ROOF;
    }
    break;
  case PICKUP_ROOF:
    if (pickUpPanelRoof(robotSide))
    {
      runState = PLACE_PLAT;
    }
    break;
  case PLACE_PLAT:
    if (placePlat(robotSide))
    {
      runState = PICKUP_PLAT;
    }

    break;
  case PICKUP_PLAT:
    if (pickUpPlat(robotSide))
    {
      runState = PLACE_ROOF;
    }
    break;
  case PLACE_ROOF:
    if (placeRoof(robotSide))
    {
      runState = CROSS_SIDE;
    }
    break;
  case CROSS_SIDE:

    if (crossSide(robotSide))
    {
      return true;
    }
    break;
  }

  return false;
}

void startRobot()
{
  if (keyPress == ESTOP_RESTART)
  {
    allowRun = true;
  }
  switch (keyPress)
  {
  case S_RIGHT_FIRST:
    robotSide = RIGHT;
    robotRun = FIRST_ROBOT;
    allowRun = true;
    break;
  case S_RIGHT_SECOND:
    robotSide = RIGHT;
    robotRun = SECOND_ROBOT;
    allowRun = true;
    break;
  case S_LEFT_FIRST:
    robotSide = LEFT;
    robotRun = FIRST_ROBOT;
    allowRun = true;
    break;
  case S_LEFT_SECOND:
    robotSide = LEFT;
    robotRun = SECOND_ROBOT;
    allowRun = true;
    break;
  }
}

void testLoop()
{
  motor.moveTo(-500);
}

boolean didTheThing = false;

boolean startYes = false;

void loop()
{

  // Serial.println(ultra.getDistanceCM());

  // drive.lineFollowToTargetDistance(error, curDistCM, 10);
  // drive.movePanelPickUp(1, curDistIN, leftSense, rightSense, error);
  // Serial.println(decoder.getCode());
  // Serial.println(rightSense);
  // delay(100);
  // 4261527296 play/pause
  // 4194680576 button below
  // 4127833856

  // if (buttonA.isPressed())
  // {
  //   Serial.println("down");
  //   motor.setEffort(-200);
  // }
  // if (buttonB.isPressed())
  // {
  //   Serial.println("up");
  //   motor.setEffort(200);
  // }

  // if (buttonC.isPressed())
  // {
  //   Serial.println("stop");
  //   motor.setEffort(0);
  // }
  // motor.moveTo(-9500);
  // Serial.println(motor.getPosition());
  //  pickUpPanelRoof(RIGHT);

  // robotRun = FIRST_ROBOT;
  // robotSide = LEFT;

  // if (buttonB.isPressed())
  // {

  //   startYes = true;
  // }

  // if (startYes == true)
  // {
  //   if (run())
  //   {
  //     startYes = false;
  //     drive.setEffort(500);
  //   }
  // }

  //  if (didTheThing == false && pickUpPlat(LEFT)){
  //    drive.setEffort(0);
  //    //didTheThing = true;
  //  }

  // if (didTheThing == true) {
  //   if(placePlat(RIGHT)){
  //     startYes = false;
  //     drive.setEffort(0);
  //   }

  //   allowRun = false;
  // }

  // run();
  updateValues();
  if (allowRun == false)
  {
    // Serial.println("start");
    startRobot();
  }
  while (allowRun == true)
  {
    // Serial.println("loop");
    updateValues();
    if (keyPress == ESTOP) // TODO
    {
      allowRun = false;
      drive.setEffort(0);
      motor.setEffort(0);
    }

    if (run())
    {
      allowRun = false;
    }
  }
}