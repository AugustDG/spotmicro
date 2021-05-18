#include <Adafruit_PWMServoDriver.h>
#include <HC_SR04.h>

#define SERVO_FREQ 50      // Analog servos run at ~50 Hz updates
#define PROCESSING_DELAY 0 // how much delay between loop calls (in ms)

// lengths of each leg segment
#define UPPER_SEG_LENGTH 10
#define LOWER_SEG_LENGTH 12
#define STEP_SIZE 25 // how small is each degree step (larger it is, the smoother, but slower it is)

// basic struct to hold 2d coordinates
struct Vector2D {
  float x, y;
};

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
HC_SR04 sonicR(10, 11, 0);
HC_SR04 sonicL(12, 13, 0);

bool finishedMoving = false; // the bot has finished moving
bool hasRoutine = false;     // is following a routine
uint8_t routineStepCount = 4;    // how long the current routine is
uint8_t routineStep = 0;         // where, in the current routine, are we at?
uint8_t routineLegCount = 2; //how many legs do we go through in the current routine?
uint8_t routineLeg = 0; // what leg, in the current routine, is mainly doing the action?
uint8_t routineCyclesCount = 0; //how many cycles of the selected routine should it do?
uint8_t routineCycle = 0; //where, in the number of routine cycles, are we?

// bounds of each servo's pwm range
// end, mid, shoulder;

// when every angle is at max
const int16_t maxPwmVals[] = {230, 330, 345, 
                          310, 165, 270,
                          300, 165, 325, 
                          235, 330, 260};

// when every angle is at min
const int16_t minPwmVals[] = {460, 135, 265, 
                          70,  340, 350,
                          70,  330, 245, 
                          465, 140, 340};

// upper bounds of each servo's degree range
const int16_t servoDegRanges[] = {120, 90, 10, 120, 90, 10,
                              120, 90, 10, 120, 90, 10};

// steps each servo will take at each loop iteration
float servoDegSteps[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

float desiredDegs[] = {0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0}; // holds desired degree values
float currentDegs[] = {
    0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0}; // holds the current degree values that each servo is set to

// holds the positions of each leg (4) in 2 different positions, mostly for
// testing purposes
struct Vector2D standingPos[] = {{-2, 12}, {-2, 12}, {-2, 12}, {-2, 12}};
struct Vector2D lyingPos[] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};

struct Vector2D *currentPos =
    new Vector2D[4]; // for easier testing, will remove later as dynamic memory
                     // on Arduino is limited
struct Vector2D tempPos[] = {
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0}}; // holds positions for each leg until everyone has been assigned

// holds each leg's position for the forward walk routine. 
// the difference with the positions array is that it ressembles a state machine for all legs: x(horizontal) is
// time and y(vertical) are the positions of each leg
struct Vector2D forwardRoutine[4][4] = {{{-3, 11}, {-3, 9}, {2, 9}, {2, 11}}, //front legs
                                        {{-3, 11}, {-3, 11}, {-3, 11}, {-3, 11}},
                                        {{-3, 13}, {-3, 9}, {2, 9}, {2, 13}}, //back legs
                                        {{-3, 13}, {-3, 13}, {-3, 13}, {-3, 13}}};

struct Vector2D backRoutine[4][4] = {{{2, 10}, {2, 9}, {-3, 9}, {-3, 10}}, //front legs
                                     {{2, 10}, {2, 10}, {2, 10}, {2, 10}}, 
                                     {{2, 13}, {2, 9}, {-3, 13}, {-3, 13}}, //back legs
                                     {{2, 13}, {2, 13}, {2, 13}, {2, 13}}};

struct Vector2D (*currentRoutine)[4][4];

void setup() {
  // ultrasonic sensors setup
  //sonicL.begin();
  //sonicR.begin();

  // open a serial connection and print starting string
  Serial.begin(9600);
  while (!Serial) continue;
  Serial.println("Spot Booting...");

  // ultrasonic sensors start checking
  //sonicL.start();
  //sonicR.start();

  // pwm driver setup
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates

  delay(10);

  // for simpler testing purposes, the shoulder servos are fixed for now
  pwm.setPWM(2, 0, DegToPwm(2, 5));
  pwm.setPWM(5, 0, DegToPwm(5, 5));
  pwm.setPWM(8, 0, DegToPwm(8, 5));
  pwm.setPWM(11, 0, DegToPwm(11, 5));

  // sets the starting position, need to make more foolproof for the final product
  currentPos = standingPos;

  CalculateDegs();
}

// sets the routine to be followed
void CalculateRoutine() {
  if (!hasRoutine || !finishedMoving)
    return;

  // if we just started a movement routine, we're clearly moving and havent
  // finished movement
  finishedMoving = false;

  for (int i = 0; i < 4; i++) {
    tempPos[i] = (*currentRoutine)[RangeLoop(i + routineLeg, 4)][routineStep];

    continue; // remove this, testing purposes

    Serial.print("Leg #: ");
    Serial.print(i, DEC);
    Serial.print(", Routine #: ");
    Serial.print(routineStep, DEC);
    Serial.print(", Routine Leg #: ");
    Serial.print(routineLeg, DEC);
    Serial.print("; ");
    Serial.print(tempPos[i].x, DEC);
    Serial.print("/");
    Serial.println(tempPos[i].y, DEC);
    Serial.println("-----------------------");

    delay(100);
  }

  currentPos = tempPos;

  CalculateDegs();

  // goes through every step until the max number of steps is reached
  if (routineStep < routineStepCount - 1)
    routineStep++;
  else {
    routineStep = 0;

    // after the steps in the routine is completed, changes leg offset
    if (routineLeg < routineLegCount - 1)
      routineLeg++;
    else
    {
      routineLeg = 0;

      //goes through the entire routine how many times? routineCyclesCount times
      if (routineCycle < routineCyclesCount - 1)
        routineCycle++;
      else
        {
          routineCycle = 0;
          currentPos = standingPos;
          hasRoutine = false;

          CalculateDegs();
        }
    }
  }
}

// transforms the 2d coordinates to 2 angles (in degrees) for each servo and
// calculates the steps in degrees: needs to be called everytime we want the
// servos to move (when we change positions)
void CalculateDegs() {
  for (int i = 0; i < 4; i++) {
    struct Vector2D targetAngles = CoordToAngles(&currentPos[i], i * 3);

    // checks if nan in case input values for the trigo functions in
    // CoordToAngles are out of bounds: need a better solution in the future
    if (isnan(targetAngles.x))
      targetAngles.x = 0;
    if (isnan(targetAngles.y))
      targetAngles.y = 0;

    desiredDegs[i * 3] = targetAngles.x;
    desiredDegs[i * 3 + 1] = targetAngles.y;
    desiredDegs[i * 3 + 2] = 0; // the shoulder servos stay at 0 for now
  }

  // calculates servo degree steps so that each motor finishes movement at the
  // same time
  for (int i = 0; i < 12; i++) {
    servoDegSteps[i] = (desiredDegs[i] - currentDegs[i]) / STEP_SIZE;
  }
}

// Loops the supplied value between 0 and the supplied maximum
int RangeLoop(int val, int max) {
  if (val < max)
    return val;

  return abs(val - max);
}

// maps the supplied degree value of the specified servo to a pwm value
int DegToPwm(int servo, float deg) {
  return map(deg, 0, servoDegRanges[servo], minPwmVals[servo],
             maxPwmVals[servo]);
}

// transforms 2d coordinates to angles for the specified servo (servonumb to
// only bound the resulting degree value) using trigonometry
Vector2D CoordToAngles(Vector2D *vec, int servoNumb) {
  // todo:add restrictions to coord. bounds

  float x = vec->x;
  float y = vec->y;
  bool isNegative = false;

  //if (for some reason) y is negative, make it positive
  if (y < 0)
    y = -y;

  //if the x coordinate is negative, the formula differs
  if (x < 0) 
  {
    isNegative = true;
    x = -x;
  }

  // simple trigonometry, basically creating triangles and using the cosine law
  // to find the necessary angles: very basic and not at all optimized
  float hyp = pow(pow(x, 2) + pow(y, 2), 0.5f);

  struct Vector2D res;

  res.x =
      acos((pow(hyp, 2) - pow(UPPER_SEG_LENGTH, 2) - pow(LOWER_SEG_LENGTH, 2)) /
           (-2 * UPPER_SEG_LENGTH * LOWER_SEG_LENGTH));

  float tempAngle1 = atan(y / x);

  float tempAngle2 =
      acos((pow(LOWER_SEG_LENGTH, 2) - pow(hyp, 2) - pow(UPPER_SEG_LENGTH, 2)) /
           (-2 * UPPER_SEG_LENGTH * hyp));

  //if the x coordinate is negative, the formula differs
  if (isNegative) {
    res.y = tempAngle1 - tempAngle2;
  } else {
    res.y = PI - tempAngle1 - tempAngle2;
  }

  res.x = res.x * 180 / PI;
  res.y = res.y * 180 / PI;

  // todo: find a better solution
  res.x = constrain(res.x, 0, servoDegRanges[servoNumb]);
  res.y = constrain(res.y, 0, servoDegRanges[servoNumb + 1]);

  return res;
}

void loop() {
  /*if (sonicR.isFinished() && sonicL.isFinished())
  {
    // for general info data, a 0 is appended at the beginning of the message
    Serial.print("0R: ");
    Serial.print(sonicR.getRange());
    Serial.print(" / L: ");
    Serial.println(sonicL.getRange());

    sonicL.start();
  sonicR.start();
  }*/

  int servosComplete = 0; // holds how many servos reached their final destination

  for (int i = 0; i < 12; i++) {
    // if we havent reached target pwm values, add the previously calculated
    // step and set the resulting pwm if we have increment the number of
    // completed servos
    if (DegToPwm(i, currentDegs[i]) != DegToPwm(i, desiredDegs[i])) {
      currentDegs[i] += servoDegSteps[i];
      pwm.setPWM( i, 0, DegToPwm(i, currentDegs[i])); // we could directly calculate pwm steps instead
                                                      // of servo ones, would probably save quite a
                                                      // bit of processing power for each loop
    } else {
      servosComplete++;
    }
  }

  finishedMoving = servosComplete == 12; // if everyone reached their goal, the bot finished moving;

  // this line needs to be modified so that when the current routine is changed,
  // we still call CalculateRoutine and everything works: for testing purposes
  // this is fine as we are testing one routine at a time

  CalculateRoutine(); // if we're running a routine and the bot finished moving, we can progress into the next step of our routine;

  // controls which position based on input from the serial port
  if (Serial.available() > 0) {
    // read the incoming byte:
    int commandByte = Serial.readStringUntil(' ').toInt();
    int directionByte = Serial.readStringUntil(' ').toInt();
    int lengthByte = Serial.readStringUntil('\n').toInt();

    hasRoutine = false;

    switch (commandByte) {
    case 0:
      currentPos = lyingPos;
      break;

    case 1:
      currentPos = standingPos;
      break;

    case 2:
      hasRoutine = true;
      
      switch (directionByte) {
        case 0:
        currentRoutine = &forwardRoutine;
          break;
        case 1:
        currentRoutine = &backRoutine;
          break;
        case 2:
          break;
        case 3:
          break;
      }

      routineCyclesCount = lengthByte;
      break;
    }

    CalculateDegs();

    Serial.print("I received: Cmd: ");
    Serial.print(commandByte, DEC);
    Serial.print(", Dir: ");
    Serial.print(directionByte, DEC);
    Serial.print(", Len: ");
    Serial.println(lengthByte, DEC);
  }

  delay(PROCESSING_DELAY);
}
