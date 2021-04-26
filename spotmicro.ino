#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ 50       // Analog servos run at ~50 Hz updates
#define PROCESSING_DELAY 25 // how much delay between loop calls (in ms)

// lengths of each leg segment
#define UPPER_SEG_LENGTH 10
#define LOWER_SEG_LENGTH 12

//basic struct to hold 2d coordinates
struct Vector2D {
  float x, y;
};

bool finishedMoving = false; //the bot has finished moving
bool hasRoutine = false; //is following a routine
int routineStepCount = 4; //how long the routines are
int routineStep = 0; //where, in the routine, are we at?

//bounds of each servo's pwm range
// end, mid, shoulder;
const int maxPwmVals[] = {230, 340, 310, 320, 150, 310,
                          320, 150, 290, 230, 340, 310};

const int minPwmVals[] = {480, 140, 310, 70,  350, 310,
                          70,  350, 290, 480, 140, 310};

//upper bounds of each servo's degree range
const int servoDegRanges[] = {115, 90, 0, 115, 90, 0, 115, 90, 0, 115, 90, 0};

//steps each servo will take at each loop iteration
float servoDegSteps[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

float desiredDegs[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //holds desired degree values
float currentDegs[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //holds the current degree values that each servo is set to

//holds the positions of each leg (4) in 2 different positions, mostly for testing purposes
struct Vector2D standingPos[] = {{0, 15}, {0, 15}, {0, 15}, {0, 15}};
struct Vector2D lyingPos[] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};

struct Vector2D* currentPos = new Vector2D[4]; //for easier testing, will remove later as dynamic memory on Arduino is limited
struct Vector2D tempPos[] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}}; //holds positions for each leg until everyone has been assigned

//holds each leg's position for the walk routine. difference with the positions array is that it ressembles a state machine for each leg
struct Vector2D walkRoutine[][4] = {{{0, 15}, {0, 10}, {5, 10}, {5, 15}},
                                    {{0, 15}, {0, 15}, {0, 15}, {0, 15}},
                                    {{0, 15}, {0, 10}, {0, 10}, {0, 15}},
                                    {{0, 15}, {0, 10}, {0, 10}, {0, 15}}};

void setup() {
  //open a serial connection and print starting string
  Serial.begin(9600);
  Serial.println("Spot Booting...");

  //pwm driver setup
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates

  delay(10);

  //sets the starting position, need to make more foolproof for the final product
  currentPos = standingPos;

  CalculateDegs();
}

//sets the routine to be followed 
void CalculateRoutine()
{
  if (!hasRoutine || !finishedMoving) return;

  //if we just started a movement routine, we're clearly moving and havent finished movement
  finishedMoving = false;

  for (int i =0; i < 4; i++)
  {
    tempPos[i] = walkRoutine[i][routineStep];

    continue; //remove this, testing purposes

    Serial.print("Leg #: ");
    Serial.print(i, DEC);
    Serial.print(", Routine #: ");
    Serial.print(routineStep, DEC);
    Serial.print("; ");
    Serial.print(tempPos[i].x, DEC);
    Serial.print("/");
    Serial.println(tempPos[i].y, DEC);
    Serial.println("-----------------------");

    delay(500);
  }

  currentPos = tempPos;

  CalculateDegs();

  //goes through every step until the max number of steps is reached
  if (routineStep < routineStepCount - 1)
    routineStep++;
  else 
    routineStep = 0;
}

//transforms the 2d coordinates to 2 angles (in degrees) for each servo and calculates the steps in degrees: needs to be called everytime we want the servos to move (when we change positions)
void CalculateDegs()
{
  for (int i = 0; i < 4; i++)
      {
        struct Vector2D targetAngles = CoordToAngles(&currentPos[i], i*3);
      
        //checks if nan in case input values for the trigo functions in CoordToAngles are out of bounds: need a better solution in the future
        if (isnan(targetAngles.x)) targetAngles.x = 0;
        if (isnan(targetAngles.y)) targetAngles.y = 0;

        desiredDegs[i*3] = targetAngles.x;
        desiredDegs[i*3+1] = targetAngles.y;
        desiredDegs[i*3+2] = 0; //the shoulder servos stay at 0 for now
      }

    //calculates servo degree steps so that each motor finishes movement at the same time
    for (int i = 0; i < 12; i++) {
    servoDegSteps[i] = (desiredDegs[i] - currentDegs[i]) / 50;
  }
}

//maps the supplied degree value of the specified servo to a pwm value
int DegToPwm(int servo, float deg) {
  return map(deg, 0, servoDegRanges[servo], minPwmVals[servo],
             maxPwmVals[servo]);
}

//transforms 2d coordinates to angles for the specified servo (servonumb to only bound the resulting degree value) using trigonometry
Vector2D CoordToAngles(Vector2D *vec, int servoNumb) {
  //todo:add restrictions to coord. bounds

  //simple trigonometry, basically creating triangles and using the cosine law to find the necessary angles: very basic and not at all optimized
  float hyp = pow(pow(vec->x, 2) + pow(vec->y, 2), 0.5f);

  struct Vector2D res;

  res.x = acos((pow(hyp, 2) - pow(UPPER_SEG_LENGTH, 2) - pow(LOWER_SEG_LENGTH, 2)) /
           (-2 * UPPER_SEG_LENGTH * LOWER_SEG_LENGTH));

  float tempAngle1 = atan(vec->y / vec->x);

  float tempAngle2 = acos((pow(LOWER_SEG_LENGTH, 2) - pow(hyp, 2) - pow(UPPER_SEG_LENGTH, 2)) /
           (-2 * UPPER_SEG_LENGTH * hyp));

  res.y = PI - tempAngle1 - tempAngle2;

  res.x = res.x * 180/PI;
  res.y = res.y * 180/PI;

  //todo: find a better solution
  res.x = constrain(res.x, 0, servoDegRanges[servoNumb]);
  res.y = constrain(res.y, 0, servoDegRanges[servoNumb + 1]);

  return res;
}

void loop() {

  int servosComplete = 0; //holds how many servos reached their final destination

  for (int i = 0; i < 12; i++) 
  {
    //if we havent reached target pwm values, add the previously calculated step and set the resulting pwm
    //if we have increment the number of completed servos
    if (DegToPwm(i, currentDegs[i]) != DegToPwm(i, desiredDegs[i])) 
    {
      currentDegs[i] += servoDegSteps[i];
      pwm.setPWM(i, 0, DegToPwm(i, currentDegs[i])); //we could directly calculate pwm steps instead of servo ones, would probably save quite a bit of processing power for each loop
    }
    else 
    {
      servosComplete++;
    }
  }

  finishedMoving = servosComplete == 12; //if everyone reached their goal, the bot finished moving;

  //this line needs to be modified so that when the current routine is changed, we still call CalculateRoutine and everything works: for testing purposes this is fine as we are testing one routine at a time
  CalculateRoutine(); //if we're running a routine and the bot finished moving, we can progress into the next step of our routine;

  //controls which position based on input from the serial port
  if (Serial.available() > 0) {
    // read the incoming byte:
    int incomingByte = Serial.readStringUntil(' ').toInt();

    hasRoutine = false;

    switch (incomingByte) {
    case 0:
      currentPos = lyingPos;
      break;

    case 1:
      currentPos = standingPos;
      break;

    case 2:
      hasRoutine = true;

      delay(1000);
      break;
    }

    CalculateDegs();

    Serial.print("I received: ");
    Serial.println(incomingByte, DEC);
  }

  delay(PROCESSING_DELAY);
}
