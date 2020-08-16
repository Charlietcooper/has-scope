#include <AccelStepper.h>
#include <MultiStepper.h>

#define AZI_STEP  2 // Digital Pin 2
#define AZI_DIR   3 // etc.
#define ALT_STEP  4
#define ALT_DIR   5

#define DIRECTION_NORTH  0
#define DIRECTION_SOUTH  1
#define DIRECTION_WEST   0
#define DIRECTION_EAST   1    
#define MOTION_START     0
#define MOTION_STOP      1

enum scopeState {
   SCOPE_IDLE,
   SCOPE_SLEWING,
   SCOPE_TRACKING,
   SCOPE_PARKING,
   SCOPE_PARKED,
   SCOPE_MANMOVE
};

long dir_NS, dir_WE;
long motionCommand_NS, motionCommand_WE;

scopeState trackState = SCOPE_IDLE;

const long AZI_LIM_HI = 5000000;
const long AZI_LIM_LO = -5000000;
const long ALT_LIM_HI = 500000;
const long ALT_LIM_LO = -500000;

const boolean DISABLE_OUTPUT = true;

const float max_speed_SP = 900.0;    // Pulses per second (as per stepper motor library, not sure if this is what it does in reality
const float sec_to_max_speed = 10.0; // Seconds to go from zero to max speed
const float sync_speed = 5.0;        // pulses per sec. Track rate for Right Ascension.

const byte numChars = 32;
const byte cmdChars = 2;
boolean newData = false;

long RecdAZI = 0;
long RecdALT = 0;
long AZI_SP = 0; 
long ALT_SP = 0;

long messageCntr = 0;

char receivedChars[numChars];   // an array to store the received data
char tempChars[numChars];       // temporary array for use when parsing
char RecdCMD;

bool targetCMD = false;
bool sendposCMD = false;
bool trackCMD = false;
bool MoveNS_CMD = false;
bool MoveWE_CMD = false;

unsigned long time = millis();
unsigned long updateTime = time + 1000;
const float RAdrift_1msec = 24.0 / 86400 / 1000;

AccelStepper stepperAZI(AccelStepper::DRIVER, AZI_STEP, AZI_DIR);
AccelStepper stepperALT(AccelStepper::DRIVER, ALT_STEP, ALT_DIR);

void setup() {
   Serial.begin(57600);

   stepperAZI.setMaxSpeed(max_speed_SP);
   stepperAZI.setAcceleration(max_speed_SP / sec_to_max_speed); 
   
   stepperALT.setMaxSpeed(max_speed_SP * 1.0); // 3.203125
   stepperALT.setAcceleration(max_speed_SP / sec_to_max_speed); 
}

void loop() {

    // Check for time update messages on the serial bus.
    recvWithStartEndMarkers(); // Check for new data on the serial bus
    if (newData == true) { // if a new complete message has been received:
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
        parseData();
        
        if (sendposCMD) { sendCurrentPosition(); sendposCMD = false; }
        if (targetCMD) { issueDriverCommand(); targetCMD = false; }
        if (trackCMD) { setToTrackMode(); trackCMD = false; }
        if (MoveNS_CMD) { moveNorthSouth(); MoveNS_CMD = false; }
        if (MoveWE_CMD) { moveWestEast(); MoveWE_CMD = false; }
        
        newData = false;
    }

    switch (trackState) {
      case SCOPE_IDLE:
        // Do nothing.
        break;
      case SCOPE_SLEWING:
        stepperAZI.run();
        stepperALT.run();
        break;
      case SCOPE_PARKING:
        stepperAZI.run();
        stepperALT.run();
        break;
      case SCOPE_TRACKING:
        stepperAZI.runSpeed();
        break;
      case SCOPE_PARKED:
        // Do nothing
        break;
      default:
        // Do nothing.
        break;
    }
    
}

void issueDriverCommand() {

  trackState = SCOPE_SLEWING;
  
  // Limit checking
  if (RecdAZI > AZI_LIM_HI) { RecdAZI = AZI_LIM_HI; }
  if (RecdAZI < AZI_LIM_LO) { RecdAZI = AZI_LIM_LO; }
  if (RecdALT > ALT_LIM_HI) { RecdALT = ALT_LIM_HI; }
  if (RecdALT < ALT_LIM_LO) { RecdALT = ALT_LIM_LO; }

  // Issue moveTo if the setpoint has changed.
  if (RecdAZI != AZI_SP) { 
    AZI_SP = RecdAZI;
    stepperAZI.moveTo(AZI_SP);
  }
  if (RecdALT != ALT_SP) { 
    ALT_SP = RecdALT;
    stepperALT.moveTo(ALT_SP);
  }
}

void setToTrackMode() {
  stepperAZI.setSpeed(-sync_speed);
  trackState = SCOPE_TRACKING;
}

void moveNorthSouth() {
  long diffSteps;
  long ALTTarget;
  int stepSize = 1000;
  
  if (dir_NS == DIRECTION_NORTH ) {
    diffSteps = stepSize;
  } else if (dir_NS == DIRECTION_SOUTH) {
    diffSteps = -stepSize;
  }
  if (motionCommand_NS == MOTION_START ) {
    ALTTarget = stepperALT.currentPosition() + diffSteps;
    stepperALT.moveTo(ALTTarget);
    trackState = SCOPE_SLEWING;
  } else if (motionCommand_NS == MOTION_STOP) {
    stepperALT.moveTo(stepperALT.currentPosition());
    trackState = SCOPE_IDLE;
  }
}

void moveWestEast() {
  long diffSteps;
  long AZITarget;
  int stepSize = 1000;
  
  if (dir_WE == DIRECTION_WEST ) {
    diffSteps = stepSize;
  } else if (dir_WE == DIRECTION_EAST) {
    diffSteps = -stepSize;
  }
  if (motionCommand_WE == MOTION_START ) {
    AZITarget = stepperAZI.currentPosition() + diffSteps;
    stepperAZI.moveTo(AZITarget);
    trackState = SCOPE_SLEWING;
  } else if (motionCommand_WE == MOTION_STOP) {
    stepperAZI.moveTo(stepperAZI.currentPosition());
    trackState = SCOPE_IDLE;
  }
}

/* 
 *  Functions for received serial communication
 *  
 *  Expects a packet like: <T,45,-3000>\0
 */

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index
    int dir;
    int motionCommand;
    // print the received messages
    //Serial.print("<");
    //Serial.print(tempChars);
    //Serial.println(">"); 
        
    strtokIndx = strtok(tempChars,","); // get the first part - the string
    RecdCMD = *strtokIndx; // convert and copy it to the received command.

    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    RecdAZI = atol(strtokIndx); 
    
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    RecdALT = atol(strtokIndx); 

    if (RecdCMD == 'T') {
      targetCMD = true;
    } else if (RecdCMD == 'R') {
      sendposCMD = true;
    } else if (RecdCMD == 'S') {
      trackCMD = true;
    } else if (RecdCMD == 'N') {
      MoveNS_CMD = true;
      dir_NS = RecdAZI;
      motionCommand_NS = RecdALT;
    } else if (RecdCMD == 'W') {
      MoveWE_CMD = true;
      dir_WE = RecdAZI;
      motionCommand_WE = RecdALT;
    }
}

void sendCurrentPosition() {
    Serial.print("<U,");
    Serial.print(stepperAZI.currentPosition());
    Serial.print(",");
    Serial.print(stepperALT.currentPosition());
    Serial.println(">");
}
