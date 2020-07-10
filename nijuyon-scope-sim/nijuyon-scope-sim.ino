#include <AccelStepper.h>
#include <MultiStepper.h>

#define AZI_STEP  2 // Digital Pin 2
#define AZI_DIR   3 // etc.
#define ALT_STEP  4
#define ALT_DIR   5

const long AZI_LIM_HI = 5000000;
const long AZI_LIM_LO = -5000000;
const long ALT_LIM_HI = 500000;
const long ALT_LIM_LO = -500000;

const boolean DISABLE_OUTPUT = true;

const float max_speed_SP = 900.0; // Steps per second (as per stepper motor library, not sure if this is what it does in reality
const float sec_to_max_speed = 10.0; // Seconds to go from zero to max speed

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



AccelStepper stepperAZI(AccelStepper::DRIVER, AZI_STEP, AZI_DIR);
AccelStepper stepperALT(AccelStepper::DRIVER, ALT_STEP, ALT_DIR);

void setup() {
   Serial.begin(9600);

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
        //showParsedData();
        if (sendposCMD) { sendCurrentPosition(); sendposCMD = false; }
        if (targetCMD) { issueDriverCommand(); targetCMD = false; }
        newData = false;
    }
    
    stepperAZI.run();
    stepperALT.run();
    
    if (DISABLE_OUTPUT == false && messageCntr > 90000) {
        Serial.print("Speed: "); 
        Serial.print("AZI = ");
        Serial.print(stepperAZI.speed()); 
        Serial.print(" ALT = ");
        Serial.println(stepperALT.speed()); 
        Serial.print("Current Position: "); 
        Serial.print(" AZI = ");
        Serial.print(stepperAZI.currentPosition());   
        Serial.print(" ALT = ");
        Serial.println(stepperALT.currentPosition());
        Serial.print("Target Position: "); 
        Serial.print("AZI = ");
        Serial.print(stepperAZI.targetPosition());   
        Serial.print(" ALT = ");
        Serial.println(stepperALT.targetPosition());

        messageCntr = 0;
    } else {
      messageCntr++;
    }
    //delay(1000); // milliseconds
}

void issueDriverCommand() {
 // Limit checking
  if (RecdAZI > AZI_LIM_HI) { RecdAZI = AZI_LIM_HI; }
  if (RecdAZI < AZI_LIM_LO) { RecdAZI = AZI_LIM_LO; }
  if (RecdALT > ALT_LIM_HI) { RecdALT = ALT_LIM_HI; }
  if (RecdALT < ALT_LIM_LO) { RecdALT = ALT_LIM_LO; }

  // Issue moveTo if the setpoint has changed.
  if (RecdAZI != AZI_SP) { 
    AZI_SP = RecdAZI;
    stepperAZI.moveTo(AZI_SP);
    //Serial.print("Azimuth move to: ");
    //Serial.println(AZI_SP);
  }
  if (RecdALT != ALT_SP) { 
    ALT_SP = RecdALT;
    stepperALT.moveTo(ALT_SP);
    //Serial.print("Altitude move to: ");
    //Serial.println(ALT_SP);
  }
  //Serial.println("-------------------------");
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

    // print the received messages
    //Serial.print("<");
    //Serial.print(tempChars);
    //Serial.println(">"); 
        
    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    RecdCMD = *strtokIndx; // convert and copy it to the received command.

    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    RecdAZI = atol(strtokIndx); 
    
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    RecdALT = atol(strtokIndx); 

    if (RecdCMD == 'T') {
      targetCMD = true;
    } else if (RecdCMD == 'R') {
      sendposCMD = true;
    }

}

void sendCurrentPosition() {
    Serial.print("<U,");
    Serial.print(stepperAZI.currentPosition());
    Serial.print(",");
    Serial.print(stepperALT.currentPosition());
    Serial.println(">");
}

void showParsedData() {
    Serial.print("Received CMD: ");
    Serial.println(RecdCMD);
    Serial.print("Received AZI: ");
    Serial.println(RecdAZI);
    Serial.print("Received ALT: ");
    Serial.println(RecdALT);
   
}
