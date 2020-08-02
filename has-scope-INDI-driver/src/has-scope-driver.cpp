/*
   Hamilton Astronomical Society 24" Telescope INDI Driver

*/

/** \file has-scope-driver.cpp
    \brief Construct a basic INDI telescope device that simulates GOTO commands.
    \author Richard Croy

    \example has-scope-driver.cpp
    A simple GOTO telescope that simulator slewing operation.
*/

#include <cmath>
#include <memory>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>

#include <vector>
#include <string>
#include <sstream>

#include <libnova/transform.h>
#include <libnova/julian_day.h>
#include <libnova/utility.h>
#include <libnova/ln_types.h>
#include <libnova/sidereal_time.h>

#include "indicom.h"
#include "indicontroller.h"
#include "connectionplugins/connectionserial.h"

#include "has-scope-driver.h"

#define POLLMS 2000     // Poll time in milliseconds
#define BUFFER_SIZE     40  // Maximum message length
#define ARDUINO_TIMEOUT 5   // fd timeout in seconds
#define START_BYTE 0x3C
#define END_BYTE 0x3E
#define PULSE_PER_RA  17975.0     // Pulses per hour of Right Ascension (hrs)
#define PULSE_PER_DEC 3777.77778  // Pulses per degree of Declination (deg)

// Commands available
#define TARGET_CMD  'T' 
#define REQUESTPOS_CMD  'R' // Request the current position (in number of steps).

struct {
    double RA {0}; // hrs
    double Dec {0}; // deg
} current, target, startPos; // RA in hours

struct {
    signed long stepRA {0};
    signed long stepDec {0};
} currentSteps, prevSteps, deltaSteps, targetSteps;

lnh_lnlat_posn hobserver;
ln_lnlat_posn observer;
ln_hrz_posn hrz_posn;
ln_equ_posn curr_equ_posn;
ln_date date;
double JD;

typedef std::chrono::steady_clock Clock; // A monotonic clock
Clock::time_point prevTime = Clock::now();
Clock::time_point currTime = Clock::now();

const double RAdrift_1sec = 24.0 / 86400.0;

bool waitingOnSerialResponse = false;

static std::unique_ptr<HASSTelescope> HASScope(new HASSTelescope());

void hexDump(char *buf, const char *data, int size)
{
    for (int i = 0; i < size; i++)
        sprintf(buf + 3 * i, "%02X ", (unsigned char)data[i]);
}

/**************************************************************************************
** Return properties of device.
***************************************************************************************/
void ISGetProperties(const char *dev)
{
    HASScope->ISGetProperties(dev);
}

/**************************************************************************************
** Process new switch from client
***************************************************************************************/
void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    HASScope->ISNewSwitch(dev, name, states, names, n);
}

/**************************************************************************************
** Process new text from client
***************************************************************************************/
void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    HASScope->ISNewText(dev, name, texts, names, n);
}

/**************************************************************************************
** Process new number from client
***************************************************************************************/
void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    HASScope->ISNewNumber(dev, name, values, names, n);
}

/**************************************************************************************
** Process new blob from client
***************************************************************************************/
void ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[],
               char *names[], int n)
{
    HASScope->ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names, n);
}

/**************************************************************************************
** Process snooped property from another driver
***************************************************************************************/
void ISSnoopDevice(XMLEle *root)
{
    HASScope->ISSnoopDevice(root);
}

/**************************************************************************************
** HAS Telescope Basic constructor
***************************************************************************************/
HASSTelescope::HASSTelescope()
{
    // We add an additional debug level so we can log verbose scope status
    //DBG_SCOPE = INDI::Logger::getInstance().addDebugLevel("Scope Verbose", "SCOPE");

    SetTelescopeCapability(TELESCOPE_CAN_SYNC | TELESCOPE_CAN_GOTO | TELESCOPE_CAN_ABORT);
    LOGF_INFO("\n------------------------------------------------------","");
    LOGF_INFO("Initializing HAS Telescope...","");

}

/**************************************************************************************
** We init our properties here. 
***************************************************************************************/
bool HASSTelescope::initProperties()
{
    // ALWAYS call initProperties() of parent first
    LOGF_INFO("Calling INDI::Telescope::initProperties() %s", "");
    INDI::Telescope::initProperties();

    // Add Debug control so end user can turn debugging/loggin on and off
    addDebugControl();

    SetTimer(POLLMS);

    currTime = Clock::now();
    std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(currTime - prevTime);

    double time_spand = time_span.count();

    //LOGF_INFO("time_span: %f sec", time_span);
    //LOGF_INFO("time_span double : %f sec", time_spand);
    
    TrackState = SCOPE_IDLE;
    setDriverInterface(TELESCOPE_INTERFACE);

    /* 
     * observers position
     * longitude is measured positively eastwards   
     * 
     * HAS Observatory, Hamilton, New Zealand
     */

    // For INDI
    double longitude = 175.2146, latitude = -37.7735; 
    double elevation = 40; // meters

    double sidereal;

    // For libnova. Observer position (equatorial coordinates)
    observer.lat = latitude;
    observer.lng = longitude;

    updateLocation(latitude, longitude, elevation);

    // Set initial position of the telescope
    // Forks horizontal, nose resting on the bridge.
    JD = ln_get_julian_from_sys();
    ln_get_date_from_sys(&date);
    sidereal = ln_get_apparent_sidereal_time(JD);
    curr_equ_posn.dec = 45.0;
    // libnova works in decimal degrees for RA
    curr_equ_posn.ra = (sidereal + (observer.lng / 360 * 24)) / 24 * 360; // deg
    if (curr_equ_posn.ra > 360) curr_equ_posn.ra = curr_equ_posn.ra - 360;

    NewRaDec(curr_equ_posn.ra / 360 * 24, curr_equ_posn.dec);

    startPos.RA = curr_equ_posn.ra / 360 * 24; // RA in hours
    startPos.Dec = curr_equ_posn.dec;

    prevSteps.stepRA = 0;
    prevSteps.stepDec = 0;

    LOGF_INFO("\n------------------------------------------------------","");
    LOGF_INFO("Set initial position. RA %f deg, DEC %f deg", curr_equ_posn.ra, curr_equ_posn.dec);
    LOGF_INFO("Right Ascension in hours: %f", curr_equ_posn.ra / 360 * 24);
    LOGF_INFO("System Julian date is %f", JD);
    LOGF_INFO("System Sidereal time is %f", sidereal);
    LOGF_INFO("System UTC Date is %i-%i-%i %i:%i:%f", date.years, date.months, date.days, date.hours, date.minutes, date.seconds);
    LOGF_INFO("Observer position is Latitude %f, Long %f", observer.lat, observer.lng);
    LOGF_INFO("EqN[AXIS_RA].value is %f hrs",  EqN[AXIS_RA].value);
    LOGF_INFO("EqN[AXIS_DE].value is %f deg",  EqN[AXIS_DE].value);

    // Enable simulation mode so that serial connection in INDI::Telescope does not try
    // to attempt to perform a physical connection to the serial port.
    //LOGF_INFO("Calling setSimulation(true) %s", "");
    //setSimulation(true);
   
    return true;
}

bool HASSTelescope::SendCommand(char cmd_op, signed long stepRA, signed long stepDec)
{
    int err;
    int nbytes;
    int cmd_nbytes;
    char cmd[BUFFER_SIZE];
    char *ptrCmd = cmd;
    char hexbuf[3*BUFFER_SIZE];

    LOGF_INFO("---Begin SendCommand()---","");

    if (waitingOnSerialResponse) {
        LOGF_INFO("Already waiting on a response. Not sending command.","");
    } else {
        cmd_nbytes = sprintf(cmd, "<%c,%ld,%ld>\n", cmd_op, stepRA, stepDec);
        LOGF_INFO("Command sent was: %s", cmd);
        cmd_nbytes++;
        hexDump(hexbuf, cmd, cmd_nbytes);
        //LOGF_INFO("CMD as Hex (%s)", hexbuf);

        //tcflush(fd, TCIOFLUSH);
        tcflush(fd, TCOFLUSH);

        if ((err = tty_write(fd, cmd, cmd_nbytes, &nbytes)) != TTY_OK)
        {
            LOGF_INFO("tty_write Error: %i, %i bytes", err, nbytes);
            return -5;
        } else {
            //LOGF_INFO("Wrote cmd buffer to tty, %i bytes", nbytes);
            waitingOnSerialResponse = true;
        }
    }
    
}

int HASSTelescope::ReadResponse()
{
    int nbytes = 0;
    int err = TTY_TIME_OUT;
    char rbuffer[BUFFER_SIZE];
    char *rbuff = &rbuffer[0];
    int bytesToStart = 0;
    memset(&rbuffer[0], 0x00, sizeof(rbuffer)); // clear the buffer

    bool recvInProgress = false;
    bool newData = false;
    int ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
    char hexbuf[3*BUFFER_SIZE];
        
    LOGF_INFO("--- ReadResponse() ---","");

    hexDump(hexbuf, rbuffer, BUFFER_SIZE);
      
/*
    // Look for a starting byte until time out occurs or BUFFER_SIZE bytes were read
    while (*rbuffer != START_BYTE && err == TTY_OK)
        err = tty_read(fd, rbuffer, 1, ARDUINO_TIMEOUT, &nbytes);
        LOGF_INFO("bytesToStart is %i, err is %i", bytesToStart, err);
        LOGF_INFO("*rbuff is %c. START_BYTE is %c", *rbuff, START_BYTE);
        LOGF_INFO("tty_read: nbytes read is %i", nbytes);
        LOGF_INFO("Read: %c. Start byte?", rbuffer[0]);
        bytesToStart++;

    if (err != TTY_OK) {
        LOGF_INFO("tty_read failed. error %i", err);
        return(1);
    }

    LOGF_INFO("Found START_BYTE. A tty_read of %i bytes to find start byte.", bytesToStart);
*/
    nbytes = 0;
    recvInProgress = true;
    while (newData == false) {
        err = tty_read(fd, &rc, 1, ARDUINO_TIMEOUT, &nbytes);
        //LOGF_INFO("Read: %c, byte number %i", rc, ndx);
        if (recvInProgress == true) {
            //LOGF_INFO("rc %c, endMarker %c", rc, endMarker);
            if (rc != endMarker) {
                rbuffer[ndx] = rc;
                ndx++;
                if (ndx >= BUFFER_SIZE) {
                    ndx = BUFFER_SIZE - 1;
                }
            }
            else {
                rbuffer[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
 
        }
    }
    LOGF_INFO("Received: %s", rbuffer);

    waitingOnSerialResponse = false;

    hexDump(hexbuf, rbuffer, BUFFER_SIZE);
    //LOGF_INFO("rbuffer Hex (%s)", hexbuf);

    // Update telescope position variables  

    std::string str = rbuffer; 
    std::vector<std::string> v; 
    std::stringstream ss(str);

    while (ss.good()) { 
        std::string substr; 
        getline(ss, substr, ','); 
        v.push_back(substr); 
    } 
  
    currentSteps.stepRA = std::stoi(v[1].c_str());
    currentSteps.stepDec = std::stoi(v[2].c_str());
    //LOGF_INFO("currentSteps.stepRA: %i, currentSteps.stepDec: %i", currentSteps.stepRA, currentSteps.stepDec);
}

bool HASSTelescope::Connect()
{
    LOGF_INFO("--- Connect() called. ---","");

    const char*defaultPort = "/dev/ttyACM0";
    int err = TTY_OK;

    if (isConnected()) {
        LOGF_INFO("Connect(): Already connected, according to isConnected()","");
        return true;
    }

    if (tty_connect(defaultPort, 57600, 8, 0, 1, &fd) != TTY_OK) {
        LOGF_INFO("tyy_connect failed.","");
        return false;
    } else {
        LOGF_INFO("tyy_connect succeeded. File Descriptor is: %i", fd);
    }

    // Send a dummy postion request. No reply will come on the first attempt.
    SendCommand(REQUESTPOS_CMD, 0, 0);
    waitingOnSerialResponse = false ; 

    bool status = true;
    return status;
}

/**************************************************************************************
** INDI is asking us to check communication with the device via a handshake
***************************************************************************************/
bool HASSTelescope::Handshake()
{
    // When communicating with a real mount, we check here if commands are receieved
    // and acknowledged by the mount. 

    int portFD = serialConnection->getPortFD();
    uint8_t wordSize = serialConnection->getWordSize();

    LOGF_INFO("getPortFD: %s", portFD);
    LOGF_INFO("getWordSize: %s", wordSize);

    SendCommand(REQUESTPOS_CMD, currentSteps.stepRA, currentSteps.stepDec);
    ReadResponse();

    return true;
}

/**************************************************************************************
** INDI is asking us for our default device name
***************************************************************************************/
const char *HASSTelescope::getDefaultName()
{
    return "HAS";
}

/**************************************************************************************
** Client is asking us to slew to a new position
***************************************************************************************/
bool HASSTelescope::Goto(double ra, double dec)
{
    char RAStr[64]={0}, DecStr[64]={0};
    double diffRA, diffDec;
    signed long diffStepRA, diffStepDec;

    LOGF_INFO("--- Goto() ---", "");

    target.RA  = ra;
    target.Dec = dec;

    LOGF_INFO("target.RA: %f (hrs), target.Dec: %f", target.RA, target.Dec);
    
    // Parse the RA/DEC into strings
    fs_sexa(RAStr, target.RA, 2, 3600);
    fs_sexa(DecStr, target.Dec, 2, 3600);

    // Calculate difference between target and current position.
    diffRA = target.RA - EqN[AXIS_RA].value;
    diffDec = target.Dec - EqN[AXIS_DE].value;
    //LOGF_INFO("diffRA: %f, diffDec: %f", diffRA, diffDec);

    diffStepRA = diffRA * PULSE_PER_RA;
    diffStepDec = diffDec * PULSE_PER_DEC;
    //LOGF_INFO("diffStepRA: %i, diffStepDec: %i", diffStepRA, diffStepDec);

    targetSteps.stepRA = currentSteps.stepRA + diffStepRA;
    targetSteps.stepDec = currentSteps.stepDec + diffStepDec;
    //LOGF_INFO("targetSteps.stepRA: %i, targetSteps.stepDec: %i", targetSteps.stepRA, targetSteps.stepDec);

    // Send new target to Arduino
    SendCommand(TARGET_CMD, targetSteps.stepRA, targetSteps.stepDec);

    waitingOnSerialResponse = false;

    // Mark state as slewing
    TrackState = SCOPE_SLEWING;

    // Inform client we are slewing to a new position
    //LOGF_INFO("Slewing to RA: %s - DEC: %s", RAStr, DecStr);
    LOGF_INFO("Slewing to stepRA %i, stepDec %i", targetSteps.stepRA, targetSteps.stepDec);

    // Success!
    return true;
}

/**************************************************************************************
** Client is asking us to abort our motion
***************************************************************************************/
bool HASSTelescope::Abort()
{
    SendCommand(TARGET_CMD, currentSteps.stepRA, currentSteps.stepDec);
    TrackState = SCOPE_IDLE;
    return true;
}

/**************************************************************************************
** Client is asking us to report telescope status
***************************************************************************************/
bool HASSTelescope::ReadScopeStatus()
{
    double sidereal;
    double localSidereal;

    LOGF_INFO("---ReadScopeStatus()---", "");
    LOGF_INFO("TrackState: %i", TrackState);
    
    if (!waitingOnSerialResponse) {
        ReadResponse();
        //LOGF_INFO("Already waiting on Serial Response. Just doing ReadResponse().", "");
    } else {
        //LOGF_INFO("Calling both SendCommand and ReadResponse.", "");
        tcflush(fd, TCIOFLUSH);
        SendCommand(REQUESTPOS_CMD, currentSteps.stepRA, currentSteps.stepDec);
        ReadResponse();
    }

    // Calculate delta Steps
    deltaSteps.stepRA = currentSteps.stepRA - prevSteps.stepRA;
    deltaSteps.stepDec = currentSteps.stepDec - prevSteps.stepDec;

    //LOGF_INFO("(abs(deltaSteps.stepRA) < 0.001) %i", (abs(deltaSteps.stepRA) < 0.001));
    //LOGF_INFO("(abs(deltaSteps.stepDec) < 0.001) %i", (abs(deltaSteps.stepDec) < 0.001));
    //LOGF_INFO("(TrackState == SCOPE_SLEWING) %i", (TrackState == SCOPE_SLEWING));

    if ((TrackState == SCOPE_SLEWING) && (abs(deltaSteps.stepRA) < 0.001) && (abs(deltaSteps.stepDec) < 0.001)) {
        LOGF_INFO("Set TrackState == SCOPE_IDLE","");
        TrackState == SCOPE_IDLE;
    }

    //LOGF_INFO("deltaSteps.stepRA %f, currentSteps.stepRA %f, prevSteps.stepRA %f", deltaSteps.stepRA, currentSteps.stepRA, prevSteps.stepRA );
    prevSteps.stepRA = currentSteps.stepRA;
    prevSteps.stepDec = currentSteps.stepDec;
    //LOGF_INFO("updated prevSteps.stepRA %f", prevSteps.stepRA);
    //LOGF_INFO("Change in RA %f", deltaSteps.stepRA / PULSE_PER_RA);
    //LOGF_INFO("curr_equ_posn.ra %f", curr_equ_posn.ra);

    current.RA = (deltaSteps.stepRA / PULSE_PER_RA) + EqN[AXIS_RA].value; // hrs
    current.Dec = deltaSteps.stepDec / PULSE_PER_DEC + EqN[AXIS_DE].value;

    // libnova works in decimal degrees
    curr_equ_posn.ra = current.RA * 360 / 24;
    curr_equ_posn.dec = current.Dec;

    NewRaDec(current.RA, current.Dec); // RA in hrs

    char RAStr[64]={0}, DecStr[64]={0};

    LOGF_INFO("EqN[AXIS_RA].value is %f hrs",  EqN[AXIS_RA].value);
    LOGF_INFO("EqN[AXIS_DE].value is %f deg",  EqN[AXIS_DE].value);

    // Parse the RA/DEC into strings
    fs_sexa(RAStr, curr_equ_posn.ra, 2, 3600);
    fs_sexa(DecStr, curr_equ_posn.dec, 2, 3600);

    //DEBUGF(DBG_SCOPE, "Current RA: %s Current DEC: %s", RAStr, DecStr);
 
    return true;
}

void HASSTelescope::TimerHit()
{
    std::chrono::duration<double> time_span;
    double RAdrift;

    LOGF_INFO("--- TimerHit() called ---", "");
    if (TrackState == SCOPE_IDLE) {
        currTime = Clock::now();
        time_span = std::chrono::duration_cast<std::chrono::duration<double>>(currTime - prevTime);
        //LOGF_INFO("time_span is %f sec", time_span.count());
        prevTime = Clock::now();
        RAdrift = time_span.count() * RAdrift_1sec;
        //LOGF_INFO("RAdrift is %f hrs", RAdrift);
        NewRaDec(EqN[AXIS_RA].value + RAdrift, EqN[AXIS_DE].value); // RA in hrs
    }

    if ( isConnected()) { // && TrackState == SCOPE_SLEWING
        //LOGF_INFO("IsConnected [and SCOPE_Slewing]. Calling REQUESTPOS_CMD", "");
        if (!waitingOnSerialResponse) SendCommand(REQUESTPOS_CMD,0,0);
        ReadScopeStatus();
    } else {
        //LOGF_INFO("not (IsConnected and SCOPE_Slewing). No action.", "");
    }

    SetTimer(POLLMS);
}
