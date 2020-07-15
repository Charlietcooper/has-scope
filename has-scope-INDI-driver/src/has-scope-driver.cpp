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

#define BUFFER_SIZE     40  // Maximum message length
#define ARDUINO_TIMEOUT 5   // fd timeout in seconds
#define START_BYTE 0x3C
#define END_BYTE 0x3E
#define PULSE_PER_RA  17975     // Pulses per hour of Right Ascension
#define PULSE_PER_DEC 3777.778  // Pulses per degree of Declination

// Commands available
#define TARGET_CMD  'T' 
#define REQUESTPOS_CMD  'R' // Request the current position (in number of steps).

double targetRA {0};
double targetDEC {0};
double currentRA {0};
double currentDEC {0};

struct {
    double RA {0};
    double Dec {0};
} current, target, startPos;

struct {
    signed long stepRA {100};
    signed long stepDec {200};
} currentSteps, targetSteps;

lnh_lnlat_posn hobserver;
ln_lnlat_posn observer;
ln_hrz_posn hrz_posn;
ln_equ_posn curr_equ_posn;
ln_date date;
double JD;


static std::unique_ptr<HASSTelescope> HASScope(new HASSTelescope());

void hexDump(char *buf, const char *data, int size)
{
    for (int i = 0; i < size; i++)
        sprintf(buf + 3 * i, "%02X ", (unsigned char)data[i]);

    //if (size > 0)
    //    buf[3 * size - 1] = '\0';
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
    curr_equ_posn.dec = 35.0;
    curr_equ_posn.ra = (sidereal / 24.0 * 360) + observer.lng; 
    if (curr_equ_posn.ra > 360) curr_equ_posn.ra = curr_equ_posn.ra - 360;

    NewRaDec(curr_equ_posn.ra, curr_equ_posn.dec);

    startPos.RA = curr_equ_posn.ra;
    startPos.Dec = curr_equ_posn.dec;

    LOGF_INFO("\n------------------------------------------------------","");
    LOGF_INFO("Set initial position. RA %f, DEC %f", curr_equ_posn.ra, curr_equ_posn.dec);
    LOGF_INFO("Right Ascension in hours: %f", curr_equ_posn.ra / 360.0 * 24.0);
    LOGF_INFO("System Julian date is %f", JD);
    LOGF_INFO("System Sidereal time is %f", sidereal);
    LOGF_INFO("System UTC Date is %i-%i-%i %i:%i:%f", date.years, date.months, date.days, date.hours, date.minutes, date.seconds);
    LOGF_INFO("Observer position is Latitude %f, Long %f", observer.lat, observer.lng);

    // Enable simulation mode so that serial connection in INDI::Telescope does not try
    // to attempt to perform a physical connection to the serial port.
    //LOGF_INFO("Calling setSimulation(true) %s", "");
    //setSimulation(true);
   
    return true;
}

bool HASSTelescope::SendCommand(char cmd_op)
{
    int err;
    int nbytes;
    int cmd_nbytes;
    char cmd[BUFFER_SIZE];
    char *ptrCmd = cmd;
    char hexbuf[3*BUFFER_SIZE];

    LOGF_INFO("---Begin SendCommand()---","");

    cmd_nbytes = sprintf(cmd, "<%c,%ld,%ld>", cmd_op, currentSteps.stepDec, currentSteps.stepRA);
    LOGF_INFO("cmd buffer is: %s", cmd);
    cmd_nbytes++;

    hexDump(hexbuf, cmd, cmd_nbytes);
    LOGF_INFO("CMD as Hex (%s)", hexbuf);

    //tcflush(fd, TCIOFLUSH);
    tcflush(fd, TCOFLUSH);

    if ((err = tty_write(fd, cmd, cmd_nbytes, &nbytes)) != TTY_OK)
    {
        LOGF_INFO("tty_write Error, %i bytes", nbytes);
        return -5;
    } else {
        LOGF_INFO("Wrote cmd buffer to tty, %i bytes", nbytes);
    }

}

int HASSTelescope::ReadResponse()
{
    int nbytes = 0;
    int err = TTY_OK;
    char rbuffer[BUFFER_SIZE];
    int bytesToStart = 0;
    memset(&rbuffer[0], 0, sizeof(rbuffer)); // clear the buffer

    bool recvInProgress = false;
    bool newData = false;
    int ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
    char hexbuf[3*BUFFER_SIZE];
        
    LOGF_INFO("---Begin ReadResponse()---","");

    hexDump(hexbuf, rbuffer, BUFFER_SIZE);
    LOGF_INFO("After clearing rbuffer it is: %s", hexbuf);
    
    // Look for a starting byte until time out occurs or BUFFER_SIZE bytes were read
    while (*rbuffer != START_BYTE && err == TTY_OK)
        //LOGF_INFO("*rbuffer is %c. START_BYTE is %c", *rbuffer, START_BYTE);
        err = tty_read(fd, rbuffer, 1, ARDUINO_TIMEOUT, &nbytes);
        LOGF_INFO("*tty_read: nbytes read is %i", nbytes);
        
        LOGF_INFO("Start byte? Read: %c", rbuffer[0]);
        bytesToStart++;

    LOGF_INFO("Found START_BYTE. A tty_read of %i bytes to find start byte.", bytesToStart);

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
    LOGF_INFO("Finished read. Received: %s", rbuffer);

    hexDump(hexbuf, rbuffer, BUFFER_SIZE);
    LOGF_INFO("rbuffer Hex (%s)", hexbuf);

    // Update telescope position variables  

    std::string str = rbuffer; 
    std::vector<std::string> v; 
    std::stringstream ss(str);

    while (ss.good()) { 
        std::string substr; 
        getline(ss, substr, ','); 
        v.push_back(substr); 
    } 
  
    for (size_t i = 0; i < v.size(); i++) {
        LOGF_INFO("%s", v[i].c_str());
    }

    currentSteps.stepRA = std::stoi(v[1].c_str());
    currentSteps.stepDec = std::stoi(v[2].c_str());
    LOGF_INFO("currentSteps.stepRA: %i, currentSteps.stepDec: %i", currentSteps.stepRA, currentSteps.stepDec);
    
}

bool HASSTelescope::Connect()
{
    const char*defaultPort = "/dev/ttyACM0";
    int err = TTY_OK;

    if (isConnected())
        return true;

    if (tty_connect(defaultPort, 9600, 8, 0, 1, &fd) != TTY_OK) {
        LOGF_INFO("tyy_connect failed.","");
        return false;
    } else {
        LOGF_INFO("tyy_connect succeeded. File Descriptor is: %i", fd);
    }

    ReadScopeStatus();

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

    SendCommand(REQUESTPOS_CMD);
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
    targetRA  = ra;
    targetDEC = dec;
    char RAStr[64]={0}, DecStr[64]={0};

    // Parse the RA/DEC into strings
    fs_sexa(RAStr, targetRA, 2, 3600);
    fs_sexa(DecStr, targetDEC, 2, 3600);

    // Mark state as slewing
    TrackState = SCOPE_SLEWING;

    // Inform client we are slewing to a new position
    LOGF_INFO("Slewing to RA: %s - DEC: %s", RAStr, DecStr);

    // Success!
    return true;
}

/**************************************************************************************
** Client is asking us to abort our motion
***************************************************************************************/
bool HASSTelescope::Abort()
{
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
    SendCommand(REQUESTPOS_CMD);
    ReadResponse();

    char RAStr[64]={0}, DecStr[64]={0};

    JD = ln_get_julian_from_sys();
    ln_get_date_from_sys(&date);
    sidereal = ln_get_apparent_sidereal_time(JD);
    curr_equ_posn.dec = 35.0;
    localSidereal = sidereal + observer.lng / 180 * 12;
    if (localSidereal > 24) localSidereal = localSidereal - 24;
    LOGF_INFO("Local sidereal time is %f", localSidereal);

    curr_equ_posn.ra = localSidereal; 
    //if (curr_equ_posn.ra > 360) curr_equ_posn.ra = curr_equ_posn.ra - 360;

    LOGF_INFO("EqN[AXIS_RA].value is %f",  EqN[AXIS_RA].value);
    LOGF_INFO("EqN[AXIS_DE].value is %f",  EqN[AXIS_DE].value);
    LOGF_INFO("Calling NewRaDec().",  EqN[AXIS_DE].value);
    NewRaDec(curr_equ_posn.ra, curr_equ_posn.dec);
    LOGF_INFO("EqN[AXIS_RA].value is %f",  EqN[AXIS_RA].value);
    LOGF_INFO("EqN[AXIS_DE].value is %f",  EqN[AXIS_DE].value);


    // Parse the RA/DEC into strings
    fs_sexa(RAStr, curr_equ_posn.ra, 2, 3600);
    fs_sexa(DecStr, curr_equ_posn.dec, 2, 3600);

    DEBUGF(DBG_SCOPE, "Current RA: %s Current DEC: %s", RAStr, DecStr);
    LOGF_INFO("ReadScopeStatus() Current RA: %f Current DEC: %f", curr_equ_posn.ra, curr_equ_posn.dec);

    return true;
}
