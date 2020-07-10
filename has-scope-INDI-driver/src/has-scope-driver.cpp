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

#include <libnova/transform.h>
#include <libnova/julian_day.h>
#include <libnova/utility.h>
#include <libnova/ln_types.h>

#include "indicom.h"
#include "indicontroller.h"
#include "connectionplugins/connectionserial.h"

#include "has-scope-driver.h"

#define BUFFER_SIZE     40  // Maximum message length
#define ARDUINO_TIMEOUT 5   // fd timeout in seconds
#define START_BYTE 0x3C
#define END_BYTE 0x3E

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
} current, target;

struct {
    signed long stepRA {100};
    signed long stepDec {200};
} currentSteps, targetSteps;

//lnh_equ_posn hobject, hequ;
//ln_equ_posn object, equ;
//lnh_hrz_posn hhrz;
//

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

    // For libnova. Observer position (equatorial coordinates)
    hobserver.lng.degrees = 175;
    hobserver.lng.minutes = 12;
    hobserver.lng.seconds = 52.56;
    hobserver.lat.degrees = -37;
    hobserver.lat.minutes = 46;
    hobserver.lat.seconds = 24.60;

    ln_hlnlat_to_lnlat(&hobserver, &observer);

    updateLocation(latitude, longitude, elevation);

    // Set initial position of the telescope
    // Forks horizontal, nose resting on the bridge.
    hrz_posn.az = 180;
    hrz_posn.alt = 5;
    JD = ln_get_julian_from_sys();

    ln_get_equ_from_hrz(&hrz_posn, &observer, JD, &curr_equ_posn);

    NewRaDec(curr_equ_posn.ra, curr_equ_posn.ra);

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
   
    cmd_nbytes = sprintf(cmd, "<%c,%ld,%ld>", cmd_op, currentSteps.stepDec, currentSteps.stepRA);
    LOGF_INFO("cmd buffer is: %s", cmd);
    cmd_nbytes++;

    hexDump(hexbuf, cmd, cmd_nbytes);
    LOGF_INFO("CMD as Hex (%s)", hexbuf);

    tcflush(fd, TCIOFLUSH);

    if ((err = tty_write(fd, cmd, cmd_nbytes, &nbytes)) != TTY_OK)
    {
        LOGF_INFO("tty_write Error, %i bytes", nbytes);
        return -5;
    } else {
        LOGF_INFO("Wrote cmd buffer to tty, %i bytes", nbytes);
    }

    ReadResponse();
}

int HASSTelescope::ReadResponse()
{
    int nbytes = 0;
    int err = TTY_OK;
    char rbuffer[BUFFER_SIZE];
    int bytesToStart = 0;
    rbuffer[0] = 0x00;

    // Look for a starting byte until time out occurs or BUFFER_SIZE bytes were read
    while (*rbuffer != START_BYTE && err == TTY_OK)
        err = tty_read(fd, rbuffer, 1, ARDUINO_TIMEOUT, &nbytes);
        LOGF_INFO("Start byte? Read: %c", rbuffer[0]);
        bytesToStart++;

    LOGF_INFO("Found START_BYTE. tty_read of %i bytes.", bytesToStart);

    bool recvInProgress = false;
    bool newData = false;
    int ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

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
    tcflush(fd, TCIOFLUSH);
    LOGF_INFO("Finished read. Received: %s", rbuffer);
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

    SendCommand(REQUESTPOS_CMD);
    ReadResponse();

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
    /* static struct timeval ltv { 0, 0 };
    struct timeval tv { 0, 0 };
    double dt = 0, da_ra = 0, da_dec = 0, dx = 0, dy = 0;
    int nlocked;

    // update elapsed time since last poll, don't presume exactly POLLMS 
    gettimeofday(&tv, nullptr);

    if (ltv.tv_sec == 0 && ltv.tv_usec == 0)
        ltv = tv;

    dt  = tv.tv_sec - ltv.tv_sec + (tv.tv_usec - ltv.tv_usec) / 1e6;
    ltv = tv;

    // Calculate how much we moved since last time
    da_ra  = SLEW_RATE * dt;
    da_dec = SLEW_RATE * dt;

    // Process per current state. We check the state of EQUATORIAL_EOD_COORDS_REQUEST and act acoordingly 
    switch (TrackState)
    {
        case SCOPE_SLEWING:
            // Wait until we are "locked" into positon for both RA & DEC axis
            nlocked = 0;

            // Calculate diff in RA
            dx = targetRA - currentRA;

            // If diff is very small, i.e. smaller than how much we changed since last time, then we reached target RA.
            if (fabs(dx)  <= da_ra)
            
            {
                currentRA = targetRA;
                nlocked++;
            }
            // Otherwise, increase RA
            else if (dx > 0)
                currentRA += da_ra ;
            // Otherwise, decrease RA
            else
                currentRA -= da_ra;

            // Calculate diff in DEC
            dy = targetDEC - currentDEC;

            // If diff is very small, i.e. smaller than how much we changed since last time, then we reached target DEC.
            if (fabs(dy) <= da_dec)
            {
                currentDEC = targetDEC;
                nlocked++;
            }
            // Otherwise, increase DEC
            else if (dy > 0)
                currentDEC += da_dec;
            // Otherwise, decrease DEC
            else
                currentDEC -= da_dec;

            // Let's check if we reached position for both RA/DEC
            if (nlocked == 2)
            {
                // Let's set state to TRACKING
                TrackState = SCOPE_TRACKING;

                LOG_INFO("Telescope slew is complete. Tracking...");
            }
            break;

        default:
            break;
    } */

    SendCommand(REQUESTPOS_CMD);
    ReadResponse();

    char RAStr[64]={0}, DecStr[64]={0};

    hrz_posn.az = 180;
    hrz_posn.alt = 5;
    JD = ln_get_julian_from_sys();

    ln_get_equ_from_hrz(&hrz_posn, &observer, JD, &curr_equ_posn);

    NewRaDec(curr_equ_posn.ra, curr_equ_posn.ra);

    // Parse the RA/DEC into strings
    fs_sexa(RAStr, curr_equ_posn.ra, 2, 3600);
    fs_sexa(DecStr, curr_equ_posn.ra, 2, 3600);

    DEBUGF(DBG_SCOPE, "Current RA: %s Current DEC: %s", RAStr, DecStr);
    LOGF_INFO("ReadScopeStatus():  Current RA: %s Current DEC: %s", RAStr, DecStr);

    return true;
}
