/*
   Hamilton Astronomical Society 24" Telescope INDI Driver

*/

/** \file has-scope-driver.h
    \brief Construct a basic INDI telescope device that simulates GOTO commands.
    \author Richard Croy

    \example has-scope-driver.h
    A simple GOTO telescope that simulator slewing operation.
*/

#pragma once

#include "inditelescope.h"

class HASSTelescope : public INDI::Telescope
{
  public:
    HASSTelescope();

  protected:
    bool Connect() override;
    bool Handshake() override;
    const char *getDefaultName() override;
    bool initProperties() override;

    // Telescope specific functions
    bool ReadScopeStatus() override;
    bool Goto(double, double) override;
    bool Abort() override;
    int ReadResponse();
    bool SendCommand(char cmd_op);

  private:

    // Debug channel to write mount logs to
    // Default INDI::Logger debugging/logging channel are Message, Warn, Error, and Debug
    // Since scope information can be _very_ verbose, we create another channel SCOPE specifically
    // for extra debug logs. This way the user can turn it on/off as desired.
    uint8_t DBG_SCOPE { INDI::Logger::DBG_IGNORE };

    static const uint8_t SLEW_RATE = 3; // slew rate, degrees/s
    int fd; // Port file descriptor
};
