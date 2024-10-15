/*-*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-   */
/*ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab:*/

/*
  Author(s): Peter Kazanzides

  (C) Copyright 2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <string>

#include <cisstCommon/cmnLogger.h>
#include <cisstCommon/cmnKbHit.h>
#include <cisstCommon/cmnGetChar.h>
#include <cisstCommon/cmnConstants.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsManagerLocal.h>
#include <cisstMultiTask/mtsTaskContinuous.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

#include <sawGalilController/mtsGalilController.h>

class GalilClient : public mtsTaskMain {

private:
    size_t NumAxes;

    mtsDoubleVec values;
    mtsFunctionRead GetConnected;
    mtsFunctionRead get_analog;

    void OnStatusEvent(const mtsMessage &msg) {
        std::cout << std::endl << "Status: " << msg.Message << std::endl;
    }
    void OnWarningEvent(const mtsMessage &msg) {
        std::cout << std::endl << "Warning: " << msg.Message << std::endl;
    }
    void OnErrorEvent(const mtsMessage &msg) {
        std::cout << std::endl << "Error: " << msg.Message << std::endl;
    }

public:

    GalilClient() : mtsTaskMain("GalilClient"), NumAxes(0)
    {
        mtsInterfaceRequired *req = AddInterfaceRequired("Input", MTS_OPTIONAL);
        if (req) {
            req->AddFunction("GetConnected", GetConnected);
            req->AddFunction("GetAnalogInput", get_analog);
            req->AddEventHandlerWrite(&GalilClient::OnStatusEvent, this, "status");
            req->AddEventHandlerWrite(&GalilClient::OnWarningEvent, this, "warning");
            req->AddEventHandlerWrite(&GalilClient::OnErrorEvent, this, "error");
        }
    }

    void Configure(const std::string&) {}

    void PrintHelp()
    {
        std::cout << "Available commands:" << std::endl
                  << "  q: quit" << std::endl;
    }

    void Startup()
    {
        PrintHelp();
    }

    void Run() {

        ProcessQueuedEvents();

        bool galilOK;
        GetConnected(galilOK);

        char c = 0;
        if (cmnKbHit()) {
            c = cmnGetChar();
            switch (c) {

            case 'q':   // quit program
                std::cout << std::endl << "Exiting.. " << std::endl;
                this->Kill();
                break;

            }
        }

        if (galilOK) {
            get_analog(values);
            printf("analog: ");
            for (size_t i = 0; i < values.size(); i++)
                printf("%lf ", values[i]);
            printf("\r");
        }
        else {
            printf("Galil not connected\r");
        }

        osaSleep(0.01);  // to avoid taking too much CPU time
    }

    void Cleanup() {}

};

int main(int argc, char **argv)
{
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    // Using SendStatus, SendWarning and SendError instead
    // cmnLogger::AddChannel(std::cout, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    if (argc < 3) {
        std::cout << "Syntax: sawGalilAnalog <config> <interface>" << std::endl;
        std::cout << "        <config>      Configuration file (JSON format)" << std::endl;
        std::cout << "        <interface>   Interface name" << std::endl;
        return 0;
    }

    std::cout << "Starting mtsGalilController" << std::endl;
    mtsGalilController *galilServer;
    galilServer = new mtsGalilController("galilServer");
    galilServer->Configure(argv[1]);

    mtsComponentManager * componentManager = mtsComponentManager::GetInstance();
    componentManager->AddComponent(galilServer);

    GalilClient client;
    componentManager->AddComponent(&client);

    if (!componentManager->Connect(client.GetName(), "Input", galilServer->GetName(), argv[2])) {
        std::cout << "Failed to connect: "
            << client.GetName() << "::Input to "
            << galilServer->GetName() << "::" << argv[2] << std::endl;
        delete galilServer;
        return -1;
    }

    componentManager->CreateAll();
    componentManager->StartAll();

    // Main thread passed to client task

    galilServer->Kill();
    componentManager->WaitForStateAll(mtsComponentState::FINISHED, 2.0 * cmn_s);

    componentManager->Cleanup();

    // stop all logs
    cmnLogger::SetMask(CMN_LOG_ALLOW_NONE);
    delete galilServer;

    return 0;
}
