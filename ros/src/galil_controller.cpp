/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-02-28

  (C) Copyright 2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstCommon/cmnQt.h>
#include <cisstMultiTask/mtsTaskManager.h>
#include <cisstMultiTask/mtsSystemQtWidget.h>
#include <cisstParameterTypes/prmStateRobotQtWidget.h>

#include <sawGalilController/mtsGalilController.h>

#include <QApplication>
#include <QMainWindow>

#include <cisst_ros_crtk/mts_ros_crtk_bridge.h>

int main(int argc, char * argv[])
{
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClassMatching("mtsGalilController", CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // create ROS node handle
    cisst_ral::ral ral(argc, argv, "joystick");
    auto rosNode = ral.node();

    // parse options
    cmnCommandLineOptions options;
    std::string jsonConfigFile = "";
    double rosPeriod = 2.0 * cmn_ms;
    double tfPeriod = 20.0 * cmn_ms;
    std::list<std::string> managerConfig;

    options.AddOptionOneValue("j", "json-config",
                              "json configuration file",
                              cmnCommandLineOptions::REQUIRED_OPTION, &jsonConfigFile);
    options.AddOptionOneValue("p", "ros-period",
                              "period in seconds to read all tool positions (default 0.002, 2 ms, 500Hz).  There is no point to have a period higher than the device",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &rosPeriod);
    options.AddOptionOneValue("P", "tf-ros-period",
                              "period in seconds to read all components and broadcast tf2 (default 0.02, 20 ms, 50Hz).",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &tfPeriod);
    options.AddOptionMultipleValues("m", "component-manager",
                                    "JSON files to configure component manager",
                                    cmnCommandLineOptions::OPTIONAL_OPTION, &managerConfig);
    options.AddOptionNoValue("D", "dark-mode",
                             "replaces the default Qt palette with darker colors");

    // check that all required options have been provided
    if (!options.Parse(ral.stripped_arguments(), std::cerr)) {
        return -1;
    }

    // create the components
    mtsGalilController * galilController = new mtsGalilController("GalilController");
    galilController->Configure(jsonConfigFile);

    // add the components to the component manager
    mtsManagerLocal * componentManager = mtsComponentManager::GetInstance();
    componentManager->AddComponent(galilController);

    // ROS CRTK bridge
    mts_ros_crtk_bridge_provided * crtk_bridge
        = new mts_ros_crtk_bridge_provided("galil_controller_crtk_bridge", rosNode);
    componentManager->AddComponent(crtk_bridge);
    crtk_bridge->bridge_interface_provided(galilController->GetName(), "control",
                                           rosPeriod, tfPeriod);
    crtk_bridge->Connect();

    // create a Qt user interface
    QApplication application(argc, argv);
    cmnQt::QApplicationExitsOnCtrlC();
    if (options.IsSet("dark-mode")) {
        cmnQt::SetDarkMode();
    }

    // organize all widgets in a tab widget
    QTabWidget * tabWidget = new QTabWidget;

    prmStateRobotQtWidgetComponent * stateWidget
        = new prmStateRobotQtWidgetComponent("Galil-State");
    stateWidget->SetPrismaticRevoluteFactors(1.0 / cmn_mm, cmn180_PI);
    stateWidget->Configure();
    componentManager->AddComponent(stateWidget);
    componentManager->Connect(stateWidget->GetName(), "Component",
                              galilController->GetName(), "control");
    tabWidget->addTab(stateWidget, "State");

    mtsSystemQtWidgetComponent * systemWidget
        = new mtsSystemQtWidgetComponent("Galil-System");
    systemWidget->Configure();
    componentManager->AddComponent(systemWidget);
    componentManager->Connect(systemWidget->GetName(), "Component",
                              galilController->GetName(), "control");
    tabWidget->addTab(systemWidget, "System");

    // custom user components
    if (!componentManager->ConfigureJSON(managerConfig)) {
        CMN_LOG_INIT_ERROR << "Configure: failed to configure component-manager, check cisstLog for error messages" << std::endl;
        return -1;
    }

    // create and start all components
    componentManager->CreateAllAndWait(5.0 * cmn_s);
    componentManager->StartAllAndWait(5.0 * cmn_s);

    // run Qt user interface
    tabWidget->show();
    application.exec();

    // stop all logs
    cmnLogger::Kill();

    // stop ROS node
    cisst_ral::shutdown();

    // kill all components and perform cleanup
    componentManager->KillAllAndWait(5.0 * cmn_s);
    componentManager->Cleanup();

    return 0;
}
