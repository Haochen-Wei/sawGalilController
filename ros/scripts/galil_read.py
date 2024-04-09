#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2024-02-28

# (C) Copyright 2024 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

# Start the galil_controller node
# > rosrun galil_controller galil_controller

# To communicate with the controller using ROS topics, run this script
# > rosrun galil_controller read_galil -c /galil

import crtk
import galil_controller

# example of application using controller.py
class example_application:

    # configuration
    def __init__(self, ral, robot_name, expected_interval):
        self.expected_interval = expected_interval
        self.controller = galil_controller.controller(ral = ral,
                                                      controller_name = robot_name,
                                                      expected_interval = expected_interval)
        
    # homing example
    def home(self):
        print('checking connections')
        self.controller.ral().check_connections()
        print('starting enable')
        if not self.controller.enable(10):
            sys.exit('failed to enable within 10 seconds')
        print('starting home')
        if not self.controller.home(10):
            sys.exit('failed to home within 10 seconds')

    # read data
    def read(self):
        print(self.controller.measured_jp())
        print(self.controller.measured_jv())
        print(self.controller.setpoint_jp())

    # main method
    def run(self):
        self.home()
        self.xyz_motion()

if __name__ == '__main__':
    # extract ros arguments (e.g. __ns:= for namespace)
    argv = crtk.ral.parse_argv(sys.argv[1:]) # skip argv[0], script name

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--controller', type=str, required=True,
                        help = 'controller name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
    parser.add_argument('-i', '--interval', type=float, default=0.01,
                        help = 'expected interval in seconds between messages sent by the device')
    args = parser.parse_args(argv)

    ral = crtk.ral('galil_controller')
    application = example_application(ral, args.controller, args.interval)
    ral.spin_and_execute(application.run)
