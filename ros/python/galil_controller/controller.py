#  Author(s):  Anton Deguet
#  Created on: 2021-03-04

# (C) Copyright 2021-2024 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import crtk
import numpy

class controller(object):
    """Simple controller API wrapping around ROS messages
    """
    # initialize the controller
    def __init__(self, ral, controller_name, expected_interval = 0.01):
        """Constructor.  This initializes a few data members.It
        requires a controller name, this will be used to find the ROS
        topics for the controller being controlled.  For example if the
        user wants `controller`, the ROS topics will be from the namespace
        `controller`"""
        # ros stuff
        self.__ral = ral.create_child(controller_name)
        self.__controller_name = controller_name

        # crtk features
        self.__crtk_utils = crtk.utils(self, self.__ral, expected_interval)

        # add crtk features that we need and are supported
        self.__crtk_utils.add_measured_js()
        self.__crtk_utils.add_setpoint_js()
        self.__crtk_utils.add_servo_jp()
        self.__crtk_utils.add_servo_jv()

    def ral(self):
        return self.__ral

    def name(self):
        return self.__controller_name
