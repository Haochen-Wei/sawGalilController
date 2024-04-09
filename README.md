
# sawGalilController

This SAW package contains a component (mtsGalilController) that interfaces to a Galil Controller
using the gclib library with the Data Record (DR) interface.

With the DR interface, the Galil controller periodically sends a binary packet of state information to the PC.
The period can be as high as 500 Hz, depending on the controller model number.
The PC sends commands to the Galil controller via ASCII strings.

The component is designed to be generic, and is configured using both JSON and DMC files.
The JSON file specifies the high-level configuration data, such as the controller connectivity information
and conversions between controller units (e.g., encoder counts) and SI units.
The DMC file handles the low-level Galil configuration. Note that the DMC filename is specified in
the JSON file. Examples of JSON and DMC files can be found in the [share](./core/share) subdirectory,
with the JSON file format documented in the corresponding [README](./core/share/README.md).

Most of the source code is in the [core](./core) subdirectory to facilitate building with ROS1 or ROS2.

## Download and build

You will first need to download the Galil libraries from https://www.galil.com/downloads/api.  Instructions to install the *gclib* can be found following: *Downloads* ➡ *Software* ➡ *gclib | Galil APIs*.  On Linux, you will need sudo privileges to install the software.

If you're using ROS (1 or 2), the simplest way to download and compile this repository and all the dependencies is to use *vcs*: https://github.com/jhu-saw/vcs
