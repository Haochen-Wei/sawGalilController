
# Sample configuration files

This directory contains sub-directories with sample configuration files (JSON and DMC).

  * FlashXYZ:  Configuration files for SARRP Flash XYZ stages
  * EyeRobot3:  Configuration files for Eye Robot 3.0

The JSON file contains the following fields:

| Keyword       | Default   | Description                                           |
|:--------------|:----------|:------------------------------------------------------|
| file_version  |           | Version of JSON file format                           |
| IP_address    | "auto"    | IP address of Galil controller                        |
| direct_mode   | false     | Whether to directly connect to Galil controller       |
| model         | 0         | Galil model (not recommended for normal use)          |
| DR_period_ms  | 2         | Requested DR period in msec                           |
| DMC_file      | ""        | DMC file to download to Galil controller              |
|               |           |                                                       |
| robots        |           | Array of robot configuration data (see below)         |
|  - name       |           |  - robot name (used for provided interface)           |
|  - axes       |           |  - array of robot axis configuration data (see below) |
|  -- name      | ""        |  -- name of axis (if empty, will use Galil channel letter) |
|  -- index     |           |  -- channel index on Galil controller (typically 0-7} |
|  -- type      |           |  -- prismatic (1) or revolute (2)                     |
|  -- position_bits_to_SI | |  -- conversion scale and offset (*)                   |
|  --- scale    | 1         |  --- scale factor                                     |
|  --- offset   | 0         |  --- offset (in bits)                                 |
|  -- is_absolute | false   |  -- true if absolute encoder (false if incremental)   |
|  -- home_pos  | 0         |  -- home position                                     |
|  -- position_limits |     |  -- upper and lower joint position limits             |
|  --- lower    | -MAX      |  --- lower position limit                             |
|  --- upper    | +MAX      |  --- upper position limit                             |
|               |           |                                                       |
| analog_inputs | <>        | Array of analog input configuration data (see below)  |
|  - name       |           |  - descriptive name (used for provided interface)     |
|  - axes       |           |  - array of analog input axis configuration           |
|  -- index     |           |  -- channel index on Galil controller (typically 0-7) |
|  -- volts_to_SI |         |  -- conversion scale and offset (*)                   |
|  --- scale    | 1         |  --- scale factor                                     |
|  --- offset   | 0         |  --- offset (in bits)                                 |

(*) The conversion (position_bits_to_SI or volts_to_SI) is applied as follows:

value_SI = (value_bits - offset)/scale

<> indicates that the default is an empty vector
