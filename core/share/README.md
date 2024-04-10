
# Sample configuration files

This directory contains sub-directories with sample configuration files (JSON and DMC).

  * FlashXYZ:  Configuration files for SARRP Flash XYZ stages
  * EyeRobot3:  Configuration files for Eye Robot 3.0

The JSON file contains the following fields:

| Keyword      | Default   | Description                                     |
|:-------------|:----------|:------------------------------------------------|
| file_version |           | Version of JSON file format                     |
| name         |           | Descriptive name                                |
| IP_address   | "auto"    | IP address of Galil controller                  |
| direct_mode  | false     | Whether to directly connect to Galil controller |
| model        | 0         | Galil model (not recommended for normal use)    |
| DR_period_ms | 2         | Requested DR period in msec                     |
| DMC_file     | ""        | DMC file to download to Galil controller        |
| axes         |           | Array of axis configuration data (see below)    |
|  - index     |           | - channel index on Galil controller (0-7}       |
|  - type      |           | - prismatic (1) or revolute (2)                 |
|  - position_bits_to_SI |  | - conversion scale and offset (*)              |
|  -- scale    | 1         | -- scale factor                                 |
|  -- offset   | 0         | -- offset (in bits)                             |
|  - is_absolute | false   | - true if absolute encoder (false if incremental) |
|  - home_pos  | 0         | - home position                                 |
|  - position_limits |     | - upper and lower joint position limits         |
|  -- lower    | -MAX      | -- lower position limit                         |
|  -- upper    | +MAX      | -- upper position limit                         |

(*) The conversion (position_bits_to_SI) is applied as follows:

value_SI = (value_bits - offset)/scale
