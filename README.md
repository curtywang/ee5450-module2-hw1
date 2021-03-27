# ee5450-module2-hw1: MQTT streaming of sensors



## Project file structure
| File name | Purpose |
| ------------------- | ------- |
| `Core/Inc/main.h` | System data structures |
| `Core/Src/main.c` | Thread setup and entry functions |
| `Core/Inc/sensor_telemetry_setup.h` | Constants for MQTT telemetry setup |
| `Core/Src/sensor_telemetry_setup.c` | Functions to setup NetX Duo and MQTT client for sensor telemetry |
| `Core/Inc/sensor_telemetry.h` | Declarations for sensor telemetry functions (MQTT message publishing) |
| `Core/Inc/sensor_telemetry.h` | Definitions for sensor telemetry functions (MQTT message publishing) |
| `mosquitto.conf` | Mosquitto configuration file (run `mosquitto.exe` with `-c` switch) |

## System data structures
### `main.h`: `struct global_data_t`: Global data structure
You'll notice upon opening `main.c` that a significant reformatting
has occurred.  There are only three global variables now, but in place
of the previous global variables, there is a new `struct` type
named `global_data_t`.  This data structure replaces our global data
to allow us to use test harnesses in the future.  Rather than having the
test harness attempt to modify globals, the test harness can generate
dummy `struct global_data_t` objects instead. 

This data structure contains members to ensure the operation of the system
successfully, including the ThreadX primitives and structures, NetX primitives
and structures, and the thread input parameters. This also allows for
threads and functions that require multiple inputs to be used with the
ThreadX `tx_thread_create()` interface, which only accepts one `ULONG` 
input (which will be the pointer to the global data structure).

Note the one caveat, that this type definition is available globally
in `main.c`, but the actual pointer is only allocated inside
`tx_application_define` to take advantage of ThreadX's byte allocation
feature.

### `main.c`: `static TX_EVENT_FLAGS_GROUP global_event_flags`: system-wide software interrupts
This global event flag group is to provide system-wide software interrupts. 
The event `#define`s are public in `main.h` and begin with `EVT_`.


## Task to complete this assignment
Your tasks are to modify `main.c`, `sensor_telemetry.h`, and `sensor_telemetry.c`
accordingly to achieve the following aim.  Two examples have been provided, 
as shown in the table. 

### System aim (purpose)
Aim: Use the sensors available on your B-L4S5I-IOT01A board
and MQTT to stream the following sensor data at roughly 1Hz:

| sensor type | Sensor | uC Conn. | MQTT topic | Example? |
| ----------- | ------ | -------- | ---------- | -------- |
| temperature | HTS221 | `I2C2` | `board_test/temperature` | Yes |
| acceleration (x, y, z) | LSM6DSL | `I2C2` | `board_test/accelerometer` | Yes |
| relative humidity | HTS221 | `I2C2` | `board_test/humidity` | No |
| angular velocity (x, y, z) | LSM6DSL | `I2C2` | `board_test/gyroscope` | No |
| magnetic direction | LIS3MDL | `I2C2` | `board_test/magnetometer` | No |
| air pressure | LPS22HB | `I2C2` | `board_test/air_pressure` | No |

### Tasks
1. Begin by modifying the `sensor_telemetry_setup.h` `#define`s for your own networking
   setup.  If you need to find your own IP address, run `ipconfig /all` in PowerShell
   or the Command Prompt.  Note that you may need to check this every time you turn 
   your computer on/off unless you have a reserved IP at your router end.
2. Install [MQTT Explorer](http://mqtt-explorer.com/) so you can more easily view the 
   sensor data that is streamed.
3. Run mosquitto on your computer, using the `mosquitto.conf` configuration that 
   allows for unauthenticated access.
4. Compile and use the debug button (bug-looking icon) to run the system as-is on
   your board. There is a breakpoint setup right after `setup_wifi()` since
   this part always takes a while. Make sure to hit "continue" to go to the
   rest of the program.
   
   Make sure that you receive the "hellos" and the accelerometer
   and temperature data before continuing.
5. Add the necessary threads and string formatting functions to 
   successfully accomplish the system's aim.
