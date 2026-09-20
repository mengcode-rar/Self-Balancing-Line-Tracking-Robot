# SBLTR

<<<<<<< HEAD
Self-balancing robot firmware for ESP32
=======
Firmware robot self-balancing ESP32
>>>>>>> 8f3cfc1e58407acc166720e2f240ac09491b7e51

## Code Structure

| File | Responsibility |
| --- | --- |
| `src/main.cpp` | Initializes the modules and runs the main loop. |
| `src/RobotController.cpp` | State machine, PID balancing, and fall handling. |
| `src/Imu.cpp` | MPU6050, calibration, roll angle, and complementary filter. |
| `src/Motors.cpp` | Motor direction, PWM, slew limiting, and PID output conversion. |
| `src/LineFollower.cpp` | Reading the five line sensors and steering correction. |
| `src/WebControl.cpp` | Wi-Fi, control page, and the `/set` and `/drive` endpoints. |
| `src/SerialTuning.cpp` | Tuning commands over Serial. |
| `include/HardwareConfig.h` | Pin assignments and hardware constants. |
| `include/RobotContext.h` | Shared parameters and state. |

The header for each module is located in `include/`. The initial values for pins, PID, line sensors, and control logic follow the original sketch. The motor module uses the appropriate PWM API for Arduino ESP32 core 2.x or 3.x.

## Getting Started

1. Make sure `include/WifiCredentials.h` exists. If it does not, copy `include/WifiCredentials.example.h` to that name and fill in your SSID and password. The local credentials file is excluded by `.gitignore`.
2. Run `pio run -e esp32dev` to build.
3. Run `pio run -e esp32dev -t upload` to flash the ESP32. Open the Serial monitor at 115200 baud.

Once connected to Wi-Fi, the robot's IP address is printed to Serial. Open that address in a browser to adjust the PID values and send direction commands.

## Notes from the Original Sketch

- The line sensor is enabled by default and can change the setpoint on every balancing cycle. As a result, forward/backward commands from the web interface may be overwritten on the next cycle.
- `MPU6050_light::update()` does not provide a read-failure status; the `readImu()` check still follows the original sketch's behavior. MPU presence is verified during initialization.
- The Wi-Fi connection in `setup()` blocks until it succeeds before robot control starts.