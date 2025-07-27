# RCJ Soccer Open 2025
## Introduction
This repository documents the software for Team Gloves in RoboCup Junior Soccer Open 2025. 

For our hardware CAD files, refer to [this repository](https://github.com/JiunnXiang/gloves-25-hardware).
### Repository Branches
- `nationals` [branch](https://github.com/mango-milkshake/RCJ-Soccer-Open-2025/tree/nationals): This is an older version of our software, used when we competed in RoboCup Singapore Open 2025. 
- `internationals` [branch](https://github.com/mango-milkshake/RCJ-Soccer-Open-2025/tree/internationals) (default): This is the newer updated version of our software, used when we competed in RoboCup Internationals 2025 in Salvador, Brazil. There have been quite a large amount of major code revamping and updates since our national competition, hence we forked out a new branch for it.

## Microcontrollers
Our robot makes use of a total of seven microcontrollers – one ESP32-S3-WROOM, four RP2040-Zeros, and two STM32H743VI on the OpenMV H7 cameras.
The distribution of our microcontrollers are as follows: 
- **Top Plate**
  - [`rp2040_top`](https://github.com/mango-milkshake/RCJ-Soccer-Open-2025/tree/internationals/rp2040_top): **RP2040-Zero**, which receives data from our LiDAR ring and IMUs, and converts the data into useful localisation information: robot coordinates and heading. Data is communicated to the ESP32 through UART.
- **Middle Plate**
  - [`esp32_mid`](https://github.com/mango-milkshake/RCJ-Soccer-Open-2025/tree/internationals/esp32_mid): **ESP32-S3-WROOM**, which is our main controller and handles integrating data from all sources and our overall game logic and strategies.
  - [`rp2040_upper`](https://github.com/mango-milkshake/RCJ-Soccer-Open-2025/tree/internationals/rp2040_upper): **RP2040-Zero**, which receives data from both OpenMV cameras and converts that into one final set of ball angle and distance relative to the robot. It was also intended for receiving and processing data from six Vl53L5CX ToF sensors for bot detection, however, this was not used in the end due to lack of accuracy and time for effective implementation into our strategies. Data is communicated to the ESP32 through I2C.
  - [`cameras`](https://github.com/mango-milkshake/RCJ-Soccer-Open-2025/tree/internationals/cameras): both **STM32H743VI** of the OpenMV cameras, which make use of OpenMV libraries for machine vision to detect ball and goal positions. Data is communicated to the ESP32 through UART.
- **Bottom Plate**
  - [`rp2040_lower`](https://github.com/mango-milkshake/RCJ-Soccer-Open-2025/tree/internationals/rp2040_lower): **RP2040-Zero**, which controls our motors and DRV8245 motor drivers. Data is received from the ESP32 through UART.
  - [`rp2040_bottom`](https://github.com/mango-milkshake/RCJ-Soccer-Open-2025/tree/internationals/rp2040_bottom): **RP2040-Zero**, which receives data from our ballcap LiDAR sensor and analog light sensors, though the light sensors for line detection ended up unused due to sufficiently accurate and reliable absolute positioning through the top plate localisation system. Data is communicated to the ESP32 through I2C.
  - > The bottom plate folders were originally named b1 and b2, but were renamed as such due to a potential design idea to move the motor drivers onto the underside of the middle plate. Though the design change unfortunately did not carry through, we kept the folder names as a memory. 

## Project Structure
Our processes are divided cleanly into multiple subsystems. In this way, once one subsystem is stabilised and finalised, we mostly do not have to edit the code for that at all anymore.

In roughly the order of how data is transferred, our subsystems are:
- `rp2040_top`: **Localisation** (LiDARs and IMUs). Sensor data is converted into one set of robot coordinates and heading.
- `cameras`: **Vision** (Cameras). Detects ball and goals through machine vision by analysing pixel colours.
- `rp2040_upper`: **Vision** (Cameras). Camera data is converted into one set of ball angle and distance relative to the robot, as well as some additional information as needed such as whether the goal is clear for scoring.
- `rp2040_bottom`: **Ballcap and Line Detection** (LiDAR and light sensors). Detects ballcap status (including whether the ball is in the left, middle, or right side of the ballcap area), as well as line status (whether the light sensor is on a white line). 
- `esp32_mid`: **Overall main microcontroller**, subsystems integration, game logic and strategies, Dribbler and Kicker control, ESPNow communication between robots. Makes use of all above data to decide on a strategy and the robot's desired movement, and conveys the movement requirement to the RP2040 driving the motors below.
  - Data from various sources is organised through a `Data.h` library. This splits up the data into various categories such as localisation and ball information, for use in both the functions library and the main logic. Additionally, we store various state variables and a list of strategies to be used. 
  - General robot functions are placed in a `Bot.h` library, such as various ball tracking and scoring functions. These can be used and combined in different ways to create many flexible game strategies quickly and concisely in the main logic code. 
- `rp2040_lower`: **Drivebase** (Motor Drivers and Motors). Controls robot movement through a set of translation (in each wheel axis) and rotation PWM values obtained from the main ESP32 microcontroller. There is an additional maximum motor speed cap set here for safety purposes. 

## Usage
### ESP32 and RP2040
Our software for the ESP32 and RP2040 were mainly written in C++ / Arduino framework within the [PlatformIO](https://docs.platformio.org/en/latest/) environment. PlatformIO’s unified environment allows for cross-platform builds for both the RP2040 and ESP32 boards, allowing our software development to be standardized across all our microcontrollers. For the RP2040, we use the [Arduino-Pico core](https://arduino-pico.readthedocs.io/en/latest/platformio.html).

PlatformIO can be installed as [CLI](https://docs.platformio.org/en/latest/core/index.html#piocore) or [IDE](https://docs.platformio.org/en/latest/integration/ide/pioide.html#pioide) (also a VSCode extension). Once installed, open **each** microcontroller's folder **individually** in a separate VSCode workspace, for PlatformIO to successfully identify it as a configurable project. 

### File Structure
Each of the ESP32 and RP2040 folders consists of various folders and files:
- `src/`: all the relevant source code.
  - `lib/`: all the relevant libraries used.
    - `public/`: libraries obtained from sources online.
    - `private/`: self-created libraries for more specific uses.
  - `test/`: relevant testing code, usually for testing either specific parts or small parts of the subsystem.
  - `main.cpp` and other code files: the main code we run on the microcontroller. Sometimes, there are different versions for various uses.
- `.gitignore`: gitignore file
- `platformio.ini`: PlatformIO's project configuration file. Specify the platform and boards used here, as well as other configs like build flags. You can define many different environments and the corresponding code file to upload and run.

### OpenMV H7 Cameras (STM32H743VI)
Our cameras were programmed in MicroPython using the OpenMV IDE. We have many different versions in the repository `cameras/` folder, but the final used ones are `top_cam_new.py` for our upward-facing camera pointed at a custom-made distortion-free mirror, and `front_cam_goal.py` for our front-facing camera. 

Calibration:
- colour thresholds for orange ball (both cameras) and yellow/blue goals (front camera, for checking if goal is open to score), using the OpenMV IDE Threshold Editor.
- camera centre position coordinates (top camera) using the OpenMV IDE Frame Buffer.
- pixel to real distance (both cameras): experimentally obtain data values, then fit a curve using [Mathematica](https://github.com/mango-milkshake/RCJ-Soccer-Open-2025/blob/internationals/cameras/localization%20fitting%20code.nb) or similar methods.

### Other files
- `general/`: some general tools for testing, such as I2C scanner.
- `unused/`: testing code and libraries for parts that ended up unused in the end due to being unsuitable. This was mostly in the early stages when we were sourcing for the most suitable sensors, hence trying out many different models. 
