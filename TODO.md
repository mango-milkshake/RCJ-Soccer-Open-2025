# TODO List
## Hardware
- Build the bots

## Software
- TOFSense Lidar gate
- Test front camera UART comms
- Edit top camera exposure / whitebal settings
- Test and obtain lidar gate threshold
- Implement lidar gate in main code
- Handle edge cases / bad data for coordinates (lidar ring)
- Handle case of cameras / ballcap lidar not seeing ball
- Test effectiveness of dummy points for rotating calipers
- PWM map to velocity and fit graph + edit motor libs
- Ballcap lidar library + integrate into main code
- Look ahead algorithm 
  - Inputs: ball coordinate, ball velocity (maybe)
  - Output: target coordinate for bot
- Proper ball track algorithm
- Proper ballcap condition
- Aiming / Scoring with dribbler and kicker
- IMU determine position using acceleration
- Weighted average coordinates through different sources
- Make use of line sensor data
- GAME LOGIC

## I Fried It
Fried RP2040-Zero Count: 4  
Fried ESP32-S3-Pico Count: 1  
Fried Motor Driver Count: 5  
Fried TOFSense Count: ?

Last updated: 2025-03-26