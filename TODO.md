# TODO List
## Hardware
- Build the bots

## Software
- retune PID
- possible top plate latency compensation
- Test scoring and aiming with current top plate and ballcap
  - top plate should be done, includes: 
    - rotating calipers for bot coordinate
    - heading obtained from weighted average of:
      - IMU: change in yaw between current and previous reading + previous calculated (fused) heading
      - rotating calipers algorithm (angle of rotation of fitted rectangle)
  - ballcap temporarily use lidar gate only
  - make sure bot can aim and score
  - if possible try involving kicker
- Test front camera UART comms
- Handle case of cameras / ballcap lidar not seeing ball
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
Fried RP2040-Zero Count: 5  
Fried ESP32-S3-Pico Count: 1  
Fried Motor Driver Count: 5  
Fried TOFSense Count: ?  
Fried Solenoid Count: 1

Last updated: 2025-03-28 11:11AM