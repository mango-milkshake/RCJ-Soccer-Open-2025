# TODO List
## Hardware
- Build the bots

## Software
- TOFSense Lidar gate
- Implement lidar gate in main code
- retune PID
- Test scoring and aiming with current top plate and ballcap
  - top plate should be done, includes: 
    - rotating calipers for bot coordinate
    - heading obtained from weighted average of:
      - IMU: change in yaw between current and previous reading + previous calculated (fused) heading
      - rotating calipers algorithm (angle of rotation of fitted rectangle)
  - ballcap temporarily use lidar gate only
  - recheck moving to target coordinate
  - make sure bot can aim and score
  - if possible try involving kicker
- Test front camera UART comms
- Test and obtain lidar gate threshold
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
Fried RP2040-Zero Count: 4  
Fried ESP32-S3-Pico Count: 1  
Fried Motor Driver Count: 5  
Fried TOFSense Count: ?

Last updated: 2025-03-26 11:40PM <t:1743003600:F>