# TODO List
## Hardware
- [ ] Build the bots

## Software
- [x] Test individual parts
- [ ] Write libraries for parts
- [x] Getting data from sensors
- [x] Motor drivers (single and daisy chain)
- [x] Moving Drivebase
- [x] Analog multiplexer for line sensors
- [x] OpenMV camera UART comms
- [x] TOFSense get data through I2C and data processing
- [x] ICM-20948 get data through SPI
- [x] Kicker through software
- [x] Dribbler through software
- [x] Ballcap YGLidar
- [ ] TOFSense Lidar gate
- [x] UART comms between ESP32 and RP2040
- [x] I2C comms between ESP32 and RP2040s
- [x] Bluetooth comms between ESP32s
- [x] PID calculations
- [x] NeoPixel LED control
- [x] Main code structure
- [x] Convex hull, rotating calipers algorithm
- [x] Latency compensation algorithm on OpenMV
- [ ] Test front camera UART comms
- [ ] Edit top camera exposure / whitebal settings
- [ ] Test and obtain lidar gate threshold
- [ ] Implement lidar gate in main code
- [ ] Handle edge cases / bad data for coordinates (lidar ring)
- [ ] Handle case of cameras / ballcap lidar not seeing ball
- [ ] Test effectiveness of dummy points for rotating calipers
- [ ] PWM map to velocity and fit graph + edit motor libs
- [ ] Ballcap lidar library + integrate into main code
- [ ] Look ahead algorithm 
  - Inputs: ball coordinate, ball velocity (maybe)
  - Output: target coordinate for bot
- [ ] Proper ball track algorithm
- [ ] Proper ballcap condition
- [ ] Aiming / Scoring with dribbler and kicker
- [ ] IMU determine position using acceleration
- [ ] Add second IMU data
- [ ] Weighted average heading with 2 IMUs and heading obtained from lidar ring
- [ ] Weighted average coordinates through different sources
- [ ] Make use of line sensor data
- [ ] GAME LOGIC

## I Fried It
Fried RP2040-Zero Count: 4  
Fried ESP32-S3-Pico Count: 1  
Fried Motor Driver Count: ?
Fried TOFSense Count: ?

Last updated: 2025-03-24