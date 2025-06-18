import sensor
import time
import math
from ulab import numpy as np

threshold = (20, 87, 12, 80, 26, 74) # orange ball
x = 0
y = 0

sensor.reset()  # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565)  # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)  # Set frame size to QVGA (320x240)
sensor.skip_frames(time=200)  # Wait for settings take effect.
clock = time.clock()  # Create a clock object to track the FPS.
sensor.set_auto_gain(False)  # must be turned off for color tracking
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=6000)  # Disable auto exposure



prevCoords = [0,0]
prevTime = 0
v = 0
vel_values_x = []
vel_values_y = []
vel_values_length = 10
vel_values_x = [0] * vel_values_length
vel_values_y = [0] * vel_values_length

while True:
    clock.tick()  # Update the FPS clock.
    img = sensor.snapshot()  # Take a picture and return the image.
    img.lens_corr(strength=1.48, zoom=1.0)
    blobs = img.find_blobs([threshold], area_threshold = 150, merge=True)
    img.draw_cross(120, 120, color=(0, 255, 0));
    img.draw_cross(310, 120, color=(0, 255, 0));
    for blob in blobs:
        ball = max(blobs, key=lambda b: b.area())
        img.draw_rectangle(ball.rect(), color=(0,255,0))
        img.draw_cross(ball.cx(), ball.cy(), color=(0,0,255))

        print(ball.cx(), ball.cy())
        x = ball.cx()
        y = ball.cy()

        x2 = (
            1.46886e-20   * x**3
            - 3.0779e-6   * x**2 * y
            + 0.000369348 * x**2
            - 3.91234e-21 * x * y**2
            + 0.00158733  * x * y
            - 0.190479    * x
            - 1.20828e-6  * y**3
            + 0.000434979 * y**2
            - 0.277897    * y
            + 29.1719
        )

        y2 = (
            -7.9634e-6    * x**3
            + 2.7854e-20   * x**2 * y
            + 0.00535085  * x**2
            + 9.05357e-8   * x * y**2
            - 0.0000217286* x * y
            - 1.21978     * x
            - 4.6143e-21  * y**3
            + 0.0000474553* y**2
            - 0.0113893   * y
            + 110.458
        )
        dist = (x2**2 + y2**2)**0.5
        print("dist = ", dist)
        #print("x = ", x) #image coordinates
        #print("y = ", y)
        print("x2 = ", x2) #actual coordinates
        print("y2 = ", y2)

        #update values
        coords = [x2,y2]
        t = time.ticks_ms()

        dx = coords[0]-prevCoords[0]
        dy = coords[1]-prevCoords[1]
        #print("dx = ", dx)
        #print("dy = ", dy)

        dt = (t-prevTime)/1000
        #print ("dt =", dt) #~25ms

        #calculate velocity
        if (dt != 0):
            vx = dx/dt #cm/s
            vy = dy/dt


        #find average velocity
        for i in range(vel_values_length-1):
            vel_values_x[i] = vel_values_x[i+1]
            vel_values_y[i] = vel_values_y[i+1]
        vel_values_x[-1] = vx
        vel_values_y[-1] = vy
        vx = sum(vel_values_x)/len(vel_values_x)
        vy = sum(vel_values_y)/len(vel_values_y)
        #print ("vx =", vx)
        #print ("vy =", vy)

        #log previous values
