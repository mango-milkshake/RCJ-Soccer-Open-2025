import sensor, time, math, pyb
from ulab import numpy as np
from pyb import UART

threshold = (18, 100, 1, 64, 9, 80) # orange ball
x = 0
y = 0

sensor.reset()  # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565)  # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)  # Set frame size to QVGA (320x240)
sensor.skip_frames(time=500)  # Wait for settings take effect.
clock = time.clock()  # Create a clock object to track the FPS.
sensor.set_auto_gain(False)  # must be turned off for color tracking
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=5000)  # Disable auto exposure

uart = UART(3, 115200)
uart.init(115200, bits=8, parity=None, stop=1, timeout_char=1000)

H =   [-1.859358520295239 ,  -0.2475423844694668 ,  341.95168802449524 ,
       0.1593857662399504 ,  0.6168373514426762 ,  -701.4419970037706 ,
       0.00032056366322381105 ,  -0.20157299901006417 ,  1.0 ] #homography matrix calibrated 21/3
H_inv =   [-0.56132058638221612798,  -0.27385474401661052705,  -0.1486964956693853285,
          -0.00153211422104253632,  -0.0078510190489150984,  -4.9831254360538204693,
          -0.00012889387494789154376,  -0.0014947655750417743606,  -0.0044158718953420171454]

def Homography (H, x, y):
    n = H[6]*x + H[7]*y + H[8] #normalisation
    x2 = (H[0]*x + H[1]*y + H[2]) / n
    y2 = (H[3]*x + H[4]*y + H[5]) / n
    return x2, y2

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
    if len(blobs)>0:
        no_ball = False
        ball = max(blobs, key=lambda b: b.area())
        img.draw_rectangle(ball.rect(), color=(0,255,0))
        x = ball.cx()
        y = ball.cy() + ball.h() / 2
        x2, y2 = Homography(H, x, y)
        dist = (x2**2 + y2**2)**0.5

        cur_ball_x = round(x2 * 128)
        cur_ball_y = round(y2 * 128)

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
       # print("dy = ", dy)

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
        prevCoords = [x2,y2]
        prevTime = t

        x3 = x2 + vx*0.2 / 1 #position in 1 second [cm]
        y3 = y2 + vy*0.2 / 1
        x4, y4 = Homography(H_inv, x3, y3)
        img.draw_line(int(x),int(y),int(x4),int(y4),color=(255,0,255))

        next_ball_x = round(x3 * 128)
        next_ball_y = round(y3 * 128)
    else:
        no_ball = True

    uart.writechar(1)
    if(no_ball==False):
        if cur_ball_x < 0:
            uart.writechar(0)
            cur_ball_x = -cur_ball_x
        else:
            uart.writechar(1)
        uart.writechar(cur_ball_x & 0xFF)
        uart.writechar((cur_ball_x >> 8) & 0xFF)
        uart.writechar(cur_ball_y & 0xFF)
        uart.writechar((cur_ball_y >> 8) & 0xFF)

#        if next_ball_x < 0:
#            uart.writechar(0)
#            next_ball_x = -next_ball_x
#        else:
#            uart.writechar(1)
#        uart.writechar(next_ball_x & 0xFF)
#        uart.writechar((next_ball_x >> 8) & 0xFF)
#        uart.writechar(next_ball_y & 0xFF)
#        uart.writechar((next_ball_y >> 8) & 0xFF)
    else:
        for i in range(5):
            uart.writechar(0)
    uart.sendbreak()


