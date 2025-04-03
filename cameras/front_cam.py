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

uart = UART(1, 115200)
uart.init(115200, bits=8, parity=None, stop=1, timeout_char=1000)

def sendVar(var):
    if (var < 0):
        uart.writechar(0)
    else:
        uart.writechar(1)
    var = round(abs(var) * 128)
    uart.writechar(var & 0xFF)
    uart.writechar((var >> 8) & 0xFF)
    #print(var & 0xFF, (var >> 8) & 0xFF)

prevCoords = [0,0]
prevTime = 0
v = 0
vel_values_x = []
vel_values_y = []
vel_values_length = 10
vel_values_x = [0] * vel_values_length
vel_values_y = [0] * vel_values_length
led2 = pyb.LED(2)

while True:
    clock.tick()  # Update the FPS clock.
    led2.on()
    img = sensor.snapshot()  # Take a picture and return the image.
    img.lens_corr(strength=1.48, zoom=1.0)
    blobs = img.find_blobs([threshold], area_threshold = 150, merge=True)
    if len(blobs)>0:
        no_ball = False
        ball = max(blobs, key=lambda b: b.area())
        img.draw_rectangle(ball.rect(), color=(0,255,0))
        img.draw_cross(ball.cx(), ball.cy(), color=(0,0,255))
        print(ball.cx(), ball.cy())
        x = ball.cx()
        y = ball.cy()
        x2 = 2.34678e-7* x**3 - 6.93269e-7* x**2 * y + 0.0000184518 * x**2 + 4.94946e-9 *x * y**2 + 0.000340658*x*y - 0.0405575*x + 1.26915e-7 * y**3 - 0.0000321536*y**2 - 0.0783178*y + 9.3546
        y2 = -2.17938e-6 * x**3 - 3.57085e-8 * x**2 * y + 0.00129718 * x**2 - 3.14209e-7 * x * y**2 + 0.0000760482 * x * y - 0.27871*x + 6.65652e-7 * y**3 - 0.0001672 * y**2 + 0.00840698*y + 33.1509

        dist = (x2**2 + y2**2)**0.5

        print("dist = ", dist)
        #print("x = ", x) #image coordinates
        #print("y = ", y)
        #print("x2 = ", x2) #actual coordinates
        #print("y2 = ", y2)

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

        #log previous values
        prevCoords = [x2,y2]
        prevTime = t

        x3 = x2 + vx*0.2 / 1 #position in 1 second [cm]
        y3 = y2 + vy*0.2 / 1

        print ("x3 =", x3)
        print ("y3 =", y3)
        print ("vx =", vx)
        print ("vy =", vy)

    else:
        no_ball = True

    uart.writechar(5)
    if(no_ball==False):
        sendVar(x3)
        sendVar(y3)
        sendVar(vx)
        sendVar(vy)
    else:
        for _ in range(12):
            uart.writechar(0)
    uart.sendbreak()

    print("fps: ", clock.fps())

