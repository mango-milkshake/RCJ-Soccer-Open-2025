import sensor, time, math, pyb
from pyb import UART

def pix_to_real(r_px):
    return (-0.7938274173017964
            + 1.5294566788791972*r_px
            - 0.016771506764266992*(r_px**2)
            + 0.00024722073358174443*(r_px**3)
            - 1.2846712889870513e-6*(r_px**4))

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)

sensor.set_gainceiling(32)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_contrast(0)
sensor.set_auto_exposure(False, exposure_us=6000)
sensor.set_auto_gain(False, gain_db=-8)
sensor.set_auto_whitebal(False)
sensor.set_contrast(3)

sensor.skip_frames(time=200)

clock = time.clock()  # Create a clock object to track the FPS.
#led1 = pyb.LED(1)
led2 = pyb.LED(2)
#led3 = pyb.LED(3)
#led1.on()
led2.on()
#led3.on()

window_x = 195
window_y = 143
window_width = 160
mask_radius = 100
roi_width=280

sensor.set_windowing(window_x - int((window_width/2)), window_y - int((window_width/2)), window_width, window_width)

centre_x = 78
centre_y = 81

thresh_ball = (29, 71, 8, 50, 4, 33)
thresh_yellow_goal = (45, 100, -25, 15, 24, 72) # old
thresh_blue_goal = (46, 58, -27, -7, -34, -17) # old

uart = UART(3, 115200)
uart.init(115200, bits=8, parity=None, stop=1, timeout_char=1000)
count = 0

no_ball = False

while True:
    # count+=1
    # print(count)
    clock.tick()  # Update the FPS clock.
    led2.on()
    img = sensor.snapshot()  # Take a picture and return the image.
    # ball = img.find_blobs([thresh_ball], roi=(centre_x-int(0.5*roi_width), centre_y-int(0.5*roi_width), roi_width, roi_width), pixel_threshold=0, area_threshold=0, merge=True)
    ball = img.find_blobs([thresh_ball], pixel_threshold=10, area_threshold=10, merge=True)
    img.draw_cross(centre_x, centre_y)

    if len(ball)>0:
        no_ball = False
        # print("ball")
        b = max(ball, key = lambda b:b.pixels())
        img.draw_rectangle(b.rect())
        ball_x = b.cx() - centre_x
        ball_y = b.cy() - centre_y
        ball_angle = math.atan2(ball_x, ball_y) * 180 / math.pi
        ball_angle -= 90
        if ball_angle<0:
            ball_angle += 360
        ball_dist = (ball_x ** 2 + ball_y ** 2) ** 0.5
        print("angle ", ball_angle, "dist ", ball_dist)
        actual_dist = pix_to_real(ball_dist)
        print("actual dist: ")
        print(actual_dist)
        angle_uart = round(ball_angle * 128)
        dist_uart = round(actual_dist * 128)
    else:
        no_ball = True

    uart.writechar(5)
    if(no_ball==False):
        uart.writechar(angle_uart & 0xFF)
        uart.writechar((angle_uart >> 8) & 0xFF)
        uart.writechar(dist_uart & 0xFF)
        uart.writechar((dist_uart >> 8) & 0xFF)
    else:
        uart.writechar(0)
        uart.writechar(0)
        uart.writechar(0)
        uart.writechar(0)
    uart.sendbreak()

    print("fps", clock.fps())
