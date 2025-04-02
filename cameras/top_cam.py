import sensor, time, math, pyb
from pyb import UART

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)

sensor.set_gainceiling(128)
sensor.set_auto_whitebal(False) # must be turned off for color tracking

sensor.skip_frames(time=500)
sensor.set_auto_exposure(False, exposure_us=2000)
sensor.set_auto_gain(False, gain_db = 8)

clock = time.clock()  # Create a clock object to track the FPS.
#led1 = pyb.LED(1)
led2 = pyb.LED(2)
#led3 = pyb.LED(3)
#led1.on()
led2.on()
#led3.on()

centre_x = 156
centre_y = 127
mask_radius = 100
roi_width=280

# thresh_ball = (35, 77, 7, 50, 25, 65) # old
thresh_ball = (28, 63, 38, 80, 19, 69)
thresh_yellow_goal = (45, 100, -25, 15, 24, 72) # old
thresh_blue_goal = (46, 58, -27, -7, -34, -17) # old

uart = UART(3, 115200)
uart.init(115200, bits=8, parity=None, stop=1, timeout_char=1000)
count = 0

# find use goal
#img = sensor.snapshot()
#yellow_goal = img.find_blobs([thresh_yellow_goal], pixel_threshold=100, area_threshold=0, merge=True, margin=5)
#blue_goal = img.find_blobs([thresh_blue_goal], pixel_threshold=100, area_threshold=0, merge=True, margin=5)
# while(len(yellow_goal)<1 or len(blue_goal)<1):
#     img = sensor.snapshot()
#     yellow_goal = img.find_blobs([thresh_yellow_goal], pixel_threshold=100, area_threshold=0, merge=True, margin=5)
#     blue_goal = img.find_blobs([thresh_blue_goal], pixel_threshold=100, area_threshold=0, merge=True, margin=5)
# bg = max(blue_goal, key = lambda bg:bg.pixels())
# yg = max(yellow_goal, key = lambda yg:yg.pixels())
# if(yg.cy()<centre_y):
#     use_goal = "blue"
# else:
#     use_goal = "yellow"

use_goal = "blue"
no_ball = False
no_goal = True

while True:
    # count+=1
    # print(count)
    clock.tick()  # Update the FPS clock.
    led2.on()
    img = sensor.snapshot()  # Take a picture and return the image.
    # img.mask_circle(centre_x, centre_y, mask_radius)
    ball = img.find_blobs([thresh_ball], roi=(centre_x-int(0.5*roi_width), centre_y-int(0.5*roi_width), roi_width, roi_width), pixel_threshold=0, area_threshold=0, merge=True)
    # if(use_goal=="yellow"):
    #     yellow_goal = img.find_blobs([thresh_yellow_goal], pixel_threshold=100, area_threshold=0, merge=True, margin=20)
    # else:
    #     blue_goal = img.find_blobs([thresh_blue_goal], pixel_threshold=100, area_threshold=0, merge=True, margin=20)
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
        actual_dist = 0.727104914847282 + 0.5147081989789882*(ball_dist) + 0.016366529704736815*(ball_dist)**2 - 0.00026196059702118856*(ball_dist)**3 + 1.4312212575440405e-6*(ball_dist)**4
        print("actual dist: ")
        print(actual_dist)
        angle_uart = round(ball_angle * 128)
        dist_uart = round(actual_dist * 128)
    else:
        no_ball = True

    # if (use_goal=="blue" and len(blue_goal)>0):
    #     # print("blue goal")
    #     bg = max(blue_goal, key = lambda bg:bg.pixels())
    #     img.draw_rectangle(bg.rect())
    #     img.draw_cross(bg.cx(), bg.cy())

    #     bgoal_x = bg.cx() - centre_x
    #     bgoal_y = bg.cy() - centre_y
    #     bgoal_angle = math.atan2(bgoal_x, bgoal_y) * 180 / math.pi
    #     if bgoal_angle<0:
    #         bgoal_angle += 360

    #     bgoal_dist = m * math.exp(t*abs(bgoal_y)) + c*abs(bgoal_y) + d
    #     # print("blue dist", bgoal_dist)

    #     angle_bgoal_uart = round(bgoal_angle * 128)
    #     dist_bgoal_uart = round(bgoal_dist * 128)
    # else:
    #     no_goal = True

    # if (use_goal=="yellow" and len(yellow_goal)>0):
    #     # print("yellow goal")
    #     yg = max(yellow_goal, key = lambda yg:yg.pixels())
    #     img.draw_rectangle(yg.rect())
    #     img.draw_cross(yg.cx(), yg.cy())

    #     ygoal_x = yg.cx() - centre_x
    #     ygoal_y = yg.cy() - centre_y
    #     ygoal_angle = math.atan2(ygoal_x, ygoal_y) * 180 / math.pi
    #     if ygoal_angle<0:
    #         ygoal_angle += 360

    #     ygoal_dist = m * math.exp(t*abs(ygoal_y)) + c*abs(ygoal_y) + d
    #     # print("yellow dist", ygoal_dist)

    #     angle_ygoal_uart = round(ygoal_angle * 128)
    #     dist_ygoal_uart = round(ygoal_dist * 128)
    # else:
    #     no_goal = True


    uart.writechar(1)
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
    # if(no_goal==False):
    #     if(use_goal=="blue"):
    #         uart.writechar(angle_bgoal_uart & 0xFF)
    #         uart.writechar((angle_bgoal_uart >> 8) & 0xFF)
    #         uart.writechar(dist_bgoal_uart & 0xFF)
    #         uart.writechar((dist_bgoal_uart >> 8) & 0xFF)
    #     else:
    #         uart.writechar(angle_ygoal_uart & 0xFF)
    #         uart.writechar((angle_ygoal_uart >> 8) & 0xFF)
    #         uart.writechar(dist_ygoal_uart & 0xFF)
    #         uart.writechar((dist_ygoal_uart >> 8) & 0xFF)
    # else:
    #     uart.writechar(0)
    #     uart.writechar(0)
    #     uart.writechar(0)
    #     uart.writechar(0)
    uart.sendbreak()

    print("fps", clock.fps())
