#black bot
import sensor, time, math, pyb
from pyb import UART

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)

centre_x = 173
centre_y = 109

# sensor.set_windowing((centre_x-120, 0, 240, 240))
sensor.set_gainceiling(128)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False) # must be turned off for color tracking

sensor.skip_frames(time=500)
sensor.set_auto_exposure(False, exposure_us=12000)
sensor.set_auto_gain(False, gain_db = 4)
sensor.set_auto_whitebal(False)

clock = time.clock()  # Create a clock object to track the FPS.
#led1 = pyb.LED(1)
led2 = pyb.LED(2)
#led3 = pyb.LED(3)
#led1.on()
led2.on()
#led3.on()

centre_x = 168
centre_y = 113

# thresh_ball = (35, 77, 7, 50, 25, 65) # good one
thresh_ball = (55, 100, -2, 37, 18, 62)
thresh_yellow_goal = (45, 100, -25, 15, 24, 72)
thresh_blue_goal = (46, 58, -27, -7, -34, -17)

# fitting dist
# dists = [0, 44, 72, 87, 98.5, 104, 107, 110.005]
# Y = 0.0032779180010127974 * e^(0.08938137527794597 * x) + 0.4046707205307676 * x + -0.90086019612582
m, t, c, d = 0.0032779180010127974, 0.08938137527794597, 0.4046707205307676, -0.90086019612582

uart = UART(3, 230400)
uart.init(230400, bits=8, parity=None, stop=1, timeout_char=1000)
count = 0

# find use goal
img = sensor.snapshot()
yellow_goal = img.find_blobs([thresh_yellow_goal], pixel_threshold=100, area_threshold=0, merge=True, margin=5)
blue_goal = img.find_blobs([thresh_blue_goal], pixel_threshold=100, area_threshold=0, merge=True, margin=5)
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

use_goal = "yellow"
no_ball = False
no_goal = True

while True:
    # count+=1
    # print(count)
    clock.tick()  # Update the FPS clock.
    led2.on()
    img = sensor.snapshot()  # Take a picture and return the image.
    # img.draw_circle(centre_x, centre_y-10, 40, color=(0,0,0), fill=True)
    ball = img.find_blobs([thresh_ball], pixel_threshold=0, area_threshold=0, merge=True)
    # if(use_goal=="yellow"):
    #     yellow_goal = img.find_blobs([thresh_yellow_goal], pixel_threshold=100, area_threshold=0, merge=True, margin=20)
    # else:
    #     blue_goal = img.find_blobs([thresh_blue_goal], pixel_threshold=100, area_threshold=0, merge=True, margin=20)
    # img.draw_cross(centre_x, centre_y)
    print(sensor.get_exposure_us())
    img.draw_cross(centre_x, centre_y)

    if len(ball)>0:
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
        actual_dist = 1.0629+0.768114*ball_dist-0.0110443*ball_dist**2+0.0000695113*ball_dist**3
        print("actual", actual_dist)

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

#    if(len(yellow_goal)<1):
#        # print("yellow undetected")
#        if(use_goal =="blue"):
#            bot_y = 4.6 + bgoal_dist
#        else:
#            bot_y = 4.6 + 233.8 - bgoal_dist

#    elif(len(blue_goal)<1):
#        # print("blue undetected")
#        if(use_goal == "yellow"):
#            bot_y = 4.6 + ygoal_dist
#        else:
#            bot_y = 4.6 + 233.8 - ygoal_dist

#    else:
#        if(bg.cx() == yg.cx()):
#            proj_x = bg.cx()
#            proj_y = centre_y
#        else:
#            grad = (yg.cy()-bg.cy()) / (yg.cx()-bg.cx())
#            if(grad==0):
#                proj_x = centre_x
#                proj_y = bg.cy()
#            else:
#                perp_grad = -1/grad
#                proj_x = (grad*bg.cx() - perp_grad*centre_x + centre_y - bg.cy()) / (grad-perp_grad)
#                proj_y = grad*(proj_x-bg.cx()) + bg.cy()

#        img.draw_cross(round(proj_x), round(proj_y))

#        y_pixels = ((yg.cx()-centre_x) ** 2 + (yg.cy()-centre_y) ** 2) ** 0.5
#        b_pixels = ((bg.cx()-centre_x) ** 2 + (bg.cy()-centre_y) ** 2) ** 0.5
#        dist_to_y = m * math.exp(t*y_pixels) + c*y_pixels + d
#        dist_to_b = m * math.exp(t*b_pixels) + c*b_pixels + d
#        total_y_dist = dist_to_y + dist_to_b

#        if(use_goal=="blue"): # blue below in img
#            bot_y = 4.6 + (dist_to_b / total_y_dist) * 233.8
#        else:
#            bot_y = 4.6 + (dist_to_y / total_y_dist) * 233.8

#    print("bot", bot_y)
#    uart.writechar(1)
#    uart.writechar(round(bot_y) & 0xFF)
#    uart.writechar((round(bot_y) >> 8) & 0xFF)
#    uart.sendbreak()

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
