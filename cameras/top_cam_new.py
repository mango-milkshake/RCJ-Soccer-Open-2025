import sensor, time, math, pyb
from pyb import UART
def pix_to_real(r_px):
    return (-0.19292354634084388
            + 0.9854734061269065*r_px
            - 0.000013340676293767154*(r_px**2))
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_gainceiling(32)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=2000)
sensor.set_auto_gain(False, gain_db=2)
#sensor.set_contrast(1)
sensor.skip_frames(time=200)

clock = time.clock()
led2 = pyb.LED(2)
led2.on()
window_x = 169
window_y = 111
window_width = 270
mask_radius = 100
roi_width=280
sensor.set_windowing(window_x - int((window_width/2)), window_y - int((window_width/2)), window_width, window_width)

centre_x = 132
centre_y = 111
thresh_ball = (0, 100, 16, 41, 3, 77)
thresh_yellow_goal = (45, 100, -25, 15, 24, 72)
thresh_blue_goal = (46, 58, -27, -7, -34, -17)
uart = UART(3, 115200)
uart.init(115200, bits=8, parity=None, stop=1, timeout_char=1000)
count = 0
no_ball = False
while True:
    clock.tick()
    led2.on()
    img = sensor.snapshot()
    ball = img.find_blobs([thresh_ball], merge=True)
    img.draw_cross(centre_x, centre_y)
    if len(ball)>0:
        no_ball = False
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
        actual_dist = pix_to_real(ball_dist) +2
        print("actual dist: ", actual_dist)
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
