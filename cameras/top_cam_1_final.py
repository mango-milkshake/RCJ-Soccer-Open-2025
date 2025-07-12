import sensor, time, math, pyb
from pyb import UART

# bot 1 - all black wheels
# ========== Polynomial Conversion Functions ==========
def pix_to_real(r_px):
    return (-0.7938274173017964
            + 1.5294566788791972*r_px
            - 0.016771506764266992*(r_px**2)
            + 0.00024722073358174443*(r_px**3)
            - 1.2846712889870513e-6*(r_px**4))

def real_to_pix(r_cm):
    output = 1.5974054819124033 + 0.3508026709534986*r_cm + 0.02301638290817975*(r_cm**2) - 0.0003494973335512324*(r_cm**3) + 1.7850794050003496e-6*(r_cm**4)
    # output += -0.9 * r_cm + 17.4
    return output

# ========== Camera Setup ==========
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)

sensor.set_gainceiling(32)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_contrast(0)
sensor.set_auto_exposure(False, exposure_us=30000)
sensor.set_auto_gain(False, gain_db=8)
sensor.set_auto_whitebal(False)
sensor.set_contrast(3)
sensor.skip_frames(time=200)

clock = time.clock()
led2 = pyb.LED(2)
led2.on()

ballExists = True
centre_x = 144
centre_y = 134
mask_radius = 120
thresh_ball = (21, 64, 37, 81, -39, 55) # (0, 100, 10, 48, -4, 43)
actual_data_len = 10

uart = UART(3, 115200)
uart.init(115200, bits=8, parity=None, stop=1, timeout_char=1000)

# Velocity storage in real coords (cm/s)
vx = 0.0
vy = 0.0
beta = 0.9  # EWMA smoothing factor

prev_xr = None  # real coords
prev_yr = None
prev_t_ms = time.ticks_ms()

TIME_FUTURE = 0.4

def uartwrite(var): # for confirm positive variables
    var = round(var*128)
    uart.writechar(var & 0xFF)
    uart.writechar((var >> 8) & 0xFF)

def sendVar(var): # for possibly negative variables
    if (var < 0):
        uart.writechar(0)
    else:
        uart.writechar(1)
    var = round(abs(var) * 128)
    uart.writechar(var & 0xFF)
    uart.writechar((var >> 8) & 0xFF)

while True:
    clock.tick()
    now_ms = time.ticks_ms()
    dt_ms = time.ticks_diff(now_ms, prev_t_ms)
    prev_t_ms = now_ms

    dt_s = dt_ms / 1000.0 if dt_ms > 0 else 0.001

    img = sensor.snapshot()
    #img.mask_circle(centre_x, centre_y, mask_radius)
    img.draw_cross(centre_x, centre_y, (255,255,255))

    # 1) Detect the ball in pixel coords
    blobs = img.find_blobs([thresh_ball], pixel_threshold=10, area_threshold=10, merge=True)
    if len(blobs)<1:
        print("No ball")
        ballExists = False
        uart.writechar(5)
        for _ in range (actual_data_len):
            uart.writechar(0)
        uart.sendbreak()
        continue

    b = max(blobs, key=lambda b: b.pixels())
    #img.draw_rectangle(b.rect(), (0,255,0))
    #img.draw_cross(b.cx(), b.cy(), (0,255,0))

    ball_x = b.cx() - centre_x
    ball_y = b.cy() - centre_y
    ball_angle = math.atan2(ball_x, ball_y) * 180 / math.pi
    ball_angle -= 90
    if ball_angle<0:
        ball_angle += 360

    # pixel coords (relative to centre if you want)
    px = b.cx() - centre_x
    py = centre_y - b.cy()

    # 2) Convert pixel distance => real distance
    dist_px = math.sqrt(px*px + py*py)

    angle = math.atan2(py, px)
    dist_real = pix_to_real(dist_px)  # in cm

    # convert polar -> cartesian in real coords
    xr = dist_real * math.cos(angle)
    yr = dist_real * math.sin(angle)
    _xr = xr/100
    _yr = yr/100

    # 3) Compute velocity in real coords
    if prev_xr is not None:
        ballExists = True
        inst_vx = (xr - prev_xr) / dt_s
        inst_vy = (yr - prev_yr) / dt_s
        inst_vx = inst_vx
        inst_vy = inst_vy

        # EWMA smoothing
        vx = beta*vx + (1-beta)*inst_vx
        vy = beta*vy + (1-beta)*inst_vy
    else:
        ballExists = False
        vx = 0
        vy = 0
    vx = vx / 100
    vy = vy / 100

    prev_xr, prev_yr = xr, yr

    # 4) Predict future position in real coords
    xr_future = xr + vx * TIME_FUTURE
    yr_future = yr + vy * TIME_FUTURE

    # 4.5) send to esp
    uart.writechar(5)
    if(ballExists):
        uartwrite(ball_angle)
        uartwrite(dist_real)
        # sendVar(_xr)
        # sendVar(_yr)
        sendVar(vx)
        sendVar(vy)
    else:
        for _ in range(actual_data_len):
            uart.writechar(0)

    uart.sendbreak()

    # 5) Convert that future real coords => pixel coords
    future_dist = math.sqrt(xr_future*xr_future + yr_future*yr_future)
    future_angle = math.atan2(yr_future, xr_future)
    future_dist_px = real_to_pix(future_dist)

    fx = future_dist_px * math.cos(future_angle)
    fy = future_dist_px * math.sin(future_angle)

    # Convert back to absolute image coords
    # (invert y if you're using 'centre_y - b.cy()' style)
    # current ball center in image is (b.cx(), b.cy())
    # future point is (b.cx()+..., b.cy()+...) in pixel space
    arrow_x1 = b.cx()
    arrow_y1 = b.cy()
    arrow_x2 = int(arrow_x1 + (fx - px))   # Because px was the "current" offset in pixel coords
    arrow_y2 = int(arrow_y1 - (fy - py))   # Subtract because we used 'py = centre_y - b.cy()'

    # 6) Draw arrow in a distinct color, e.g. blue
    img.draw_arrow(arrow_x1, arrow_y1, arrow_x2, arrow_y2, (0,0,255))

    # Debug print
    print("Real coords: x=%.2f cm, y=%.2f cm | v_x=%.2f, v_y=%.2f"
          % (xr, yr, vx, vy))
    print("fps:", clock.fps())


