import sensor, image, time, pyb
from pyb import SPI

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

spi = SPI(2)
cs = pyb.Pin("P3", pyb.Pin.IN)
sendBuffer = [1, 2, 3, 4, 5, 6, 7, 8, 9]

while(True):
    img = sensor.snapshot()
    if cs.value()==0:
        spi.init(SPI.PERIPHERAL)
        try:
            spi.send(bytearray(sendBuffer), timeout=100)
            print("SPI sent")
        except OSError:
            pass
        spi.deinit()
