enable_lens_corr = False
f_x = (2.8 / 3.984) * 160 # find_apriltags defaults to this if not set
f_y = (2.8 / 2.952) * 120 # find_apriltags defaults to this if not set
c_x = 160 // 2
c_y = 120 // 2
line_range = (31, 101, 100, 20)

import pyb, sensor, image, time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)  # 160x120 pixels
sensor.set_vflip(True)              # The camera is mounted upside-down on the car.
sensor.set_hmirror(True)
sensor.set_auto_gain(False)         # must turn this off to prevent image washout...
sensor.set_auto_whitebal(False)     # must turn this off to prevent image washout...
sensor.skip_frames(time = 2000)

uart = pyb.UART(3, 9600, timeout_char=1000)
uart.init(9600, bits=8, parity=None, stop=1, timeout_char=1000)

clock = time.clock()

while(True):
    clock.tick()
    img = sensor.snapshot()
    if enable_lens_corr: img.lens_corr(1.8) # for 2.8mm lens...

    y_max = 0
    x_goal = None
    theta_goal = None

    Tx = None
    Ry = None

    #img.draw_rectangle(line_range, color = (255, 0, 0))
    # about line following:
    # https://openmv.io/blogs/news/linear-regression-line-following
    for l in img.find_line_segments(line_range, merge_distance = 20, max_theta_diff = 15):
        x = l.x2() - c_x
        y = l.y2()
        rho = l.rho()
        theta = l.theta()
        if rho < 0:
            rho = -rho
        if theta >= 90:
            theta -= 180
        if l.magnitude() > 8:
            #img.draw_line(l.line(), color = (255, 0, 0), thickness=1)
            if y > y_max:
                y_max = y
                x_goal = x
                theta_goal = theta

    for tag in img.find_apriltags(fx=f_x, fy=f_y, cx=c_x, cy=c_y): # defaults to TAG36H11
        #img.draw_rectangle(tag.rect(), color = (255, 0, 0))
        #img.draw_cross(tag.cx(), tag.cy(), color = (0, 255, 0))
        Tx = tag.cx() - c_x  # centroid x position of the apriltag
        Ry = tag.y_rotation() * 180 / 3.141592
        if Ry > 180:
            Ry -= 360

    if x_goal != None:
        #print("[LINE]  x: %3d theta: %3d" % (x_goal, theta_goal))
        uart.write(("[LINE]  x: %3d theta: %3d\r\n" % (x_goal, theta_goal)).encode())
    if Tx != None:
        #print("[AprT] Tx: %3d    Ry: %3d" % (Tx, Ry))
        uart.write(("[AprT] Tx: %3d    Ry: %3d\r\n" % (Tx, Ry)).encode())
    #print("FPS %f\n" % clock.fps())
