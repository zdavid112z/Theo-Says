from queue import Queue

import numpy as np
import cv2
import math
from collections import deque


class Pointer:
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.radius = r

    def inside(self, outer):
        dx = self.x - outer.x
        dy = self.y - outer.y
        return outer.radius ** 2 > dx * dx + dy * dy


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


def print_direction(direction):
    str_dir = "????"
    if direction == 1:
        str_dir = "top left"
    elif direction == 2:
        str_dir = "top"
    elif direction == 3:
        str_dir = "top right"
    elif direction == 4:
        str_dir = "right"
    elif direction == 5:
        str_dir = "bottom right"
    elif direction == 6:
        str_dir = "bottom"
    elif direction == 7:
        str_dir = "bottom left"
    elif direction == 0:
        str_dir = "left"
    print(str_dir)


def plus(a, b):
    return a[0] + b[0], a[1] + b[1]


def minus(a, b):
    return a[0] - b[0], a[1] - b[1]


def length(point):
    return math.sqrt(point[0] ** 2 + point[1] ** 2)


def normalized(v):
    l = length(v)
    if l < 0.0001:
        return 0, 0
    return v[0] / l, v[1] / l


def sqrt_pair(v):
    return math.sqrt(v[0]), math.sqrt(v[1])


class Tracker:

    def __init__(self, n):
        self.n = n
        self.q = deque()
        for _ in range(n):
            self.q.append((0, 0))
        self.m = [0, 0]
        self.s = [0, 0]
        self.nm = [0, 0]
        self.ns = [0, 0]
        self.d = [0, 0]
        self.nd = [0, 0]
        self.count = 0

    def clear(self):
        self.q.clear()
        for _ in range(self.n):
            self.q.append((0, 0))
        self.m = [0, 0]
        self.s = [0, 0]
        self.nm = [0, 0]
        self.ns = [0, 0]
        self.d = [0, 0]
        self.nd = [0, 0]
        self.count = 0

    def update_single(self, old_val, new_val, old_m, old_s):
        new_m = old_m + (new_val - old_val) / (self.n - 1)
        new_s = old_s + (new_val - old_val) * (new_val - new_m + old_val - old_m) / (self.n - 2)
        return new_m, max(new_s, 0)

    def add_point(self, p):
        if isinstance(p, Pointer):
            p = (p.x, p.y)
        new_val = minus(p, self.q[self.n - 1])
        old_val = minus(self.q[1], self.q[0])
        new_val_norm = normalized(new_val)
        old_val_norm = normalized(old_val)
        self.q.append(p)
        self.q.popleft()
        self.m[0], self.s[0] = self.update_single(old_val[0], new_val[0], self.m[0], self.s[0])
        self.m[1], self.s[1] = self.update_single(old_val[1], new_val[1], self.m[1], self.s[1])
        self.nm[0], self.ns[0] = self.update_single(old_val_norm[0], new_val_norm[0], self.nm[0], self.ns[0])
        self.nm[1], self.ns[1] = self.update_single(old_val_norm[1], new_val_norm[1], self.nm[1], self.ns[1])
        self.d = sqrt_pair(self.s)
        self.nd = sqrt_pair(self.ns)
        self.count = min(self.n, self.count + 1)

    def is_ready(self):
        return self.count == self.n


class AverageLine:

    def __init__(self):
        self.a = 0
        self.b = 0
        self.c = 0
        self.n = 0

    def add_line(self, x0, y0, x1, y1):
        if y0 == y1:
            y1 += 0.1
        a = y0 - y1
        b = x1 - x0
        c = x0 * y1 - x1 * y0
        b /= a
        c /= a
        a /= a
        if a < 0:
            a = -a
            b = -b
            c = -c
        self.a += a
        self.b += b
        self.c += c
        self.n += 1

    def finalize(self):
        self.a /= self.n
        self.b /= self.n
        self.c /= self.n

    def intersect(self, line):
        a0 = self.a
        b0 = self.b
        c0 = self.c
        a1 = line.a
        b1 = line.b
        c1 = line.c
        x = (b0 * c1 - b1 * c0) / (b1 * a0 - a1 * b0)
        y = (a0 * c1 - a1 * c0) / (a1 * b0 - b1 * a0)
        return x, y


def on_change(_):
    pass

# 230 x 187 cm


NUM_POINTS = 60
MAX_DIST = 4
MIN_MEAN_LENGTH = 5.5
MAX_STD_LENGTH = 13

vid = cv2.VideoCapture("v5.mp4")
hsv = True

cv2.namedWindow("Trackbar")
cv2.resizeWindow("Trackbar", 640, 480)
# backSub = cv2.createBackgroundSubtractorMOG2()
backSub = cv2.createBackgroundSubtractorKNN()
pointer = None
old_pointer = None
tracker = Tracker(NUM_POINTS)
last_direction = -1
left_line = AverageLine()
right_line = AverageLine()
bottom_line = AverageLine()
top_line = AverageLine()

if hsv:
    cv2.createTrackbar("Hue Min", "Trackbar", 0, 180, on_change)
    cv2.createTrackbar("Hue Max", "Trackbar", 180, 180, on_change)
    cv2.createTrackbar("Sat Min", "Trackbar", 35, 255, on_change)
    cv2.createTrackbar("Sat Max", "Trackbar", 255, 255, on_change)
    cv2.createTrackbar("Val Min", "Trackbar", 174, 255, on_change)
    cv2.createTrackbar("Val Max", "Trackbar", 255, 255, on_change)
else:
    cv2.createTrackbar("R Min", "Trackbar", 150, 255, on_change)
    cv2.createTrackbar("R Max", "Trackbar", 255, 255, on_change)
    cv2.createTrackbar("G Min", "Trackbar", 0, 255, on_change)
    cv2.createTrackbar("G Max", "Trackbar", 50, 255, on_change)
    cv2.createTrackbar("B Min", "Trackbar", 0, 255, on_change)
    cv2.createTrackbar("B Max", "Trackbar", 50, 255, on_change)

nr_frame = 0
start_at_frame = 0
calibrate_before = 30
update_calib_after = 5
orig_W = 690
orig_H = 561
small_W = 690
small_H = 561

best_left = None
best_right = None
best_top = None
best_bottom = None

src_points = np.array([[0, 0], [orig_W, 0], [orig_W, orig_H], [0, orig_H]], dtype=np.float32)
dst_points = np.array([[0, 0], [small_W, 0], [small_W, small_H], [0, small_H]], dtype=np.float32)
M = cv2.getPerspectiveTransform(src_points, dst_points)

for i in range(start_at_frame):
    _, _ = vid.read()

while True:
    nr_frame += 1
    success, img = vid.read()

    img_orig_small = cv2.resize(img, (orig_W, orig_H))
    if nr_frame < calibrate_before:
        canny_img = cv2.Canny(img_orig_small, 20, 100, None, 3)
        lines = cv2.HoughLinesP(canny_img, 1, np.pi / 180, 4, None, 200, 20)
        img_lines = cv2.copyTo(img_orig_small, None)

        if nr_frame % update_calib_after == 0:

            if best_left is not None:
                left_line.add_line(best_left[0], best_left[1], best_left[2], best_left[3])
            if best_right is not None:
                right_line.add_line(best_right[0], best_right[1], best_right[2], best_right[3])
            if best_top is not None:
                top_line.add_line(best_top[0], best_top[1], best_top[2], best_top[3])
            if best_bottom is not None:
                bottom_line.add_line(best_bottom[0], best_bottom[1], best_bottom[2], best_bottom[3])

            best_left = None
            best_right = None
            best_top = None
            best_bottom = None

        if lines is not None:
            for i in range(0, len(lines)):
                l = lines[i][0]
                cv2.line(img_lines, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 3, cv2.LINE_AA)

                if l[0] < orig_W / 3 and l[2] < orig_W / 3:
                    if best_left is None or max(best_left[0], best_left[2]) < max(l[0], l[2]):
                        best_left = l

                elif l[0] > orig_W * 2 / 3 and l[2] > orig_W * 2 / 3:
                    if best_right is None or min(best_right[0], best_right[2]) > min(l[0], l[2]):
                        best_right = l

                elif l[1] < orig_H / 3 and l[3] < orig_H / 3:
                    if best_top is None or max(best_top[1], best_top[3]) < max(l[1], l[3]):
                        best_top = l

                elif l[1] > orig_H * 2 / 3 and l[3] > orig_H * 2 / 3:
                    if best_bottom is None or min(best_bottom[1], best_bottom[3]) > min(l[1], l[3]):
                        best_bottom = l

    elif nr_frame == calibrate_before:
        top_line.finalize()
        bottom_line.finalize()
        left_line.finalize()
        right_line.finalize()

        top_left_point = top_line.intersect(left_line)
        top_right_point = top_line.intersect(right_line)
        bottom_left_point = bottom_line.intersect(left_line)
        bottom_right_point = bottom_line.intersect(right_line)

        src_points = np.array([top_left_point, top_right_point, bottom_right_point, bottom_left_point], dtype=np.float32)
        dst_points = np.array([[0, 0], [small_W, 0], [small_W, small_H], [0, small_H]], dtype=np.float32)
        M = cv2.getPerspectiveTransform(src_points, dst_points)

    img_small = cv2.warpPerspective(img_orig_small, M, (small_W, small_H))
    img_hsv = cv2.cvtColor(img_small, cv2.COLOR_BGR2HSV)

    if hsv:
        hue_min = cv2.getTrackbarPos("Hue Min", "Trackbar")
        hue_max = cv2.getTrackbarPos("Hue Max", "Trackbar")
        sat_min = cv2.getTrackbarPos("Sat Min", "Trackbar")
        sat_max = cv2.getTrackbarPos("Sat Max", "Trackbar")
        val_min = cv2.getTrackbarPos("Val Min", "Trackbar")
        val_max = cv2.getTrackbarPos("Val Max", "Trackbar")
        lower = np.array([hue_min, sat_min, val_min])
        upper = np.array([hue_max, sat_max, val_max])
        mask = cv2.inRange(img_hsv, lower, upper)
    else:
        r_min = cv2.getTrackbarPos("R Min", "Trackbar")
        r_max = cv2.getTrackbarPos("R Max", "Trackbar")
        g_min = cv2.getTrackbarPos("G Min", "Trackbar")
        g_max = cv2.getTrackbarPos("G Max", "Trackbar")
        b_min = cv2.getTrackbarPos("B Min", "Trackbar")
        b_max = cv2.getTrackbarPos("B Max", "Trackbar")
        lower = np.array([b_min, g_min, r_min])
        upper = np.array([b_max, g_max, r_max])
        mask = cv2.inRange(img_small, lower, upper)

    background_mask = backSub.apply(img_small)
    # background_mask = backSub.apply(img_small)
    # background_mask = backSub.apply(img_small)
    # background_mask = backSub.apply(img_small)

    masked = cv2.bitwise_and(img_small, img_small, mask=mask)

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    img_contour = cv2.copyTo(img_small, None)

    closest_pointer = None
    min_dist = 100000
    for cnt in contours:
        (x, y), radius = cv2.minEnclosingCircle(cnt)
        if closest_pointer is None or pointer is None:
            closest_pointer = Pointer(x, y, radius)
        else:
            dist = (x - pointer.x) ** 2 + (y - pointer.y) ** 2
            if dist < min_dist:
                closest_pointer = Pointer(x, y, radius)
                min_dist = dist

    pointer = closest_pointer
    if pointer is not None:
        cv2.circle(img_contour, (int(pointer.x), int(pointer.y)), int(pointer.radius), (0, 255, 0))

    if pointer is not None and old_pointer is not None:
        dx = pointer.x - old_pointer.x
        dy = pointer.y - old_pointer.y
        plength = math.sqrt(dx * dx + dy * dy)
        if plength > 0.0001:
            n = int(math.ceil(plength / MAX_DIST))
            p = (old_pointer.x, old_pointer.y)
            dx /= n
            dy /= n
            for i in range(n):
                p = (p[0] + dx, p[1] + dy)
                tracker.add_point(p)
    elif pointer is not None:
        tracker.add_point(pointer)
    else:
        tracker.clear()
        last_direction = -1
    old_pointer = pointer

    tml = length(tracker.m)
    tdl = length(tracker.d)
    tnml = length(tracker.nm)
    tndl = length(tracker.nd)
    if tracker.is_ready() and tml >= 2.75 and tndl <= 0.2:
        direction = int((math.atan2(tracker.m[1], tracker.m[0]) + math.pi * 1.125) * 4 / math.pi) % 8
        if direction != last_direction:
            last_direction = direction
            print(str(nr_frame) + "   " + str(tml) + "  " + str(tdl) + "   " + str(tnml) + "   " + str(tndl) + "   ", end="")
            print_direction(direction)

    # direction = int((math.atan2(tracker.m[1], tracker.m[0]) + math.pi * 1.125) * 4 / math.pi) % 8
    # print(str(tml) + "  " + str(tdl) + "   " + str(tnml) + "   " + str(tndl) + "   ", end="")
    # print_direction(direction)

    for i in range(NUM_POINTS):
        cv2.circle(img_contour, (int(tracker.q[i][0]), int(tracker.q[i][1])), 10, (255, 0, 0))

    u = cv2.hconcat([img_small, img_lines, img_contour])
    d = cv2.hconcat([cv2.cvtColor(canny_img, cv2.COLOR_GRAY2BGR), cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), masked])
    output = cv2.vconcat([u, d])

    cv2.imshow("Output", output)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
