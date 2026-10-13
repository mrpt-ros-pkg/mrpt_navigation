#!/usr/bin/env python3
# Generates the gridmaps of the "street" demo world (mvsim/demo_street.world.xml):
#  - street_walls.png: fixed infrastructure only (loaded by mvsim, which adds
#    the parked cars as polygonal blocks).
#  - street.png: infrastructure plus parked cars (static map for localization
#    and planning).
# Prints the <block> elements for the parked cars to paste into the world file.
import math
import os

from PIL import Image, ImageDraw

RES = 0.25  # [m/pixel]
X0, Y0 = -2.0, -2.0  # origin [m]
W, H = 64.0, 32.0  # extent [m]

CAR_LEN, CAR_WID = 4.5, 2.2

# Parked cars (x, y, yaw_deg). Free bays are the gaps between them.
CARS = [
    (4.0, 2.85, 90), (10.5, 2.85, 90), (17.0, 2.85, 90),  # perpendicular bays
    (26.5, 3.1, 60), (33.0, 3.1, 60),  # 60 deg angled bay
    (41.0, 3.1, 120), (47.5, 3.1, 120),  # 120 deg angled bay
    (55.0, 2.85, 90),
    (8.0, 24.1, 0), (22.5, 24.1, 0),  # long parallel bay
    (32.0, 24.1, 0), (41.0, 24.1, 0),  # short parallel bay
    (50.0, 24.1, 0),
]


def box(cx, cy, yaw_deg, length, width):
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return [(cx + c * dx - s * dy, cy + s * dx + c * dy)
            for dx, dy in [(-length / 2, -width / 2), (length / 2, -width / 2),
                           (length / 2, width / 2), (-length / 2, width / 2)]]


def octagon(cx, cy, r):
    return [(cx + r * math.cos(math.radians(22.5 + 45 * k)),
             cy + r * math.sin(math.radians(22.5 + 45 * k))) for k in range(8)]


def render(with_cars):
    nx, ny = int(W / RES), int(H / RES)
    img = Image.new('L', (nx, ny), 0)
    d = ImageDraw.Draw(img)

    def px(p):
        return ((p[0] - X0) / RES, (Y0 + H - p[1]) / RES)

    def fill(poly, v):
        d.polygon([px(p) for p in poly], fill=v)

    # Free area inside the street walls:
    fill([(0.5, 0.5), (59.5, 0.5), (59.5, 27.0), (0.5, 27.0)], 254)
    # Fixed obstacles: planter, kiosk and bollards
    fill(octagon(28.0, 17.0, 1.1), 0)
    fill(box(52.0, 15.0, 0, 3.0, 2.0), 0)
    for bx in (21.0, 23.0, 25.0):
        fill(box(bx, 11.8, 0, 0.5, 0.5), 0)
    if with_cars:
        for x, y, yaw in CARS:
            fill(box(x, y, yaw, CAR_LEN, CAR_WID), 0)
    return img


here = os.path.dirname(os.path.abspath(__file__))
render(False).save(os.path.join(here, 'street_walls.png'))
render(True).save(os.path.join(here, 'street.png'))

for i, (x, y, yaw) in enumerate(CARS):
    cls = 'parked_car_a' if i % 2 == 0 else 'parked_car_b'
    print(f'\t<block class="{cls}"><init_pose>{x} {y} {yaw}</init_pose></block>')
