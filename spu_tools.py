#!/usr/bin/env python3
"""spu_tools.py -- everything the Mac's internal sensors can do, in one window.

tabs:

  angles    the hinge angle and the screen-to-table angle
  level     spirit level, from the accelerometer in the base
  quake     seismograph: scrolling trace of ground/desk vibration
  knock     detects taps on the chassis and times them
  heartbeat pulse felt through the chassis (ballistocardiogram)
  3d        a model of the machine that tilts and opens with the real one
  ambient   temperature and light inside the case
  trackpad  Force Touch pressure sensors, and weight on the pad

nothing here makes any sound.

    python3 spu_tools.py [--rate HZ]
"""

import math
import sys
import time
import tkinter as tk
from collections import deque
from tkinter import ttk

from macimu import IMU

# ---------------------------------------------------------------- appearance

BG = '#0e0e12'
PANEL = '#15151c'
GRID = '#23232e'
TEXT = '#e8e8ee'
DIM = '#7a7a8a'
ACCENT = '#f0b429'
GREEN = '#3ddc84'
BLUE = '#5fa8f5'
RED = '#ff5f56'
PURPLE = '#b98cff'

FS = 400.0                 # sample rate we ask the sensor for

# the accelerometer sits in the base and its axes do not line up with the
# model's. sensor +z points down when the machine is flat, and its two
# in-plane axes can point any of four ways. only two of the four keep the
# frame right-handed, so the signs must be flipped together -- the gyro fix
# below handles it either way, but a mixed pair means the sensor is being
# read as a mirror image, which is almost certainly not what the part does.
BODY_X_IS_MODEL_X = True   # sensor x is the left-right axis (else it is front-back)
BODY_Y_IS_MODEL_Z = True
BODY_X_SIGN = -1           # sensor +x points to the machine's LEFT
BODY_Y_SIGN = -1           # sensor +y points BACKWARD

# flip these live in the 3d tab with the x / y / s keys.


# ---------------------------------------------------------------------- 3d

def q_from_to(a, b):
    """quaternion rotating unit vector a onto unit vector b."""
    dot = max(-1.0, min(1.0, sum(x * y for x, y in zip(a, b))))
    if dot > 0.9999995:
        return (1.0, 0.0, 0.0, 0.0)
    if dot < -0.9999995:
        axis = (1.0, 0.0, 0.0) if abs(a[0]) < 0.9 else (0.0, 1.0, 0.0)
        cx = a[1] * axis[2] - a[2] * axis[1]
        cy = a[2] * axis[0] - a[0] * axis[2]
        cz = a[0] * axis[1] - a[1] * axis[0]
        ln = math.sqrt(cx * cx + cy * cy + cz * cz) or 1.0
        return (0.0, cx / ln, cy / ln, cz / ln)
    cx = a[1] * b[2] - a[2] * b[1]
    cy = a[2] * b[0] - a[0] * b[2]
    cz = a[0] * b[1] - a[1] * b[0]
    s = math.sqrt((1.0 + dot) * 2.0)
    return (s * 0.5, cx / s, cy / s, cz / s)


def q_rotate(q, v):
    """rotate vector v by quaternion q = (w, x, y, z)."""
    w, x, y, z = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def q_mul(a, b):
    """quaternion product a * b."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def q_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def q_norm(q):
    n = math.sqrt(sum(c * c for c in q)) or 1.0
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def q_to_axis_angle(q):
    w = max(-1.0, min(1.0, q[0]))
    ang = 2.0 * math.acos(w)
    sn = math.sqrt(max(0.0, 1.0 - w * w))
    if sn < 1e-9:
        return (0.0, 1.0, 0.0), 0.0
    return (q[1] / sn, q[2] / sn, q[3] / sn), ang


def q_from_axis_angle(axis, ang):
    h = ang * 0.5
    sn = math.sin(h)
    return (math.cos(h), axis[0] * sn, axis[1] * sn, axis[2] * sn)


WORLD_UP = (0.0, 1.0, 0.0)


def body_to_device(v):
    """sensor axes -> machine axes: x right, y up, z front.

    sensor +z points down through the base when the machine is flat, so the
    machine's up is -z. this mapping being missing is why the model first
    rendered upside down.
    """
    x, y, z = v
    return (BODY_X_SIGN * (x if BODY_X_IS_MODEL_X else y),
            -z,
            BODY_Y_SIGN * (y if BODY_Y_IS_MODEL_Z else x))


def _mapping_det_sign():
    """+1 if body_to_device is a rotation, -1 if it is a reflection."""
    cols = [body_to_device(e) for e in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
    (a, d, g), (b, e, h), (c, f, i) = cols
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    return 1.0 if det >= 0.0 else -1.0


def body_to_device_gyro(v):
    """angular velocity through the same mapping.

    angular velocity is a pseudovector, so if the mapping is a reflection it
    has to be negated as well -- otherwise the gyro drives the model the
    wrong way round and the filter fights itself.
    """
    sgn = _mapping_det_sign()
    return tuple(sgn * c for c in body_to_device(v))


def box(cx, cy, cz, w, h, d):
    """8 corners of an axis-aligned box."""
    hw, hh, hd = w / 2.0, h / 2.0, d / 2.0
    return [(cx - hw, cy - hh, cz - hd), (cx + hw, cy - hh, cz - hd),
            (cx + hw, cy + hh, cz - hd), (cx - hw, cy + hh, cz - hd),
            (cx - hw, cy - hh, cz + hd), (cx + hw, cy - hh, cz + hd),
            (cx + hw, cy + hh, cz + hd), (cx - hw, cy + hh, cz + hd)]


BOX_FACES = [(0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4),
             (3, 2, 6, 7), (0, 3, 7, 4), (1, 2, 6, 5)]


# --------------------------------------------------------------------- hub

def _hp_coeff(fc, fs):
    """one-pole high-pass coefficient for cutoff fc."""
    return math.exp(-2.0 * math.pi * fc / fs)


def _lp_coeff(fc, fs):
    """one-pole low-pass coefficient for cutoff fc."""
    return 1.0 - math.exp(-2.0 * math.pi * fc / fs)


class Hub:
    """owns the IMU and keeps rolling buffers every view reads from."""

    # a high-pass alone is not enough: it passes slow rocking and gravity
    # leakage straight through, which swamps the tiny real vibration. these
    # are proper bands.
    SEISMO_LO, SEISMO_HI = 0.7, 25.0     # ground/desk rumble band
    TAP_LO = 50.0                        # taps are sharp; keep only the top
    BCG_LO, BCG_HI = 0.8, 3.0            # heartbeat: 48 - 180 bpm

    def __init__(self, rate=FS):
        self.rate = rate
        n = int(rate * 10)
        self.accel = deque(maxlen=n)        # (t, x, y, z) in g
        self.gyro = deque(maxlen=n)         # (t, x, y, z) in deg/s
        self.lid = None
        self.t0 = time.monotonic()
        self.imu = IMU(accel=True, gyro=True, lid=True, als=True,
                       temp=True, sample_rate=int(rate))
        self.imu.start()

        a_s = _hp_coeff(self.SEISMO_LO, rate)
        b_s = _lp_coeff(self.SEISMO_HI, rate)
        a_t = _hp_coeff(self.TAP_LO, rate)

        self.seismo = [0.0, 0.0, 0.0]       # bandpassed, for the trace
        self.s_prev_in = [0.0, 0.0, 0.0]
        self.s_lp = [0.0, 0.0, 0.0]
        self._a_s, self._b_s = a_s, b_s

        self.tapf = [0.0, 0.0, 0.0]         # high band, for tap detection
        self.t_prev_in = [0.0, 0.0, 0.0]
        self._a_t = a_t

        # ballistocardiogram band -- the pulse you feel through the chassis
        # when your wrists rest on the machine
        a_b = _hp_coeff(self.BCG_LO, rate)
        b_b = _lp_coeff(self.BCG_HI, rate)
        self.bcg_f = [0.0, 0.0, 0.0]
        self.bcg_prev = [0.0, 0.0, 0.0]
        self.bcg_lp = [0.0, 0.0, 0.0]
        self._a_b, self._b_b = a_b, b_b
        self.bcg = deque(maxlen=n)          # (t, magnitude)

        self.dyn = deque(maxlen=n)          # seismo band magnitude
        self.tapmag = deque(maxlen=n)       # tap band magnitude

        self.temp = None                    # ambient degC
        self.lux = None
        self.temp_hist = deque(maxlen=int(rate))   # ~1 s of readings
        self.g_lp = [0.0, 0.0, -1.0]        # slow average = gravity direction
        self.g_alpha = 0.004                # ~0.6 s at 400 Hz
        self.warm = False

        # orientation: device -> world. gyro drives it, gravity stops the
        # pitch/roll part drifting. yaw has no absolute reference in these
        # sensors, so it will creep slowly -- that is physics, not a bug.
        self.q = None
        self.ahrs_gain = 0.01               # gravity correction per sample
        self.roll = self.pitch = self.yaw = 0.0

    def poll(self):
        t = time.monotonic() - self.t0
        acc = self.imu.read_accel()
        gyr = self.imu.read_gyro()
        dt = 1.0 / self.rate
        for n, s in enumerate(acc):
            v3 = (s.x, s.y, s.z)
            gsample = gyr[n] if n < len(gyr) else (gyr[-1] if gyr else None)
            self.accel.append((t, s.x, s.y, s.z))
            if not self.warm:
                # seed the filters with the first real sample, otherwise the
                # step from zero to 1g rings for half a second
                self.s_prev_in = list(v3)
                self.t_prev_in = list(v3)
                self.g_lp = list(v3)
                self.warm = True
                continue
            for i, v in enumerate(v3):
                self.g_lp[i] += self.g_alpha * (v - self.g_lp[i])

                y = self._a_s * (self.seismo[i] + v - self.s_prev_in[i])
                self.seismo[i] = y
                self.s_prev_in[i] = v
                self.s_lp[i] += self._b_s * (y - self.s_lp[i])

                self.tapf[i] = self._a_t * (self.tapf[i] + v - self.t_prev_in[i])
                self.t_prev_in[i] = v

            for i, v in enumerate(v3):
                y = self._a_b * (self.bcg_f[i] + v - self.bcg_prev[i])
                self.bcg_f[i] = y
                self.bcg_prev[i] = v
                self.bcg_lp[i] += self._b_b * (y - self.bcg_lp[i])

            self.dyn.append((t, math.sqrt(sum(c * c for c in self.s_lp))))
            self.tapmag.append((t, math.sqrt(sum(c * c for c in self.tapf))))
            self.bcg.append((t, math.sqrt(sum(c * c for c in self.bcg_lp))))

            if gsample is not None:
                self.gyro.append((t, gsample.x, gsample.y, gsample.z))
                self._fuse(v3, (gsample.x, gsample.y, gsample.z), dt)
        l = self.imu.read_lid()
        if l is not None:
            self.lid = float(l)
        t = self.imu.read_temperature()
        if t is not None:
            self.temp = t
            self.temp_hist.append((time.monotonic() - self.t0, t))
        a = self.imu.read_als()
        if a is not None:
            self.lux = a.lux

    def _fuse(self, acc_body, gyro_body, dt):
        """complementary filter: integrate the gyro, let gravity correct it."""
        a_dev = body_to_device(acc_body)
        mag = math.sqrt(sum(c * c for c in a_dev))
        if mag < 1e-6:
            return
        a_hat = (a_dev[0] / mag, a_dev[1] / mag, a_dev[2] / mag)

        if self.q is None:
            self.q = q_from_to(a_hat, WORLD_UP)
            return

        gdev = body_to_device_gyro(gyro_body)
        wx = math.radians(gdev[0])
        wy = math.radians(gdev[1])
        wz = math.radians(gdev[2])

        # integrate the measured rotation
        dq = q_mul(self.q, (0.0, wx, wy, wz))
        h = 0.5 * dt
        qg = q_norm((self.q[0] + dq[0] * h, self.q[1] + dq[1] * h,
                     self.q[2] + dq[2] * h, self.q[3] + dq[3] * h))

        # nudge it so the predicted up matches the measured up. derived
        # geometrically rather than by hand-tuning a sign, so it cannot end
        # up feeding back the wrong way.
        if abs(mag - 1.0) < 0.2:                     # ignore hard shakes
            v_est = q_rotate(q_conj(qg), WORLD_UP)
            axis, ang = q_to_axis_angle(q_from_to(a_hat, v_est))
            qc = q_from_axis_angle(axis, ang * self.ahrs_gain)
            self.q = q_norm(q_mul(qg, qc))
        else:
            self.q = qg

        self.roll, self.pitch, self.yaw = self._euler(self.q)

    @staticmethod
    def _euler(q):
        w, x, y, z = q
        roll = math.degrees(math.atan2(2 * (w * x + y * z),
                                       1 - 2 * (x * x + y * y)))
        sp = 2 * (w * y - z * x)
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, sp))))
        yaw = math.degrees(math.atan2(2 * (w * z + x * y),
                                      1 - 2 * (y * y + z * z)))
        return roll, pitch, yaw

    def gravity(self):
        """which way is down, in body coordinates, heavily smoothed."""
        m = math.sqrt(sum(c * c for c in self.g_lp)) or 1.0
        return (self.g_lp[0] / m, self.g_lp[1] / m, self.g_lp[2] / m)

    def stop(self):
        self.imu.stop()


# -------------------------------------------------------------------- views

class View(tk.Frame):
    """plain tk frame, not ttk -- ttk widgets take styles, not a bg colour."""
    title = 'view'

    def __init__(self, parent, hub):
        super().__init__(parent, bg=BG)
        self.hub = hub
        self.canvas = tk.Canvas(self, bg=BG, highlightthickness=0)
        self.canvas.pack(fill='both', expand=True)

    def draw(self):
        pass


class AnglesView(View):
    """hinge angle A, and B = 180 - A behind the screen."""
    title = 'angles'

    TABLE_Y = 395
    HINGE_X = 300
    DECK_LEN = 215
    SCREEN_LEN = 235
    ARC_A_R = 95
    ARC_B_R = 150

    def __init__(self, parent, hub):
        super().__init__(parent, hub)
        self.canvas.config(width=780, height=470)
        self.canvas.pack_propagate(False)

    def draw(self):
        c = self.canvas
        a = self.hub.lid
        c.delete('all')
        W = 780
        hy = self.TABLE_Y
        hx = self.HINGE_X

        c.create_rectangle(0, hy, W, hy + 80, fill='#3a3a44', width=0)
        c.create_line(0, hy, W, hy, fill='#4a4a58', width=2)
        c.create_line(hx, hy - 7, hx - self.DECK_LEN, hy - 7, fill='#5a5f6b',
                      width=17, capstyle='round')
        c.create_line(hx, hy - 13, hx - self.DECK_LEN, hy - 13, fill='#7d8496',
                      width=3)

        if a is None:
            c.create_text(W / 2, 200, text='waiting for the lid sensor...',
                          fill=DIM, font=('Menlo', 14))
            return

        rad = math.radians(180.0 - a)
        sx = hx + self.SCREEN_LEN * math.cos(rad)
        sy = hy - self.SCREEN_LEN * math.sin(rad)
        c.create_line(hx, hy - 7, sx, sy - 7, fill='#2f6fbf', width=15,
                      capstyle='round')
        c.create_line(hx, hy - 7, sx, sy - 7, fill=BLUE, width=3)

        c.create_arc(hx - self.ARC_A_R, hy - self.ARC_A_R,
                     hx + self.ARC_A_R, hy + self.ARC_A_R,
                     start=180.0 - a, extent=a, style='arc',
                     outline=ACCENT, width=3)
        mid = math.radians(180.0 - a / 2.0)
        c.create_text(hx + (self.ARC_A_R + 26) * math.cos(mid),
                      hy - (self.ARC_A_R + 26) * math.sin(mid),
                      text=f'A  {a:.1f}°', fill=ACCENT,
                      font=('Menlo', 14, 'bold'))

        b = 180.0 - a
        c.create_arc(hx - self.ARC_B_R, hy - self.ARC_B_R,
                     hx + self.ARC_B_R, hy + self.ARC_B_R,
                     start=0.0, extent=b, style='arc',
                     outline=GREEN, width=3, dash=(6, 4))
        midb = math.radians(b / 2.0)
        c.create_text(hx + (self.ARC_B_R + 24) * math.cos(midb),
                      hy - (self.ARC_B_R + 24) * math.sin(midb),
                      text=f'B  {b:.1f}°', fill=GREEN,
                      font=('Menlo', 14, 'bold'))
        c.create_line(sx, sy - 7, hx, hy - 7, fill=GREEN, width=1, dash=(3, 5))

        c.create_text(20, 30, anchor='w', fill=TEXT, font=('Menlo', 13),
                      text=(f"A  keyboard <-> screen      {a:6.1f}°   "
                            f"(lid sensor)\n"
                            f"B  screen back <-> table   {b:6.1f}°   "
                            f"(180 - A)"))


class LevelView(View):
    """spirit level. the accelerometer is in the base, so this measures how
    the keyboard deck sits -- useful for a desk, a stand, or a shelf."""
    title = 'level'

    def __init__(self, parent, hub):
        super().__init__(parent, hub)
        self.canvas.config(width=780, height=470)

    def draw(self):
        c = self.canvas
        gx, gy, gz = self.hub.gravity()
        c.delete('all')
        W, H = 780, 470

        # body x is left-right, body y is front-back, body z points down.
        # tilt of each axis away from vertical
        roll = math.degrees(math.atan2(gx, -gz))
        pitch = math.degrees(math.atan2(gy, -gz))
        total = math.degrees(math.acos(max(-1.0, min(1.0, -gz))))

        c.create_text(30, 28, anchor='w', fill=TEXT, font=('Menlo', 14, 'bold'),
                      text='spirit level')
        c.create_text(30, 50, anchor='w', fill=DIM, font=('Menlo', 11),
                      text='measures the keyboard half, not the screen')

        self._tube(c, 60, 130, 660, 46, roll, 'left / right', -90, 90)
        self._tube(c, 60, 220, 660, 46, pitch, 'front / back', -90, 90)
        self._bubble(c, 640, 360, 95, roll, pitch)

        c.create_text(60, 300, anchor='w', fill=TEXT, font=('Menlo', 13),
                      text=f"left/right  {roll:+6.1f}°\n"
                           f"front/back  {pitch:+6.1f}°\n"
                           f"total tilt  {total:6.1f}°")

    def _tube(self, c, x, y, w, h, value, label, lo, hi):
        c.create_rectangle(x, y, x + w, y + h, outline=GRID, width=2,
                           fill=PANEL)
        mid = x + w / 2.0
        for frac in (0.25, 0.5, 0.75):
            gx = x + w * frac
            c.create_line(gx, y, gx, y + h, fill=GRID)
        frac = max(0.0, min(1.0, (value - lo) / (hi - lo)))
        bx = x + w * frac
        c.create_oval(bx - 18, y + 6, bx + 18, y + h - 6,
                      fill=GREEN if abs(value) < 1.0 else ACCENT, width=0)
        c.create_text(x, y - 12, anchor='w', text=label, fill=DIM,
                      font=('Menlo', 11))

    def _bubble(self, c, cx, cy, r, roll, pitch):
        c.create_oval(cx - r, cy - r, cx + r, cy + r, outline=GRID, width=2,
                      fill=PANEL)
        for rr in (r * 0.33, r * 0.66):
            c.create_oval(cx - rr, cy - rr, cx + rr, cy + rr, outline=GRID)
        c.create_line(cx - r, cy, cx + r, cy, fill=GRID)
        c.create_line(cx, cy - r, cx, cy + r, fill=GRID)
        lim = 30.0
        px = cx + r * max(-0.95, min(0.95, roll / lim))
        py = cy + r * max(-0.95, min(0.95, pitch / lim))
        c.create_oval(px - 9, py - 9, px + 9, py + 9,
                      fill=GREEN if abs(roll) < 1 and abs(pitch) < 1 else BLUE,
                      width=0)
        c.create_text(cx, cy + r + 20, text='30° full scale', fill=DIM,
                      font=('Menlo', 10))


class QuakeView(View):
    """seismograph: scrolling trace of vibration with event detection."""
    title = 'quake'

    def __init__(self, parent, hub):
        super().__init__(parent, hub)
        self.canvas.config(width=780, height=470)
        self.span = 6.0                       # seconds on screen
        self.gain = 40.0
        self.events = deque(maxlen=200)
        self.sta = 0.0
        self.lta = 1e-9
        self.active = False
        self.count = 0
        self._cursor = -1.0

    def draw(self):
        c = self.canvas
        c.delete('all')
        W, H = 780, 470
        x0, x1 = 60, W - 20
        y0, y1 = 70, 360
        now = self.hub.dyn[-1][0] if self.hub.dyn else self.hub.t0

        c.create_text(20, 26, anchor='w', fill=TEXT, font=('Menlo', 14, 'bold'),
                      text='seismograph')
        c.create_text(20, 48, anchor='w', fill=DIM, font=('Menlo', 11),
                      text='gravity removed -- this is movement only')

        c.create_rectangle(x0, y0, x1, y1, outline=GRID, fill=PANEL)
        mid = (y0 + y1) / 2.0
        c.create_line(x0, mid, x1, mid, fill=GRID)
        for f in (0.25, 0.75):
            yy = y0 + (y1 - y0) * f
            c.create_line(x0, yy, x1, yy, fill=GRID, dash=(2, 6))

        # sta/lta event detection. the cursor means each sample advances the
        # averages once, however often we redraw.
        for t, v in self.hub.dyn:
            if t <= self._cursor:
                continue
            self._cursor = t
            self.sta += 0.02 * (v - self.sta)
            self.lta += 0.0002 * (v - self.lta)
            ratio = self.sta / (self.lta + 1e-12)
            if not self.active and ratio > 3.0 and v > self.lta * 3.0:
                self.active = True
                self.count += 1
                self.events.append(t)
            elif self.active and ratio < 1.4:
                self.active = False

        pts = []
        t_lo = now - self.span
        for t, v in self.hub.dyn:
            if t < t_lo:
                continue
            x = x0 + (t - t_lo) / self.span * (x1 - x0)
            y = mid - v * self.gain * (y1 - y0) / 2.0
            pts.append((x, max(y0, min(y1, y))))

        if len(pts) > 1:
            c.create_line(*[v for p in pts for v in p], fill=GREEN, width=2)

        if self.active:
            c.create_oval(x1 - 60, y0 - 40, x1 - 20, y0 - 20, fill=RED, width=0)
            c.create_text(x1 - 70, y0 - 30, anchor='e', text='EVENT', fill=RED,
                          font=('Menlo', 11, 'bold'))

        c.create_text(x0, y1 + 28, anchor='w', fill=DIM, font=('Menlo', 11),
                      text=f'{self.span:.0f}s window     gain x{self.gain:.0f}')
        c.create_text(x1, y1 + 28, anchor='e', fill=TEXT, font=('Menlo', 12),
                      text=f'events detected: {self.count}')
        c.create_text(x0, H - 20, anchor='w', fill=DIM, font=('Menlo', 10),
                      text='try a gentle tap on the desk, a footstep, or set '
                           'something vibrating nearby')


class KnockView(View):
    """detects knocks/taps on the chassis and times the intervals.

    tuned against measured idle levels: in the >50 Hz band this machine sits
    at ~1.5 mg median and ~11 mg worst case while untouched, so the default
    threshold sits above that. arrow up/down moves it while you watch.
    """
    title = 'knock'

    THRESH = 0.020          # g, in the >50 Hz band
    REFRACTORY = 0.06       # s, minimum spacing between taps
    REARM = 0.4             # must fall to this fraction of thresh to re-arm

    def __init__(self, parent, hub):
        super().__init__(parent, hub)
        self.canvas.config(width=780, height=470)
        self.taps = deque(maxlen=64)
        self.last_tap = -1.0
        self._cursor = -1.0
        self.span = 4.0                 # seconds of waveform on screen
        self.armed = True
        self.thresh = self.THRESH
        self.peak = 0.0
        self.peak_t = -1.0
        self.canvas.bind_all('<Up>', lambda e: self._bump(0.001))
        self.canvas.bind_all('<Down>', lambda e: self._bump(-0.001))

    def _bump(self, d):
        self.thresh = max(0.002, round(self.thresh + d, 4))

    def reset(self):
        """clear all detector state, including the re-arm latch."""
        self.taps.clear()
        self.last_tap = -1.0
        self.peak = 0.0
        self.peak_t = -1.0
        self.armed = True

    def draw(self):
        c = self.canvas
        W, H = 780, 470
        x0, x1, y0, y1 = 60, W - 20, 90, 320

        # each sample advances detection once, however often we redraw.
        # a single impact rings for several samples, so one threshold crossing
        # is not one tap: the detector must see the signal fall back below the
        # re-arm level before it will fire again.
        for t, v in self.hub.tapmag:
            if t <= self._cursor:
                continue
            self._cursor = t
            if t - self.peak_t > 1.0:     # peak-hold window of one second
                self.peak, self.peak_t = 0.0, t
            if v > self.peak:
                self.peak = v

            if not self.armed:
                if v < self.thresh * self.REARM:
                    self.armed = True
                continue
            if v > self.thresh and t > self.last_tap + self.REFRACTORY:
                self.armed = False
                self.last_tap = t
                self.taps.append(t)

        c.delete('all')
        c.create_text(20, 26, anchor='w', fill=TEXT, font=('Menlo', 14, 'bold'),
                      text='knock detector')
        c.create_text(20, 48, anchor='w', fill=DIM, font=('Menlo', 11),
                      text='sharp high-frequency transients only. up/down '
                           'arrows change the threshold 1 mg at a time')

        c.create_rectangle(x0, y0, x1, y1, outline=GRID, fill=PANEL)

        now = self.hub.tapmag[-1][0] if self.hub.tapmag else 0.0
        t_lo = now - self.span
        mid = (y0 + y1) / 2.0
        pts = []
        for t, v in self.hub.tapmag:
            if t < t_lo:
                continue
            x = x0 + (t - t_lo) / self.span * (x1 - x0)
            y = mid - min(1.0, v / (self.thresh * 3.0)) * (y1 - y0) / 2.2
            pts.append((x, max(y0, min(y1, y))))
        if len(pts) > 1:
            c.create_line(*[v for p in pts for v in p], fill=BLUE, width=1)

        # threshold line, so the margin is visible
        ty = mid - (1.0 / 3.0) * (y1 - y0) / 2.2
        c.create_line(x0, ty, x1, ty, fill=ACCENT, dash=(5, 4))
        c.create_text(x1 - 4, ty - 12, anchor='e', fill=ACCENT,
                      font=('Menlo', 10),
                      text=f'threshold {self.thresh * 1000:.0f} mg')

        for t in self.taps:
            if t < t_lo:
                continue
            x = x0 + (t - t_lo) / self.span * (x1 - x0)
            c.create_line(x, y0, x, y1, fill=GREEN, dash=(3, 3))
            c.create_oval(x - 5, mid - 5, x + 5, mid + 5, fill=GREEN, width=0)

        recent = [t for t in self.taps if t > now - 3.0]
        c.create_text(x0, y1 + 32, anchor='w', fill=TEXT, font=('Menlo', 13),
                      text=f'taps in last 3s: {len(recent)}      '
                           f'total: {len(self.taps)}')
        c.create_text(x1, y1 + 32, anchor='e', fill=DIM, font=('Menlo', 11),
                      text=f'peak this second: {self.peak * 1000:.1f} mg')
        if len(self.taps) > 1:
            gaps = [b - a for a, b in zip(list(self.taps), list(self.taps)[1:])]
            c.create_text(x0, y1 + 58, anchor='w', fill=DIM,
                          font=('Menlo', 11),
                          text='intervals (ms): ' +
                               '  '.join(f'{g * 1000:.0f}' for g in gaps[-7:]))
        c.create_text(x0, y0 - 16, anchor='w', fill=DIM, font=('Menlo', 10),
                      text='idle noise here is 1-4 mg, so anything that trips '
                           'this is a real impact')


class ModelView(View):
    """a 3d model of the machine, driven by the gyro and levelled by gravity.

    the gyro supplies the rotation, including spinning flat on the desk; the
    accelerometer stops the pitch and roll from drifting. yaw has no absolute
    reference, so it creeps slowly over minutes.
    """
    title = '3d'

    def __init__(self, parent, hub):
        super().__init__(parent, hub)
        self.canvas.config(width=780, height=470)
        self.smooth_lid = None
        for key, fn in (('x', 'flip_x'), ('y', 'flip_y'), ('s', 'swap_axes')):
            self.canvas.bind_all(f'<KeyPress-{key}>',
                                 getattr(self, f'_on_{fn}'))

    @staticmethod
    def _on_flip_x(e):
        import spu_tools
        spu_tools.BODY_X_SIGN *= -1

    @staticmethod
    def _on_flip_y(e):
        import spu_tools
        spu_tools.BODY_Y_SIGN *= -1

    @staticmethod
    def _on_swap_axes(e):
        import spu_tools
        spu_tools.BODY_X_IS_MODEL_X = not spu_tools.BODY_X_IS_MODEL_X
        spu_tools.BODY_Y_IS_MODEL_Z = not spu_tools.BODY_Y_IS_MODEL_Z

    def draw(self):
        c = self.canvas
        c.delete('all')
        W, H = 780, 470
        c.create_text(20, 26, anchor='w', fill=TEXT, font=('Menlo', 14, 'bold'),
                      text='orientation')
        c.create_text(20, 48, anchor='w', fill=DIM, font=('Menlo', 11),
                      text='gyro + gravity fusion. move or turn the machine '
                           'and it follows')

        a = self.hub.lid if self.hub.lid is not None else 0.0
        if self.smooth_lid is None:
            self.smooth_lid = a
        self.smooth_lid += 0.25 * (a - self.smooth_lid)
        a = self.smooth_lid

        q = self.hub.q
        if q is None:
            c.create_text(W / 2, H / 2, text='waiting for the sensors...',
                          fill=DIM, font=('Menlo', 12))
            return

        # base: 300 x 15 x 210 mm, hinge along the back top edge
        BASE_W, BASE_T, BASE_D = 300.0, 15.0, 210.0
        SCR_H, SCR_T = 200.0, 8.0
        hy = BASE_T / 2.0
        hz = -BASE_D / 2.0

        base = box(0, 0, 0, BASE_W, BASE_T, BASE_D)

        # screen, hinged at the back edge. 0 lies forward over the keyboard,
        # 90 stands up, 180 falls back past vertical.
        rad = math.radians(a)
        u = (0.0, math.sin(rad), math.cos(rad))       # up the screen
        nrm = (0.0, math.cos(rad), -math.sin(rad))    # screen normal
        scr = []
        for sx, sy, sz in ((-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
                           (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)):
            lx = sx * BASE_W / 2.0
            ly = 0.0 if sy < 0 else SCR_H
            lz = sz * SCR_T / 2.0
            scr.append((lx,
                        hy + ly * u[1] + lz * nrm[1],
                        hz + ly * u[2] + lz * nrm[2]))

        scale = 0.95
        cx, cy = W / 2.0, H / 2.0 + 30

        # camera sits at +z, which is the front of the machine -- the side
        # you sit at. with it at -z you are looking at the back of the lid.
        def proj(p):
            p = q_rotate(q, p)
            f = 1400.0 / (1400.0 - p[2])
            return (cx + p[0] * scale * f, cy - p[1] * scale * f, p[2])

        faces = []
        for idx, pts in ((0, base), (1, scr)):
            for fi, face in enumerate(BOX_FACES):
                pr = [proj(pts[i]) for i in face]
                depth = sum(p[2] for p in pr) / 4.0
                faces.append((depth, idx, fi, pr))

        # painter's algorithm: with the camera at +z, small z is far away,
        # so draw ascending -- anything else puts the keyboard behind the lid
        faces.sort(key=lambda f: f[0])
        for depth, idx, fi, pr in faces:
            flat = [v for p in pr for v in (p[0], p[1])]
            if idx == 0:
                col = '#4a4e58' if fi == 0 else (
                    '#5a5f6b' if fi in (1, 4, 5) else '#7d8496')
            else:
                col = '#24507f' if fi == 0 else (
                    '#2f6fbf' if fi in (1, 4, 5) else '#5fa8f5')
            c.create_polygon(*flat, fill=col, outline=BG, width=1)

        c.create_text(20, H - 92, anchor='w', fill=TEXT, font=('Menlo', 12),
                      text=f'lid  {a:5.1f}°')
        c.create_text(20, H - 68, anchor='w', fill=DIM, font=('Menlo', 11),
                      text=f'roll {self.hub.roll:+6.1f}°   '
                           f'pitch {self.hub.pitch:+6.1f}°   '
                           f'yaw {self.hub.yaw:+6.1f}°')
        c.create_text(20, H - 44, anchor='w', fill=DIM, font=('Menlo', 10),
                      text='yaw drifts slowly: no magnetometer to anchor it')
        c.create_text(20, H - 24, anchor='w', fill=DIM, font=('Menlo', 10),
                      text=f'viewed from the front.   keys:  x flip x   '
                           f'y flip y   s swap axes     '
                           f'[{"x" if BODY_X_SIGN > 0 else "-x"} '
                           f'{"y" if BODY_Y_SIGN > 0 else "-y"}]')



class AmbientView(View):
    """ambient temperature and light, from the light sensor's companion parts."""
    title = 'ambient'

    def __init__(self, parent, hub):
        super().__init__(parent, hub)
        self.canvas.config(width=780, height=470)

    def draw(self):
        c = self.canvas
        c.delete('all')
        W, H = 780, 470
        c.create_text(20, 26, anchor='w', fill=TEXT, font=('Menlo', 14, 'bold'),
                      text='ambient')
        c.create_text(20, 48, anchor='w', fill=DIM, font=('Menlo', 11),
                      text='the light sensor and its companion thermometer, '
                           'both inside the machine')

        t = self.hub.temp
        if t is None:
            c.create_text(W / 2, H / 2, text='waiting for the sensor...',
                          fill=DIM, font=('Menlo', 12))
            return

        c.create_text(60, 130, anchor='w', fill=ACCENT, font=('Menlo', 54, 'bold'),
                      text=f'{t:.1f}°C')
        c.create_text(60, 180, anchor='w', fill=DIM, font=('Menlo', 11),
                      text='sits inside the case, so it reads a degree or two '
                           'above room air')

        # 60 s trace
        x0, x1, y0, y1 = 60, W - 40, 240, 380
        c.create_rectangle(x0, y0, x1, y1, outline=GRID, fill=PANEL)
        hist = list(self.hub.temp_hist)
        if len(hist) > 2:
            t_now = hist[-1][0]
            span = 60.0
            lo = min(v for _, v in hist)
            hi = max(v for _, v in hist)
            pad = max(0.2, (hi - lo) * 0.2)
            lo, hi = lo - pad, hi + pad
            pts = []
            for tt, vv in hist:
                if t_now - tt > span:
                    continue
                x = x1 - (t_now - tt) / span * (x1 - x0)
                y = y1 - (vv - lo) / (hi - lo) * (y1 - y0)
                pts.extend((x, y))
            if len(pts) > 3:
                c.create_line(*pts, fill=ACCENT, width=2)
            c.create_text(x0 + 8, y0 + 10, anchor='nw', fill=DIM,
                          font=('Menlo', 10), text=f'{hi:.1f}')
            c.create_text(x0 + 8, y1 - 10, anchor='sw', fill=DIM,
                          font=('Menlo', 10), text=f'{lo:.1f}')
            c.create_text((x0 + x1) / 2, y1 + 18, text='last 60 s', fill=DIM,
                          font=('Menlo', 10))

        lux = self.hub.lux
        light = f'{lux:.0f} lux' if lux is not None else '--'
        if lux is not None:
            if lux < 10:
                desc = 'dim / dark room'
            elif lux < 100:
                desc = 'low light'
            elif lux < 500:
                desc = 'normal indoor light'
            else:
                desc = 'bright / daylight'
        else:
            desc = ''
        c.create_text(60, 415, anchor='w', fill=BLUE, font=('Menlo', 20, 'bold'),
                      text=light)
        c.create_text(240, 415, anchor='w', fill=DIM, font=('Menlo', 12),
                      text=desc)


class TrackpadView(View):
    """the Force Touch trackpad's pressure sensors, and weight on the pad."""
    title = 'trackpad'

    def __init__(self, parent, hub):
        super().__init__(parent, hub)
        self.canvas.config(width=780, height=470)
        import trackpad as tp
        self.tp_mod = tp
        self.pad = tp.Trackpad()
        if self.pad.available:
            self.pad.start()
        self.offset = self.pad.pressure_offset
        self.canvas.bind_all('<KeyPress-bracketright>',
                             lambda e: self._cycle(1))
        self.canvas.bind_all('<KeyPress-bracketleft>',
                             lambda e: self._cycle(-1))
        self.canvas.bind_all('<KeyPress-r>', lambda e: self.pad.reset_peak())

    def _cycle(self, d):
        cands = self.tp_mod.FIELD_CANDIDATES
        i = cands.index(self.offset) if self.offset in cands else 0
        self.offset = cands[(i + d) % len(cands)]
        self.pad.pressure_offset = self.offset
        self.pad.reset_peak()

    def draw(self):
        c = self.canvas
        c.delete('all')
        W, H = 780, 470
        c.create_text(20, 26, anchor='w', fill=TEXT, font=('Menlo', 14, 'bold'),
                      text='trackpad force')

        if not self.pad.available:
            c.create_text(20, 60, anchor='w', fill=RED, font=('Menlo', 12),
                          text=f'not available: {self.pad.error}')
            return

        s = self.pad.snapshot()
        c.create_text(20, 48, anchor='w', fill=DIM, font=('Menlo', 11),
                      text='force in grams. [ and ] pick which raw field to '
                           'read, r resets the peak')

        p = s['pressure']
        mx = self.tp_mod.PAD_MAX_GRAMS
        c.create_text(60, 130, anchor='w', fill=GREEN, font=('Menlo', 54, 'bold'),
                      text=f'{p:6.1f} g')
        c.create_text(60, 182, anchor='w', fill=DIM, font=('Menlo', 11),
                      text=f'peak {s["peak"]:.1f} g  (r resets)     '
                           f'fingers {s["nfingers"]}     '
                           f'field {self.offset}')

        # bar, full scale = the pad's rated 1 kg maximum
        bx0, bx1, by = 60, W - 60, 215
        c.create_rectangle(bx0, by, bx1, by + 26, outline=GRID, fill=PANEL)
        c.create_rectangle(bx0, by, bx0 + (bx1 - bx0) * min(1.0, p / mx),
                           by + 26, fill=GREEN, width=0)
        px = bx0 + (bx1 - bx0) * min(1.0, s['peak'] / mx)
        c.create_line(px, by - 4, px, by + 30, fill=ACCENT, width=2)
        c.create_text(bx1, by + 40, anchor='e', fill=DIM, font=('Menlo', 10),
                      text=f'full scale {mx:.0f} g (pad maximum)')

        # the raw field table, so the right one is obvious when you press
        c.create_text(60, 282, anchor='w', fill=TEXT, font=('Menlo', 12, 'bold'),
                      text='all raw fields -- 52 is pressure, 60/64 are contact '
                           'size, 56 is a fixed angle')
        x = 60
        for off in self.tp_mod.FIELD_CANDIDATES:
            v = s['fields'][off]
            sel = off == self.offset
            c.create_text(x, 310, anchor='w',
                          fill=ACCENT if sel else DIM, font=('Menlo', 10),
                          text=f'@{off}')
            c.create_text(x, 332, anchor='w',
                          fill=TEXT if sel else DIM, font=('Menlo', 11, 'bold'),
                          text=f'{v:7.2f}')
            x += 88

        c.create_text(20, H - 54, anchor='w', fill=DIM, font=('Menlo', 10),
                      text='the pad senses capacitance, so it only reports '
                           'while something conductive touches it --')
        c.create_text(20, H - 36, anchor='w', fill=DIM, font=('Menlo', 10),
                      text='rest a finger on an object to weigh it, or the '
                           'object alone will read nothing.')



class HeartbeatView(View):
    """ballistocardiogram: the pulse you can feel through the chassis.

    Rest your wrists on the machine and keep still. Each heartbeat nudges the
    whole laptop very slightly, and the accelerometer in the base picks that
    up. The band is 0.8-3 Hz (48-180 bpm), and the rate is found by
    autocorrelation -- sliding the signal against a delayed copy of itself and
    looking for the delay where it best matches its own rhythm.
    """
    title = 'heartbeat'

    MIN_LAG, MAX_LAG = 0.40, 1.50       # 40 - 150 bpm
    WINDOW = 10.0                       # seconds of signal to correlate

    def __init__(self, parent, hub):
        super().__init__(parent, hub)
        self.canvas.config(width=780, height=470)
        self.bpm = None
        self.conf = 0.0
        self.acorr = []
        self._last_calc = 0.0

    def draw(self):
        import math as _m
        c = self.canvas
        W, H = 780, 470
        now = time.monotonic() - self.hub.t0

        # recompute twice a second; it is O(n^2) and not worth doing faster
        if now - self._last_calc > 0.5 and len(self.hub.bcg) > self.hub.rate * 6:
            self._last_calc = now
            self._analyse()

        c.delete('all')
        c.create_text(20, 26, anchor='w', fill=TEXT, font=('Menlo', 14, 'bold'),
                      text='heartbeat')
        c.create_text(20, 48, anchor='w', fill=DIM, font=('Menlo', 11),
                      text='rest your wrists on the machine and keep still -- '
                           'it feels the pulse through the chassis')

        # waveform
        x0, x1, y0, y1 = 60, W - 20, 80, 250
        c.create_rectangle(x0, y0, x1, y1, outline=GRID, fill=PANEL)
        span = 6.0
        t_lo = now - span
        pts = []
        peak = 1e-9
        for t, v in self.hub.bcg:
            if t >= t_lo:
                peak = max(peak, abs(v))
        mid = (y0 + y1) / 2
        for t, v in self.hub.bcg:
            if t < t_lo:
                continue
            x = x0 + (t - t_lo) / span * (x1 - x0)
            y = mid - (v / peak) * (y1 - y0) * 0.45
            pts.extend((x, max(y0, min(y1, y))))
        if len(pts) > 3:
            c.create_line(*pts, fill=RED, width=2)
        c.create_text(x1 - 6, y0 + 12, anchor='ne', fill=DIM,
                      font=('Menlo', 10), text=f'+-{peak * 1000:.2f} mg')

        # rate
        if self.bpm is None:
            c.create_text(60, 300, anchor='w', fill=DIM, font=('Menlo', 20),
                          text='no steady pulse found')
            c.create_text(60, 335, anchor='w', fill=DIM, font=('Menlo', 11),
                          text='press your wrists down a little and stay still '
                               'for ~15 s')
        else:
            col = GREEN if self.conf > 0.35 else ACCENT
            c.create_text(60, 300, anchor='w', fill=col,
                          font=('Menlo', 46, 'bold'), text=f'{self.bpm:.0f} bpm')
            c.create_text(250, 300, anchor='w', fill=DIM, font=('Menlo', 12),
                          text=f'confidence {self.conf:.2f}')

        # autocorrelation curve
        ax0, ax1, ay0, ay1 = 60, W - 20, 370, 445
        c.create_rectangle(ax0, ay0, ax1, ay1, outline=GRID, fill=PANEL)
        if self.acorr:
            hi = max(r for _, r in self.acorr) or 1.0
            pl = []
            for lag, r in self.acorr:
                x = ax0 + (lag - self.MIN_LAG) / (self.MAX_LAG - self.MIN_LAG) * (ax1 - ax0)
                y = ay1 - max(0.0, r / hi) * (ay1 - ay0)
                pl.extend((x, y))
            if len(pl) > 3:
                c.create_line(*pl, fill=BLUE, width=2)
            if self.bpm:
                bl = 60.0 / self.bpm
                x = ax0 + (bl - self.MIN_LAG) / (self.MAX_LAG - self.MIN_LAG) * (ax1 - ax0)
                c.create_line(x, ay0, x, ay1, fill=GREEN, dash=(4, 3))
                c.create_text(x, ay0 - 10, text=f'{self.bpm:.0f}', fill=GREEN,
                              font=('Menlo', 11, 'bold'))
        for bpm in (40, 60, 80, 100, 120, 150):
            lag = 60.0 / bpm
            if self.MIN_LAG <= lag <= self.MAX_LAG:
                x = ax0 + (lag - self.MIN_LAG) / (self.MAX_LAG - self.MIN_LAG) * (ax1 - ax0)
                c.create_text(x, ay1 + 14, text=str(bpm), fill=DIM,
                              font=('Menlo', 9))
        c.create_text(ax0, ay0 - 12, anchor='w', fill=DIM, font=('Menlo', 10),
                      text='autocorrelation -- the tallest peak is the beat period')

    def _analyse(self):
        import math as _m
        rate = self.hub.rate
        buf = [v for _, v in self.hub.bcg][-int(rate * self.WINDOW):]
        n = len(buf)
        if n < rate * 5:
            return
        mean = sum(buf) / n
        cen = [x - mean for x in buf]
        var = sum(x * x for x in cen)
        if var < 1e-20:
            self.bpm, self.conf, self.acorr = None, 0.0, []
            return
        lo = int(rate * self.MIN_LAG)
        hi = min(int(rate * self.MAX_LAG), n - 1)
        curve = []
        best_r, best_lag = -1.0, lo
        for lag in range(lo, hi):
            s = 0.0
            for i in range(0, n - lag, 4):      # stride 4: plenty for 100 Hz
                s += cen[i] * cen[i + lag]
            r = s / var
            curve.append((lag / rate, r))
            if r > best_r:
                best_r, best_lag = r, lag
        self.acorr = curve
        if best_r > 0.15:
            self.bpm = 60.0 / (best_lag / rate)
            self.conf = min(1.0, best_r)
        else:
            self.bpm, self.conf = None, 0.0


# ---------------------------------------------------------------------- app

class App:
    def __init__(self, root, rate=FS):
        self.root = root
        root.title('spu tools')
        root.configure(bg=BG)
        self.hub = Hub(rate)

        nb = ttk.Notebook(root)
        nb.pack(fill='both', expand=True)
        self.views = []
        for cls in (AnglesView, LevelView, QuakeView, KnockView,
                    HeartbeatView, ModelView, AmbientView, TrackpadView):
            v = cls(nb, self.hub)
            nb.add(v, text=cls.title)
            self.views.append(v)
        self.nb = nb

        self._alive = True
        self._after = None
        self.last_draw = 0.0
        self.pump()

    def pump(self):
        if not self._alive:
            return
        self.hub.poll()
        # the trackpad's event source lives on the main run loop, so it has to
        # be serviced from here -- a background thread would spin without
        # receiving anything. a couple of ms per tick is enough.
        for v in self.views:
            pad = getattr(v, 'pad', None)
            if pad is not None and pad.available:
                pad.pump(0.002)
        now = time.monotonic()
        if now - self.last_draw > 0.045:
            self.last_draw = now
            try:
                idx = self.nb.index(self.nb.select())
                self.views[idx].draw()
            except Exception:
                pass
        self._after = self.root.after(12, self.pump)

    def quit(self):
        self._alive = False
        if self._after is not None:
            try:
                self.root.after_cancel(self._after)
            except Exception:
                pass
        try:
            for v in self.views:
                pad = getattr(v, 'pad', None)
                if pad is not None:
                    pad.stop()
            self.hub.stop()
        finally:
            self.root.destroy()


def main():
    import argparse
    import signal

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--rate', type=float, default=FS,
                    help=f'sensor sample rate in Hz (default {FS:.0f})')
    args = ap.parse_args()

    root = tk.Tk()
    app = App(root, args.rate)

    def _sig(signum, frame):
        app.quit()
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    root.mainloop()


if __name__ == '__main__':
    main()
