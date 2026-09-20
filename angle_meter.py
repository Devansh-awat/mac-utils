#!/usr/bin/env python3
"""angle_meter.py -- how far is the screen tilted?

one diagram, two angles:

  A  the angle inside the hinge, between the keyboard and the screen
  B  the angle between the BACK of the screen and the table, drawn behind
     the screen. B is simply 180 - A.

  A comes from the lid angle sensor. no accelerometer, no calibration,
  no zeroing -- the sensor reports the physical hinge angle directly.

    python3 angle_meter.py
"""

import math
import tkinter as tk

from macimu import IMU

# layout, in canvas pixels
CANVAS_W, CANVAS_H = 780, 470
TABLE_Y = 395
HINGE_X = 300
DECK_LEN = 215
SCREEN_LEN = 235
ARC_A_R = 95
ARC_B_R = 150

# colours
BG = '#101014'
TABLE = '#3a3a44'
TABLE_TOP = '#4a4a58'
DECK = '#5a5f6b'
DECK_TOP = '#7d8496'
SCREEN = '#2f6fbf'
SCREEN_EDGE = '#5fa8f5'
TEXT = '#e8e8ee'
DIM = '#8a8a99'
A_COL = '#f0b429'
B_COL = '#3ddc84'


class AngleMeter:
    def __init__(self, root):
        self.root = root
        root.title('screen angle')
        root.configure(bg=BG)
        root.resizable(False, False)

        self.canvas = tk.Canvas(root, width=CANVAS_W, height=CANVAS_H,
                                bg=BG, highlightthickness=0)
        self.canvas.pack()

        self.readout = tk.Label(root, bg=BG, fg=TEXT, justify='left',
                                font=('Menlo', 15), padx=18, pady=10)
        self.readout.pack(fill='x')

        self.imu = IMU(gyro=False, lid=True, sample_rate=50)
        self.imu.start()

        root.protocol('WM_DELETE_WINDOW', self.quit)
        self.angle_a = None
        self._alive = True
        self._after_id = None
        self.tick()

    # ---------------------------------------------------------------- drawing

    def draw(self, a):
        c = self.canvas
        c.delete('all')

        # table surface
        c.create_rectangle(0, TABLE_Y, CANVAS_W, CANVAS_H, fill=TABLE, width=0)
        c.create_line(0, TABLE_Y, CANVAS_W, TABLE_Y, fill=TABLE_TOP, width=2)

        hx, hy = HINGE_X, TABLE_Y

        # keyboard deck, lying on the table
        c.create_line(hx, hy - 7, hx - DECK_LEN, hy - 7, fill=DECK, width=17,
                      capstyle='round')
        c.create_line(hx, hy - 13, hx - DECK_LEN, hy - 13, fill=DECK_TOP, width=3)

        # screen: direction (180 - a) measured anticlockwise from +x, y-up,
        # which in canvas coordinates (y-down) flips the sine
        rad = math.radians(180.0 - a)
        ux, uy = math.cos(rad), -math.sin(rad)
        sx, sy = hx + SCREEN_LEN * ux, hy + SCREEN_LEN * uy
        c.create_line(hx, hy - 7, sx, sy - 7, fill=SCREEN, width=15,
                      capstyle='round')
        c.create_line(hx, hy - 7, sx, sy - 7, fill=SCREEN_EDGE, width=3)

        # angle A -- at the hinge, between deck (pointing left, 180 deg) and
        # the screen. tk arcs run anticlockwise from +x.
        c.create_arc(hx - ARC_A_R, hy - ARC_A_R, hx + ARC_A_R, hy + ARC_A_R,
                     start=180.0 - a, extent=a, style='arc',
                     outline=A_COL, width=3)
        mid = math.radians(180.0 - a / 2.0)
        c.create_text(hx + (ARC_A_R + 26) * math.cos(mid),
                      hy - (ARC_A_R + 26) * math.sin(mid),
                      text=f'A  {a:.1f}°', fill=A_COL,
                      font=('Menlo', 14, 'bold'))

        # angle B -- behind the screen, between the table (pointing right,
        # 0 deg) and the screen
        b = 180.0 - a
        c.create_arc(hx - ARC_B_R, hy - ARC_B_R, hx + ARC_B_R, hy + ARC_B_R,
                     start=0.0, extent=b, style='arc',
                     outline=B_COL, width=3, dash=(6, 4))
        midb = math.radians(b / 2.0)
        c.create_text(hx + (ARC_B_R + 24) * math.cos(midb),
                      hy - (ARC_B_R + 24) * math.sin(midb),
                      text=f'B  {b:.1f}°', fill=B_COL,
                      font=('Menlo', 14, 'bold'))

        # a faint line continuing the screen's plane down to the table, so
        # angle B is visibly measured against the table
        c.create_line(sx, sy - 7, hx, hy - 7, fill=B_COL, width=1, dash=(3, 5))

        c.create_text(hx - 40, hy + 22, text='hinge', fill=DIM,
                      font=('Menlo', 11))
        c.create_text(hx - DECK_LEN + 20, hy + 22, text='keyboard', fill=DIM,
                      font=('Menlo', 11))
        c.create_text(CANVAS_W - 14, hy + 22, text='table', fill=DIM,
                      anchor='e', font=('Menlo', 11))

    # ------------------------------------------------------------------- loop

    def tick(self):
        if not self._alive:
            return
        lid = self.imu.read_lid()
        if lid is not None:
            self.angle_a = float(lid)

        if self.angle_a is None:
            self.readout.config(text='waiting for the lid sensor...')
        else:
            a = self.angle_a
            self.draw(a)
            self.readout.config(
                text=f"A   keyboard <-> screen      {a:6.1f}°     (lid sensor)\n"
                     f"B   screen back <-> table   {180.0 - a:6.1f}°     (180 - A)")

        self._after_id = self.root.after(40, self.tick)

    def quit(self):
        # stop the timer first, otherwise a queued tick fires after the
        # window is gone and Tk raises "invalid command name .!canvas"
        self._alive = False
        if self._after_id is not None:
            try:
                self.root.after_cancel(self._after_id)
            except Exception:
                pass
            self._after_id = None
        try:
            self.imu.stop()
        finally:
            self.root.destroy()


if __name__ == '__main__':
    import signal

    root = tk.Tk()
    app = AngleMeter(root)

    # shut down cleanly when killed from the terminal, so the shared-memory
    # segments get unlinked instead of leaking
    def _sig(signum, frame):
        app.quit()
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    root.mainloop()
