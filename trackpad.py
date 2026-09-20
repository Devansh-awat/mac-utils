"""Read the Force Touch trackpad through Apple's private MultitouchSupport.

The trackpad's pressure sensors are not exposed by any public API. The only
route is `MultitouchSupport.framework`, which Apple does not document and
whose per-finger struct has changed between macOS releases. The offsets below
were confirmed here by calibration rather than copied from another project,
and the raw field table is still exposed so a future macOS that shuffles the
struct can be re-solved the same way -- see `FIELD_CANDIDATES`.

Layout confirmed on macOS 27 by press calibration (30x22 grid trackpad).
Read the raw little-endian values at these offsets:

    offset  0  int32   frame counter
    offset  8  double  timestamp
    offset 16  int32   identifier
    offset 20  int32   state
    offset 32  float   position x   (normalized 0..1)
    offset 36  float   position y
    offset 40  float   velocity x
    offset 44  float   velocity y
    offset 52  float   PRESSURE IN GRAMS   <- the one that matters
    offset 56  float   constant 1.5708 (pi/2); a default angle, never moves
    offset 60  float   contact size, ~7-9 for a fingertip, pressure-independent
    offset 64  float   contact size, same behaviour, slightly smaller
    offset 48  float   varies 0.3-1.0, not force
    offset 68  float   varies, negative, not force
    offset 72  float   varies, not force
    offset 76  float   noisy, not force

How offset 52 was identified: it is the only field that reads exactly 0 with
no finger, tens of grams at a light rest, and ~980 g under a hard press --
and 1 kg is the Force Touch pad's rated maximum. The size fields stay flat
across that whole range.

Weight: the value is already in grams. The pad senses capacitance, so it only
reports while something conductive is in contact -- an inert object alone on
the glass produces no frames, because no capacitance means no contact.
"""

import ctypes
import ctypes.util
import struct
import threading
import time

_FRAMEWORK = ('/System/Library/PrivateFrameworks/'
              'MultitouchSupport.framework/MultitouchSupport')

# float offsets inside a finger record, in bytes
FIELD_CANDIDATES = (48, 52, 56, 60, 64, 68, 72, 76)

# confirmed by press calibration; the view can still cycle it
DEFAULT_PRESSURE_OFFSET = 52
PAD_MAX_GRAMS = 1000.0          # Force Touch rated maximum
FINGER_RECORD_MIN = 80


class Trackpad:
    """Live trackpad readings.

    IMPORTANT: the multitouch source lives on the *main* run loop. Pumping a
    run loop from a worker thread returns immediately every time -- measured
    at 6.6 million iterations in 3 seconds -- which both burns a core and
    receives no callbacks at all. So `start()` and `pump()` must both be
    called from the main thread: the app pumps from its Tk tick, and a
    standalone script uses `run_forever()`.
    """

    def __init__(self, pressure_offset: int = DEFAULT_PRESSURE_OFFSET):
        self.available = False
        self.error = ''
        self.pressure_offset = pressure_offset
        self.lock = threading.Lock()
        self.pressure = 0.0
        self.nfingers = 0
        self.peak = 0.0
        self.frames = 0
        self.fields = {off: 0.0 for off in FIELD_CANDIDATES}
        self.pos = (0.0, 0.0)
        self._mt = None
        self._cf = None
        self._cb = None            # MUST be kept alive: a collected callback
                                   # segfaults the moment the pad reports
        self._mode = None
        self._started = False
        self._stop = threading.Event()
        self._probe()

    # ------------------------------------------------------------- setup

    def _probe(self):
        try:
            self._mt = ctypes.CDLL(_FRAMEWORK)
        except OSError as e:
            self.error = f'MultitouchSupport unavailable: {e}'
            return
        try:
            self._cf = ctypes.CDLL(ctypes.util.find_library('CoreFoundation'))
        except OSError as e:
            self.error = f'CoreFoundation unavailable: {e}'
            return

        mt, cf = self._mt, self._cf
        cf.CFArrayGetCount.restype = ctypes.c_long
        cf.CFArrayGetCount.argtypes = [ctypes.c_void_p]
        cf.CFArrayGetValueAtIndex.restype = ctypes.c_void_p
        cf.CFArrayGetValueAtIndex.argtypes = [ctypes.c_void_p, ctypes.c_long]
        cf.CFRunLoopRunInMode.restype = ctypes.c_int32
        cf.CFRunLoopRunInMode.argtypes = [ctypes.c_void_p, ctypes.c_double,
                                          ctypes.c_bool]
        cf.CFStringCreateWithCString.restype = ctypes.c_void_p
        cf.CFStringCreateWithCString.argtypes = [ctypes.c_void_p,
                                                 ctypes.c_char_p,
                                                 ctypes.c_uint32]
        mt.MTDeviceCreateList.restype = ctypes.c_void_p

        try:
            devs = mt.MTDeviceCreateList()
            count = cf.CFArrayGetCount(devs) if devs else 0
        except Exception as e:
            self.error = f'device list failed: {e}'
            return
        if count == 0:
            self.error = 'no multitouch device found (needs a Force Touch pad)'
            return

        self._dev = cf.CFArrayGetValueAtIndex(devs, 0)
        self._cb_type = ctypes.CFUNCTYPE(
            ctypes.c_int, ctypes.c_void_p, ctypes.c_void_p,
            ctypes.c_int, ctypes.c_double, ctypes.c_int)
        self._mode = cf.CFStringCreateWithCString(
            None, b'kCFRunLoopDefaultMode', 0x08000100)
        self.available = True

    # -------------------------------------------------------------- run

    def start(self):
        """Register the callback and start the device. Call from the main thread."""
        if not self.available or self._started:
            return
        mt = self._mt

        def on_frame(device, data, nfingers, timestamp, frame):
            try:
                if nfingers > 0 and data:
                    raw = ctypes.string_at(data, FINGER_RECORD_MIN)
                    vals = {}
                    for off in FIELD_CANDIDATES:
                        vals[off] = struct.unpack_from('<f', raw, off)[0]
                    px = struct.unpack_from('<f', raw, 32)[0]
                    py = struct.unpack_from('<f', raw, 36)[0]
                    with self.lock:
                        self.fields = vals
                        self.pos = (px, py)
                        self.nfingers = nfingers
                        self.frames += 1
                        p = vals.get(self.pressure_offset, 0.0)
                        if 0.0 <= p < 5000.0:      # guard a bad offset
                            self.pressure = p
                            if p > self.peak:
                                self.peak = p
                else:
                    with self.lock:
                        self.nfingers = 0
                        self.pressure = 0.0
            except Exception:
                pass
            return 0

        keep = self._cb_type(on_frame)
        self._cb = keep                    # keep alive or it segfaults
        mt.MTRegisterContactFrameCallback.restype = None
        mt.MTRegisterContactFrameCallback.argtypes = [ctypes.c_void_p,
                                                      self._cb_type]
        mt.MTRegisterContactFrameCallback(self._dev, keep)
        mt.MTDeviceStart.restype = None
        mt.MTDeviceStart.argtypes = [ctypes.c_void_p, ctypes.c_int]
        mt.MTDeviceStart(self._dev, 0)
        self._started = True

    def pump(self, timeout: float = 0.0):
        """Service pending trackpad events. Call from the main thread.

        A zero timeout processes whatever is queued and returns at once, so
        this is safe to call from a UI tick at ~100 Hz.
        """
        if self.available and self._started:
            self._cf.CFRunLoopRunInMode(self._mode, timeout, False)

    def run_forever(self, seconds=None):
        """Standalone helper: pump on the calling thread until stopped."""
        self.start()
        t0 = time.time()
        while not self._stop.is_set():
            if seconds is not None and time.time() - t0 >= seconds:
                break
            self.pump(0.1)

    def snapshot(self):
        with self.lock:
            return {
                'pressure': self.pressure,
                'nfingers': self.nfingers,
                'peak': self.peak,
                'frames': self.frames,
                'fields': dict(self.fields),
                'pos': self.pos,
            }

    def reset_peak(self):
        with self.lock:
            self.peak = 0.0

    def stop(self):
        self._stop.set()


if __name__ == '__main__':
    import sys
    pad = Trackpad()
    print(f'available: {pad.available}  {pad.error}')
    if pad.available:
        print('press the trackpad -- showing every candidate field, 12s\n')
        print('  time  nf  ' + '  '.join(f'{o:>8}' for o in FIELD_CANDIDATES))
        pad.start()
        end = time.time() + 12
        while time.time() < end:
            pad.pump(0.05)
            st = pad.snapshot()
            row = '  '.join(f'{st["fields"][o]:8.2f}' for o in FIELD_CANDIDATES)
            print(f'  {time.time()%100:5.1f} {st["nfingers"]:2d}  {row}', flush=True)
        pad.stop()
        print(f'\npeak at offset {pad.pressure_offset}: {pad.snapshot()["peak"]:.2f}')
