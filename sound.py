"""Listen to the microphone: level, spectrum and the musical note being played.

Wraps the `sound_probe` helper built from `_sound.swift`. That helper does the
capture (AVAudioEngine) and the analysis (vDSP FFT) in Swift and streams
compact results here, because doing a 4096-point FFT per frame in pure Python
would not keep up.

The helper needs a usage description or macOS kills it on launch, so the build
embeds `_sound.plist` into the binary. First run will prompt for microphone
access; nothing is recorded, saved or sent -- samples are analysed in memory
and discarded.
"""

import os
import shutil
import subprocess
import threading
import time

_HELPER = 'sound_probe'
_SOURCE = '_sound.swift'
_PLIST = '_sound.plist'
_BUILD_TIMEOUT = 240
BANDS = 40


def _package_dir() -> str:
    return os.path.dirname(os.path.abspath(__file__))


def _cache_dir() -> str:
    base = os.environ.get('XDG_CACHE_HOME') or os.path.join(
        os.path.expanduser('~'), '.cache')
    return os.path.join(base, 'macutils')


def helper_path() -> str:
    override = os.environ.get('MACUTILS_SOUND_PROBE')
    if override and os.path.isfile(override) and os.access(override, os.X_OK):
        return override
    local = os.path.join(_package_dir(), _HELPER)
    if os.path.isfile(local) and os.access(local, os.X_OK):
        return local
    cached = os.path.join(_cache_dir(), _HELPER)
    if os.path.isfile(cached) and os.access(cached, os.X_OK):
        return cached
    return ''


def build_helper(force: bool = False) -> str:
    """Compile the probe. Returns its path, or '' if swiftc is unavailable."""
    d = _package_dir()
    src, plist = os.path.join(d, _SOURCE), os.path.join(d, _PLIST)
    if not (os.path.isfile(src) and os.path.isfile(plist)):
        return ''
    if not shutil.which('swiftc'):
        return ''
    cand = [os.path.join(d, _HELPER), os.path.join(_cache_dir(), _HELPER)]
    for path in cand:
        if os.path.isfile(path) and not force:
            return path
    dest = cand[0] if os.access(d, os.W_OK) else cand[1]
    try:
        os.makedirs(os.path.dirname(dest), exist_ok=True)
        tmp = dest + '.tmp'
        proc = subprocess.run(
            ['swiftc', '-O', '-o', tmp, src,
             '-Xlinker', '-sectcreate', '-Xlinker', '__TEXT',
             '-Xlinker', '__info_plist', '-Xlinker', plist],
            capture_output=True, text=True, timeout=_BUILD_TIMEOUT)
        if proc.returncode != 0:
            return ''
        os.replace(tmp, dest)
        os.chmod(dest, 0o755)
        return dest
    except Exception:
        return ''


def available() -> bool:
    return bool(helper_path()) or bool(build_helper())


class Sound:
    """Live microphone analysis. Reads on a background thread."""

    def __init__(self):
        self.available = False
        self.error = ''
        self.lock = threading.Lock()
        self.level_db = -120.0
        self.note = None
        self.freq = 0.0
        self.cents = 0.0
        self.amp = 0.0
        self.bands = [0] * BANDS
        self.peak_level = -120.0
        self.sample_rate = 0
        self._proc = None
        self._thread = None
        self._stop = threading.Event()

    def start(self):
        if self._thread is not None:
            return
        binary = helper_path() or build_helper()
        if not binary:
            self.error = ('sound_probe unavailable -- needs the Swift toolchain '
                          'and _sound.swift/_sound.plist next to this module')
            return
        try:
            self._proc = subprocess.Popen(
                [binary], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                text=True, bufsize=1)
        except Exception as e:
            self.error = f'could not start sound_probe: {e}'
            return
        self.available = True
        self._thread = threading.Thread(target=self._read, daemon=True)
        self._thread.start()

    def _read(self):
        proc = self._proc
        for line in proc.stdout:
            if self._stop.is_set():
                break
            try:
                p = line.split()
                if not p:
                    continue
                kind = p[0]
                if kind == 'R':
                    self.sample_rate = int(p[1])
                elif kind == 'L':
                    v = float(p[1])
                    with self.lock:
                        self.level_db = v
                        self.peak_level = max(self.peak_level, v)
                elif kind == 'N':
                    with self.lock:
                        self.freq = float(p[1])
                        self.note = p[2]
                        self.cents = float(p[3])
                        self.amp = float(p[4]) if len(p) > 4 else 0.0
                elif kind == 'S':
                    vals = [int(x) for x in p[1:1 + BANDS]]
                    if len(vals) == BANDS:
                        with self.lock:
                            self.bands = vals
            except (ValueError, IndexError):
                continue

    def snapshot(self):
        with self.lock:
            return {
                'level': self.level_db,
                'peak': self.peak_level,
                'note': self.note,
                'freq': self.freq,
                'cents': self.cents,
                'amp': self.amp,
                'bands': list(self.bands),
                'rate': self.sample_rate,
            }

    def reset_peak(self):
        with self.lock:
            self.peak_level = -120.0

    def stop(self):
        self._stop.set()
        if self._proc:
            try:
                self._proc.terminate()
                self._proc.wait(timeout=2)
            except Exception:
                try:
                    self._proc.kill()
                except Exception:
                    pass


if __name__ == '__main__':
    import sys
    s = Sound()
    s.start()
    if not s.available:
        print('unavailable:', s.error)
        sys.exit(1)
    print(f'sample rate {s.snapshot()["rate"]} Hz')
    print('listen for 12 s -- talk, whistle, or play an instrument\n')
    end = time.time() + 12
    while time.time() < end:
        d = s.snapshot()
        note = d['note'] or '--'
        print(f"  {d['level']:7.1f} dB   {note:5} "
              f"{d['freq']:7.1f} Hz  {d['cents']:+5.0f} cents   "
              + ''.join('#' if b > 40 else ('.' if b > 12 else ' ')
                        for b in d['bands']))
        time.sleep(0.4)
    s.stop()
