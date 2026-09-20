"""CoreHID backend for the SPU IMU.

spawns the `spu_hid` helper (built from _spu_hid.swift) and pumps its records
into the same shared-memory ring buffers the ctypes path in _spu.py uses, so
`IMU.read_accel()` and friends work unchanged regardless of backend.

why this exists: CoreHID (public since macOS 15) exposes the SPU sensors as
built-in HID devices with a dedicated `.spu` transport, so this path needs no
ctypes, no CFRunLoop pumping, no manual callback lifetime management, and no
privileged IORegistry property writes. see _spu.py for the ctypes fallback,
which is still used when the helper cannot be built.
"""

import os
import shutil
import struct
import subprocess
import sys

from ._spu import (
    SHM_HEADER, SHM_SNAP_HDR, ALS_REPORT_LEN, IMU_DECIMATION,
    shm_write_sample, shm_snap_write,
)

_HELPER_NAME = 'spu_hid'
_SWIFT_SOURCE = '_spu_hid.swift'
_BUILD_TIMEOUT = 180


def _package_dir() -> str:
    return os.path.dirname(os.path.abspath(__file__))


def _cache_dir() -> str:
    base = os.environ.get('XDG_CACHE_HOME') or os.path.join(
        os.path.expanduser('~'), '.cache')
    return os.path.join(base, 'macimu')


def helper_path() -> str:
    """Locate the built helper, or '' if it is not available.

    checks, in order: the MACIMU_SPU_HID override, a binary shipped next to
    the package, then a previously built copy in the user cache.
    """
    override = os.environ.get('MACIMU_SPU_HID')
    if override and os.path.isfile(override) and os.access(override, os.X_OK):
        return override

    bundled = os.path.join(_package_dir(), 'bin', _HELPER_NAME)
    if os.path.isfile(bundled) and os.access(bundled, os.X_OK):
        return bundled

    cached = os.path.join(_cache_dir(), _HELPER_NAME)
    if os.path.isfile(cached) and os.access(cached, os.X_OK):
        return cached

    return ''


def build_helper(force: bool = False) -> str:
    """Compile the helper into the user cache. Returns its path, or ''.

    needs the Swift toolchain (Command Line Tools provides swiftc). Returns
    '' rather than raising so callers can fall back to the ctypes backend.
    """
    src = os.path.join(_package_dir(), _SWIFT_SOURCE)
    if not os.path.isfile(src):
        return ''
    if not shutil.which('swiftc'):
        return ''

    dest_dir = _cache_dir()
    dest = os.path.join(dest_dir, _HELPER_NAME)
    if os.path.isfile(dest) and not force:
        return dest

    try:
        os.makedirs(dest_dir, exist_ok=True)
        tmp = dest + '.tmp'
        proc = subprocess.run(
            ['swiftc', '-O', '-o', tmp, src],
            capture_output=True, text=True, timeout=_BUILD_TIMEOUT)
        if proc.returncode != 0:
            return ''
        os.replace(tmp, dest)
        os.chmod(dest, 0o755)
        return dest
    except Exception:
        return ''


def available() -> bool:
    """True if the CoreHID backend can be used (helper present or buildable)."""
    return bool(helper_path()) or bool(build_helper())


def hid_worker(shm_name, restart_count, gyro_shm_name=None,
               als_shm_name=None, lid_shm_name=None, decimation=None,
               shutdown_event=None, temp_shm_name=None):
    """Drop-in replacement for _spu.sensor_worker using the CoreHID helper.

    Same signature and same shared-memory contract, so the caller does not
    care which backend produced the samples.
    """
    import multiprocessing.shared_memory

    binary = helper_path() or build_helper()
    if not binary:
        raise RuntimeError(
            'spu_hid helper unavailable -- build it with '
            '`swiftc -O -o <path> macimu/_spu_hid.swift` or set MACIMU_SPU_HID')

    dec_n = decimation if decimation is not None else IMU_DECIMATION

    # keep the SharedMemory objects themselves alive -- .buf alone is a view
    # onto an mmap that is closed as soon as the object is collected
    accel_shm = multiprocessing.shared_memory.SharedMemory(
        name=shm_name, create=False)
    accel_buf = accel_shm.buf
    struct.pack_into('<I', accel_buf, 12, restart_count)

    gyro_shm = als_shm = lid_shm = None
    gyro_buf = als_buf = lid_buf = None

    if gyro_shm_name:
        gyro_shm = multiprocessing.shared_memory.SharedMemory(
            name=gyro_shm_name, create=False)
        gyro_buf = gyro_shm.buf

    if als_shm_name:
        als_shm = multiprocessing.shared_memory.SharedMemory(
            name=als_shm_name, create=False)
        als_buf = als_shm.buf

    if lid_shm_name:
        lid_shm = multiprocessing.shared_memory.SharedMemory(
            name=lid_shm_name, create=False)
        lid_buf = lid_shm.buf

    if temp_shm_name:
        temp_shm = multiprocessing.shared_memory.SharedMemory(
            name=temp_shm_name, create=False)
        temp_buf = temp_shm.buf

    cmd = [binary, '--decimation', str(dec_n)]
    cmd.append('--gyro' if gyro_buf is not None else '--no-gyro')
    cmd.append('--als' if als_buf is not None else '--no-als')
    cmd.append('--lid' if lid_buf is not None else '--no-lid')
    cmd.append('--temp' if temp_buf is not None else '--no-temp')

    proc = subprocess.Popen(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
        text=True, bufsize=1)

    try:
        for line in proc.stdout:
            if shutdown_event is not None and shutdown_event.is_set():
                break
            try:
                parts = line.split()
                if not parts:
                    continue
                kind = parts[0]

                if kind == 'A':
                    t = float(parts[1])
                    shm_write_sample(accel_buf, int(parts[2]), int(parts[3]),
                                     int(parts[4]), t)
                elif kind == 'G':
                    if gyro_buf is None:
                        continue
                    t = float(parts[1])
                    shm_write_sample(gyro_buf, int(parts[2]), int(parts[3]),
                                     int(parts[4]), t)
                elif kind == 'S':
                    if als_buf is None:
                        continue
                    payload = bytes.fromhex(parts[2])
                    if len(payload) == ALS_REPORT_LEN:
                        shm_snap_write(als_buf, payload)
                elif kind == 'T':
                    if temp_buf is None:
                        continue
                    struct.pack_into('<f', temp_buf, SHM_SNAP_HDR,
                                     float(parts[2]))
                    cnt, = struct.unpack_from('<I', temp_buf, 0)
                    struct.pack_into('<I', temp_buf, 0, cnt + 1)
                elif kind == 'D':
                    if lid_buf is None:
                        continue
                    angle = float(parts[2])
                    struct.pack_into('<f', lid_buf, SHM_SNAP_HDR, angle)
                    cnt, = struct.unpack_from('<I', lid_buf, 0)
                    struct.pack_into('<I', lid_buf, 0, cnt + 1)
            except (ValueError, IndexError, TypeError, struct.error):
                continue
    finally:
        try:
            proc.terminate()
            proc.wait(timeout=2)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass
