# Third-party notices

This project is licensed AGPL-3.0 (see `LICENSE`). It includes code from the
project below, under its own license. That license is reproduced in full here
because it requires its notice to be preserved.

---

## macimu

Files: everything under `macimu/` except `_spu_hid.py` and `_spu_hid.swift`,
which are original to this project. The library provides the sensor-reading
layer that all the apps here depend on.

Upstream: https://github.com/olvvier/apple-silicon-accelerometer

```
MIT License

Copyright (c) 2026 olvvier

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

---

## not included

The upstream project also vendors **KBPulse** (MIT, Copyright (c) 2021 Ethan
Chaffin), a keyboard-backlight driver. It is not used by anything here and has
deliberately been left out, so that this repository carries only one third-party
notice.

No Apple code is included or redistributed. The tools call Apple frameworks
(`CoreHID`, and the private `MultitouchSupport`) at runtime through their
system-installed binaries.
