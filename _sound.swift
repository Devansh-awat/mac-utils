// _sound.swift -- listen to the microphone and report what it hears.
//
// Companion to sound.py. Captures from the default input with AVAudioEngine,
// runs an FFT via vDSP, and streams compact results on stdout so the Python
// side only has to draw them:
//
//   L <db>                        RMS level, dBFS (negative; 0 is full scale)
//   N <freq> <note> <cents> <amp> dominant pitch, nearest note, deviation
//   S <b0> <b1> ... <bN>          log-spaced spectrum, integers 0-100
//
// build: swiftc -O -o sound_probe _sound.swift

import Foundation
import AVFoundation
import Accelerate

// MARK: - output

let outLock = NSLock()
let outHandle = FileHandle.standardOutput

func emit(_ line: String) {
    outLock.lock()
    outHandle.write(Data((line + "\n").utf8))
    outLock.unlock()
}

// MARK: - tuner maths

let NOTE_NAMES = ["C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"]

/// nearest equal-tempered note to a frequency, assuming A4 = 440 Hz
func nearestNote(_ hz: Double) -> (String, Double, Double) {
    let midi = 69.0 + 12.0 * log2(hz / 440.0)
    let nearest = midi.rounded()
    let cents = (midi - nearest) * 100.0
    let idx = ((Int(nearest) % 12) + 12) % 12
    let octave = Int(nearest) / 12 - 1
    return ("\(NOTE_NAMES[idx])\(octave)", cents, nearest)
}

// MARK: - capture

let fftSize = 4096
let log2n = vDSP_Length(log2(Double(fftSize)))
guard let fftSetup = vDSP_create_fftsetup(log2n, FFTRadix(kFFTRadix2)) else {
    FileHandle.standardError.write(Data("could not create FFT setup\n".utf8))
    exit(1)
}

// log-spaced display bands, 40 Hz .. 16 kHz
let bandCount = 40
let bandEdges: [Double] = (0...bandCount).map { i in
    40.0 * pow(16000.0 / 40.0, Double(i) / Double(bandCount))
}

var window = [Float](repeating: 0, count: fftSize)
vDSP_hann_window(&window, vDSP_Length(fftSize), Int32(vDSP_HANN_NORM))

let engine = AVAudioEngine()
let input = engine.inputNode
let format = input.outputFormat(forBus: 0)
guard format.sampleRate > 0 else {
    FileHandle.standardError.write(Data("no usable audio input\n".utf8))
    exit(1)
}
let sampleRate = format.sampleRate

// Tracks how loud the loudest spectral peak usually is when nothing musical
// is happening, so the note threshold adapts to the room instead of being a
// constant that is wrong everywhere except where it was tuned.
var noiseFloor: Double = 25.0
var emitted = 0
input.installTap(onBus: 0, bufferSize: AVAudioFrameCount(fftSize), format: format) {
    buffer, _ in
    guard let chans = buffer.floatChannelData else { return }
    let n = Int(buffer.frameLength)
    guard n >= fftSize else { return }
    let samples = Array(UnsafeBufferPointer(start: chans[0], count: n))

    // ---- level in dBFS
    var rms: Float = 0
    vDSP_rmsqv(samples, 1, &rms, vDSP_Length(n))
    let db = rms > 1e-9 ? 20.0 * log10(Double(rms)) : -120.0

    // ---- windowed magnitude spectrum
    var windowed = [Float](repeating: 0, count: fftSize)
    vDSP_vmul(samples, 1, window, 1, &windowed, 1, vDSP_Length(fftSize))

    var real = [Float](repeating: 0, count: fftSize / 2)
    var imag = [Float](repeating: 0, count: fftSize / 2)
    var mags = [Float](repeating: 0, count: fftSize / 2)
    real.withUnsafeMutableBufferPointer { rp in
        imag.withUnsafeMutableBufferPointer { ip in
            var split = DSPSplitComplex(realp: rp.baseAddress!, imagp: ip.baseAddress!)
            windowed.withUnsafeBufferPointer { wp in
                wp.baseAddress!.withMemoryRebound(to: DSPComplex.self,
                                                  capacity: fftSize / 2) { cp in
                    vDSP_ctoz(cp, 2, &split, 1, vDSP_Length(fftSize / 2))
                }
            }
            vDSP_fft_zrip(fftSetup, &split, 1, log2n, FFTDirection(FFT_FORWARD))
            vDSP_zvabs(&split, 1, &mags, 1, vDSP_Length(fftSize / 2))
        }
    }

    // ---- log bands, 0-100
    let binHz = sampleRate / Double(fftSize)
    var bands = [Int](repeating: 0, count: bandCount)
    for b in 0..<bandCount {
        let lo = max(1, Int(bandEdges[b] / binHz))
        let hi = min(mags.count - 1, max(lo, Int(bandEdges[b + 1] / binHz)))
        var peak: Float = 0
        for i in lo...hi { peak = max(peak, mags[i]) }
        // map roughly -70..0 dBFS onto 0..100
        let d = peak > 1e-9 ? 20.0 * log10(Double(peak) / Double(fftSize)) : -90.0
        bands[b] = max(0, min(100, Int((d + 70.0) / 70.0 * 100.0)))
    }

    // ---- dominant pitch, searching 60 Hz .. 2 kHz
    var bestIdx = 0
    var bestVal: Float = 0
    let loBin = max(1, Int(60.0 / binHz))
    let hiBin = min(mags.count - 1, Int(2000.0 / binHz))
    if loBin < hiBin {
        for i in loBin...hiBin where mags[i] > bestVal {
            bestVal = mags[i]
            bestIdx = i
        }
    }
    // parabolic interpolation around the peak for sub-bin accuracy
    var peakHz = Double(bestIdx) * binHz
    if bestIdx > 0 && bestIdx < mags.count - 1 {
        let a = Double(mags[bestIdx - 1]), b0 = Double(mags[bestIdx]), c = Double(mags[bestIdx + 1])
        let denom = a - 2 * b0 + c
        if abs(denom) > 1e-12 {
            peakHz = (Double(bestIdx) + 0.5 * (a - c) / denom) * binHz
        }
    }

    emitted += 1
    if emitted % 2 != 0 { return }        // ~20 Hz is plenty

    // A tuner that names a note for room hum is worse than no tuner. Require
    // the peak to stand clearly above the surrounding spectrum before
    // claiming there is a pitch at all.
    var band = [Float]()
    if loBin < hiBin { for i in loBin...hiBin { band.append(mags[i]) } }
    var prominent = false
    if !band.isEmpty {
        let sorted = band.sorted()
        let median = sorted[sorted.count / 2]
        // Ratio test: the peak must tower over the rest of the spectrum.
        // Level test: it must also beat this room's own noise floor, learnt
        // continuously below. Either alone is fooled -- room hum here is tonal
        // enough to pass a ratio, and a quiet room can spike in absolute terms.
        let ratioOK = median > 0 ? Double(bestVal) > Double(median) * 10.0
                                 : bestVal > 1e-5
        let levelOK = Double(bestVal) > max(40.0, noiseFloor * 4.0)
        prominent = ratioOK && levelOK
        if !prominent {
            noiseFloor += 0.05 * (Double(bestVal) - noiseFloor)
        }
    }

    emit(String(format: "L %.1f", db))
    if prominent && bestVal > 1e-6 && db > -60.0 && peakHz > 60 {
        let (note, cents, _) = nearestNote(peakHz)
        emit(String(format: "N %.1f %@ %.0f %.3f", peakHz, note, cents, Double(bestVal)))
    } else {
        emit("N 0 - 0 0")
    }
    emit("S " + bands.map(String.init).joined(separator: " "))
}

engine.prepare()
do {
    try engine.start()
} catch {
    FileHandle.standardError.write(Data("audio engine failed: \(error)\n".utf8))
    exit(1)
}

emit("R \(Int(sampleRate))")
RunLoop.main.run()
