// _spu_hid.swift -- read the Apple Silicon SPU IMU via the public CoreHID framework.
//
// alternative backend to the ctypes/IOKit path in _spu.py. CoreHID exposes the
// SPU sensors as built-in HID devices with a dedicated `.spu` transport, so no
// ctypes, no CFRunLoop pumping, no manual callback lifetime management, and no
// privileged IORegistry property writes to wake the drivers.
//
// emits newline-delimited records on stdout, one per sample:
//   A <t> <x> <y> <z>        accel, raw q16 ints
//   G <t> <x> <y> <z>        gyro,  raw q16 ints
//   S <t> <hex>              ambient light, raw report bytes
//   D <t> <angle>            lid angle, degrees
//   T <t> <celsius>          ambient temperature, degrees C
// <t> is seconds since boot (mach_absolute_time), matching _spu.py's timestamps.
//
// build: swiftc -O -o macimu/bin/spu_hid macimu/_spu_hid.swift

import Foundation
import CoreHID
import Darwin

// MARK: - timing

var timebase = mach_timebase_info_data_t()
mach_timebase_info(&timebase)
let machToSec = Double(timebase.numer) / Double(timebase.denom) * 1e-9
func nowSec() -> Double { Double(mach_absolute_time()) * machToSec }

// MARK: - stdout writer
//
// several sensor tasks emit concurrently, so serialize whole lines

let outLock = NSLock()
let stdoutHandle = FileHandle.standardOutput

func emit(_ line: String) {
    outLock.lock()
    stdoutHandle.write(Data((line + "\n").utf8))
    outLock.unlock()
}

// MARK: - args

var decimation = 8
var wantAccel = true, wantGyro = true, wantAls = true, wantLid = true
var wantTemp = true
var tempDecimation = 200          // als-temp runs ~630 Hz; that is far more
                                  // than a thermometer needs

var argv = Array(CommandLine.arguments.dropFirst())
var ai = 0
while ai < argv.count {
    switch argv[ai] {
    case "--decimation":
        ai += 1
        if ai < argv.count, let v = Int(argv[ai]), v > 0 { decimation = v }
    case "--accel": wantAccel = true
    case "--gyro": wantGyro = true
    case "--als": wantAls = true
    case "--lid": wantLid = true
    case "--temp": wantTemp = true
    case "--no-temp": wantTemp = false
    case "--no-accel": wantAccel = false
    case "--no-gyro": wantGyro = false
    case "--no-als": wantAls = false
    case "--no-lid": wantLid = false
    case "-h", "--help":
        print("usage: spu_hid [--decimation N] [--no-accel] [--no-gyro] [--no-als] [--no-lid] [--no-temp]")
        exit(0)
    default:
        break
    }
    ai += 1
}

// MARK: - report decoding
//
// bmi286 imu reports are 22 bytes with the xyz payload as 3 little-endian
// int32 at offset 6, in q16 fixed point (same layout _spu.py decodes)

let IMU_REPORT_LEN = 22
let IMU_DATA_OFF = 6
let ALS_REPORT_LEN = 122
let LID_REPORT_LEN = 3
let TEMP_REPORT_LEN = 14
let TEMP_OFF = 2                  // 32-bit value at offset 2, /65536 = degC

func i32(_ d: Data, _ o: Int) -> Int32 {
    d.withUnsafeBytes { $0.loadUnaligned(fromByteOffset: o, as: Int32.self) }
}

// MARK: - sensor tasks

let manager = HIDDeviceManager()

func stream(product: String, stopOnError: Bool = false,
            onReport: @escaping (Data) -> Void) async {
    let criteria = HIDDeviceManager.DeviceMatchingCriteria(
        product: product, isBuiltIn: true)
    do {
        let devs = await manager.monitorNotifications(matchingCriteria: [criteria])
        for try await note in devs {
            guard case .deviceMatched(let ref) = note else { continue }
            guard let client = HIDDeviceClient(deviceReference: ref) else { continue }
            let reports = await client.monitorNotifications(
                reportIDsToMonitor: [HIDReportID.allReports], elementsToMonitor: [])
            for try await ev in reports {
                if case .inputReport(_, let data, _) = ev {
                    onReport(data)
                }
            }
        }
    } catch {
        if stopOnError {
            FileHandle.standardError.write(Data("spu_hid: \(product): \(error)\n".utf8))
        }
    }
}

// MARK: - dispatchers

let group = DispatchGroup()

func run(_ name: String, _ body: @escaping () async -> Void) {
    group.enter()
    Task {
        await body()
        group.leave()
    }
}

if wantAccel {
    run("accel") {
        var dec = 0
        await stream(product: "accel") { data in
            guard data.count == IMU_REPORT_LEN else { return }
            dec += 1
            if dec < decimation { return }
            dec = 0
            emit("A \(nowSec()) \(i32(data, IMU_DATA_OFF)) \(i32(data, IMU_DATA_OFF + 4)) \(i32(data, IMU_DATA_OFF + 8))")
        }
    }
}

if wantGyro {
    run("gyro") {
        var dec = 0
        await stream(product: "gyro") { data in
            guard data.count == IMU_REPORT_LEN else { return }
            dec += 1
            if dec < decimation { return }
            dec = 0
            emit("G \(nowSec()) \(i32(data, IMU_DATA_OFF)) \(i32(data, IMU_DATA_OFF + 4)) \(i32(data, IMU_DATA_OFF + 8))")
        }
    }
}

if wantAls {
    run("als") {
        await stream(product: "als") { data in
            guard data.count == ALS_REPORT_LEN else { return }
            emit("S \(nowSec()) \(data.map { String(format: "%02x", $0) }.joined())")
        }
    }
}

if wantTemp {
    run("als-temp") {
        var dec = 0
        await stream(product: "als-temp") { data in
            guard data.count == TEMP_REPORT_LEN else { return }
            dec += 1
            if dec < tempDecimation { return }
            dec = 0
            // little-endian u32 at offset 2, q16 fixed point
            let raw = UInt32(data[TEMP_OFF]) | (UInt32(data[TEMP_OFF+1]) << 8)
                    | (UInt32(data[TEMP_OFF+2]) << 16) | (UInt32(data[TEMP_OFF+3]) << 24)
            emit("T \(nowSec()) \(Double(raw) / 65536.0)")
        }
    }
}

if wantLid {
    run("las") {
        await stream(product: "las") { data in
            guard data.count >= LID_REPORT_LEN, data[0] == 1 else { return }
            let raw = UInt16(data[1]) | (UInt16(data[2]) << 8)
            emit("D \(nowSec()) \(raw & 0x1FF)")
        }
    }
}

group.wait()
