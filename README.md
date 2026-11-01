# M5Unit-ToF90-Go

[![test](https://github.com/nicolasmmb/M5Unit-ToF90-Go/actions/workflows/test.yml/badge.svg)](https://github.com/nicolasmmb/M5Unit-ToF90-Go/actions/workflows/test.yml)

Pure-Go driver for the **M5Stack Unit Mini ToF-90°** (SKU U196), built around
the **ST VL53L0CXV0DH** laser ranging sensor (VL53L0X register set). No C, C++,
CGo or Arduino.

## Features

- Full init sequence from the official M5Stack/ST/Pololu driver: soft reset,
  identification, SPAD map, tuning table, VHV and phase calibration.
- Single-shot and continuous ranging, with range status and validity.
- Measurement timing budget (20–200 ms) and signal rate limit (0–511.99 MCPS).
- **`LongRange()`**: 200 ms budget + 0.1 MCPS for the 200 cm mode.
- I2C address change (`0x29` → any free address).
- **Zero heap allocations per operation**: fixed scratch buffers, cached
  timing budget, no `fmt`.
- Every polling operation has a **Context variant** for cancellation (and the
  device lock is cancellable too).
- Timeouts on every polling loop and sentinel errors (`errors.Is`).
- Host tests with a full register-level sensor emulator, allocation contracts,
  RAM footprint budget and I2C transaction benchmarks.

## Hardware

| Parameter | Value |
|---|---|
| Unit | M5Stack Unit Mini ToF-90° (U196) |
| Sensor | VL53L0CXV0DH (VL53L0X registers) |
| Interface | I2C, **0x29**, up to 400 kHz |
| Range / resolution | 3–200 cm / 1 mm (200 cm in long-range, dark) |
| Field of view | 25°, emitter rotated 90° |
| Wiring (M5StickC) | Grove I2C1: **SDA=G32, SCL=G33** |

The Grove port is shared with the other units (Encoder, GPS, Fingerprint2):
connect only one at a time.

## Install

```bash
go get github.com/nicolasmmb/M5Unit-ToF90-Go@v0.1.0
```

## Quick start

```go
i2c := machine.I2C1
_ = i2c.Configure(machine.I2CConfig{
    SDA:       machine.GPIO32,
    SCL:       machine.GPIO33,
    Frequency: 400 * machine.KHz,
})

d := tof90.New(i2c)
d.SetTimeout(100 * time.Millisecond)
if err := d.Init(); err != nil {
    println("tof:", err.Error())
}

// Continuous mode: ~3 I2C transactions per sample.
_ = d.StartContinuous()
for {
    m, err := d.ReadRange()
    if err == nil {
        println(m.RangeMM, "mm", m.Valid)
    }
}
```

Single shot and long range:

```go
mm, err := d.Distance()          // -1 when out of range
_ = d.SetTimingBudget(50 * time.Millisecond)
_ = d.SetSignalRateLimit(0.1)
// Changing the budget while streaming requires a restart: the sensor latches
// the timing configuration when continuous mode starts.
_ = d.StopContinuous()
_ = d.StartContinuous()

_ = d.LongRange()                // 200 ms + 0.1 MCPS, up to 2 m
```

## API

| Function | Description |
|---|---|
| `New(bus I2C) *Device` | Driver on any bus implementing `Tx(addr, w, r)` |
| `NewAt(bus, address)` | Construct for a sensor already at another address |
| `Init()` / `InitContext(ctx)` | Reset, identify and load the tuning/calibration sequence |
| `StartMeasurement() error` | Start a single shot without waiting (do_range_measurement) |
| `Measure()` / `MeasureContext(ctx)` | One single-shot reading |
| `DistanceCM() (float32, error)` | Centimetres (official `get_distance`), `-0.1` invalid |
| `Distance() (int16, error)` | Millimetres, `-1` when invalid |
| `StartContinuous()` / `StopContinuous()` | Continuous mode (both have Context variants) |
| `StartMeasurement()` / `StartMeasurementContext(ctx)` | Start a single shot without waiting |
| `Ready() (bool, error)` | Result waiting (continuous mode) |
| `ReadRange()` / `ReadRangeContext(ctx)` | Read the pending result |
| `TimingBudget()` / `SetTimingBudget(d)` | 20–200 ms, cached / applied |
| `SignalRateLimit()` / `SetSignalRateLimit(v)` | MCPS |
| `LongRange() error` | 200 ms budget + 0.1 MCPS |
| `SetAddress(a) error` | Move the sensor to `0x08–0x77` |
| `ModelID()` / `RevisionID()` | Identification registers |
| `Reset()` / `ResetContext(ctx)` | Soft reset (call `Init` again) |

`Measurement`: `RangeMM int16`, `Status uint8` (raw VL53L0X range status),
`Valid bool` (1-4000 mm, like the official M5Stack driver). Use
`StatusValid(status)` to check the ST "range valid" codes (0, 3, 6 and 11 —
11 is the merged-pulse variant reported by this unit).

## Timeouts and cancellation

Every polling loop has a deadline, so no call can wait forever on the sensor:

| Operation | Worst case (defaults) |
|---|---|
| `ReadRange` / `Measure` / `Distance` | `timeout + timing budget` = 200 ms + 200 ms |
| `StartContinuous` / `StartMeasurement` | `timeout` = 200 ms (start bit) |
| `Init` / `Reset` | one `timeout` per poll stage + 138 transactions |
| `SetTimingBudget` / `SetSignalRateLimit` / `SetAddress` | a few I2C transactions (TinyGo bounds each at 40 ms) |

The operations above also accept a `context.Context` (`InitContext`,
`MeasureContext`, `DistanceContext`, `ReadRangeContext`,
`StartMeasurementContext`, `StartContinuousContext`, `StopContinuousContext`,
`ResetContext`). The context is checked before every poll and between groups
of I2C transactions, and it also cancels while waiting for the device lock:

```go
ctx, cancel := context.WithTimeout(context.Background(), time.Second)
defer cancel()
m, err := d.ReadRangeContext(ctx) // returns ctx.Err() when cancelled
```

The blocking `I2C.Tx` itself cannot be interrupted; TinyGo's implementation
bounds it at 40 ms.

## Performance

Measured on the host against a register-level fake, counting I2C transactions
(the real cost at 400 kHz). See [BENCHMARKS.md](BENCHMARKS.md).

| Operation | I2C transactions | Allocations |
|---|---:|---:|
| `Init` | 138 (one-time) | 0 |
| `ReadRange` (continuous) | 3 | 0 |
| `Measure` (single shot) | 12 | 0 |
| `TimingBudget` | **0** (cached) | 0 |
| `SetTimingBudget` | 7 | 0 |
| `LongRange` | 8 | 0 |

`Device` is **72 bytes**; `Measurement` is **4 bytes**. Continuous sampling is
dominated by the sensor's timing budget (20–200 ms), not by the MCU.

```bash
make bench    # benchmarks with tx/op and allocs
make alloc    # 0-allocation and footprint contracts
```

## References

- [M5Stack Unit Mini ToF-90°](https://docs.m5stack.com/en/unit/Unit_Mini_ToF-90)
- [M5Stack UIFlow driver](https://github.com/m5stack/uiflow-micropython/blob/master/m5stack/libs/driver/vl53l0x.py)
- ST VL53L0X datasheet and API, Pololu VL53L0X Arduino library

## License

MIT — see [LICENSE](LICENSE).
