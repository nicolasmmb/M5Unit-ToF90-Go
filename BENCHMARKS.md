# Benchmarks

The package is pure Go, so every benchmark runs on the host with a fake
VL53L0X register set (`fakeSensor` in `device_test.go`). The fake counts I2C
transactions, which is the quantity that dominates on real hardware: at
400 kHz each transaction costs roughly 60–80 µs, while the sensor itself needs
20–200 ms per measurement (timing budget).

## What is measured

| Benchmark | Realistic quantity |
|---|---|
| `BenchmarkInit` | full reset + 80-entry tuning table + SPAD + calibration |
| `BenchmarkMeasure` | single-shot: start, wait, read 12 B, clear interrupt |
| `BenchmarkReadRangeContinuous` | one sample in continuous mode (the POC path) |
| `BenchmarkTimingBudget` | cached budget lookup |
| `BenchmarkSetTimingBudget` | recompute + write the timeout registers |
| `BenchmarkSignalRateLimit` | read the 9.7 fixed-point limit |
| `BenchmarkLongRange` | 200 ms budget + 0.1 MCPS signal limit |
| `TestNoAlloc` | `AllocsPerRun == 0` for every public operation |
| `TestFootprint` | `unsafe.Sizeof(Device{})` budget (96 B) |

`tx/op` is reported by every benchmark. Allocation and footprint contracts run
in CI.

## How to run

```bash
make test                  # go test -race
make bench                 # all benchmarks, -count=8
make alloc                 # allocation + footprint contracts
make bench-save NAME=before
# ... change the code ...
make bench-save NAME=after
make bench-compare FROM=before TO=after
make profile               # CPU + alloc profiles (pprof)
make escape                # compiler heap-escape report
```

`scripts/bench.sh` accepts `COUNT`, `BENCH` and `BENCHTIME`.

## Results: first working version vs optimized (Apple M2 Pro, `benchstat -count=8`)

| Metric | Before | After | Change |
|---|---:|---:|---:|
| `Init` tx/op | 160 | **138** | −13.8% |
| `TimingBudget` tx/op | 6 | **0** | −100% |
| `TimingBudget` ns/op | 50.5 | **7.8** | −84.5% |
| `Measure` tx/op | 12 | 12 | — |
| `ReadRangeContinuous` tx/op | 3 | 3 | — |
| `SetTimingBudget` tx/op | 7 | 7 | — |
| `LongRange` tx/op | 8 | 8 | — |
| allocations (all ops) | 0 | **0** | — |
| `Device` size | 72 B | **72 B** | — |

`Init` ns/op on the host rose 9.7% (963 → 1056 ns) because the fake I2C
transaction costs ~5 ns, so the batching loop is visible. On real hardware the
opposite applies: 22 fewer transactions at ~70 µs each saves about **1.5 ms**
per boot, far more than the loop overhead.

### What changed

1. **Batched tuning table**: the 80 static register writes contain 59 runs of
   consecutive registers; `writeTableLocked` sends each run as one I2C
   transaction (22 fewer transactions during `Init`).
2. **Cached timing budget**: `Init` reads the factory budget once and
   `SetTimingBudget`/`LongRange` update it, so `TimingBudget()` is a plain
   field read with **zero bus traffic** — safe to call from a display loop.

### Why the remaining numbers are already minimal

- Continuous-mode sample = read interrupt status + read 12-byte result block
  + clear interrupt = **3 transactions**; each is required by the sensor.
- Single-shot `Measure` = 7 configuration writes (stop-variable dance used by
  ST/Pololu) + start + start-bit poll + 3 read/clear = **12**.
- `Init` = one-time cost, 138 transactions ≈ 10 ms at 400 kHz.
- Every operation is allocation-free: `Device` owns its 8-byte TX and 12-byte
  RX scratch buffers, so slices passed to the `I2C` interface never escape.
- `Device` is **72 bytes** (mutex, bus interface, address, timeout, budget,
  stop variable, state and scratch buffers). `Measurement` is 4 bytes.

Host numbers are for Go on arm64. TinyGo/ESP32 executes the same fixed-buffer
code; the transaction counts and allocation counts are what matter for RAM and
bus load, and they carry over directly.
