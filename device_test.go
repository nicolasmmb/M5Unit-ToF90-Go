package tof90

import (
	"errors"
	"testing"
	"time"
	"unsafe"
)

// fakeSensor emulates the VL53L0X register set used by the driver.
type fakeSensor struct {
	regs [256]byte
	addr uint16

	tx   int
	fail bool

	model     uint8
	revision1 uint8

	readyAfter int // number of interrupt-status reads before a result is ready
	polls      int
	started    bool
	pending    bool
	cont       bool
	rangeMM    uint16
	status     uint8
	spad       uint8
}

func newFakeSensor() *fakeSensor {
	f := &fakeSensor{
		addr:       DefaultAddress,
		model:      ModelID,
		revision1:  0xAA,
		readyAfter: 0,
		rangeMM:    1234,
		spad:       0x86, // aperture, 6 reference SPADs
	}
	f.regs[regIdentificationModelID] = f.model
	f.regs[0xC1] = f.revision1
	f.regs[regIdentificationRevision] = 0x10
	f.regs[0x89] = 0x00
	f.regs[0x60] = 0x00
	f.regs[0x84] = 0x00
	f.regs[0x91] = 0x3C
	f.regs[regSystemSequenceConfig] = 0xFF
	f.regs[regPreRangeVcselPeriod] = 0x06
	f.regs[regFinalRangeVcselPeriod] = 0x04
	f.regs[regMsrcConfigTimeout] = 0x25
	f.regs[regPreRangeTimeoutHi] = 0x00
	f.regs[regPreRangeTimeoutHi+1] = 0x96
	f.regs[regFinalRangeTimeoutHi] = 0x01
	f.regs[regFinalRangeTimeoutHi+1] = 0xFE
	for i := 0; i < 6; i++ {
		f.regs[regSpadEnablesRef0+i] = 0xFF
	}
	f.setRange(f.rangeMM, f.status)
	return f
}

func (f *fakeSensor) setRange(mm uint16, status uint8) {
	f.rangeMM = mm
	f.status = status
	f.regs[regResultRangeStatus] = status << 3
	f.regs[regResultRangeStatus+10] = byte(mm >> 8)
	f.regs[regResultRangeStatus+11] = byte(mm)
}

func (f *fakeSensor) Tx(addr uint16, w, r []byte) error {
	if f.fail {
		return errors.New("i2c: no ack")
	}
	if addr != f.addr {
		return errors.New("i2c: no ack")
	}
	f.tx++
	if len(w) == 0 {
		return nil
	}
	reg := w[0]

	if len(r) > 0 {
		switch reg {
		case regResultInterruptStatus:
			if !f.pending && f.started {
				f.polls++
				if f.polls > f.readyAfter {
					f.pending = true
				}
			}
			ready := byte(0)
			if f.pending {
				ready = 0x01
			}
			for i := range r {
				r[i] = 0
			}
			r[0] = ready
			return nil
		case 0x83:
			// SPAD acquisition finishes immediately.
			for i := range r {
				r[i] = 0x01
			}
			return nil
		case 0x92:
			for i := range r {
				r[i] = f.spad
			}
			return nil
		case 0x00:
			for i := range r {
				r[i] = f.regs[reg] &^ 0x01 // start bit clears itself
			}
			return nil
		}
		for i := range r {
			r[i] = f.regs[reg+uint8(i)]
		}
		return nil
	}

	if len(w) > 2 {
		for i := 1; i < len(w); i++ {
			f.regs[reg+uint8(i)-1] = w[i]
		}
		return nil
	}

	val := w[1]
	switch reg {
	case regSoftReset:
		if val == 0 {
			f.regs[regIdentificationModelID] = 0x00
			f.started = false
			f.pending = false
			f.cont = false
		} else {
			f.regs[regIdentificationModelID] = f.model
			f.regs[0xC1] = f.revision1
			f.regs[regIdentificationRevision] = 0x10
		}
	case regI2CSlaveAddress:
		f.addr = uint16(val & 0x7F)
	case regSysRangeStart:
		if val&0x02 != 0 {
			f.cont = true
			f.started = true
			f.pending = false
			f.polls = 0
		} else if val&0x01 != 0 {
			f.started = true
			f.pending = false
			f.polls = 0
		} else {
			f.started = false
		}
		// The start bit self-clears.
		f.regs[reg] = val &^ 0x01
		return nil
	case regSystemInterruptClear:
		if val&0x01 != 0 {
			f.pending = false
			f.polls = 0
		}
	}
	f.regs[reg] = val
	return nil
}

func newTestDevice(tb testing.TB) (*Device, *fakeSensor) {
	tb.Helper()
	f := newFakeSensor()
	d := New(f)
	d.SetTimeout(50 * time.Millisecond)
	if err := d.Init(); err != nil {
		tb.Fatalf("Init: %v", err)
	}
	return d, f
}

func TestInit(t *testing.T) {
	d, f := newTestDevice(t)
	if got, err := d.ModelID(); err != nil || got != ModelID {
		t.Fatalf("ModelID = %#x, %v", got, err)
	}
	if got, err := d.RevisionID(); err != nil || got != 0x10 {
		t.Fatalf("RevisionID = %#x, %v", got, err)
	}
	if got := d.Address(); got != DefaultAddress {
		t.Fatalf("Address = %#x", got)
	}
	if f.tx < 130 || f.tx > 145 {
		t.Fatalf("Init used %d I2C transactions, want ~139", f.tx)
	}
	if f.regs[regSystemSequenceConfig] != 0xE8 {
		t.Fatalf("sequence config = %#x, want 0xE8", f.regs[regSystemSequenceConfig])
	}
	if f.regs[regGpioHvMuxActiveHigh]&0x10 != 0 {
		t.Fatal("GPIO interrupt not configured active-low")
	}
	// Aperture mode keeps SPADs 12..17 (6 SPADs) and clears the rest.
	if got := f.regs[regSpadEnablesRef0+1]; got != 0xF0 {
		t.Fatalf("SPAD map[1] = %#x, want 0xF0", got)
	}
	if got := f.regs[regSpadEnablesRef0+2]; got != 0x03 {
		t.Fatalf("SPAD map[2] = %#x, want 0x03", got)
	}

	budget, err := d.TimingBudget()
	if err != nil {
		t.Fatal(err)
	}
	if budget < 20*time.Millisecond || budget > 40*time.Millisecond {
		t.Fatalf("default budget = %v, want ~33 ms", budget)
	}
	rate, err := d.SignalRateLimit()
	if err != nil {
		t.Fatal(err)
	}
	if rate < 0.24 || rate > 0.26 {
		t.Fatalf("signal rate = %v, want 0.25", rate)
	}
}

func TestInitNotConnected(t *testing.T) {
	f := newFakeSensor()
	f.fail = true
	d := New(f)
	d.SetTimeout(5 * time.Millisecond)
	if err := d.Init(); !errors.Is(err, ErrNotConnected) {
		t.Fatalf("Init = %v, want ErrNotConnected", err)
	}
}

func TestInitBadModel(t *testing.T) {
	f := newFakeSensor()
	f.revision1 = 0xA0
	d := New(f)
	d.SetTimeout(20 * time.Millisecond)
	if err := d.Init(); !errors.Is(err, ErrBadModel) {
		t.Fatalf("Init = %v, want ErrBadModel", err)
	}
}

func TestMeasure(t *testing.T) {
	d, f := newTestDevice(t)
	f.setRange(432, 0)
	m, err := d.Measure()
	if err != nil {
		t.Fatal(err)
	}
	if !m.Valid || m.RangeMM != 432 || m.Status != RangeValid {
		t.Fatalf("Measure = %+v", m)
	}

	mm, err := d.Distance()
	if err != nil || mm != 432 {
		t.Fatalf("Distance = %d, %v", mm, err)
	}
}

func TestMeasureOutOfRange(t *testing.T) {
	d, f := newTestDevice(t)
	f.setRange(8190, RangeValid)
	m, err := d.Measure()
	if err != nil {
		t.Fatal(err)
	}
	if m.Valid {
		t.Fatalf("out-of-range reading marked valid: %+v", m)
	}
	if mm, err := d.Distance(); err != nil || mm != -1 {
		t.Fatalf("Distance = %d, %v; want -1", mm, err)
	}

	f.setRange(500, RangeSigmaFail)
	m, err = d.Measure()
	if err != nil {
		t.Fatal(err)
	}
	if m.Valid || m.Status != RangeSigmaFail {
		t.Fatalf("status failure = %+v", m)
	}
}

func TestMeasureTimeout(t *testing.T) {
	d, f := newTestDevice(t)
	d.SetTimeout(2 * time.Millisecond)
	f.readyAfter = 1 << 30 // never ready
	if _, err := d.Measure(); !errors.Is(err, ErrTimeout) {
		t.Fatalf("Measure = %v, want ErrTimeout", err)
	}
}

func TestContinuous(t *testing.T) {
	d, f := newTestDevice(t)
	f.setRange(777, 0)
	if err := d.StartContinuous(); err != nil {
		t.Fatal(err)
	}
	if !d.IsContinuous() {
		t.Fatal("IsContinuous = false after StartContinuous")
	}
	ready, err := d.Ready()
	if err != nil || !ready {
		t.Fatalf("Ready = %v, %v", ready, err)
	}
	m, err := d.ReadRange()
	if err != nil || !m.Valid || m.RangeMM != 777 {
		t.Fatalf("ReadRange = %+v, %v", m, err)
	}
	if err := d.StopContinuous(); err != nil {
		t.Fatal(err)
	}
	if d.IsContinuous() {
		t.Fatal("IsContinuous = true after StopContinuous")
	}
}

func TestTimingBudgetCached(t *testing.T) {
	d, f := newTestDevice(t)
	before := f.tx
	got, err := d.TimingBudget()
	if err != nil || got <= 0 {
		t.Fatalf("TimingBudget = %v, %v", got, err)
	}
	if f.tx != before {
		t.Fatalf("TimingBudget used %d I2C transactions", f.tx-before)
	}
}

func TestTimingBudget(t *testing.T) {
	d, _ := newTestDevice(t)
	for _, want := range []time.Duration{20 * time.Millisecond, 33 * time.Millisecond, 50 * time.Millisecond, 100 * time.Millisecond, 200 * time.Millisecond} {
		if err := d.SetTimingBudget(want); err != nil {
			t.Fatalf("SetTimingBudget(%v): %v", want, err)
		}
		got, err := d.TimingBudget()
		if err != nil {
			t.Fatal(err)
		}
		delta := got - want
		if delta < 0 {
			delta = -delta
		}
		if delta > time.Millisecond {
			t.Fatalf("budget roundtrip %v -> %v", want, got)
		}
	}
	for _, bad := range []time.Duration{19 * time.Millisecond, 201 * time.Millisecond, 0} {
		if err := d.SetTimingBudget(bad); !errors.Is(err, ErrBudget) {
			t.Fatalf("SetTimingBudget(%v) = %v, want ErrBudget", bad, err)
		}
	}
}

func TestSignalRateLimit(t *testing.T) {
	d, _ := newTestDevice(t)
	for _, want := range []float32{0.1, 0.25, 1, 511.99} {
		if err := d.SetSignalRateLimit(want); err != nil {
			t.Fatalf("SetSignalRateLimit(%v): %v", want, err)
		}
		got, err := d.SignalRateLimit()
		if err != nil {
			t.Fatal(err)
		}
		if diff := got - want; diff > 0.01 || diff < -0.01 {
			t.Fatalf("signal roundtrip %v -> %v", want, got)
		}
	}
	for _, bad := range []float32{-0.1, 512} {
		if err := d.SetSignalRateLimit(bad); !errors.Is(err, ErrSignalRate) {
			t.Fatalf("SetSignalRateLimit(%v) = %v, want ErrSignalRate", bad, err)
		}
	}
}

func TestLongRange(t *testing.T) {
	d, _ := newTestDevice(t)
	if err := d.LongRange(); err != nil {
		t.Fatal(err)
	}
	budget, err := d.TimingBudget()
	if err != nil {
		t.Fatal(err)
	}
	if delta := budget - 200*time.Millisecond; delta < -time.Millisecond || delta > time.Millisecond {
		t.Fatalf("LongRange budget = %v", budget)
	}
	rate, err := d.SignalRateLimit()
	if err != nil {
		t.Fatal(err)
	}
	if rate < 0.09 || rate > 0.11 {
		t.Fatalf("LongRange signal rate = %v", rate)
	}
}

func TestSetAddress(t *testing.T) {
	d, f := newTestDevice(t)
	if err := d.SetAddress(0x2A); err != nil {
		t.Fatal(err)
	}
	if d.Address() != 0x2A || f.addr != 0x2A {
		t.Fatalf("address = %#x / fake %#x", d.Address(), f.addr)
	}
	if _, err := d.ModelID(); err != nil {
		t.Fatalf("ModelID at new address: %v", err)
	}
	var b [1]byte
	if err := f.Tx(DefaultAddress, []byte{regIdentificationModelID}, b[:]); err == nil {
		t.Fatal("old address still responds")
	}
	for _, bad := range []uint8{0x00, 0x07, 0x78, 0xFF} {
		if err := d.SetAddress(bad); !errors.Is(err, ErrAddress) {
			t.Fatalf("SetAddress(%#x) = %v, want ErrAddress", bad, err)
		}
	}
}

func TestNoAlloc(t *testing.T) {
	f := newFakeSensor()
	d := New(f)
	if allocs := testing.AllocsPerRun(5, func() {
		if err := d.Init(); err != nil {
			t.Fatal(err)
		}
	}); allocs != 0 {
		t.Fatalf("Init allocates %v per call", allocs)
	}
	if err := d.StartContinuous(); err != nil {
		t.Fatal(err)
	}
	checks := []struct {
		name string
		fn   func()
	}{
		{"Measure", func() { _, _ = d.Measure() }},
		{"ReadRange", func() { _, _ = d.ReadRange() }},
		{"Distance", func() { _, _ = d.Distance() }},
		{"TimingBudget", func() { _, _ = d.TimingBudget() }},
		{"SetTimingBudget", func() { _ = d.SetTimingBudget(50 * time.Millisecond) }},
		{"SignalRateLimit", func() { _, _ = d.SignalRateLimit() }},
		{"SetSignalRateLimit", func() { _ = d.SetSignalRateLimit(0.25) }},
		{"LongRange", func() { _ = d.LongRange() }},
		{"SetAddress", func() {
			_ = d.SetAddress(0x2B)
			_ = d.SetAddress(0x29)
		}},
	}
	for _, c := range checks {
		if allocs := testing.AllocsPerRun(50, c.fn); allocs != 0 {
			t.Errorf("%s allocates %v per call", c.name, allocs)
		}
	}
}

func TestFootprint(t *testing.T) {
	t.Logf("Measurement=%d B  Device=%d B", unsafe.Sizeof(Measurement{}), unsafe.Sizeof(Device{}))
	if n := unsafe.Sizeof(Device{}); n > 96 {
		t.Fatalf("Device is %d B, budget 96 B", n)
	}
}
