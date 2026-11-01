// Package tof90 drives the M5Stack Unit Mini ToF-90° (SKU U196), built around
// the ST VL53L0CXV0DH (VL53L0X register set) laser ranging sensor.
//
// The driver is pure Go, uses fixed buffers and performs no heap allocation
// per operation. Register access goes through a minimal I2C interface so the
// same code runs on TinyGo targets and in host tests.
package tof90

import (
	"errors"
	"sync"
	"time"
)

// I2C is the subset of tinygo.org/x/drivers.I2C used by the sensor.
// machine.I2C satisfies it.
type I2C interface {
	// Tx writes w to addr, then reads len(r) bytes into r.
	Tx(addr uint16, w, r []byte) error
}

const (
	// DefaultAddress is the 7-bit I2C address of the VL53L0X family.
	DefaultAddress = 0x29

	// ModelID is the value of the identification register (0xC0).
	ModelID = 0xEE

	defaultTimeout = 200 * time.Millisecond
	pollInterval   = 1 * time.Millisecond
)

// Register map (VL53L0X datasheet and the official M5Stack driver).
const (
	regSysRangeStart          = 0x00
	regSystemSequenceConfig   = 0x01
	regSystemInterruptConfig  = 0x0A
	regSystemInterruptClear   = 0x0B
	regResultInterruptStatus  = 0x13
	regResultRangeStatus      = 0x14 // 12-byte block, range at +10
	regMsrcConfigControl      = 0x60
	regMsrcConfigTimeout      = 0x46
	regPreRangeVcselPeriod    = 0x50
	regFinalRangeVcselPeriod  = 0x70
	regPreRangeTimeoutHi      = 0x51
	regFinalRangeTimeoutHi    = 0x71
	regSignalRateLimitHi      = 0x44
	regSoftReset              = 0xBF
	regI2CSlaveAddress        = 0x8A
	regVhvConfigPadSclSda     = 0x89
	regGpioHvMuxActiveHigh    = 0x84
	regSpadEnablesRef0        = 0xB0
	regRefEnStartSelect       = 0xB6
	regDynamicSpadNumRequest  = 0x4E
	regDynamicSpadRefOffset   = 0x4F
	regIdentificationModelID  = 0xC0
	regIdentificationRevision = 0xC2
	regStopVariable           = 0x91
)

// Errors returned by the driver. They are sentinels: compare with errors.Is.
var (
	ErrNotConnected = errors.New("tof90: sensor not responding")
	ErrBadModel     = errors.New("tof90: unexpected model ID")
	ErrTimeout      = errors.New("tof90: timeout")
	ErrBudget       = errors.New("tof90: timing budget must be 20-200 ms")
	ErrSignalRate   = errors.New("tof90: signal rate limit must be 0-511.99 MCPS")
	ErrAddress      = errors.New("tof90: address must be 0x08-0x77")
	ErrNoBudget     = errors.New("tof90: timing budget too small for the sequence")
)

// StatusValid reports whether a VL53L0X range status still carries a usable
// distance. Several codes are "valid" variants:
//
//	0  range valid
//	3  range valid, minimum range clipped
//	6  range valid, wrap check not performed
//	11 range valid, merged pulse
//
// The official M5Stack driver ignores the status and only rejects readings
// above 4 m; Measurement.Valid follows that behaviour.
func StatusValid(status uint8) bool {
	switch status {
	case 0, 3, 6, 11:
		return true
	}
	return false
}

// Measurement is one range reading.
type Measurement struct {
	// RangeMM is the measured distance in millimetres.
	RangeMM int16
	// Status is the raw sensor range status (bits 3-6 of RESULT_RANGE_STATUS).
	Status uint8
	// Valid is true when the range is usable (1-4000 mm), matching the
	// official M5Stack driver. Check StatusValid for the status nuance.
	Valid bool
}

// Device is a VL53L0X-compatible ranging sensor. It is safe for concurrent
// use: every public method serializes on an internal mutex.
type Device struct {
	mu sync.Mutex

	bus     I2C
	addr    uint8
	timeout time.Duration

	budget     time.Duration
	stopVar    uint8
	continuous bool

	// Scratch buffers owned by the device: reusing them keeps every
	// transaction allocation-free even though Tx takes slices.
	tx [8]byte
	rx [12]byte
}

// New returns a Device on bus with the default address and timeout.
func New(bus I2C) *Device {
	return &Device{
		bus:     bus,
		addr:    DefaultAddress,
		timeout: defaultTimeout,
		budget:  33 * time.Millisecond,
	}
}

// SetTimeout sets the maximum time spent in each polling loop. Call it before
// Init. A zero or negative value restores the default (200 ms).
func (d *Device) SetTimeout(timeout time.Duration) {
	d.mu.Lock()
	if timeout <= 0 {
		timeout = defaultTimeout
	}
	d.timeout = timeout
	d.mu.Unlock()
}

// Address returns the current 7-bit I2C address.
func (d *Device) Address() uint8 {
	d.mu.Lock()
	a := d.addr
	d.mu.Unlock()
	return a
}

// ModelID reads the identification register.
func (d *Device) ModelID() (uint8, error) {
	d.mu.Lock()
	defer d.mu.Unlock()
	return d.readU8(regIdentificationModelID)
}

// RevisionID reads the revision register.
func (d *Device) RevisionID() (uint8, error) {
	d.mu.Lock()
	defer d.mu.Unlock()
	return d.readU8(regIdentificationRevision)
}

// SetAddress changes the I2C address of the sensor (0x08-0x77). The new
// address stays in effect until the sensor is powered off or reset.
func (d *Device) SetAddress(addr uint8) error {
	if addr < 0x08 || addr > 0x77 {
		return ErrAddress
	}
	d.mu.Lock()
	defer d.mu.Unlock()
	if addr == d.addr {
		return nil
	}
	if err := d.write(regI2CSlaveAddress, addr&0x7F); err != nil {
		return err
	}
	d.addr = addr
	return nil
}

// --- register access ---

func (d *Device) write(reg, val uint8) error {
	d.tx[0] = reg
	d.tx[1] = val
	return d.bus.Tx(uint16(d.addr), d.tx[:2], nil)
}

func (d *Device) writeN(reg uint8, vals []byte) error {
	d.tx[0] = reg
	copy(d.tx[1:], vals)
	return d.bus.Tx(uint16(d.addr), d.tx[:1+len(vals)], nil)
}

func (d *Device) read(reg uint8, buf []byte) error {
	d.tx[0] = reg
	return d.bus.Tx(uint16(d.addr), d.tx[:1], buf)
}

func (d *Device) readU8(reg uint8) (uint8, error) {
	d.tx[0] = reg
	if err := d.bus.Tx(uint16(d.addr), d.tx[:1], d.rx[:1]); err != nil {
		return 0, err
	}
	return d.rx[0], nil
}

func (d *Device) readU16(reg uint8) (uint16, error) {
	d.tx[0] = reg
	if err := d.bus.Tx(uint16(d.addr), d.tx[:1], d.rx[:2]); err != nil {
		return 0, err
	}
	return uint16(d.rx[0])<<8 | uint16(d.rx[1]), nil
}

func (d *Device) writeU16(reg uint8, val uint16) error {
	d.tx[0] = reg
	d.tx[1] = uint8(val >> 8)
	d.tx[2] = uint8(val)
	return d.bus.Tx(uint16(d.addr), d.tx[:3], nil)
}

// wait polls cond until it returns true or the timeout elapses.
func (d *Device) wait(cond func() (bool, error)) error {
	deadline := time.Now().Add(d.timeout)
	for {
		ok, err := cond()
		if err != nil {
			return err
		}
		if ok {
			return nil
		}
		if !time.Now().Before(deadline) {
			return ErrTimeout
		}
		time.Sleep(pollInterval)
	}
}

// waitStart polls the SYSRANGE_START start bit until the measurement starts.
func (d *Device) waitStart() error {
	return d.wait(func() (bool, error) {
		v, err := d.readU8(regSysRangeStart)
		return v&0x01 == 0, err
	})
}

// waitReady waits for a range result. The deadline includes the timing budget.
func (d *Device) waitReady() error {
	timeout := d.timeout + d.budget
	deadline := time.Now().Add(timeout)
	for {
		v, err := d.readU8(regResultInterruptStatus)
		if err != nil {
			return err
		}
		if v&0x07 != 0 {
			return nil
		}
		if !time.Now().Before(deadline) {
			return ErrTimeout
		}
		time.Sleep(pollInterval)
	}
}
