package tof90

import "time"

// TimingBudget returns the measurement timing budget. The value is read from
// the sensor during Init and updated by SetTimingBudget/LongRange, so this call
// does not touch the I2C bus.
func (d *Device) TimingBudget() (time.Duration, error) {
	d.lock()
	budget := d.budget
	d.unlock()
	return budget, nil
}

// SetTimingBudget sets the measurement timing budget (20-200 ms). A longer
// budget averages more light and extends the usable range.
//
// The sensor latches its timing configuration when continuous mode starts, so
// stop and restart continuous mode for a new budget to take effect.
func (d *Device) SetTimingBudget(budget time.Duration) error {
	d.lock()
	defer d.unlock()
	return d.setBudgetLocked(budget)
}

// SignalRateLimit returns the final range signal rate limit in MCPS.
func (d *Device) SignalRateLimit() (float32, error) {
	d.lock()
	defer d.unlock()
	return d.getSignalRateLimitLocked()
}

// SetSignalRateLimit sets the final range signal rate limit in MCPS
// (0-511.99). Lower values accept weaker returns and extend the range, at the
// cost of more noise.
func (d *Device) SetSignalRateLimit(mcps float32) error {
	d.lock()
	defer d.unlock()
	return d.setSignalRateLimitLocked(mcps)
}

// LongRange configures the 200 cm mode advertised by M5Stack: a 200 ms timing
// budget and a 0.1 MCPS signal rate limit. It needs a dark environment with
// little infrared interference.
func (d *Device) LongRange() error {
	d.lock()
	defer d.unlock()
	if err := d.setBudgetLocked(200 * time.Millisecond); err != nil {
		return err
	}
	return d.setSignalRateLimitLocked(0.1)
}

// --- timing budget ---

const (
	minBudget = 20 * time.Millisecond
	maxBudget = 200 * time.Millisecond
)

type sequenceStep struct {
	tcc, dss, msrc, preRange, finalRange bool
}

type stepTimeouts struct {
	msrcDssTccUs         int32
	preRangeUs           int32
	finalRangeUs         int32
	finalRangeVcselPclks int32
	preRangeMclks        int32
}

func (d *Device) sequenceEnablesLocked() (sequenceStep, error) {
	cfg, err := d.readU8(regSystemSequenceConfig)
	if err != nil {
		return sequenceStep{}, err
	}
	return sequenceStep{
		tcc:        cfg>>4&1 != 0,
		dss:        cfg>>3&1 != 0,
		msrc:       cfg>>2&1 != 0,
		preRange:   cfg>>6&1 != 0,
		finalRange: cfg>>7&1 != 0,
	}, nil
}

func (d *Device) vcselPeriodLocked(final bool) (int32, error) {
	reg := uint8(regPreRangeVcselPeriod)
	if final {
		reg = regFinalRangeVcselPeriod
	}
	v, err := d.readU8(reg)
	if err != nil {
		return 0, err
	}
	return int32((v+1)&0xFF) << 1, nil
}

func (d *Device) stepTimeoutsLocked(preRange bool) (stepTimeouts, error) {
	prePclks, err := d.vcselPeriodLocked(false)
	if err != nil {
		return stepTimeouts{}, err
	}
	msrcMclks, err := d.readU8(regMsrcConfigTimeout)
	if err != nil {
		return stepTimeouts{}, err
	}
	out := stepTimeouts{
		msrcDssTccUs: mclksToUs(int32(msrcMclks)+1, prePclks),
	}
	preTimeout, err := d.readU16(regPreRangeTimeoutHi)
	if err != nil {
		return stepTimeouts{}, err
	}
	out.preRangeMclks = decodeTimeout(preTimeout)
	out.preRangeUs = mclksToUs(out.preRangeMclks, prePclks)

	out.finalRangeVcselPclks, err = d.vcselPeriodLocked(true)
	if err != nil {
		return stepTimeouts{}, err
	}
	finalTimeout, err := d.readU16(regFinalRangeTimeoutHi)
	if err != nil {
		return stepTimeouts{}, err
	}
	finalMclks := decodeTimeout(finalTimeout)
	if preRange {
		finalMclks -= out.preRangeMclks
	}
	out.finalRangeUs = mclksToUs(finalMclks, out.finalRangeVcselPclks)
	return out, nil
}

func (d *Device) getBudgetLocked() (time.Duration, error) {
	budget := int32(1910 + 960) // start + end overhead
	steps, err := d.sequenceEnablesLocked()
	if err != nil {
		return 0, err
	}
	t, err := d.stepTimeoutsLocked(steps.preRange)
	if err != nil {
		return 0, err
	}
	if steps.tcc {
		budget += t.msrcDssTccUs + 590
	}
	if steps.dss {
		budget += 2 * (t.msrcDssTccUs + 690)
	} else if steps.msrc {
		budget += t.msrcDssTccUs + 660
	}
	if steps.preRange {
		budget += t.preRangeUs + 660
	}
	if steps.finalRange {
		budget += t.finalRangeUs + 550
	}
	d.budget = time.Duration(budget) * time.Microsecond
	return d.budget, nil
}

func (d *Device) setBudgetLocked(budget time.Duration) error {
	if budget < minBudget || budget > maxBudget {
		return ErrBudget
	}
	used := int32(1320 + 960) // start + end overhead
	steps, err := d.sequenceEnablesLocked()
	if err != nil {
		return err
	}
	t, err := d.stepTimeoutsLocked(steps.preRange)
	if err != nil {
		return err
	}
	if steps.tcc {
		used += t.msrcDssTccUs + 590
	}
	if steps.dss {
		used += 2 * (t.msrcDssTccUs + 690)
	} else if steps.msrc {
		used += t.msrcDssTccUs + 660
	}
	if steps.preRange {
		used += t.preRangeUs + 660
	}
	if steps.finalRange {
		used += 550
	}
	budgetUs := int32(budget / time.Microsecond)
	if used > budgetUs {
		return ErrNoBudget
	}
	finalUs := budgetUs - used
	finalMclks := usToMclks(finalUs, t.finalRangeVcselPclks)
	if steps.preRange {
		finalMclks += t.preRangeMclks
	}
	if err := d.writeU16(regFinalRangeTimeoutHi, encodeTimeout(finalMclks)); err != nil {
		return err
	}
	d.budget = budget
	return nil
}

// --- signal rate limit (9.7 fixed point) ---

func (d *Device) getSignalRateLimitLocked() (float32, error) {
	raw, err := d.readU16(regSignalRateLimitHi)
	if err != nil {
		return 0, err
	}
	return float32(raw) / 128.0, nil
}

func (d *Device) setSignalRateLimitLocked(mcps float32) error {
	if mcps < 0 || mcps > 511.99 {
		return ErrSignalRate
	}
	raw := uint16(mcps*128 + 0.5)
	return d.writeU16(regSignalRateLimitHi, raw)
}

// --- timeout encoding (VL53L0X datasheet, same math as the official driver) ---

func decodeTimeout(v uint16) int32 {
	ls := int32(v & 0xFF)
	ms := int32(v >> 8)
	if ms > 24 { // keep the shift representable; real values are small
		ms = 24
	}
	return ls<<ms + 1
}

func encodeTimeout(mclks int32) uint16 {
	if mclks <= 0 {
		return 0
	}
	ls := mclks - 1
	var ms int32
	for ls > 255 {
		ls >>= 1
		ms++
	}
	return uint16(ms<<8) | uint16(ls&0xFF)
}

func macroPeriodNs(pclks int32) int32 {
	return (2304*pclks*1655 + 500) / 1000
}

func mclksToUs(mclks, pclks int32) int32 {
	return (mclks*macroPeriodNs(pclks) + 500) / 1000
}

func usToMclks(us, pclks int32) int32 {
	macro := macroPeriodNs(pclks)
	return (us*1000 + macro/2) / macro
}
