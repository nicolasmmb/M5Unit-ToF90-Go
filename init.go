package tof90

import "context"

// Init resets the sensor, validates its identification registers and loads the
// ST/M5Stack tuning sequence (SPAD map, VHV/phase calibration, timing budget).
func (d *Device) Init() error {
	return d.InitContext(context.Background())
}

// InitContext is Init with cancellation. The context is checked between
// polling loops and I2C transactions.
func (d *Device) InitContext(ctx context.Context) error {
	ctx = ctxOrBackground(ctx)
	if err := d.lockContext(ctx); err != nil {
		return err
	}
	defer d.unlock()
	return d.initLocked(ctx)
}

func (d *Device) initLocked(ctx context.Context) error {
	if err := d.resetLocked(ctx); err != nil {
		return err
	}
	if err := ctxErr(ctx); err != nil {
		return err
	}

	model, err := d.readU8(regIdentificationModelID)
	if err != nil {
		return err
	}
	revMajor, err := d.readU8(0xC1)
	if err != nil {
		return err
	}
	revMinor, err := d.readU8(regIdentificationRevision)
	if err != nil {
		return err
	}
	if model != ModelID || revMajor != 0xAA || revMinor != 0x10 {
		return ErrBadModel
	}

	// Switch the I/O pads to 2V8 mode (the module runs at 3V3).
	extsup, err := d.readU8(regVhvConfigPadSclSda)
	if err != nil {
		return err
	}
	if err := d.write(regVhvConfigPadSclSda, extsup|0x01); err != nil {
		return err
	}

	// Standard I2C mode, and save the stop variable used by continuous mode.
	if err := d.write(0x88, 0x00); err != nil {
		return err
	}
	if err := d.write(0x80, 0x01); err != nil {
		return err
	}
	if err := d.write(0xFF, 0x01); err != nil {
		return err
	}
	if err := d.write(0x00, 0x00); err != nil {
		return err
	}
	stopVar, err := d.readU8(regStopVariable)
	if err != nil {
		return err
	}
	d.stopVar = stopVar
	if err := d.write(0x00, 0x01); err != nil {
		return err
	}
	if err := d.write(0xFF, 0x00); err != nil {
		return err
	}
	if err := d.write(0x80, 0x00); err != nil {
		return err
	}

	// Disable the SIGNAL_RATE_MSRC/PRE_RANGE limit checks.
	msrc, err := d.readU8(regMsrcConfigControl)
	if err != nil {
		return err
	}
	if err := d.write(regMsrcConfigControl, msrc|0x12); err != nil {
		return err
	}

	// Default signal rate limit: 0.25 MCPS.
	if err := d.setSignalRateLimitLocked(0.25); err != nil {
		return err
	}
	if err := d.write(regSystemSequenceConfig, 0xFF); err != nil {
		return err
	}

	spadCount, aperture, err := d.spadInfoLocked(ctx)
	if err != nil {
		return err
	}

	// Read the reference SPAD map, keep the first spadCount (aperture) SPADs
	// enabled and write it back.
	if err := d.read(regSpadEnablesRef0, d.rx[:6]); err != nil {
		return err
	}
	first := uint8(0)
	if aperture {
		first = 12
	}
	var enabled uint8
	for i := uint8(0); i < 48; i++ {
		if i < first || enabled == spadCount {
			d.rx[i/8] &^= 1 << (i % 8)
		} else if d.rx[i/8]>>(i%8)&0x01 != 0 {
			enabled++
		}
	}
	if err := d.writeN(regSpadEnablesRef0, d.rx[:6]); err != nil {
		return err
	}

	if err := d.write(0xFF, 0x01); err != nil {
		return err
	}
	if err := d.write(regDynamicSpadRefOffset, 0x00); err != nil {
		return err
	}
	if err := d.write(regDynamicSpadNumRequest, 0x2C); err != nil {
		return err
	}
	if err := d.write(0xFF, 0x00); err != nil {
		return err
	}
	if err := d.write(regRefEnStartSelect, 0xB4); err != nil {
		return err
	}

	if err := d.writeTableLocked(ctx, tuningTable[:]); err != nil {
		return err
	}

	// GPIO interrupt: active low, cleared.
	if err := d.write(regSystemInterruptConfig, 0x04); err != nil {
		return err
	}
	mux, err := d.readU8(regGpioHvMuxActiveHigh)
	if err != nil {
		return err
	}
	if err := d.write(regGpioHvMuxActiveHigh, mux&^0x10); err != nil {
		return err
	}
	if err := d.write(regSystemInterruptClear, 0x01); err != nil {
		return err
	}

	// Keep the factory timing budget, then run both reference calibrations.
	if err := ctxErr(ctx); err != nil {
		return err
	}
	budget, err := d.getBudgetLocked()
	if err != nil {
		return err
	}
	if err := d.write(regSystemSequenceConfig, 0xE8); err != nil {
		return err
	}
	if err := d.setBudgetLocked(budget); err != nil {
		return err
	}
	if err := d.write(regSystemSequenceConfig, 0x01); err != nil {
		return err
	}
	if err := d.refCalLocked(ctx, 0x40); err != nil {
		return err
	}
	if err := d.write(regSystemSequenceConfig, 0x02); err != nil {
		return err
	}
	if err := d.refCalLocked(ctx, 0x00); err != nil {
		return err
	}
	if err := d.write(regSystemSequenceConfig, 0xE8); err != nil {
		return err
	}

	d.continuous = false
	return nil
}

// resetLocked performs the VL53L0X soft reset and waits for the model ID.
func (d *Device) resetLocked(ctx context.Context) error {
	// Wait for the sensor to answer after power-up.
	if err := d.wait(ctx, func() (bool, error) {
		_, err := d.readU8(regIdentificationModelID)
		return err == nil, nil
	}); err != nil {
		if err == ErrTimeout {
			return ErrNotConnected
		}
		return err
	}

	if err := d.write(regSoftReset, 0x00); err != nil {
		return err
	}
	if err := d.wait(ctx, func() (bool, error) {
		v, err := d.readU8(regIdentificationModelID)
		return err == nil && v != ModelID, nil
	}); err != nil {
		return err
	}
	if err := d.write(regSoftReset, 0x01); err != nil {
		return err
	}
	if err := d.wait(ctx, func() (bool, error) {
		v, err := d.readU8(regIdentificationModelID)
		return err == nil && v == ModelID, nil
	}); err != nil {
		return err
	}
	return nil
}

// spadInfoLocked reads the reference SPAD count and type from the sensor.
func (d *Device) spadInfoLocked(ctx context.Context) (count uint8, aperture bool, err error) {
	for _, rv := range [...]regVal{
		{0x80, 0x01}, {0xFF, 0x01}, {0x00, 0x00},
	} {
		if err := d.write(rv.reg, rv.val); err != nil {
			return 0, false, err
		}
	}
	if err := d.write(0xFF, 0x06); err != nil {
		return 0, false, err
	}
	tmp, err := d.readU8(0x83)
	if err != nil {
		return 0, false, err
	}
	if err := d.write(0x83, tmp|0x04); err != nil {
		return 0, false, err
	}
	for _, rv := range [...]regVal{
		{0xFF, 0x07}, {0x81, 0x01}, {0x80, 0x01},
		{0x94, 0x6B}, {0x83, 0x00},
	} {
		if err := d.write(rv.reg, rv.val); err != nil {
			return 0, false, err
		}
	}
	if err := d.wait(ctx, func() (bool, error) {
		v, err := d.readU8(0x83)
		return v != 0x00, err
	}); err != nil {
		return 0, false, err
	}
	if err := d.write(0x83, 0x01); err != nil {
		return 0, false, err
	}
	tmp, err = d.readU8(0x92)
	if err != nil {
		return 0, false, err
	}
	count = tmp & 0x7F
	aperture = (tmp>>7)&0x01 == 1

	if err := d.write(0x81, 0x00); err != nil {
		return 0, false, err
	}
	if err := d.write(0xFF, 0x06); err != nil {
		return 0, false, err
	}
	tmp, err = d.readU8(0x83)
	if err != nil {
		return 0, false, err
	}
	for _, rv := range [...]regVal{
		{0x83, tmp &^ 0x04}, {0xFF, 0x01}, {0x00, 0x01},
		{0xFF, 0x00}, {0x80, 0x00},
	} {
		if err := d.write(rv.reg, rv.val); err != nil {
			return 0, false, err
		}
	}
	return count, aperture, nil
}

// refCalLocked runs one VHV or phase reference calibration.
func (d *Device) refCalLocked(ctx context.Context, vhv uint8) error {
	if err := d.write(regSysRangeStart, 0x01|vhv); err != nil {
		return err
	}
	if err := d.wait(ctx, func() (bool, error) {
		v, err := d.readU8(regResultInterruptStatus)
		return v&0x07 != 0, err
	}); err != nil {
		return err
	}
	if err := d.write(regSystemInterruptClear, 0x01); err != nil {
		return err
	}
	return d.write(regSysRangeStart, 0x00)
}

type regVal struct {
	reg, val uint8
}

// writeTableLocked writes table entries, batching consecutive registers into a
// single I2C transaction (the tuning table has 80 writes but only 59 runs).
func (d *Device) writeTableLocked(ctx context.Context, table []regVal) error {
	for i := 0; i < len(table); {
		if err := ctxErr(ctx); err != nil {
			return err
		}
		n := 1
		for i+n < len(table) && table[i+n].reg == table[i].reg+uint8(n) && n < len(d.tx)-1 {
			n++
		}
		d.tx[0] = table[i].reg
		for j := 0; j < n; j++ {
			d.tx[1+j] = table[i+j].val
		}
		if err := d.bus.Tx(uint16(d.addr), d.tx[:1+n], nil); err != nil {
			return err
		}
		i += n
	}
	return nil
}

// tuningTable is the ST/M5Stack init sequence, identical to the official
// UIFlow driver (which follows ST's API and Pololu's VL53L0X port).
var tuningTable = [...]regVal{
	{0xFF, 0x01}, {0x00, 0x00}, {0xFF, 0x00}, {0x09, 0x00},
	{0x10, 0x00}, {0x11, 0x00}, {0x24, 0x01}, {0x25, 0xFF},
	{0x75, 0x00}, {0xFF, 0x01}, {0x4E, 0x2C}, {0x48, 0x00},
	{0x30, 0x20}, {0xFF, 0x00}, {0x30, 0x09}, {0x54, 0x00},
	{0x31, 0x04}, {0x32, 0x03}, {0x40, 0x83}, {0x46, 0x25},
	{0x60, 0x00}, {0x27, 0x00}, {0x50, 0x06}, {0x51, 0x00},
	{0x52, 0x96}, {0x56, 0x08}, {0x57, 0x30}, {0x61, 0x00},
	{0x62, 0x00}, {0x64, 0x00}, {0x65, 0x00}, {0x66, 0xA0},
	{0xFF, 0x01}, {0x22, 0x32}, {0x47, 0x14}, {0x49, 0xFF},
	{0x4A, 0x00}, {0xFF, 0x00}, {0x7A, 0x0A}, {0x7B, 0x00},
	{0x78, 0x21}, {0xFF, 0x01}, {0x23, 0x34}, {0x42, 0x00},
	{0x44, 0xFF}, {0x45, 0x26}, {0x46, 0x05}, {0x40, 0x40},
	{0x0E, 0x06}, {0x20, 0x1A}, {0x43, 0x40}, {0xFF, 0x00},
	{0x34, 0x03}, {0x35, 0x44}, {0xFF, 0x01}, {0x31, 0x04},
	{0x4B, 0x09}, {0x4C, 0x05}, {0x4D, 0x04}, {0xFF, 0x00},
	{0x44, 0x00}, {0x45, 0x20}, {0x47, 0x08}, {0x48, 0x28},
	{0x67, 0x00}, {0x70, 0x04}, {0x71, 0x01}, {0x72, 0xFE},
	{0x76, 0x00}, {0x77, 0x00}, {0xFF, 0x01}, {0x0D, 0x01},
	{0xFF, 0x00}, {0x80, 0x01}, {0x01, 0xF8}, {0xFF, 0x01},
	{0x8E, 0x01}, {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00},
}
