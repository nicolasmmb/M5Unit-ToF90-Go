package tof90

import "context"

// StartContinuous puts the sensor in continuous ranging mode: it measures as
// fast as the timing budget allows and asserts the interrupt when a result is
// ready. Use Ready and ReadRange to consume results.
func (d *Device) StartContinuous() error {
	return d.StartContinuousContext(context.Background())
}

// StartContinuousContext is StartContinuous with cancellation.
func (d *Device) StartContinuousContext(ctx context.Context) error {
	ctx = ctxOrBackground(ctx)
	if err := d.lockContext(ctx); err != nil {
		return err
	}
	defer d.unlock()
	if d.continuous {
		return nil
	}
	if err := d.continuousConfigLocked(ctx, d.stopVar); err != nil {
		return err
	}
	if err := d.write(regSysRangeStart, 0x02); err != nil {
		return err
	}
	if err := d.waitStart(ctx); err != nil {
		return err
	}
	d.continuous = true
	return nil
}

// StopContinuous returns the sensor to single-shot mode and starts one
// measurement, matching the official driver.
func (d *Device) StopContinuous() error {
	return d.StopContinuousContext(context.Background())
}

// StopContinuousContext is StopContinuous with cancellation.
func (d *Device) StopContinuousContext(ctx context.Context) error {
	ctx = ctxOrBackground(ctx)
	if err := d.lockContext(ctx); err != nil {
		return err
	}
	defer d.unlock()
	if err := d.write(regSysRangeStart, 0x01); err != nil {
		return err
	}
	if err := d.continuousConfigLocked(ctx, 0x00); err != nil {
		return err
	}
	d.continuous = false
	return d.startSingleLocked(ctx)
}

// IsContinuous reports whether continuous mode is active.
func (d *Device) IsContinuous() bool {
	d.lock()
	c := d.continuous
	d.unlock()
	return c
}

// Ready reports whether a range result is waiting to be read.
func (d *Device) Ready() (bool, error) {
	d.lock()
	defer d.unlock()
	v, err := d.readU8(regResultInterruptStatus)
	if err != nil {
		return false, err
	}
	return v&0x07 != 0, nil
}

// Measure performs one ranging measurement, starting a single measurement when
// the sensor is not in continuous mode.
func (d *Device) Measure() (Measurement, error) {
	return d.MeasureContext(context.Background())
}

// MeasureContext is Measure with cancellation.
func (d *Device) MeasureContext(ctx context.Context) (Measurement, error) {
	ctx = ctxOrBackground(ctx)
	if err := d.lockContext(ctx); err != nil {
		return Measurement{}, err
	}
	defer d.unlock()
	if !d.continuous {
		if err := d.startSingleLocked(ctx); err != nil {
			return Measurement{}, err
		}
	}
	return d.readRangeLocked(ctx)
}

// StartMeasurement starts one single-shot measurement and returns as soon as
// the sensor accepts it; use Ready and ReadRange to fetch the result (the
// official driver's do_range_measurement).
func (d *Device) StartMeasurement() error {
	return d.StartMeasurementContext(context.Background())
}

// StartMeasurementContext is StartMeasurement with cancellation.
func (d *Device) StartMeasurementContext(ctx context.Context) error {
	ctx = ctxOrBackground(ctx)
	if err := d.lockContext(ctx); err != nil {
		return err
	}
	defer d.unlock()
	return d.startSingleLocked(ctx)
}

// ReadRange reads the pending result without starting a new measurement. Use
// it in continuous mode after Ready reports true.
func (d *Device) ReadRange() (Measurement, error) {
	return d.ReadRangeContext(context.Background())
}

// ReadRangeContext is ReadRange with cancellation.
func (d *Device) ReadRangeContext(ctx context.Context) (Measurement, error) {
	ctx = ctxOrBackground(ctx)
	if err := d.lockContext(ctx); err != nil {
		return Measurement{}, err
	}
	defer d.unlock()
	return d.readRangeLocked(ctx)
}

// Distance returns the range in millimetres for convenience, or -1 when the
// reading is not valid (out of range or a sensor status failure), matching the
// official driver's get_range.
func (d *Device) Distance() (int16, error) {
	return d.DistanceContext(context.Background())
}

// DistanceContext is Distance with cancellation.
func (d *Device) DistanceContext(ctx context.Context) (int16, error) {
	ctx = ctxOrBackground(ctx)
	if err := d.lockContext(ctx); err != nil {
		return 0, err
	}
	defer d.unlock()
	if !d.continuous {
		if err := d.startSingleLocked(ctx); err != nil {
			return 0, err
		}
	}
	m, err := d.readRangeLocked(ctx)
	if err != nil {
		return 0, err
	}
	if !m.Valid {
		return -1, nil
	}
	return m.RangeMM, nil
}

// DistanceCM returns the range in centimetres (official get_distance), or
// -0.1 when the reading is invalid.
func (d *Device) DistanceCM() (float32, error) {
	return d.DistanceCMContext(context.Background())
}

// DistanceCMContext is DistanceCM with cancellation.
func (d *Device) DistanceCMContext(ctx context.Context) (float32, error) {
	mm, err := d.DistanceContext(ctx)
	if err != nil {
		return 0, err
	}
	return float32(mm) / 10, nil
}

// Reset performs a soft reset of the sensor. Call Init again before ranging.
func (d *Device) Reset() error {
	return d.ResetContext(context.Background())
}

// ResetContext is Reset with cancellation.
func (d *Device) ResetContext(ctx context.Context) error {
	ctx = ctxOrBackground(ctx)
	if err := d.lockContext(ctx); err != nil {
		return err
	}
	defer d.unlock()
	d.continuous = false
	return d.resetLocked(ctx)
}

// --- internals ---

func (d *Device) startSingleLocked(ctx context.Context) error {
	if err := d.continuousConfigLocked(ctx, d.stopVar); err != nil {
		return err
	}
	if err := d.write(regSysRangeStart, 0x01); err != nil {
		return err
	}
	return d.waitStart(ctx)
}

func (d *Device) continuousConfigLocked(ctx context.Context, stopVar uint8) error {
	for _, rv := range [...]regVal{
		{0x80, 0x01}, {0xFF, 0x01}, {0x00, 0x00},
	} {
		if err := ctxErr(ctx); err != nil {
			return err
		}
		if err := d.write(rv.reg, rv.val); err != nil {
			return err
		}
	}
	if err := d.write(regStopVariable, stopVar); err != nil {
		return err
	}
	for _, rv := range [...]regVal{
		{0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00},
	} {
		if err := ctxErr(ctx); err != nil {
			return err
		}
		if err := d.write(rv.reg, rv.val); err != nil {
			return err
		}
	}
	return nil
}

func (d *Device) readRangeLocked(ctx context.Context) (Measurement, error) {
	if err := d.waitReady(ctx); err != nil {
		return Measurement{}, err
	}
	d.tx[0] = regResultRangeStatus
	if err := d.bus.Tx(uint16(d.addr), d.tx[:1], d.rx[:12]); err != nil {
		return Measurement{}, err
	}
	status := (d.rx[0] & 0x78) >> 3
	mm := int16(uint16(d.rx[10])<<8 | uint16(d.rx[11]))
	if err := d.write(regSystemInterruptClear, 0x01); err != nil {
		return Measurement{}, err
	}
	return Measurement{
		RangeMM: mm,
		Status:  status,
		Valid:   mm > 0 && mm <= 4000,
	}, nil
}
