package tof90

// StartContinuous puts the sensor in continuous ranging mode: it measures as
// fast as the timing budget allows and asserts the interrupt when a result is
// ready. Use Ready and ReadRange to consume results.
func (d *Device) StartContinuous() error {
	d.mu.Lock()
	defer d.mu.Unlock()
	if d.continuous {
		return nil
	}
	if err := d.continuousConfigLocked(d.stopVar); err != nil {
		return err
	}
	if err := d.write(regSysRangeStart, 0x02); err != nil {
		return err
	}
	if err := d.waitStart(); err != nil {
		return err
	}
	d.continuous = true
	return nil
}

// StopContinuous returns the sensor to single-shot mode and starts one
// measurement, matching the official driver.
func (d *Device) StopContinuous() error {
	d.mu.Lock()
	defer d.mu.Unlock()
	if err := d.write(regSysRangeStart, 0x01); err != nil {
		return err
	}
	if err := d.continuousConfigLocked(0x00); err != nil {
		return err
	}
	d.continuous = false
	return d.startSingleLocked()
}

// IsContinuous reports whether continuous mode is active.
func (d *Device) IsContinuous() bool {
	d.mu.Lock()
	c := d.continuous
	d.mu.Unlock()
	return c
}

// Ready reports whether a range result is waiting to be read.
func (d *Device) Ready() (bool, error) {
	d.mu.Lock()
	defer d.mu.Unlock()
	v, err := d.readU8(regResultInterruptStatus)
	if err != nil {
		return false, err
	}
	return v&0x07 != 0, nil
}

// Measure performs one ranging measurement, starting a single measurement when
// the sensor is not in continuous mode.
func (d *Device) Measure() (Measurement, error) {
	d.mu.Lock()
	defer d.mu.Unlock()
	if !d.continuous {
		if err := d.startSingleLocked(); err != nil {
			return Measurement{}, err
		}
	}
	return d.readRangeLocked()
}

// StartMeasurement starts one single-shot measurement and returns as soon as
// the sensor accepts it; use Ready and ReadRange to fetch the result (the
// official driver's do_range_measurement).
func (d *Device) StartMeasurement() error {
	d.mu.Lock()
	defer d.mu.Unlock()
	return d.startSingleLocked()
}

// ReadRange reads the pending result without starting a new measurement. Use
// it in continuous mode after Ready reports true.
func (d *Device) ReadRange() (Measurement, error) {
	d.mu.Lock()
	defer d.mu.Unlock()
	return d.readRangeLocked()
}

// Distance returns the range in millimetres for convenience, or -1 when the
// reading is not valid (out of range or a sensor status failure), matching the
// official driver's get_range.
func (d *Device) Distance() (int16, error) {
	d.mu.Lock()
	defer d.mu.Unlock()
	if !d.continuous {
		if err := d.startSingleLocked(); err != nil {
			return 0, err
		}
	}
	m, err := d.readRangeLocked()
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
	mm, err := d.Distance()
	if err != nil {
		return 0, err
	}
	return float32(mm) / 10, nil
}

// Reset performs a soft reset of the sensor. Call Init again before ranging.
func (d *Device) Reset() error {
	d.mu.Lock()
	defer d.mu.Unlock()
	d.continuous = false
	return d.resetLocked()
}

// --- internals ---

func (d *Device) startSingleLocked() error {
	if err := d.continuousConfigLocked(d.stopVar); err != nil {
		return err
	}
	if err := d.write(regSysRangeStart, 0x01); err != nil {
		return err
	}
	return d.waitStart()
}

func (d *Device) continuousConfigLocked(stopVar uint8) error {
	for _, rv := range [...]regVal{
		{0x80, 0x01}, {0xFF, 0x01}, {0x00, 0x00},
	} {
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
		if err := d.write(rv.reg, rv.val); err != nil {
			return err
		}
	}
	return nil
}

func (d *Device) readRangeLocked() (Measurement, error) {
	if err := d.waitReady(); err != nil {
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
