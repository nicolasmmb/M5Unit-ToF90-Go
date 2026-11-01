//go:build esp32

// Example: read the M5Stack Unit Mini ToF-90° (VL53L0C) over Grove I2C.
package main

import (
	"time"

	"machine"

	tof90 "github.com/nicolasmmb/M5Unit-ToF90-Go"
)

func main() {
	time.Sleep(time.Second)

	i2c := machine.I2C1
	if err := i2c.Configure(machine.I2CConfig{
		SDA:       machine.GPIO32,
		SCL:       machine.GPIO33,
		Frequency: 400 * machine.KHz,
	}); err != nil {
		println("tof90: i2c:", err.Error())
		return
	}

	d := tof90.New(i2c)
	d.SetTimeout(100 * time.Millisecond)
	if err := d.Init(); err != nil {
		println("tof90: init:", err.Error())
		return
	}
	_ = d.SetTimingBudget(33 * time.Millisecond)
	if err := d.StartContinuous(); err != nil {
		println("tof90: start:", err.Error())
		return
	}
	println("tof90: ranging at", d.Address())

	for {
		m, err := d.ReadRange()
		if err != nil {
			println("tof90: read:", err.Error())
			time.Sleep(100 * time.Millisecond)
			continue
		}
		println("tof90:", m.RangeMM, "mm status", m.Status, "valid", m.Valid)
		time.Sleep(100 * time.Millisecond)
	}
}
