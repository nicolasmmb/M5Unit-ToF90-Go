package tof90

import (
	"testing"
	"time"
)

func reportTx(b *testing.B, f *fakeSensor, before int) {
	b.Helper()
	b.ReportMetric(float64(f.tx-before)/float64(b.N), "tx/op")
}

func BenchmarkInit(b *testing.B) {
	f := newFakeSensor()
	d := New(f)
	d.SetTimeout(50 * time.Millisecond)
	b.ReportAllocs()
	b.ResetTimer()
	start := f.tx
	for i := 0; i < b.N; i++ {
		if err := d.Init(); err != nil {
			b.Fatal(err)
		}
	}
	reportTx(b, f, start)
}

func BenchmarkMeasure(b *testing.B) {
	d, f := newTestDevice(b)
	f.setRange(500, RangeValid)
	b.ReportAllocs()
	b.ResetTimer()
	start := f.tx
	for i := 0; i < b.N; i++ {
		if _, err := d.Measure(); err != nil {
			b.Fatal(err)
		}
	}
	reportTx(b, f, start)
}

func BenchmarkReadRangeContinuous(b *testing.B) {
	d, f := newTestDevice(b)
	f.setRange(500, RangeValid)
	if err := d.StartContinuous(); err != nil {
		b.Fatal(err)
	}
	b.ReportAllocs()
	b.ResetTimer()
	start := f.tx
	for i := 0; i < b.N; i++ {
		if _, err := d.ReadRange(); err != nil {
			b.Fatal(err)
		}
	}
	reportTx(b, f, start)
}

func BenchmarkTimingBudget(b *testing.B) {
	d, f := newTestDevice(b)
	b.ReportAllocs()
	b.ResetTimer()
	start := f.tx
	for i := 0; i < b.N; i++ {
		if _, err := d.TimingBudget(); err != nil {
			b.Fatal(err)
		}
	}
	reportTx(b, f, start)
}

func BenchmarkSetTimingBudget(b *testing.B) {
	d, f := newTestDevice(b)
	b.ReportAllocs()
	b.ResetTimer()
	start := f.tx
	for i := 0; i < b.N; i++ {
		if err := d.SetTimingBudget(50 * time.Millisecond); err != nil {
			b.Fatal(err)
		}
	}
	reportTx(b, f, start)
}

func BenchmarkSignalRateLimit(b *testing.B) {
	d, f := newTestDevice(b)
	b.ReportAllocs()
	b.ResetTimer()
	start := f.tx
	for i := 0; i < b.N; i++ {
		if _, err := d.SignalRateLimit(); err != nil {
			b.Fatal(err)
		}
	}
	reportTx(b, f, start)
}

func BenchmarkLongRange(b *testing.B) {
	d, f := newTestDevice(b)
	b.ReportAllocs()
	b.ResetTimer()
	start := f.tx
	for i := 0; i < b.N; i++ {
		if err := d.LongRange(); err != nil {
			b.Fatal(err)
		}
	}
	reportTx(b, f, start)
}
