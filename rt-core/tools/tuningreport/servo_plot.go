package main

import (
	"fmt"
	"image/color"
	"math"
	"path/filepath"
)

// viridis-like ramp: dark (quiet) to yellow (loud).
var heatRamp = []color.RGBA{{68, 1, 84, 255}, {59, 82, 139, 255}, {33, 145, 140, 255}, {94, 201, 98, 255}, {253, 231, 37, 255}}

func heatColor(v float64) color.RGBA {
	if math.IsNaN(v) {
		return color.RGBA{235, 235, 235, 255}
	}
	v = math.Max(0, math.Min(1, v)) * float64(len(heatRamp)-1)
	i := int(math.Min(math.Floor(v), float64(len(heatRamp)-2)))
	f := v - float64(i)
	a, b := heatRamp[i], heatRamp[i+1]
	mix := func(x, y uint8) uint8 { return uint8(float64(x) + (float64(y)-float64(x))*f) }
	return color.RGBA{mix(a.R, b.R), mix(a.G, b.G), mix(a.B, b.B), 255}
}

// heat draws columns of spectra at times t (s), each col[i] covering
// [t - half, t + half); rows are bins of width df from 0 Hz. Values in dB
// map zmin..zmax onto the ramp; time with no column stays white.
func (c *canvas) heat(x0, y0, w, h int, title string, t []float64, half float64, cols [][]float32, df, zmin, zmax float64) {
	black, light := color.RGBA{0, 0, 0, 255}, color.RGBA{200, 200, 200, 255}
	l, top, r, b := x0+70, y0+28, x0+w-90, y0+h-34
	c.text(l, y0+8, title, 1, black)
	if len(t) == 0 || len(cols[0]) == 0 {
		c.text(l, top+20, "no data", 1, light)
		return
	}
	tmin, tmax := t[0]-half, t[len(t)-1]+half
	rows := len(cols[0])
	fmax := float64(rows-1) * df
	px := func(x float64) float64 { return float64(l) + (x-tmin)/(tmax-tmin)*float64(r-l) }
	for i, ti := range t {
		xa, xb := int(px(ti-half)), int(math.Ceil(px(ti+half)))
		for x := xa; x < max(xb, xa+1); x++ {
			for y := top; y < b; y++ {
				f := float64(b-y) / float64(b-top) * fmax
				k := int(math.Round(f / df))
				if k >= rows {
					k = rows - 1
				}
				c.set(x, y, heatColor((float64(cols[i][k])-zmin)/(zmax-zmin)))
			}
		}
	}
	c.line(float64(l), float64(top), float64(l), float64(b), black)
	c.line(float64(l), float64(b), float64(r), float64(b), black)
	ys := niceStep(fmax)
	for v := 0.0; v <= fmax; v += ys {
		y := float64(b) - v/fmax*float64(b-top)
		c.line(float64(l-4), y, float64(l), y, black)
		s := fmtTick(v, ys)
		c.text(l-6-6*len(s), int(y)-3, s, 1, black)
	}
	xs := niceStep(tmax - tmin)
	for v := math.Ceil(tmin/xs) * xs; v <= tmax; v += xs {
		c.line(px(v), float64(b), px(v), float64(b+4), black)
		s := fmtTick(v, xs)
		c.text(int(px(v))-3*len(s), b+6, s, 1, black)
	}
	c.text(r-6*len("s"), b+20, "s", 1, black)
	c.text(x0+4, top-14, "Hz", 1, black)
	// colour bar
	bx := r + 20
	for y := top; y < b; y++ {
		col := heatColor(float64(b-y) / float64(b-top))
		for x := bx; x < bx+12; x++ {
			c.set(x, y, col)
		}
	}
	c.text(bx+16, top, fmt.Sprintf("%.0f", zmax), 1, black)
	c.text(bx+16, b-8, fmt.Sprintf("%.0f", zmin), 1, black)
	c.text(bx-4, b+6, "dB", 1, black)
}

// servoPlots writes the servo-loop figures: the per-joint torque
// spectrogram over the session, the band-energy timeline, the per-state
// torque spectra and the loudest hold's spectra.
func (a *Analysis) servoPlots(dir string, r *Report) ([]string, error) {
	sq := r.Servo
	nj := len(a.cfg.Joints)
	var names []string
	save := func(name string, c *canvas) error {
		names = append(names, name)
		return savePNG(filepath.Join(dir, name), c)
	}
	lp := bandLabel(sq.LoopBand)
	// 1. spectrogram of torque per joint
	{
		W, H := 1600, 220
		c := newCanvas(W, H*nj)
		for j := 0; j < nj; j++ {
			var t []float64
			var cols [][]float32
			for _, p := range sq.tl {
				if p.j[j].hasTorque {
					t, cols = append(t, p.t), append(cols, p.j[j].spec)
				}
			}
			js := sq.Joints[j]
			c.heat(0, j*H, W, H, fmt.Sprintf("%s torque spectrogram (dB re 1 permille^2 per %.1f Hz bin; s from %s): worst %s window %s permille RMS at %s, %d buzz episodes",
				js.Joint, sq.TimeBinHz, firstStamp(a), lp, f1(js.LoopPeakRMS, 0), js.LoopPeakAt, js.Episodes), t, sq.step/2, cols, sq.TimeBinHz, -30, 50)
		}
		if err := save("servo-spectrogram.png", c); err != nil {
			return names, err
		}
	}
	// 2. band-energy timeline
	{
		W, H := 1600, 230
		c := newCanvas(W, H*nj)
		for j := 0; j < nj; j++ {
			var ss []series
			for bi, b := range sq.Bands {
				var x, y []float64
				last := math.Inf(-1)
				for _, p := range sq.tl {
					if !p.j[j].hasTorque {
						continue
					}
					if p.t-last > 1.5*sq.step {
						x, y = append(x, math.NaN()), append(y, math.NaN())
					}
					x, y = append(x, p.t), append(y, p.j[j].band[bi])
					last = p.t
				}
				ss = append(ss, series{name: bandLabel(b), x: x, y: y, col: bi})
			}
			var ex, ey []float64
			for _, e := range sq.Episodes {
				if e.Joint == sq.Joints[j].Joint {
					ex, ey = append(ex, e.t0, e.t1+sq.step, math.NaN()), append(ey, e.PeakRMS*1.5, e.PeakRMS*1.5, math.NaN())
				}
			}
			if len(ex) > 0 {
				ss = append(ss, series{name: "buzz episode", x: ex, y: ey, col: 7})
			}
			c.panel(0, j*H, W, H, panelSpec{title: fmt.Sprintf("%s torque band RMS per %d ms window (s from %s); dashed: buzz thresholds %g still / %g moving (%s)",
				sq.Joints[j].Joint, r.Params.Servo.TimelineMs, firstStamp(a), r.Params.Servo.BuzzStillOn, r.Params.Servo.BuzzMovingOn, lp),
				xl: "s", yl: "permille RMS", ss: ss, hlines: []float64{r.Params.Servo.BuzzStillOn, r.Params.Servo.BuzzMovingOn}, logy: true})
		}
		if err := save("servo-bands.png", c); err != nil {
			return names, err
		}
	}
	// 3. torque spectra per state
	{
		W, H := 1200, 220
		c := newCanvas(W, H*nj)
		for j := 0; j < nj; j++ {
			var ss []series
			for s := 0; s < nStates; s++ {
				if p := sq.Joints[j].psd[s]; p != nil {
					ss = append(ss, psdSeries(stateNames[s], p, sq.BinHz, s))
				}
			}
			c.panel(0, j*H, W, H, panelSpec{title: fmt.Sprintf("%s torque spectrum per state (permille RMS per %.2f Hz bin)", sq.Joints[j].Joint, sq.BinHz),
				xl: "Hz", yl: "permille", ss: ss, logy: true})
		}
		if err := save("servo-spectra.png", c); err != nil {
			return names, err
		}
	}
	// 4. loudest hold vs armed idle
	if len(sq.Holds) > 0 {
		hi, hj := -1, 0
		for i, h := range sq.Holds {
			for j, v := range h.LoopRMS {
				if !math.IsNaN(v) && (hi < 0 || v > sq.Holds[hi].LoopRMS[hj]) {
					hi, hj = i, j
				}
			}
		}
		if hi >= 0 {
			h := sq.Holds[hi]
			W, H := 1200, 220
			c := newCanvas(W, H*nj)
			for j := 0; j < nj; j++ {
				var ss []series
				if h.psd[j] != nil {
					ss = append(ss, psdSeries(fmt.Sprintf("hold %s %s", h.At, h.Axis), h.psd[j], sq.BinHz, 1))
				}
				if p := sq.Joints[j].psd[stIdle]; p != nil {
					ss = append(ss, psdSeries("armed idle", p, sq.BinHz, 0))
				}
				c.panel(0, j*H, W, H, panelSpec{title: fmt.Sprintf("%s torque spectrum in the loudest hold (%s at %s, %s %s RMS on %s) vs armed idle",
					sq.Joints[j].Joint, h.Axis, h.At, f1(h.LoopRMS[hj], 0), lp, a.cfg.Joints[hj].Name), xl: "Hz", yl: "permille", ss: ss, logy: true})
			}
			if err := save("servo-worst-hold.png", c); err != nil {
				return names, err
			}
		}
	}
	return names, nil
}

func psdSeries(name string, p []float64, df float64, col int) series {
	x, y := make([]float64, len(p)), make([]float64, len(p))
	for k, v := range p {
		x[k], y[k] = float64(k)*df, math.Sqrt(v)
	}
	x[0], y[0] = math.NaN(), math.NaN()
	return series{name: name, x: x, y: y, col: col}
}

func firstStamp(a *Analysis) string {
	if len(a.wins) == 0 {
		return "-"
	}
	return a.stamp(a.wins[0], 0)
}
