package main

import (
	"fmt"
	"image"
	"image/color"
	"image/png"
	"math"
	"os"
	"path/filepath"
	"strconv"
)

// A minimal raster plotter: stacked panels of line series with axes, ticks
// and labels, written as PNG with the standard library only.

var palette = []color.RGBA{{31, 119, 180, 255}, {214, 39, 40, 255}, {44, 160, 44, 255}, {255, 127, 14, 255},
	{148, 103, 189, 255}, {140, 86, 75, 255}, {227, 119, 194, 255}, {127, 127, 127, 255}, {188, 189, 34, 255}, {23, 190, 207, 255}}

type canvas struct{ img *image.RGBA }

func newCanvas(w, h int) *canvas {
	c := &canvas{image.NewRGBA(image.Rect(0, 0, w, h))}
	for i := range c.img.Pix {
		c.img.Pix[i] = 255
	}
	return c
}

func (c *canvas) set(x, y int, col color.RGBA) {
	if image.Pt(x, y).In(c.img.Rect) {
		c.img.SetRGBA(x, y, col)
	}
}

func (c *canvas) line(x0, y0, x1, y1 float64, col color.RGBA) {
	dx, dy := x1-x0, y1-y0
	n := int(math.Max(math.Abs(dx), math.Abs(dy))) + 1
	if n > 20000 {
		n = 20000
	}
	for i := 0; i <= n; i++ {
		t := float64(i) / float64(n)
		c.set(int(math.Round(x0+dx*t)), int(math.Round(y0+dy*t)), col)
	}
}

func (c *canvas) text(x, y int, s string, scale int, col color.RGBA) {
	for _, r := range s {
		if r < 0x20 || r > 0x7E {
			r = '?'
		}
		g := font5x7[r-0x20]
		for cx := 0; cx < 5; cx++ {
			for cy := 0; cy < 8; cy++ {
				if g[cx]&(1<<cy) != 0 {
					for sx := 0; sx < scale; sx++ {
						for sy := 0; sy < scale; sy++ {
							c.set(x+cx*scale+sx, y+cy*scale+sy, col)
						}
					}
				}
			}
		}
		x += 6 * scale
	}
}

type series struct {
	name string
	x, y []float64 // NaN in y breaks the line
	col  int
	dots bool
	mark bool // a 7-pixel cross per point, no line
}

type panelSpec struct {
	title, xl, yl string
	ss            []series
	hlines        []float64
	ymin, ymax    *float64
	logy          bool // decade axis; values <= 0 are not drawn
}

func logPanel(p panelSpec) panelSpec {
	lg := func(v float64) float64 {
		if v > 0 {
			return math.Log10(v)
		}
		return math.NaN()
	}
	var ss []series
	for _, s := range p.ss {
		y := make([]float64, len(s.y))
		for i, v := range s.y {
			y[i] = lg(v)
		}
		s.y = y
		ss = append(ss, s)
	}
	p.ss = ss
	var hl []float64
	for _, v := range p.hlines {
		hl = append(hl, lg(v))
	}
	p.hlines = hl
	if p.ymin != nil {
		v := lg(*p.ymin)
		p.ymin = &v
	}
	if p.ymax != nil {
		v := lg(*p.ymax)
		p.ymax = &v
	}
	return p
}

func niceStep(span float64) float64 {
	if span <= 0 || math.IsNaN(span) {
		return 1
	}
	raw := span / 5
	p := math.Pow(10, math.Floor(math.Log10(raw)))
	for _, m := range []float64{1, 2, 5, 10} {
		if raw <= m*p {
			return m * p
		}
	}
	return 10 * p
}

func fmtTick(v, step float64) string {
	prec := 0
	if step < 1 {
		prec = int(math.Ceil(-math.Log10(step)))
	}
	if prec > 6 {
		return strconv.FormatFloat(v, 'g', 3, 64)
	}
	return strconv.FormatFloat(v, 'f', prec, 64)
}

func (c *canvas) panel(x0, y0, w, h int, p panelSpec) {
	logy := p.logy
	if logy {
		p = logPanel(p)
	}
	black, grey, light := color.RGBA{0, 0, 0, 255}, color.RGBA{150, 150, 150, 255}, color.RGBA{225, 225, 225, 255}
	l, t, r, b := x0+70, y0+28, x0+w-20, y0+h-34
	xmin, xmax, ymin, ymax := math.Inf(1), math.Inf(-1), math.Inf(1), math.Inf(-1)
	for _, s := range p.ss {
		for i := range s.x {
			if math.IsNaN(s.y[i]) || math.IsInf(s.y[i], 0) {
				continue
			}
			xmin, xmax = math.Min(xmin, s.x[i]), math.Max(xmax, s.x[i])
			ymin, ymax = math.Min(ymin, s.y[i]), math.Max(ymax, s.y[i])
		}
	}
	for _, hl := range p.hlines {
		ymin, ymax = math.Min(ymin, hl), math.Max(ymax, hl)
	}
	if p.ymin != nil {
		ymin = *p.ymin
	}
	if p.ymax != nil {
		ymax = *p.ymax
	}
	c.text(l, y0+8, p.title, 1, black)
	if math.IsInf(xmin, 1) {
		c.text(l, t+20, "no data", 1, grey)
		return
	}
	if xmax <= xmin {
		xmax = xmin + 1
	}
	if ymax <= ymin {
		ymax, ymin = ymin+1, ymin-1
	}
	pad := (ymax - ymin) * 0.05
	ymin, ymax = ymin-pad, ymax+pad
	px := func(x float64) float64 { return float64(l) + (x-xmin)/(xmax-xmin)*float64(r-l) }
	py := func(y float64) float64 { return float64(b) - (y-ymin)/(ymax-ymin)*float64(b-t) }
	ys := niceStep(ymax - ymin)
	if logy {
		ys = math.Max(1, math.Round(ys))
	}
	for v := math.Ceil(ymin/ys) * ys; v <= ymax; v += ys {
		c.line(float64(l), py(v), float64(r), py(v), light)
		s := fmtTick(v, ys)
		if logy {
			s = strconv.FormatFloat(math.Pow(10, v), 'g', 3, 64)
		}
		c.text(l-6-6*len(s), int(py(v))-3, s, 1, black)
	}
	xs := niceStep(xmax - xmin)
	for v := math.Ceil(xmin/xs) * xs; v <= xmax; v += xs {
		c.line(px(v), float64(t), px(v), float64(b), light)
		s := fmtTick(v, xs)
		c.text(int(px(v))-3*len(s), b+6, s, 1, black)
	}
	for _, hl := range p.hlines {
		for x := l; x < r; x += 6 {
			c.line(float64(x), py(hl), float64(x+3), py(hl), grey)
		}
	}
	c.line(float64(l), float64(t), float64(l), float64(b), black)
	c.line(float64(l), float64(b), float64(r), float64(b), black)
	c.text(r-6*len(p.xl), b+20, p.xl, 1, black)
	c.text(x0+4, t-14, p.yl, 1, black)
	lx := l + 8
	for _, s := range p.ss {
		col := palette[s.col%len(palette)]
		if s.name != "" {
			c.line(float64(lx), float64(t+8), float64(lx+14), float64(t+8), col)
			c.text(lx+18, t+5, s.name, 1, col)
			lx += 30 + 6*len(s.name)
		}
		var pxp, pyp float64
		have := false
		for i := range s.x {
			if math.IsNaN(s.y[i]) || math.IsInf(s.y[i], 0) {
				have = false
				continue
			}
			X, Y := px(s.x[i]), py(math.Max(ymin, math.Min(ymax, s.y[i])))
			if s.dots {
				c.set(int(X), int(Y), col)
				continue
			}
			if s.mark {
				c.line(X-3, Y, X+3, Y, col)
				c.line(X, Y-3, X, Y+3, col)
				continue
			}
			if have {
				c.line(pxp, pyp, X, Y, col)
			}
			pxp, pyp, have = X, Y, true
		}
	}
}

func savePNG(path string, c *canvas) error {
	f, err := os.Create(path)
	if err != nil {
		return err
	}
	defer f.Close()
	return png.Encode(f, c.img)
}

// decimate keeps every n-th point so a 100 s window stays drawable.
func decimate(n, max int) int {
	if n <= max {
		return 1
	}
	return (n + max - 1) / max
}

// plots writes the report's figures and returns their file names.
func (a *Analysis) plots(dir string, r *Report) ([]string, error) {
	var names []string
	nj := len(a.cfg.Joints)
	save := func(name string, c *canvas) error {
		names = append(names, name)
		return savePNG(filepath.Join(dir, name), c)
	}
	// 1. following error against the scaled bound, per joint, all windows on
	// one wall-clock axis (s from the first record).
	{
		W, H := 1600, 230
		c := newCanvas(W, H*nj)
		t0 := a.wins[0].s.TNs[0] + a.wins[0].s.ClockOffsetNs
		for j := 0; j < nj; j++ {
			jc := a.cfg.Joints[j]
			var x, fe, bp, bn []float64
			for _, w := range a.wins {
				step := decimate(w.n, 20000)
				for k := 1; k < w.n; k += step {
					x = append(x, float64(w.s.TNs[k]+w.s.ClockOffsetNs-t0)/1e9)
					if !w.on[j][k] || !w.on[j][k-1] {
						fe, bp, bn = append(fe, math.NaN()), append(bp, math.NaN()), append(bn, math.NaN())
						continue
					}
					// worst cycle in the decimation step keeps peaks visible
					worst, wb := 0.0, 0.0
					for q := k; q < k+step && q < w.n; q++ {
						if !w.on[j][q] || !w.on[j][q-1] {
							continue
						}
						e := w.qc[j][q-1] - w.qa[j][q]
						bnd := jc.FollowingError + math.Abs(w.vc[j][q])*a.cfg.LagAllowanceS
						if math.Abs(e)/bnd >= math.Abs(worst)/math.Max(wb, 1e-12) {
							worst, wb = e, bnd
						}
					}
					fe, bp, bn = append(fe, worst*deg), append(bp, wb*deg), append(bn, -wb*deg)
				}
				x, fe, bp, bn = append(x, math.NaN()), append(fe, math.NaN()), append(bp, math.NaN()), append(bn, math.NaN())
			}
			m := r.Joints[j]
			c.panel(0, j*H, W, H, panelSpec{title: fmt.Sprintf("%s following error vs scaled bound (static %.3f deg + |v| x %.0f ms): max FE/bound %.3f, longest over %.0f ms",
				jc.Name, jc.FollowingError*deg, a.cfg.LagAllowanceS*1e3, m.FERatioMax, m.FEOverLongestMs), xl: "s", yl: "deg",
				ss: []series{{name: "FE", x: x, y: fe, col: 0}, {name: "+bound", x: x, y: bp, col: 1}, {name: "-bound", x: x, y: bn, col: 1}}})
		}
		if err := save("fe-bound.png", c); err != nil {
			return names, err
		}
	}
	// 2. residual spectra
	{
		W, H := 1000, 200
		c := newCanvas(W, H*nj)
		for j := 0; j < nj; j++ {
			g := r.Joints[j].Ring
			a2 := make([]float64, len(g.spectrumA))
			for i, v := range g.spectrumA {
				a2[i] = v * deg
			}
			c.panel(0, j*H, W, H, panelSpec{title: fmt.Sprintf("%s lag-compensated residual spectrum: peak %s Hz, %s deg", a.cfg.Joints[j].Name, f1(g.SpectrumPeakHz, 2), f1(g.SpectrumAmpDeg, 4)),
				xl: "Hz", yl: "deg", ss: []series{{x: g.spectrumF, y: a2, col: 4}}})
		}
		if err := save("ring-spectra.png", c); err != nil {
			return names, err
		}
	}
	// 3. worst ring episode per joint
	{
		W, H := 1000, 200
		c := newCanvas(W, H*nj)
		for j := 0; j < nj; j++ {
			g := r.Joints[j].Ring
			var best *RingEpisode
			for i := range g.Episodes {
				if best == nil || math.Abs(g.Episodes[i].PeakDeg) > math.Abs(best.PeakDeg) {
					best = &g.Episodes[i]
				}
			}
			ps := panelSpec{title: a.cfg.Joints[j].Name + " worst ring episode: none", xl: "ms from peak", yl: "deg"}
			if best != nil {
				x := make([]float64, len(best.trace))
				for i := range x {
					x[i] = float64(i - 200)
				}
				tol := a.cfg.Joints[j].CompletionTol * deg
				ps.title = fmt.Sprintf("%s worst ring at %s: %+.3f deg, %s Hz, zeta %s, settle %s ms; %s", a.cfg.Joints[j].Name, best.PeakAt, best.PeakDeg, f1(best.FreqZCHz, 2), f1(best.Damping, 3), f1(best.SettleMs, 0), best.Near)
				ps.ss = []series{{name: "actual - commanded(t - lag)", x: x, y: best.trace, col: 1}}
				ps.hlines = []float64{tol, -tol}
			}
			c.panel(0, j*H, W, H, ps)
		}
		if err := save("ring-worst.png", c); err != nil {
			return names, err
		}
	}
	// 4. stops
	{
		W, H := 1000, 200
		c := newCanvas(W, H*nj)
		for j := 0; j < nj; j++ {
			var ss []series
			for i, s := range r.Joints[j].Stops {
				if i >= 10 {
					break
				}
				x := make([]float64, len(s.trace))
				for q := range x {
					x[q] = float64(q)
				}
				ss = append(ss, series{x: x, y: s.trace, col: i})
			}
			tol := a.cfg.Joints[j].CompletionTol * deg
			c.panel(0, j*H, W, H, panelSpec{title: fmt.Sprintf("%s stops: actual - final target x direction (+ = past target), first 10 of %d; dashed = completion tolerance",
				a.cfg.Joints[j].Name, len(r.Joints[j].Stops)), xl: "ms after v_cmd = 0", yl: "deg", ss: ss, hlines: []float64{tol, -tol}})
		}
		if err := save("stops.png", c); err != nil {
			return names, err
		}
	}
	// 5. path holds: actual off-path traces
	if len(r.Holds) > 0 {
		W, H := 1200, 360
		c := newCanvas(W, 2*H)
		var ss []series
		var bx, bc, ba []float64
		for i, h := range r.Holds {
			x := make([]float64, len(h.actOffTrace))
			for q := range x {
				x[q] = float64(q)
			}
			if h.Linear {
				ss = append(ss, series{x: x, y: h.actOffTrace, col: i})
			}
			bx, bc, ba = append(bx, float64(i+1)), append(bc, h.CmdOffMax*1000), append(ba, h.ActOffMax)
		}
		c.panel(0, 0, W, H, panelSpec{title: "linear path holds: actual TCP distance from the commanded line (incl. stop and tail)", xl: "ms from hold start", yl: "mm", ss: ss})
		c.panel(0, H, W, H, panelSpec{title: "per hold: commanded off-line max (um, blue) and actual off-line max (mm, red)", xl: "hold #", yl: "um | mm",
			ss: []series{{name: "commanded um", x: bx, y: bc, col: 0, dots: false}, {name: "actual mm", x: bx, y: ba, col: 1}}})
		if err := save("path-holds.png", c); err != nil {
			return names, err
		}
	}
	if r.Servo != nil {
		sn, err := a.servoPlots(dir, r)
		names = append(names, sn...)
		if err != nil {
			return names, err
		}
	}
	return names, nil
}
