//go:build linux

package control

import (
	"testing"

	"rosieos/rt-core/ipcclient"
)

// Preparation never authorizes input or native Begin without calibration;
// protocol refusals must leave the native generation and input count unchanged.
func TestRemoteJogPreparationFailsClosed(t *testing.T) {
	for _, mode := range []string{"begin", "binary"} {
		t.Run(mode, func(t *testing.T) {
			f := startRemoteMotion(t)
			before, err := f.controller.JogStatus()
			if err != nil {
				t.Fatal(err)
			}
			w := f.dial(t, 0)
			if mode == "begin" {
				w.send(t, 1, []byte(`{"type":"begin","seq":3,"axis_mask":1,"host_tx_ns":1}`))
				w.rejection(t, 3, ErrJogClockInvalid.Error())
			} else {
				w.update(t, w.frame(f, 0, 1))
				w.rejection(t, 0, ReasonJogSessionStale)
			}
			after := waitRemoteJog(t, f, func(o ipcclient.JogObservation) bool { return o.Open == 0 })
			if after.JogGeneration != before.JogGeneration || after.ObservedSourceSequence != 0 || after.HasInput != 0 {
				t.Fatalf("preparation changed native jog: before=%+v after=%+v", before, after)
			}
		})
	}
}
