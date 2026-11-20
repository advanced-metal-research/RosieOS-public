//go:build linux

package denseexec

import (
	"context"
	"fmt"
	"math"
	"strings"
	"testing"
	"time"

	"rosieos/rt-core/cartesian"
	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

func returnKinematics(t *testing.T) *cartesian.Kinematics {
	t.Helper()
	k, err := cartesian.LoadKinematics("../../../../robot_description/robots/rosie_1420_v1")
	if err != nil {
		t.Fatal(err)
	}
	return k
}

func returnSpeedFixture() (control.Description, control.Status, []JointMoveAxis, []float64) {
	d := armMoveDescription()
	s := control.Status{Axes: make([]ipcclient.LogicalAxisStatus, 6)}
	// Reproduce the Oct 31 Fast return's approximate starting joint angles,
	// with nonzero J4 as well to exercise its higher machine limit.
	origins := []float64{27.79, -28.02, 6.92, 93.44, -34.93, 28.03}
	var moves []JointMoveAxis
	for i := range origins {
		origins[i] *= math.Pi / 180
		d.Axes[i].ID, d.Axes[i].Index = fmt.Sprintf("J%d", i+1), uint32(i)
		d.Axes[i].MaxVelocity = 240 * math.Pi / 180
		acceleration := 257.962688682132
		if i == 3 {
			d.Axes[i].MaxVelocity, acceleration = 1333.3333333333333*math.Pi/180, 1433.1260482340667
		}
		d.Axes[i].JogAcceleration, d.Axes[i].MaxAcceleration = acceleration, &acceleration
		s.Axes[i].LogicalTarget, s.Axes[i].TimeNS = origins[i], 100
		zero := 0.0
		moves = append(moves, JointMoveAxis{Axis: i, PositionRad: &zero})
	}
	return d, s, moves, origins
}

// Measure the command reconstructed from the uploaded Hermite knots, through
// independent FK pose differences in absolute m/s and rad/s. Checking profile
// inputs alone would miss timing/interpolation errors.
func TestReturnZeroUsesSelectedCartesianJogSpeeds(t *testing.T) {
	k := returnKinematics(t)
	d, s, moves, origins := returnSpeedFixture()
	for _, tc := range []struct {
		name                      string
		fraction, linear, angular float64
	}{
		{"Slow", .1, 400, 30}, {"Medium", .25, 400, 30}, {"Fast", 1, 400, 30},
		{"custom travel", 1, 25, 180}, {"custom rotation", .5, 1000, 8},
	} {
		t.Run(tc.name, func(t *testing.T) {
			rate, ready, err := returnProgressVelocity(k, d, s, 63, 100, moves, tc.fraction, returnJogSpeeds{tc.linear, tc.angular})
			if err != nil || !ready {
				t.Fatalf("rate=%g ready=%v err=%v", rate, ready, err)
			}
			points, corners, err := jointMovesProfileAtRate(d, moves, tc.fraction, origins, 53616, rate)
			if err != nil {
				t.Fatal(err)
			}
			knots := knotPoints(points, time.Millisecond, corners...)
			q := make([][]float64, 6)
			for j := range q {
				q[j] = hermiteSamples(knots, time.Millisecond, j)
			}
			var previous [12]float64
			var peakLinear, peakAngular float64
			for n := range q[0] {
				pose := make([]float64, 6)
				progress := (origins[0] - q[0][n]) / origins[0]
				for j := range pose {
					pose[j] = q[j][n]
					if math.Abs(pose[j]-origins[j]*(1-progress)) > 1e-9 {
						t.Fatalf("joint path changed: sample %d J%d", n, j+1)
					}
				}
				fk := k.FK(pose)
				if n > 0 {
					var distance float64
					var relative [3][3]float64
					for row := 0; row < 3; row++ {
						distance += math.Pow(fk[4*row+3]-previous[4*row+3], 2)
						for col := 0; col < 3; col++ {
							for z := 0; z < 3; z++ {
								relative[row][col] += fk[4*row+z] * previous[4*col+z]
							}
						}
					}
					skew := []float64{relative[2][1] - relative[1][2], relative[0][2] - relative[2][0], relative[1][0] - relative[0][1]}
					angle := math.Atan2(math.Sqrt(skew[0]*skew[0]+skew[1]*skew[1]+skew[2]*skew[2])/2, (relative[0][0]+relative[1][1]+relative[2][2]-1)/2)
					peakLinear = math.Max(peakLinear, math.Sqrt(distance)/.001)
					peakAngular = math.Max(peakAngular, angle/.001)
				}
				previous = fk
			}
			linearLimit, angularLimit := tc.linear/1000*tc.fraction, tc.angular*math.Pi/180*tc.fraction
			if peakLinear > linearLimit+1e-8 || peakAngular > angularLimit+1e-8 {
				t.Fatalf("TCP exceeded preset: %g/%g m/s; %g/%g rad/s", peakLinear, linearLimit, peakAngular, angularLimit)
			}
			// Prevent an accidental second percentage or unnecessary slowdown.
			if math.Max(peakLinear/linearLimit, peakAngular/angularLimit) < .97 {
				t.Fatalf("unexpected crawl: ratios %g %g", peakLinear/linearLimit, peakAngular/angularLimit)
			}
			for j := range q {
				if math.Abs(q[j][len(q[j])-1]) > 1e-12 {
					t.Fatalf("J%d not exactly zero", j+1)
				}
				v, a, jerk := derivativePeaks(q[j])
				if v > d.Axes[j].MaxVelocity*tc.fraction+1e-6 || a > d.Axes[j].JogAcceleration*tc.fraction+1e-4 || jerk > 53616+.1 { // one machine jerk on the longest joint
					t.Fatalf("native limits exceeded J%d: %g %g %g", j+1, v, a, jerk)
				}
			}
			t.Logf("%s: %s; peak %.3f mm/s / %.3f deg/s", tc.name, points[len(points)-1].Time, peakLinear*1000, peakAngular*180/math.Pi)
		})
	}
}

func TestReturnSpeedNeedsFreshStationaryArmAndGeometry(t *testing.T) {
	k := returnKinematics(t)
	d, s, moves, _ := returnSpeedFixture()
	speeds := returnJogSpeeds{400, 30}
	if _, _, err := returnProgressVelocity(nil, d, s, 63, 100, moves, 1, speeds); err == nil {
		t.Fatal("missing model accepted")
	}
	// J6 is not moving, but its held coordinate still affects the TCP.
	moves = moves[:5]
	s.Axes[5].TimeNS = 99
	if _, ready, err := returnProgressVelocity(k, d, s, 63, 100, moves, 1, speeds); err != nil || ready {
		t.Fatalf("stale held J6: %v %v", ready, err)
	}
	s.Axes[5].TimeNS = 100
	moves = append(moves, JointMoveAxis{Axis: 6, Degrees: 1})
	if _, _, err := returnProgressVelocity(k, d, s, 127, 100, moves, 1, speeds); err == nil {
		t.Fatal("moving external axis silently ignored")
	}
	d.Axes[0].ID = "J2"
	if _, _, err := returnProgressVelocity(k, d, s, 63, 100, moves[:5], 1, speeds); err == nil {
		t.Fatal("ambiguous model mapping accepted")
	}
}

func TestRecoverToZeroPropagatesJogSpeeds(t *testing.T) {
	r, c := recoverFixture(t)
	k := returnKinematics(t)
	r.cfg.CartesianModel = "rosie_1420_v1"
	r.SetRobotGeometrySource(func(string) (RobotGeometry, error) { return RobotGeometry{Kinematics: k}, nil })
	c.mu.Lock()
	for i := range c.description.Axes {
		c.description.Axes[i].Index = uint32(i)
	}
	c.mu.Unlock()
	linear, angular := 100., 15.
	in := RecoverToZeroRequest{Fraction: .25, LinearSpeedMMs: &linear, AngularSpeedDegs: &angular}
	result, err := r.RecoverToZero(context.Background(), in)
	if err != nil || result.State != "confirm_arm" {
		t.Fatalf("%+v %v", result, err)
	}
	in.ConfirmArm, in.Confirmed = true, result.Joints
	result, err = r.RecoverToZero(context.Background(), in)
	if err != nil || result.State != "done" {
		t.Fatalf("%+v %v", result, err)
	}
	if !strings.Contains(result.Steps[1].Detail, "25.000 mm/s and 3.750 deg/s") {
		t.Fatal(result.Steps)
	}
	if !r.Capabilities(context.Background()).RecoverZeroJogSpeeds {
		t.Fatal("missing capability")
	}
}

func TestRecoverInvalidJogSpeedsDoNotArm(t *testing.T) {
	for _, v := range []float64{0, -1, math.NaN(), math.Inf(1), 1001} {
		r, c := recoverFixture(t)
		angular := 30.
		result, err := r.RecoverToZero(context.Background(), RecoverToZeroRequest{Fraction: 1, LinearSpeedMMs: &v, AngularSpeedDegs: &angular})
		recoverRefusal(t, result, err, "start", "recover_jog_speed_invalid")
		if calls := recoverCalls(c, "reset_fault", "enable", "arm", "prepare_trajectory"); len(calls) > 0 {
			t.Fatal(calls)
		}
	}
	linear := 100.
	if _, err := (RecoverToZeroRequest{LinearSpeedMMs: &linear}).jogSpeeds(); err == nil {
		t.Fatal("partial rates accepted")
	}
}

func TestRecoverMissingJogGeometryDoesNotArm(t *testing.T) {
	r, c := recoverFixture(t)
	linear, angular := 100., 15.
	result, err := r.RecoverToZero(context.Background(), RecoverToZeroRequest{Fraction: 1, LinearSpeedMMs: &linear, AngularSpeedDegs: &angular})
	recoverRefusal(t, result, err, "speed", "recover_jog_speed_unavailable")
	if calls := recoverCalls(c, "enable", "arm", "prepare_trajectory", "start_trajectory"); len(calls) != 0 {
		t.Fatal(calls)
	}
}

func TestRecoverVerySlowReturnRefusesBeforeArm(t *testing.T) {
	r, c := recoverFixture(t)
	k := returnKinematics(t)
	r.cfg.CartesianModel = "rosie_1420_v1"
	r.SetRobotGeometrySource(func(string) (RobotGeometry, error) { return RobotGeometry{Kinematics: k}, nil })
	c.mu.Lock()
	for i := range c.description.Axes {
		c.description.Axes[i].Index = uint32(i)
	}
	c.description.MaxTrajectoryPoints = 250000
	c.mu.Unlock()
	linear, angular := .001, .001
	result, err := r.RecoverToZero(context.Background(), RecoverToZeroRequest{Fraction: .1, LinearSpeedMMs: &linear, AngularSpeedDegs: &angular})
	detail := recoverRefusal(t, result, err, "speed", "recover_jog_speed_unavailable")
	if !strings.Contains(detail, "too long") {
		t.Fatal(detail)
	}
	if calls := recoverCalls(c, "enable", "arm", "prepare_trajectory", "start_trajectory"); len(calls) != 0 {
		t.Fatal(calls)
	}
}
