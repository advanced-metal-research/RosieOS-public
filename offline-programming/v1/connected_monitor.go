package main

import (
	"context"
	"errors"
	"fmt"
	"sync"
	"time"

	"rosieos/offline-programming/v1/internal/targetplanning"
	"github.com/nats-io/nats.go"
)

type ConnectedNATSMonitor struct {
	registry *ConnectedRegistry
	resolver targetplanning.NATSURLResolver
	cancel   context.CancelFunc
	wg       sync.WaitGroup
}

func NewConnectedNATSMonitor(
	registry *ConnectedRegistry,
	resolver targetplanning.NATSURLResolver,
) (*ConnectedNATSMonitor, error) {
	if registry == nil || resolver == nil {
		return nil, errors.New(
			"connected observation requires a target registry and daemon NATS resolver",
		)
	}
	return &ConnectedNATSMonitor{
		registry: registry,
		resolver: resolver,
	}, nil
}

func (monitor *ConnectedNATSMonitor) Start(parent context.Context) {
	if monitor == nil || monitor.cancel != nil {
		return
	}
	ctx, cancel := context.WithCancel(parent)
	monitor.cancel = cancel
	for _, descriptor := range monitor.registry.Discovery().Targets {
		target, ok := monitor.registry.PlanningTarget(descriptor.ProfileID)
		if !ok {
			continue
		}
		monitor.wg.Add(1)
		go func(profileID string, serviceRef string) {
			defer monitor.wg.Done()
			monitor.runTarget(ctx, profileID, serviceRef)
		}(descriptor.ProfileID, target.Binding.MotionServerNATSServiceRef)
	}
}

func (monitor *ConnectedNATSMonitor) Close() {
	if monitor == nil {
		return
	}
	if monitor.cancel != nil {
		monitor.cancel()
	}
	monitor.wg.Wait()
}

func (monitor *ConnectedNATSMonitor) runTarget(
	ctx context.Context,
	profileID string,
	serviceRef string,
) {
	var statusSequence uint64
	var planSequence uint64
	var rtcoreSequence uint64
	for ctx.Err() == nil {
		natsURL, err := monitor.resolver.ResolveNATS(ctx, serviceRef)
		if err == nil {
			err = monitor.observeConnection(
				ctx,
				profileID,
				natsURL,
				&statusSequence,
				&planSequence,
				&rtcoreSequence,
			)
		}
		if ctx.Err() != nil {
			return
		}
		timer := time.NewTimer(time.Second)
		select {
		case <-ctx.Done():
			if !timer.Stop() {
				<-timer.C
			}
			return
		case <-timer.C:
		}
	}
}

func (monitor *ConnectedNATSMonitor) observeConnection(
	ctx context.Context,
	profileID string,
	natsURL string,
	statusSequence *uint64,
	planSequence *uint64,
	rtcoreSequence *uint64,
) error {
	target, ok := monitor.registry.PlanningTarget(profileID)
	if !ok {
		return fmt.Errorf("connected target %q disappeared", profileID)
	}
	connection, err := nats.Connect(
		natsURL,
		nats.Name("offline-programming-v1-connected-observation"),
		nats.Timeout(3*time.Second),
		nats.MaxReconnects(0),
		nats.NoEcho(),
	)
	if err != nil {
		return err
	}
	defer connection.Close()

	statuses := make(chan []byte, 1)
	plans := make(chan []byte, 1)
	rtcoreStatuses := make(chan []byte, 1)
	replaceLatest := func(destination chan []byte, raw []byte) {
		copy := append([]byte(nil), raw...)
		select {
		case destination <- copy:
			return
		default:
		}
		select {
		case <-destination:
		default:
		}
		select {
		case destination <- copy:
		default:
		}
	}
	if _, err := connection.Subscribe(
		target.Binding.MotionServerStatusSubject,
		func(message *nats.Msg) {
			replaceLatest(statuses, message.Data)
		},
	); err != nil {
		return err
	}
	if _, err := connection.Subscribe(
		target.Binding.MotionServerPlanSubject,
		func(message *nats.Msg) {
			replaceLatest(plans, message.Data)
		},
	); err != nil {
		return err
	}
	if _, err := connection.Subscribe(
		target.Binding.RealtimeCoreStatusSubject,
		func(message *nats.Msg) {
			replaceLatest(rtcoreStatuses, message.Data)
		},
	); err != nil {
		return err
	}
	if err := connection.FlushWithContext(ctx); err != nil {
		return err
	}
	closed := connection.StatusChanged(nats.CLOSED)
	for {
		select {
		case <-ctx.Done():
			return nil
		case raw := <-statuses:
			(*statusSequence)++
			_ = monitor.registry.AcceptNATSStatus(
				profileID,
				raw,
				*statusSequence,
				time.Now().UTC(),
			)
		case raw := <-plans:
			(*planSequence)++
			_ = monitor.registry.AcceptNATSPlan(
				profileID,
				raw,
				*planSequence,
				time.Now().UTC(),
			)
		case raw := <-rtcoreStatuses:
			(*rtcoreSequence)++
			_ = monitor.registry.AcceptNATSRTCoreStatus(
				profileID,
				raw,
				*rtcoreSequence,
				time.Now().UTC(),
			)
		case <-closed:
			return errors.New("connected observation NATS connection closed")
		}
	}
}
