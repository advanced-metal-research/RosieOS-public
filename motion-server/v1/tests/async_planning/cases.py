"""Exercise application acknowledgements through the real owner loop over NATS."""
import time


def cold_results(broker, command, status, lease, fence, sender, boot):
    results = []
    sequence = 0
    def send(kind, accepted, reason, **overrides):
        nonlocal sequence
        sequence += 1
        fields = dict(controller_boot_id=boot, leader_lease_id=lease,
                      leader_fence_epoch=fence, motion_intent_seq=sequence,
                      plan_revision=101, program_id='async-offline-fixture-1')
        # command() already supplies controller_boot_id.
        fields.pop('controller_boot_id')
        fields.update(overrides)
        if fields.get('motion_intent_seq') is None:
            fields.pop('motion_intent_seq')
        override_id = fields.pop('command_id', None)
        before = status()
        msg = command(kind, **fields)
        if override_id is not None:
            msg['command_id'] = override_id
        started = time.monotonic()
        broker.emit(msg)
        end = started + 1
        current = {}
        while time.monotonic() < end:
            current = status()
            if current.get('latest_processed_cold_command_id') == msg['command_id']:
                break
            time.sleep(.005)
        assert current.get('latest_processed_cold_command_id') == msg['command_id'], (kind, reason, current)
        assert current['latest_processed_cold_command_kind'] == kind
        assert current['latest_processed_cold_accepted'] is accepted, current
        assert current['latest_processed_cold_result'] == reason, current
        if accepted:
            assert current['latest_applied_cold_command_id'] == msg['command_id']
        else:
            assert {k: v for k, v in current.items() if k.startswith('latest_applied_cold_')} == {
                k: v for k, v in before.items() if k.startswith('latest_applied_cold_')}
        results.append(dict(kind=kind, accepted=accepted, result=reason,
                            latency_ms=(time.monotonic()-started)*1000))
        return msg, current

    send('stop', True, 'stop_zero_hold_sent')
    send('play', False, 'fresh_exact_leader_lease_required', leader_lease_id='wrong')
    send('play', False, 'ordered_motion_intent_sequence_required', motion_intent_seq=None)
    send('play', False, 'plan_revision_not_server_installed', plan_revision=999)
    send('plan_preload', False, 'exact_target_accepted_plan_reference_required', accepted_plan={'plan_id': 'missing'})
    send('play', False, 'cold_motion_identity_exceeds_v4_packet_field', command_id='x'*97)
    _, active = send('play', True, 'playback_started')
    assert active['plan_playback_active'] and active['plan_abort_reason'] == 'none'
    _, rejected = send('play', False, 'fresh_exact_leader_lease_required', leader_lease_id='wrong-again')
    assert rejected['plan_playback_active'] and rejected['plan_abort_reason'] == 'none'
    _, rejected = send('play', False, 'stale_motion_intent_sequence', motion_intent_seq=1)
    assert rejected['plan_playback_active'] and rejected['plan_abort_reason'] == 'none'
    send('play', True, 'playback_already_active')
    send('pause', True, 'playback_paused')
    send('pause', True, 'playback_already_paused_or_inactive')
    send('move_to_row', False, 'requested_row_not_in_server_plan', row_index=999)
    send('play', True, 'playback_resumed')
    msg, stopped = send('stop', True, 'stop_zero_hold_sent')
    assert not stopped['plan_playback_active']
    count = stopped['plan_stop_commands']
    broker.emit(msg)  # Existing at-least-once dedupe must not reapply it.
    time.sleep(.05)
    assert status()['plan_stop_commands'] == count
    return dict(cases=results, duplicate_not_reapplied=True,
                rejected_preserves_latest_applied=True,
                rejected_during_active_run_preserves_health=True)
