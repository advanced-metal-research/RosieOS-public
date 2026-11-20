# Load recovery after encoder reference migration

The LA Rosie 1420 bench reported all six Home references invalid with
`anchor_identity_mismatch`, changed field `anchor.source`. The independent
raw encoder source introduced by the prior correction cannot reuse anchors
recorded against the previous source. Establishing Home remains an explicit
operator operation; this change does not reinterpret old anchors.

The same screenshot retained `RTCore rejected application grant request 4
(reason 3)`. Native reason 3 is `GrantReasonNativeGenerationMismatch`, a stale
authority identity, not a Home reason. Fresh grant acquisition reads the current
native generation; renewal deliberately remains bound to the admitted generation.
No evidence justified weakening that check. The snapshot alone does not establish
which earlier event advanced the generation.

## Changes

- Load checks current native Home validity before acquiring authority or uploading
  program bytes, for both dry-run and normal loads. Native admission remains intact.
- The UI blocks Load while Home is invalid or machine status is stale and provides
  a Review Home requirement button. The button only scrolls/focuses the Home step.
- Encoder-source mismatch and legacy native generation refusal have distinct
  operator explanations. Backend `home_required` has an actionable explanation.

## Validation and delivery

- Full Go denseexec package passed, including all/single-axis reference loss,
  dry-run/normal refusal before acquisition/upload, and explicit Home then Load.
- 65 focused UI tests and TypeScript checking passed.
- OLP API rebuilt and OLP/UI restarted; served UI module contains the new controls.
- Read-only checks before/after restart confirmed the same physical daemon
  incarnation, disabled drives, no active grant/jog, and no active motion.
- No physical Home, Reset, Arm, Load, or motion was issued. Operator Home and
  subsequent physical run acceptance remain outstanding.
