// Code generated from testdata/operator_feedback_kinds.json; DO NOT EDIT.
// Unverified display policy: unknown clears on acceptance of any listed kind;
// known refusals clear only on matching acceptance. Unlisted acceptance fails closed.
export const operatorKinds: Readonly<Record<string, boolean>> = {
  "jog": false,
  "program_start": false,
  "prepare_program": false,
  "arm": false,
  "home": false,
  "enable": false,
  "reset": false,
  "stop": false,
  "unknown": true,
};

export function operatorAcceptanceClears(refused: string, accepted: string): boolean {
  return Object.hasOwn(operatorKinds, refused) && Object.hasOwn(operatorKinds, accepted) && (operatorKinds[refused] || refused === accepted);
}
