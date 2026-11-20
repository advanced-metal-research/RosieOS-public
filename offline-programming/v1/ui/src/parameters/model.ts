/**
 * A parameter set: current values, the preset they are being compared
 * against, and which fields have moved off it.
 *
 * This is the headless half of the parameter windows -- no DOM, so the rules
 * that matter can be asserted directly. `view.ts` draws it and `library.ts`
 * stores the presets.
 *
 * ## What a preset is here
 *
 * A starting point and a COMPARISON, never a reference the values resolve
 * through later. `rails/presets.ts` says why and this keeps the property: a
 * set hands out values, and whatever consumes them -- a request to the seam
 * worker, a node's `search_provenance` -- records those values. Editing a
 * preset afterwards therefore cannot reach back and change what a weld was
 * searched with. The preset id in here is for the window's own use: it is how
 * the window knows what "revert" means, and nothing downstream sees it.
 *
 * So "modified" is a comparison against the LOADED preset, not a dirty flag
 * against what was saved. A field sitting on the preset's number reads as
 * unmodified even if the operator typed it there by hand, because the
 * question the window answers is "how does this differ from the preset", and
 * a value equal to the preset's differs from it in no way worth a marker.
 *
 * ## Out of range is refused, not clamped
 *
 * A number typed past a field's bound comes back as a refusal with a reason,
 * and the previous value stands. Clamping would leave the operator believing
 * they set one number while the request carried another, which for a welding
 * parameter is the difference between a procedure and a guess.
 */

export type ParameterValue = number | string | boolean;
export type ParameterValues = Readonly<Record<string, ParameterValue>>;

interface FieldBase {
  key: string;
  label: string;
  /** Fields sharing a group are drawn together under its name. */
  group?: string;
  /** One sentence: what the number does, in the operator's terms. Shown as
   *  the field's title, so it is read on hover and by a screen reader. */
  help?: string;
}

export interface NumberField extends FieldBase {
  kind: "number";
  /** As shown -- "°", "mm", "mm/°". The value is in this unit; nothing here
   *  converts, because a window that displayed one unit and stored another
   *  would put the conversion between the operator and the number. */
  unit?: string;
  min?: number;
  max?: number;
  step?: number;
  /** Decimals in the readout. Defaults to what `step` implies. */
  digits?: number;
}

export interface ChoiceField extends FieldBase {
  kind: "choice";
  options: readonly { value: string; label: string }[];
}

export interface ToggleField extends FieldBase {
  kind: "toggle";
}

export type ParameterField = NumberField | ChoiceField | ToggleField;
export type ParameterSchema = readonly ParameterField[];

/** A named set of values for one schema.
 *
 *  `shipped` presets are the ones the code ships with: they are the reference
 *  every window opens against, so they are read-only. Overwriting one would
 *  destroy the thing "revert to preset" means. */
export interface ParameterPreset {
  id: string;
  label: string;
  origin: "shipped" | "user";
  values: ParameterValues;
  updatedAt?: string;
}

export interface FieldState {
  field: ParameterField;
  value: ParameterValue;
  /** The loaded preset's value for this field. */
  presetValue: ParameterValue;
  /** Whether the two differ. */
  modified: boolean;
}

export type SetOutcome =
  | { ok: true }
  /** The value was not applied and the old one stands. `reason` is written
   *  for the operator, not the log. */
  | { ok: false; reason: string };

/** A rule no single field can state.
 *
 *  A torch whose barrel is shorter than the standoff it is held at is two
 *  perfectly valid numbers and one impossible gun, and per-field bounds
 *  cannot say so. Returns the problems in the operator's words, empty when
 *  the set is coherent. Advisory by design: the values are still held and
 *  still shown, because refusing the first edit of a pair would make the
 *  second one unreachable. */
export type ParameterConstraints = (values: Record<string, ParameterValue>) => string[];

export interface ParameterSetOptions {
  /** Opening values. Missing keys come from the preset. */
  initial?: ParameterValues;
  constraints?: ParameterConstraints;
}

export interface ParameterSet {
  readonly schema: ParameterSchema;
  /** The preset the values are being compared against. */
  readonly preset: ParameterPreset;
  /** Whether anything differs from it. */
  readonly modified: boolean;
  /** The keys that differ, in schema order. */
  modifiedKeys(): string[];
  values(): Record<string, ParameterValue>;
  get(key: string): ParameterValue;
  state(key: string): FieldState;
  states(): FieldState[];
  set(key: string, value: ParameterValue): SetOutcome;
  /** One field back to the preset's value. */
  revert(key: string): void;
  /** Every field back to the preset's values. */
  revertAll(): void;
  /** Compare against a different preset, taking its values. */
  loadPreset(preset: ParameterPreset): void;
  /** What is wrong across fields, if anything. Empty when coherent. */
  problems(): string[];
  /** The values were just written INTO a preset (Save, or Save as). The
   *  values do not change; what changes is what they are compared against,
   *  so every field reads as unmodified again. */
  rebase(preset: ParameterPreset): void;
}

/** Fill in what a preset does not mention, so a schema that gained a field
 *  does not read as a hole. A preset missing a key is not an error: a saved
 *  preset predates every field added after it. */
export function completeValues(
  schema: ParameterSchema,
  values: ParameterValues,
  fallback: ParameterValues,
): Record<string, ParameterValue> {
  const complete: Record<string, ParameterValue> = {};
  for (const field of schema) {
    const value = values[field.key] ?? fallback[field.key];
    complete[field.key] = value ?? defaultFor(field);
  }
  return complete;
}

function defaultFor(field: ParameterField): ParameterValue {
  if (field.kind === "toggle") return false;
  if (field.kind === "choice") return field.options[0]?.value ?? "";
  return field.min ?? 0;
}

/** Check one value against its field. Exported because the view refuses a
 *  bad entry at the input and the set refuses it again on the way in; the
 *  two must not be able to disagree. */
export function validate(field: ParameterField, value: ParameterValue): SetOutcome {
  if (field.kind === "toggle") {
    return typeof value === "boolean" ? { ok: true } : { ok: false, reason: "expects on or off" };
  }
  if (field.kind === "choice") {
    return field.options.some((option) => option.value === value)
      ? { ok: true }
      : { ok: false, reason: `${String(value)} is not one of the choices` };
  }
  if (typeof value !== "number" || !Number.isFinite(value)) {
    return { ok: false, reason: "needs a number" };
  }
  const unit = field.unit ? ` ${field.unit}` : "";
  if (field.min !== undefined && value < field.min) {
    return { ok: false, reason: `below the minimum, ${field.min}${unit}` };
  }
  if (field.max !== undefined && value > field.max) {
    return { ok: false, reason: `above the maximum, ${field.max}${unit}` };
  }
  return { ok: true };
}

export function createParameterSet(
  schema: ParameterSchema,
  preset: ParameterPreset,
  options: ParameterSetOptions = {},
): ParameterSet {
  const byKey = new Map(schema.map((field) => [field.key, field]));
  const constraints = options.constraints ?? (() => []);
  let loaded = preset;
  let baseline = completeValues(schema, preset.values, {});
  let current = completeValues(schema, options.initial ?? preset.values, baseline);

  const field = (key: string): ParameterField => {
    const found = byKey.get(key);
    if (!found) throw new Error(`no parameter ${key} in this schema`);
    return found;
  };

  const state = (key: string): FieldState => ({
    field: field(key),
    value: current[key]!,
    presetValue: baseline[key]!,
    modified: current[key] !== baseline[key],
  });

  return {
    schema,
    get preset() {
      return loaded;
    },
    get modified() {
      return schema.some((entry) => current[entry.key] !== baseline[entry.key]);
    },
    modifiedKeys: () =>
      schema.filter((entry) => current[entry.key] !== baseline[entry.key]).map((entry) => entry.key),
    values: () => ({ ...current }),
    problems: () => constraints({ ...current }),
    get: (key) => state(key).value,
    state,
    states: () => schema.map((entry) => state(entry.key)),
    set(key, value) {
      const outcome = validate(field(key), value);
      if (!outcome.ok) return outcome;
      current[key] = value;
      return { ok: true };
    },
    revert(key) {
      field(key);
      current[key] = baseline[key]!;
    },
    revertAll() {
      current = { ...baseline };
    },
    loadPreset(next) {
      loaded = next;
      baseline = completeValues(schema, next.values, {});
      current = { ...baseline };
    },
    rebase(next) {
      loaded = next;
      baseline = completeValues(schema, next.values, current);
      // Deliberately not `{...baseline}`: rebase means "these values are now
      // the preset", so a field the new preset does not mention keeps what
      // the operator has rather than snapping to a default.
      current = { ...current };
    },
  };
}
