/**
 * The preset library for one parameter window: what it ships with, what the
 * operator has saved, and the rules about which of those can be written.
 *
 * Scoped by name -- `seam.search`, `weld.process`, `planner.freespace` -- so
 * one store holds every window's presets without them seeing each other's.
 * The scope is part of the key, not a filter applied afterwards, because two
 * schemas with a field of the same name are still two different parameters.
 *
 * ## Shipped presets are read-only
 *
 * A shipped preset is the reference every window opens against and the thing
 * "revert to preset" reverts TO. Overwriting one would remove the only fixed
 * point in the window, so Save refuses on a shipped preset and says to use
 * Save as. This is not a permissions rule; it is what makes revert mean
 * something.
 *
 * ## Values, never a reference
 *
 * Saving a preset copies the values in. Loading one copies them out. Nothing
 * downstream ever holds a preset id, so editing a preset changes what the
 * NEXT window opens with and nothing that has already been authored --
 * `rails/presets.ts` states the same rule for the same reason.
 */

import type { ParameterPreset, ParameterSchema, ParameterValues } from "./model.ts";
import { completeValues } from "./model.ts";

/** Where user presets live between sessions. A port, so the model is
 *  testable without IndexedDB and so a window can be given a store that
 *  keeps nothing. */
export interface PresetStore {
  list(scope: string): Promise<ParameterPreset[]>;
  put(scope: string, preset: ParameterPreset): Promise<void>;
}

/** A store that forgets. Used when persistence is unavailable, so a window
 *  still opens and Save as still works for the session rather than the
 *  window failing to draw. */
export function memoryPresetStore(): PresetStore {
  const byScope = new Map<string, Map<string, ParameterPreset>>();
  return {
    async list(scope) {
      return [...(byScope.get(scope)?.values() ?? [])];
    },
    async put(scope, preset) {
      const scoped = byScope.get(scope) ?? new Map();
      scoped.set(preset.id, preset);
      byScope.set(scope, scoped);
    },
  };
}

export type SaveOutcome =
  | { ok: true; preset: ParameterPreset }
  | { ok: false; reason: string };

export interface PresetLibrary {
  readonly scope: string;
  /** Shipped first, then the operator's by label. Shipped first because the
   *  reference belongs at the top of the list, not wherever the alphabet
   *  puts it. */
  all(): ParameterPreset[];
  get(id: string): ParameterPreset | null;
  /** The one a window opens against when it has no other instruction. */
  readonly fallback: ParameterPreset;
  /** Read the operator's saved presets in. Until this resolves the library
   *  holds only what ships, which is enough to draw. */
  load(): Promise<void>;
  /** Write values over an existing preset. Refuses on a shipped one. */
  save(id: string, values: ParameterValues): Promise<SaveOutcome>;
  /** Write values to a new preset under a name the operator typed. */
  saveAs(label: string, values: ParameterValues): Promise<SaveOutcome>;
}

export interface PresetLibraryOptions {
  scope: string;
  schema: ParameterSchema;
  /** At least one. The first is the fallback. */
  shipped: readonly ParameterPreset[];
  store: PresetStore;
}

export function createPresetLibrary(options: PresetLibraryOptions): PresetLibrary {
  const { scope, schema, store } = options;
  const shipped = options.shipped.map((preset) => ({ ...preset, origin: "shipped" as const }));
  if (shipped.length === 0) throw new Error(`preset scope ${scope} ships nothing to open against`);
  const fallback = shipped[0]!;
  let user: ParameterPreset[] = [];

  const all = (): ParameterPreset[] => [
    ...shipped,
    ...[...user].sort((a, b) => a.label.localeCompare(b.label)),
  ];

  const write = async (preset: ParameterPreset): Promise<SaveOutcome> => {
    try {
      await store.put(scope, preset);
    } catch (error) {
      // Reported, never swallowed: a Save that quietly kept nothing is how an
      // operator loses a procedure they believe is written down.
      return {
        ok: false,
        reason: `could not be saved: ${error instanceof Error ? error.message : String(error)}`,
      };
    }
    user = [...user.filter((entry) => entry.id !== preset.id), preset];
    return { ok: true, preset };
  };

  return {
    scope,
    fallback,
    all,
    get: (id) => all().find((preset) => preset.id === id) ?? null,
    async load() {
      try {
        user = (await store.list(scope)).map((preset) => ({ ...preset, origin: "user" as const }));
      } catch {
        // An unreadable store leaves the shipped presets, which is a working
        // window. Refusing to draw would be worse than losing the saved ones.
        user = [];
      }
    },
    async save(id, values) {
      const existing = all().find((preset) => preset.id === id);
      if (!existing) return { ok: false, reason: `there is no preset ${id} to save over` };
      if (existing.origin === "shipped") {
        return {
          ok: false,
          reason: `"${existing.label}" is a shipped preset and stays as it is — use Save as`,
        };
      }
      return write({
        ...existing,
        values: completeValues(schema, values, existing.values),
        updatedAt: new Date().toISOString(),
      });
    },
    async saveAs(label, values) {
      const name = label.trim();
      if (!name) return { ok: false, reason: "a preset needs a name" };
      const taken = all().find(
        (preset) => preset.label.toLowerCase() === name.toLowerCase(),
      );
      if (taken?.origin === "shipped") {
        return { ok: false, reason: `"${taken.label}" is a shipped preset — pick another name` };
      }
      // Same name as one of the operator's own overwrites it, which is what
      // typing an existing name into Save as means everywhere else.
      const id = taken?.id ?? uniqueId(name, new Set(all().map((preset) => preset.id)));
      return write({
        id,
        label: name,
        origin: "user",
        values: completeValues(schema, values, {}),
        updatedAt: new Date().toISOString(),
      });
    },
  };
}

/** A readable id from a typed name, suffixed until it is free. Readable
 *  because it ends up in a store an operator may one day have to read. */
function uniqueId(label: string, taken: ReadonlySet<string>): string {
  const base = label
    .toLowerCase()
    .replace(/[^a-z0-9]+/g, "_")
    .replace(/^_+|_+$/g, "") || "preset";
  if (!taken.has(base)) return base;
  for (let suffix = 2; suffix < 1000; suffix += 1) {
    const candidate = `${base}_${suffix}`;
    if (!taken.has(candidate)) return candidate;
  }
  return `${base}_${Date.now()}`;
}
