/**
 * The project store as a preset store.
 *
 * Presets live beside the projects rather than inside one, because a
 * procedure worked out on one part is meant to be reachable from the next.
 * That also means they are per browser ORIGIN, like everything else in this
 * store -- presets saved on `:5189` are not visible from `:8794`, and
 * "Clear site data" destroys them.
 */

import type { ProjectRepository, StoredPreset } from "../storage/repository.ts";
import type { PresetStore } from "./library.ts";
import type { ParameterPreset } from "./model.ts";

export function repositoryPresetStore(repository: ProjectRepository): PresetStore {
  return {
    async list(scope) {
      return (await repository.listPresets(scope)).map(toPreset);
    },
    async put(scope, preset) {
      await repository.putPreset({
        scope,
        id: preset.id,
        label: preset.label,
        values: { ...preset.values },
        updatedAt: preset.updatedAt ?? new Date().toISOString(),
      });
    },
  };
}

function toPreset(stored: StoredPreset): ParameterPreset {
  return {
    id: stored.id,
    label: stored.label,
    // Everything read back from here is the operator's. What ships is in
    // code, and the library adds it separately.
    origin: "user",
    values: stored.values,
    updatedAt: stored.updatedAt,
  };
}
