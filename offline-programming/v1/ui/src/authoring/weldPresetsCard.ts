import { ToolCard } from "../ui/tool-card.ts";

/**
 * The Weld presets tool's dialog: the process defaults a CAD seam's
 * approach / weld / retract rows are generated with -- process, speeds,
 * electrical, torch, weave -- as collapsible sections of one card, the
 * same ToolCard the other tools use.
 *
 * The fields themselves are the ones the setup sidebar carried, moved in
 * with their handlers: `adopt` takes every element tagged with a section
 * key and re-parents it under that section. Nothing about how a value is
 * read or written changed; only where the field lives.
 */
export const WELD_PRESET_SECTIONS: ReadonlyArray<{ key: string; title: string }> = [
  { key: "weld.process", title: "Process" },
  { key: "weld.speeds", title: "Speeds" },
  { key: "weld.electrical", title: "Electrical" },
  { key: "weld.torch", title: "Torch" },
  { key: "weld.weave", title: "Weave" },
];

export class WeldPresetsCard extends ToolCard {
  readonly sections = new Map<string, HTMLDivElement>();

  constructor(onClose: () => void) {
    super({ title: "Weld presets", className: "weld-presets-card", onClose });
    for (const [index, { key, title }] of WELD_PRESET_SECTIONS.entries()) {
      this.sections.set(key, this.addSection({ title, key, open: index === 0 }));
    }
  }

  /** Move every `[data-setup-item="<key>"]` under `root` into its section. */
  adopt(root: ParentNode): void {
    for (const [key, host] of this.sections) {
      for (const field of root.querySelectorAll<HTMLElement>(`[data-setup-item="${key}"]`)) {
        host.append(field);
      }
    }
  }
}
