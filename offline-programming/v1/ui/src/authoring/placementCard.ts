import { NumberField } from "../ui/fields.ts";
import { ToolCard } from "../ui/tool-card.ts";

/**
 * The Move part tool's dialog: a ToolCard of six NumberFields, the part's
 * offsets and rotations, floating in the viewport while the placement
 * triad is on -- the way a CAD transform feature shows its dialog beside
 * the handles.
 *
 * It is a second view of the same placement the Part panel shows: values
 * come in through `setValues` on every render and edits go out through
 * `onChange`, so the card never holds a copy of the project. ✓ keeps the
 * placement and closes the tool. ✕ restores the placement the tool opened
 * with -- one gesture to undo a whole session of drags.
 *
 * The fields carry no axis colours: the numbers are in the part's own
 * axes, which after a rotation are not the world's red/green/blue.
 */
export type PlacementField = "x" | "y" | "z" | "rx" | "ry" | "rz";

export type PlacementValues = Record<PlacementField, number>;

export interface PlacementCardCallbacks {
  onChange: (field: PlacementField, value: number) => void;
  /** Put the part at the middle of the work frame. */
  onCentre: () => void;
  onAccept: () => void;
  onCancel: () => void;
}

const FIELDS: ReadonlyArray<{ field: PlacementField; label: string; unit: string }> = [
  { field: "x", label: "X", unit: "mm" },
  { field: "y", label: "Y", unit: "mm" },
  { field: "z", label: "Z", unit: "mm" },
  { field: "rx", label: "Rotate X", unit: "°" },
  { field: "ry", label: "Rotate Y", unit: "°" },
  { field: "rz", label: "Rotate Z", unit: "°" },
];

export class PlacementCard extends ToolCard {
  readonly #fields = new Map<PlacementField, NumberField>();

  constructor(callbacks: PlacementCardCallbacks) {
    super({
      title: "Move part",
      className: "placement-card",
      onAccept: callbacks.onAccept,
      onCancel: callbacks.onCancel,
      acceptTitle: "Keep this placement and close the tool",
      cancelTitle: "Put the part back where it was when the tool opened",
    });
    this.addTool("Centre", callbacks.onCentre, "Put the part at the middle of the work frame");
    const groups: Array<[string, typeof FIELDS]> = [
      ["Position", FIELDS.slice(0, 3)],
      ["Rotation", FIELDS.slice(3)],
    ];
    for (const [title, group] of groups) {
      const section = this.addSection({ title });
      for (const { field, label, unit } of group) {
        const number = new NumberField({
          label,
          unit,
          onChange: (value) => callbacks.onChange(field, value),
          onEscape: callbacks.onCancel,
        });
        this.add(number, section);
        this.#fields.set(field, number);
      }
    }
  }

  /** Push the project's placement in; a field being edited keeps its text. */
  setValues(values: PlacementValues, format: (value: number) => string): void {
    for (const [field, number] of this.#fields) number.setValue(format(values[field]));
  }
}
