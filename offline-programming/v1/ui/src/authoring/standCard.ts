import { STAND_ID, type WorkcellFixture, type WorkcellFixtureField } from "../domain/workcell.ts";
import { NumberField } from "../ui/fields.ts";
import { ToolCard } from "../ui/tool-card.ts";

/**
 * The Stand tool's dialog: the fixture's size (a box), position and
 * rotation as NumberFields, floating over the viewport like the Move part
 * card -- a second view of the fixture the Workspace tree row and the
 * inspector show. Values come in through `setFixture` on every render and
 * edits go out through `onChange`, so the card never holds a copy of the
 * project. ✓ keeps the fixture and closes; ✕ restores the fixture the card
 * opened with.
 *
 * "Replace with STEP" hands the real fixture's file in: the box becomes the
 * tessellated part, placed by its own origin, and the size rows fold away.
 * "Cell default" puts the planner's placeholder back. "Remove" deletes the
 * fixture: the planner then clears nothing there, which is a decision the
 * operator makes knowing the bench.
 */
export interface StandCardCallbacks {
  onChange: (field: WorkcellFixtureField, value: number) => void;
  onReplaceWithStep: () => void;
  onCellDefault: () => void;
  onRemove: () => void;
  onAccept: () => void;
  onCancel: () => void;
}

const SIZE: ReadonlyArray<{ field: WorkcellFixtureField; label: string }> = [
  { field: "sx", label: "Length X" },
  { field: "sy", label: "Width Y" },
  { field: "sz", label: "Height Z" },
];
const POSITION: ReadonlyArray<{ field: WorkcellFixtureField; label: string }> = [
  { field: "x", label: "X" },
  { field: "y", label: "Y" },
  { field: "z", label: "Z" },
];
const ROTATION: ReadonlyArray<{ field: WorkcellFixtureField; label: string }> = [
  { field: "rx", label: "Rotate X" },
  { field: "ry", label: "Rotate Y" },
  { field: "rz", label: "Rotate Z" },
];

export class StandCard extends ToolCard {
  readonly #fields = new Map<WorkcellFixtureField, NumberField>();
  readonly #size: HTMLDivElement;
  readonly #cellDefault: HTMLButtonElement;

  constructor(callbacks: StandCardCallbacks) {
    super({
      title: "Fixture",
      className: "placement-card",
      onAccept: callbacks.onAccept,
      onCancel: callbacks.onCancel,
      acceptTitle: "Keep this fixture and close the tool",
      cancelTitle: "Put the fixture back as it was when the tool opened",
    });
    this.addTool("Replace with STEP", callbacks.onReplaceWithStep, "Replace this fixture's geometry with a STEP file");
    this.#cellDefault = this.addTool("Cell default", () => {
      if (!this.#cellDefault.disabled) callbacks.onCellDefault();
    }, "Put the cell's placeholder stand back");
    this.#cellDefault.hidden = true;
    this.#cellDefault.disabled = true;
    this.addTool("Remove", callbacks.onRemove, "Remove this fixture from the workspace and collision checking");
    this.#size = this.addSection({ title: "Size" });
    this.#fieldsFor(SIZE, "mm", this.#size, callbacks);
    this.#fieldsFor(POSITION, "mm", this.addSection({ title: "Position" }), callbacks);
    this.#fieldsFor(ROTATION, "°", this.addSection({ title: "Rotation" }), callbacks);
  }

  #fieldsFor(
    rows: ReadonlyArray<{ field: WorkcellFixtureField; label: string }>,
    unit: string,
    section: HTMLDivElement,
    callbacks: StandCardCallbacks,
  ): void {
    for (const { field, label } of rows) {
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

  /** Push the project's fixture in; a field being edited keeps its text. */
  setFixture(fixture: WorkcellFixture): void {
    this.setTitle(fixture.name || "Fixture");
    this.#cellDefault.hidden = fixture.id !== STAND_ID;
    this.#cellDefault.disabled = fixture.id !== STAND_ID;
    const shape = fixture.shape;
    // addSection returns the body; hide its surrounding collapsible section too.
    this.#size.parentElement!.hidden = shape.kind !== "box";
    this.setNote(shape.kind === "box"
      ? "A box standing on its footprint: position is the middle of its base in the cell frame."
      : `${shape.filename}: the fixture's STEP, placed by its own origin in the cell frame.`);
    const mm = (v: number) => (v * 1000).toFixed(1);
    const deg = (v: number) => (v * 180 / Math.PI).toFixed(1);
    if (shape.kind === "box") {
      const [sx, sy, sz] = shape.sizeM;
      this.#fields.get("sx")?.setValue(mm(sx));
      this.#fields.get("sy")?.setValue(mm(sy));
      this.#fields.get("sz")?.setValue(mm(sz));
    }
    const [x, y, z] = fixture.translationM;
    this.#fields.get("x")?.setValue(mm(x));
    this.#fields.get("y")?.setValue(mm(y));
    this.#fields.get("z")?.setValue(mm(z));
    const [rx, ry, rz] = fixture.rotationRpyRad;
    this.#fields.get("rx")?.setValue(deg(rx));
    this.#fields.get("ry")?.setValue(deg(ry));
    this.#fields.get("rz")?.setValue(deg(rz));
  }
}
