import { Widget, element } from "./widget.ts";

/**
 * Form fields for tool dialogs.
 *
 * A `NumberField` is one labelled number with a unit: "X  [ 1139.6 ] mm".
 * It reports a finished edit (change, or Enter) through `onChange` and is
 * otherwise a view: `setValue` pushes the model's number in, except while
 * the operator is typing in it, so a render pass never eats a half-typed
 * value. The label can take an axis accent (X red, Y green, Z blue) to
 * match the part triad and the axis indicator.
 */
export type AxisAccent = "x" | "y" | "z";

export interface NumberFieldOptions {
  label: string;
  unit?: string;
  accent?: AxisAccent | null;
  step?: number | "any";
  min?: number;
  max?: number;
  onChange: (value: number) => void;
  /** Escape in the field; the dialog usually maps it to cancel. */
  onEscape?: () => void;
}

export class NumberField extends Widget<HTMLLabelElement> {
  readonly input: HTMLInputElement;

  constructor(options: NumberFieldOptions) {
    super(element("label", "number-field"));
    if (options.accent) this.el.dataset.accent = options.accent;
    this.el.append(element("span", "number-field-label", options.label));
    this.input = element("input");
    this.input.type = "number";
    this.input.step = String(options.step ?? 1);
    if (options.min !== undefined) this.input.min = String(options.min);
    if (options.max !== undefined) this.input.max = String(options.max);
    this.input.inputMode = "decimal";
    this.input.setAttribute("aria-label", options.label);
    this.listen(this.input, "change", () => {
      const value = Number(this.input.value);
      if (Number.isFinite(value)) options.onChange(value);
    });
    this.listen(this.input, "keydown", (event) => {
      if (event.key === "Enter") this.input.blur();
      if (event.key === "Escape" && options.onEscape) {
        event.preventDefault();
        event.stopPropagation();
        options.onEscape();
      }
    });
    this.el.append(this.input);
    if (options.unit) this.el.append(element("span", "number-field-unit", options.unit));
  }

  /** True while the operator is editing this field. */
  get editing(): boolean {
    return document.activeElement === this.input;
  }

  /** Push a value in; a field being edited keeps its text. */
  setValue(text: string): void {
    if (this.editing) return;
    this.input.value = text;
  }

  setEnabled(enabled: boolean): void {
    this.input.disabled = !enabled;
  }
}
