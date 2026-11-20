import { ToolCard, type ToolCardOptions } from "../ui/tool-card.ts";
import { element } from "../ui/widget.ts";

/**
 * A tool card for a tool that works ON the part -- Rails welds, Waypoint
 * welds: the cell fades while it is open, so the card carries the two view
 * controls that go with that, "Centre camera" in the tool row and the
 * transparency slider at the bottom. Right is see-through.
 */
export interface PartFocusCardOptions extends ToolCardOptions {
  /** Swing the camera back onto the part. */
  onCentreCamera: () => void;
  /** How faint the rest of the cell is drawn: opacity 1 solid, 0 gone. */
  onTransparency: (opacity: number) => void;
}

export class PartFocusCard extends ToolCard {
  readonly #transparency: HTMLInputElement;

  constructor(options: PartFocusCardOptions) {
    super(options);
    this.el.classList.add("part-focus-card");
    this.addTool("Centre camera", options.onCentreCamera, "Frame the part and orbit about its centre");
    const row = element("label", "tool-card-slider");
    row.append(element("span", "tool-card-slider-label", "Transparency"));
    this.#transparency = element("input");
    this.#transparency.type = "range";
    this.#transparency.min = "0";
    this.#transparency.max = "100";
    this.#transparency.step = "1";
    this.#transparency.title = "How faint everything but the part is drawn";
    this.listen(this.#transparency, "input", () => {
      options.onTransparency(1 - Number(this.#transparency.value) / 100);
    });
    row.append(this.#transparency);
    this.footer.append(row);
  }

  /** Show the current level: `opacity` of the rest of the cell, 1 solid. */
  setTransparency(opacity: number): void {
    this.#transparency.value = String(Math.round((1 - opacity) * 100));
  }
}
