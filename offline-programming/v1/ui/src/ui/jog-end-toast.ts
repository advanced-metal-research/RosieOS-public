import { Widget, element } from "./widget.ts";
import type { JogEndNotice } from "../execution/jogEnd.ts";

/**
 * "Jog stopped" (execution/jogEnd.ts): a small warning toast when a jog or
 * Cartesian hold ends without the operator releasing it. It blocks nothing
 * -- no scrim, no focus taken, so keyboard jog and the title bar's STOP keep
 * working underneath -- and goes by itself after a few seconds. Details opens
 * the technical detail (and holds the toast open); × dismisses it.
 *
 * The client that owned the hold gets the amber toast; any other client a
 * quiet note in the same place.
 */
export interface JogEndToastOptions {
  onDismiss: () => void;
  onDetails: () => void;
}

let toastSequence = 0;

export class JogEndToast extends Widget<HTMLDivElement> {
  readonly #kicker: HTMLParagraphElement;
  readonly #text: HTMLParagraphElement;
  readonly #details: HTMLUListElement;
  readonly #toggle: HTMLButtonElement;
  readonly #dismiss: HTMLButtonElement;

  constructor(options: JogEndToastOptions) {
    super(element("div", "jog-end-toast"));
    const id = `jog-end-toast-${++toastSequence}`;
    this.el.hidden = true;
    this.el.setAttribute("role", "status");
    this.el.setAttribute("aria-live", "polite");
    this.el.setAttribute("aria-labelledby", `${id}-text`);
    const head = element("div", "jog-end-head");
    this.#kicker = element("p", "jog-end-kicker");
    this.#dismiss = element("button", "jog-end-dismiss", "×");
    this.#dismiss.type = "button";
    this.#dismiss.title = "Dismiss";
    this.#dismiss.setAttribute("aria-label", "Dismiss");
    head.append(this.#kicker, this.#dismiss);
    this.#text = element("p", "jog-end-text");
    this.#text.id = `${id}-text`;
    this.#toggle = element("button", "mini-button jog-end-toggle", "Details");
    this.#toggle.type = "button";
    this.#toggle.setAttribute("aria-expanded", "false");
    this.#toggle.setAttribute("aria-controls", `${id}-details`);
    this.#details = element("ul", "jog-end-details");
    this.#details.id = `${id}-details`;
    this.#details.hidden = true;
    this.el.append(head, this.#text, this.#toggle, this.#details);
    // Pressing a button must not leave focus on it: a held arrow key after
    // a click would otherwise drive the button instead of the jog.
    for (const button of [this.#toggle, this.#dismiss]) {
      this.listen(button, "mousedown", (event) => event.preventDefault());
    }
    this.listen(this.#dismiss, "click", () => options.onDismiss());
    this.listen(this.#toggle, "click", () => options.onDetails());
  }

  render(notice: JogEndNotice): void {
    if (!notice.active) {
      this.el.hidden = true;
      return;
    }
    this.el.classList.toggle("is-note", notice.tone === "note");
    this.el.dataset.code = notice.code;
    setText(this.#kicker, notice.kicker);
    setText(this.#text, notice.text);
    this.#toggle.textContent = notice.expanded ? "Hide details" : "Details";
    this.#toggle.setAttribute("aria-expanded", notice.expanded ? "true" : "false");
    this.#details.hidden = !notice.expanded;
    if (notice.expanded) {
      this.#details.replaceChildren(...notice.details.map((line) => element("li", "", line)));
    }
    this.el.hidden = false;
  }
}

function setText(node: HTMLElement, value: string): void {
  if (node.textContent !== value) node.textContent = value;
}
