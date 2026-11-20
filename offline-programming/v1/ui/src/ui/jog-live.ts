import { Widget, element } from "./widget.ts";
import type { JogLiveView } from "../execution/jogLive.ts";

/**
 * The jog pad's live line during a Cartesian path hold (execution/jogLive.ts):
 * speed against the speed asked for, the server's "limited by …" sentence,
 * and the singularity proximity meter while the resolver's slowing band is
 * near. Display only: it has no controls and never takes focus.
 */
export class JogLiveReadout extends Widget<HTMLDivElement> {
  readonly #kicker: HTMLSpanElement;
  readonly #speed: HTMLSpanElement;
  readonly #speedBar: HTMLSpanElement;
  readonly #text: HTMLParagraphElement;
  readonly #meter: HTMLDivElement;
  readonly #meterFill: HTMLSpanElement;
  readonly #meterLabel: HTMLSpanElement;

  constructor() {
    super(element("div", "jog-live"));
    this.el.hidden = true;
    this.el.setAttribute("role", "status");
    this.el.setAttribute("aria-live", "polite");
    const head = element("div", "jog-live-head");
    this.#kicker = element("span", "jog-live-kicker");
    this.#speed = element("span", "jog-live-speed");
    head.append(this.#kicker, this.#speed);
    const bar = element("span", "jog-live-speedbar");
    this.#speedBar = element("span", "jog-live-speedbar-fill");
    bar.append(this.#speedBar);
    this.#text = element("p", "jog-live-text");
    this.#meter = element("div", "jog-live-meter");
    this.#meter.setAttribute("role", "meter");
    this.#meter.setAttribute("aria-label", "Singularity proximity");
    this.#meter.setAttribute("aria-valuemin", "0");
    this.#meter.setAttribute("aria-valuemax", "100");
    const track = element("span", "jog-live-meter-track");
    this.#meterFill = element("span", "jog-live-meter-fill");
    track.append(this.#meterFill);
    this.#meterLabel = element("span", "jog-live-meter-label");
    this.#meter.append(element("span", "jog-live-meter-name", "SINGULARITY"), track, this.#meterLabel);
    this.el.append(head, bar, this.#text, this.#meter);
  }

  render(view: JogLiveView): void {
    if (!view.visible) {
      this.el.hidden = true;
      return;
    }
    this.el.dataset.tone = view.tone;
    this.el.classList.toggle("is-stopped", !view.active);
    setText(this.#kicker, view.kicker);
    setText(this.#speed, view.speed);
    this.#speedBar.parentElement!.hidden = !view.active;
    this.#speedBar.style.width = `${(view.speedFraction * 100).toFixed(1)}%`;
    setText(this.#text, view.text);
    this.#text.hidden = !view.text;
    this.#meter.hidden = !view.meter.visible;
    const pct = Math.round(view.meter.closeness * 100);
    this.#meter.setAttribute("aria-valuenow", String(pct));
    this.#meterFill.style.width = `${pct}%`;
    setText(this.#meterLabel, view.meter.label);
    this.el.hidden = false;
  }
}

function setText(node: HTMLElement, value: string): void {
  if (node.textContent !== value) node.textContent = value;
}
