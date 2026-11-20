import { Widget, element } from "./widget.ts";
import type { ControlNotice } from "../execution/controlNotice.ts";

/**
 * "Another controller is using the robot" (execution/controlNotice.ts). Unlike
 * the maintenance overlay it blocks nothing: the server refuses the other
 * client's control requests itself, and this only says who has the robot. It
 * sits at the bottom of the window with no scrim, so the title bar's STOP
 * stays visible and pressable above it, and it carries its own Stop for the
 * same robot whenever this server can stop it.
 *
 * Wait collapses it to a slim bar until the situation changes; it disappears
 * when the robot is free. Take control appears only when the server said it
 * would accept it (a disarmed, idle holder).
 */
export interface ControlPopupOptions {
  onWait: () => void;
  onExpand: () => void;
  onStop: () => void;
  onTake: () => void;
}

let popupSequence = 0;

export class ControlPopup extends Widget<HTMLDivElement> {
  readonly #card: HTMLDivElement;
  readonly #kicker: HTMLParagraphElement;
  readonly #title: HTMLHeadingElement;
  readonly #who: HTMLParagraphElement;
  readonly #what: HTMLParagraphElement;
  readonly #since: HTMLParagraphElement;
  readonly #note: HTMLParagraphElement;
  readonly #stopNote: HTMLParagraphElement;
  readonly #wait: HTMLButtonElement;
  readonly #stop: HTMLButtonElement;
  readonly #take: HTMLButtonElement;
  readonly #bar: HTMLDivElement;
  readonly #summary: HTMLSpanElement;
  readonly #details: HTMLButtonElement;
  #taking = false;

  constructor(options: ControlPopupOptions) {
    super(element("div", "control-popup"));
    const id = `control-popup-${++popupSequence}`;
    this.el.hidden = true;
    this.el.setAttribute("role", "dialog");
    this.el.setAttribute("aria-modal", "false");
    this.el.setAttribute("aria-live", "assertive");
    this.el.setAttribute("aria-labelledby", `${id}-title`);
    this.#card = element("div", "control-card");
    this.#kicker = element("p", "control-kicker", "ROBOT IN USE");
    this.#title = element("h2", "control-title", "Another controller is using the robot");
    this.#title.id = `${id}-title`;
    this.#who = element("p", "control-who");
    this.#what = element("p", "control-what");
    this.#since = element("p", "control-since");
    this.#note = element("p", "control-note");
    this.#stopNote = element("p", "control-stop-note");
    const actions = element("div", "control-actions");
    this.#wait = element("button", "mini-button control-wait", "Wait");
    this.#wait.type = "button";
    this.#wait.title = "Hide this until the situation changes; it disappears when the robot is free.";
    this.#take = element("button", "mini-button control-take", "Take control");
    this.#take.type = "button";
    this.#take.title = "The other controller is disarmed and idle: make this tab the controller. Nothing moves.";
    this.#stop = element("button", "control-stop", "■ Stop robot");
    this.#stop.type = "button";
    this.#stop.title = "Stop the robot (any operator may). Stop also disarms it and releases control.";
    actions.append(this.#wait, this.#take, this.#stop);
    this.#card.append(this.#kicker, this.#title, this.#who, this.#what, this.#since, this.#note, this.#stopNote, actions);
    this.#bar = element("div", "control-bar");
    this.#summary = element("span", "control-summary");
    this.#details = element("button", "mini-button control-details", "Details");
    this.#details.type = "button";
    this.#bar.append(this.#summary, this.#details);
    this.el.append(this.#card, this.#bar);
    this.listen(this.#wait, "click", () => options.onWait());
    this.listen(this.#details, "click", () => options.onExpand());
    this.listen(this.#stop, "click", () => options.onStop());
    this.listen(this.#take, "click", () => {
      if (this.#take.disabled) return;
      options.onTake();
    });
  }

  /** Take control is in flight: its button waits for the answer. */
  set taking(value: boolean) {
    this.#taking = value;
    this.#take.disabled = value;
    this.#take.textContent = value ? "Taking control…" : "Take control";
  }

  get taking(): boolean {
    return this.#taking;
  }

  render(notice: ControlNotice): void {
    if (!notice.active) {
      this.el.hidden = true;
      return;
    }
    this.el.classList.toggle("is-armed", notice.kicker.includes("ARMED"));
    this.el.classList.toggle("is-collapsed", !notice.expanded);
    setText(this.#kicker, notice.kicker);
    setText(this.#title, notice.title);
    setText(this.#who, notice.who);
    setText(this.#what, notice.what);
    setText(this.#since, notice.since);
    this.#since.hidden = !notice.since;
    setText(this.#note, notice.note);
    this.#note.hidden = !notice.note;
    setText(this.#stopNote, notice.stopNote);
    this.#stopNote.hidden = !notice.stopNote;
    this.#stop.hidden = !notice.stopAvailable;
    this.#take.hidden = !notice.canTake;
    setText(this.#summary, notice.summary);
    this.#card.hidden = !notice.expanded;
    this.#bar.hidden = notice.expanded;
    this.el.hidden = false;
  }
}

function setText(node: HTMLElement, value: string): void {
  if (node.textContent !== value) node.textContent = value;
}
