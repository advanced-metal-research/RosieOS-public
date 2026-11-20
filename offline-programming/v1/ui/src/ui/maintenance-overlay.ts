import { Widget, element } from "./widget.ts";
import type { MaintenanceDisplay } from "../execution/maintenance.ts";
import type { MaintenanceSessionLine } from "../execution/maintenanceSessions.ts";

/**
 * The full-screen notice while the cell is being updated. Everything under it
 * is `inert` and every input that could start something -- a press, a click, a
 * key, the wheel -- is stopped before the app sees it. There is no dismiss:
 * it goes when the server says the update is over.
 *
 * Two things are never blocked. Releases (pointerup, pointercancel, keyup,
 * blur) still reach the app, so a jog the operator was holding when the notice
 * appeared ends on release exactly as before. And the overlay carries its own
 * STOP, because the title bar's STOP is underneath it; the server accepts
 * Stop, Disarm and Disconnect throughout an update.
 */
export interface MaintenanceOverlayOptions {
  onStop?: () => void;
  /** Show STOP only while a cell is connected (the title bar's rule). */
  stopAvailable?: () => boolean;
  onReload?: () => void;
  /** The reload suggestion was declined. */
  onDismissReload?: () => void;
}

/** Inputs that start something. Releases are deliberately absent. */
const BLOCKED_EVENTS = [
  "pointerdown", "mousedown", "touchstart", "click", "dblclick", "auxclick", "contextmenu",
  "wheel", "keydown", "keypress", "dragstart", "drop", "submit",
] as const;

let overlaySequence = 0;

export class MaintenanceOverlay extends Widget<HTMLDivElement> {
  #options: MaintenanceOverlayOptions;
  readonly #card: HTMLDivElement;
  readonly #kicker: HTMLParagraphElement;
  readonly #title: HTMLHeadingElement;
  readonly #reason: HTMLParagraphElement;
  readonly #phase: HTMLParagraphElement;
  readonly #step: HTMLParagraphElement;
  readonly #suite: HTMLParagraphElement;
  readonly #progress: HTMLDivElement;
  readonly #progressFill: HTMLDivElement;
  readonly #progressText: HTMLSpanElement;
  readonly #steps: HTMLOListElement;
  readonly #countdownLabel: HTMLSpanElement;
  readonly #countdown: HTMLSpanElement;
  readonly #elapsed: HTMLParagraphElement;
  readonly #source: HTMLParagraphElement;
  readonly #notes: HTMLUListElement;
  readonly #connected: HTMLDivElement;
  readonly #connectedList: HTMLUListElement;
  readonly #stop: HTMLButtonElement;
  readonly #reload: HTMLButtonElement;
  readonly #later: HTMLButtonElement;
  #blocking = false;
  #inerted: Element[] = [];
  #observer: MutationObserver | null = null;
  readonly #guard = (event: Event) => this.#intercept(event);
  readonly #focusGuard = (event: Event) => {
    if (this.#blocking && !this.el.contains(event.target as Node | null)) this.#card.focus();
  };

  constructor(options: MaintenanceOverlayOptions = {}) {
    super(element("div", "maintenance-overlay"));
    this.#options = options;
    const id = `maintenance-${++overlaySequence}`;
    this.el.hidden = true;
    this.el.setAttribute("role", "alertdialog");
    this.el.setAttribute("aria-modal", "true");
    this.el.setAttribute("aria-labelledby", `${id}-title`);
    this.el.setAttribute("aria-describedby", `${id}-reason`);
    this.#card = element("div", "maintenance-card");
    this.#card.tabIndex = -1;
    this.#kicker = element("p", "maintenance-kicker", "MAINTENANCE");
    this.#title = element("h2", "maintenance-title", "Update in progress");
    this.#title.id = `${id}-title`;
    this.#reason = element("p", "maintenance-reason");
    this.#reason.id = `${id}-reason`;
    this.#phase = element("p", "maintenance-phase");
    // What the run is doing: its step, the gate's suite, a bar weighted by
    // estimated time and the steps with their times (notice progress).
    this.#step = element("p", "maintenance-step");
    this.#suite = element("p", "maintenance-suite");
    this.#progress = element("div", "maintenance-progress");
    this.#progress.setAttribute("role", "progressbar");
    this.#progress.setAttribute("aria-valuemin", "0");
    this.#progress.setAttribute("aria-valuemax", "100");
    this.#progress.setAttribute("aria-label", "Update progress");
    this.#progressFill = element("div", "maintenance-progress-fill");
    this.#progressText = element("span", "maintenance-progress-text");
    this.#progress.append(this.#progressFill, this.#progressText);
    this.#steps = element("ol", "maintenance-steps");
    const clock = element("div", "maintenance-clock");
    clock.setAttribute("role", "timer");
    this.#countdownLabel = element("span", "maintenance-countdown-label");
    this.#countdown = element("span", "maintenance-countdown");
    clock.append(this.#countdownLabel, this.#countdown);
    this.#elapsed = element("p", "maintenance-elapsed");
    this.#source = element("p", "maintenance-source");
    this.#notes = element("ul", "maintenance-notes");
    this.#notes.setAttribute("aria-live", "polite");
    // Who is connected: whoever holds the session or has the machine armed
    // first, so the people to ask are named before anyone else.
    this.#connected = element("div", "maintenance-connected");
    this.#connectedList = element("ul", "maintenance-connected-list");
    this.#connected.append(element("p", "maintenance-connected-title", "CONNECTED"), this.#connectedList);
    this.#connected.hidden = true;
    const actions = element("div", "maintenance-actions");
    this.#stop = element("button", "maintenance-stop", "■ STOP");
    this.#stop.type = "button";
    this.#stop.title = "Stop the connected robot. Stop also disarms and releases control.";
    this.#reload = element("button", "mini-button maintenance-reload", "Reload now");
    this.#reload.type = "button";
    this.#later = element("button", "mini-button maintenance-later", "Later");
    this.#later.type = "button";
    actions.append(this.#stop, this.#reload, this.#later);
    this.#card.append(this.#kicker, this.#title, this.#reason, this.#step, this.#suite, this.#progress, this.#phase, clock, this.#elapsed,
      this.#steps, this.#source, this.#notes, this.#connected, actions);
    this.el.append(this.#card);
    this.listen(this.#stop, "click", () => this.#options.onStop?.());
    this.listen(this.#reload, "click", () => this.#options.onReload?.());
    this.listen(this.#later, "click", () => {
      this.el.hidden = true;
      this.#options.onDismissReload?.();
    });
    // Clicks inside the overlay reach its buttons, then stop here: the app's
    // own document-level listeners (menus closing, viewport picks) never run.
    for (const type of BLOCKED_EVENTS) this.listen(this.el, type as keyof HTMLElementEventMap, (event) => event.stopPropagation());
    this.own(() => this.#release());
  }

  get blocking(): boolean {
    return this.#blocking;
  }

  /** STOP's target, bound once the machine panel exists: the overlay is up
   *  before the app's boot waits on the server, so it cannot depend on it. */
  bindStop(onStop: () => void, stopAvailable: () => boolean): void {
    this.#options = { ...this.#options, onStop, stopAvailable };
  }

  /** Show `display`: the blocking notice, the reload suggestion, or nothing.
   *  `sessions` lists who is connected while the notice is up. */
  render(display: MaintenanceDisplay, sessions: MaintenanceSessionLine[] = []): void {
    if (display.blocking) {
      this.#renderNotice(display);
      this.#renderSessions(sessions);
      this.#block();
      return;
    }
    this.#release();
    if (display.reloadSuggested) {
      this.#renderFinished(display);
      return;
    }
    this.el.hidden = true;
  }

  #renderNotice(display: MaintenanceDisplay): void {
    this.el.classList.add("is-blocking");
    this.el.classList.remove("is-finished");
    this.el.classList.toggle("is-overrun", display.overrun);
    this.el.classList.toggle("is-unreachable", display.unreachable);
    this.el.classList.toggle("is-stale", display.stale);
    this.el.setAttribute("role", "alertdialog");
    this.#kicker.textContent = display.unreachable ? "MAINTENANCE · SERVER NOT ANSWERING" : "MAINTENANCE";
    setText(this.#title, display.title);
    setText(this.#reason, display.reason);
    setText(this.#phase, display.phase ? `Now: ${display.phase}` : "");
    this.#phase.hidden = !display.phase;
    this.#renderProgress(display);
    setText(this.#countdownLabel, display.countdownLabel);
    setText(this.#countdown, display.countdown);
    this.#countdown.hidden = !display.countdown;
    setText(this.#elapsed, display.elapsed);
    this.#elapsed.hidden = !display.elapsed;
    setText(this.#source, display.source);
    this.#source.hidden = !display.source;
    const notes = display.notes.join("\n");
    if (this.#notes.dataset.text !== notes) {
      this.#notes.dataset.text = notes;
      this.#notes.replaceChildren(...display.notes.map((note) => element("li", "", note)));
    }
    this.#stop.hidden = !(this.#options.stopAvailable?.() ?? false);
    this.#reload.hidden = true;
    this.#later.hidden = true;
    this.el.hidden = false;
  }

  #renderProgress(display: MaintenanceDisplay): void {
    setText(this.#step, display.stepLine);
    this.#step.hidden = !display.stepLine;
    setText(this.#suite, display.suiteLine);
    this.#suite.hidden = !display.suiteLine;
    const shown = display.progress !== null;
    this.#progress.hidden = !shown;
    if (shown) {
      const fraction = display.progress ?? 0;
      this.#progressFill.style.width = `${Math.round(fraction * 1000) / 10}%`;
      this.#progress.setAttribute("aria-valuenow", String(Math.floor(fraction * 100)));
      setText(this.#progressText, display.progressText);
    }
    const key = display.steps.map((step) => `${step.state}|${step.label}|${step.time}`).join("\n");
    if (this.#steps.dataset.text !== key) {
      this.#steps.dataset.text = key;
      this.#steps.replaceChildren(...display.steps.map((step) => {
        const item = element("li", `maintenance-step-row is-${step.state}`);
        item.append(element("span", "maintenance-step-mark", step.mark), element("span", "maintenance-step-label", step.label),
          element("span", "maintenance-step-time", step.time));
        if (step.state === "running") item.setAttribute("aria-current", "step");
        return item;
      }));
    }
    this.#steps.hidden = display.steps.length === 0;
  }

  #renderSessions(sessions: MaintenanceSessionLine[]): void {
    const ordered = [...sessions.filter((line) => line.holds), ...sessions.filter((line) => !line.holds)];
    const key = ordered.map((line) => `${line.holds ? "!" : ""}${line.kind}:${line.text}`).join("\n");
    if (this.#connectedList.dataset.text !== key) {
      this.#connectedList.dataset.text = key;
      this.#connectedList.replaceChildren(...ordered.map((line) => {
        const item = element("li", `maintenance-connected-${line.kind}${line.holds ? " is-holder" : ""}`, line.text);
        if (line.holds) item.setAttribute("aria-label", `Holds the machine: ${line.text}`);
        return item;
      }));
    }
    this.#connected.hidden = ordered.length === 0;
  }

  #renderFinished(display: MaintenanceDisplay): void {
    this.el.classList.remove("is-blocking", "is-overrun", "is-unreachable", "is-stale");
    this.el.classList.add("is-finished");
    this.el.setAttribute("role", "dialog");
    this.#kicker.textContent = "MAINTENANCE";
    this.#title.textContent = "Update finished";
    this.#reason.textContent = `The OLP server was updated${display.buildBefore && display.buildAfter ? ` (${display.buildBefore} → ${display.buildAfter})` : ""}. Reload to use the new version; programs stay in this browser.`;
    for (const hidden of [this.#phase, this.#step, this.#suite, this.#progress, this.#steps, this.#countdown, this.#elapsed, this.#source, this.#stop, this.#connected]) hidden.hidden = true;
    this.#countdownLabel.textContent = "";
    this.#notes.replaceChildren();
    this.#notes.dataset.text = "";
    this.#reload.hidden = false;
    this.#later.hidden = false;
    this.el.hidden = false;
  }

  #block(): void {
    if (this.#blocking) return;
    this.#blocking = true;
    const doc = this.el.ownerDocument;
    const view = doc.defaultView;
    for (const type of BLOCKED_EVENTS) view?.addEventListener(type, this.#guard, { capture: true, passive: false });
    view?.addEventListener("focusin", this.#focusGuard, true);
    this.#inertSiblings();
    const ObserverClass = view && (view as unknown as { MutationObserver?: typeof MutationObserver }).MutationObserver;
    if (ObserverClass && doc.body) {
      // Dialogs opened after the notice (a planning progress popup) go under it too.
      this.#observer = new ObserverClass(() => this.#inertSiblings());
      this.#observer.observe(doc.body, { childList: true });
    }
    this.#card.focus();
  }

  #release(): void {
    if (!this.#blocking) return;
    this.#blocking = false;
    const view = this.el.ownerDocument.defaultView;
    for (const type of BLOCKED_EVENTS) view?.removeEventListener(type, this.#guard, { capture: true });
    view?.removeEventListener("focusin", this.#focusGuard, true);
    this.#observer?.disconnect();
    this.#observer = null;
    for (const node of this.#inerted) {
      node.removeAttribute("inert");
      node.removeAttribute("aria-hidden");
    }
    this.#inerted = [];
    this.el.classList.remove("is-blocking");
  }

  #inertSiblings(): void {
    const body = this.el.ownerDocument.body;
    if (!body) return;
    for (const child of Array.from(body.children)) {
      if (child === this.el || child.hasAttribute("inert") || child.tagName === "SCRIPT") continue;
      child.setAttribute("inert", "");
      child.setAttribute("aria-hidden", "true");
      this.#inerted.push(child);
    }
  }

  #intercept(event: Event): void {
    if (!this.#blocking) return;
    const inside = this.el.contains(event.target as Node | null);
    if (event.type === "keydown" || event.type === "keypress") {
      // No app shortcut runs while blocked; Tab moves between the overlay's
      // own controls and Enter/Space press them. Escape dismisses nothing.
      event.stopImmediatePropagation();
      const key = (event as KeyboardEvent).key;
      const onButton = inside && (event.target as Element | null)?.tagName === "BUTTON";
      if (key !== "Tab" && !(onButton && (key === "Enter" || key === " "))) event.preventDefault();
      return;
    }
    if (inside) return;
    if (event.cancelable) event.preventDefault();
    event.stopImmediatePropagation();
  }
}

function setText(node: HTMLElement, value: string): void {
  // Unchanged text is left alone: replacing it every tick would restart
  // screen-reader announcements and cancel a press on the lettering.
  if (node.textContent !== value) node.textContent = value;
}
