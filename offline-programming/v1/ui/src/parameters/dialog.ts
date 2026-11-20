/**
 * A parameter window as a modal, which is where a sheet belongs.
 *
 * These first lived folded into the authoring panel, which is about 290 px
 * wide -- and a grid whose `Preset` column is off the right edge cannot be
 * read down, which is the only reason it is a grid. A modal gives the sheet
 * the width it needs and gives the panel back to the welds, which is what
 * the operator opened it for.
 *
 * The view is moved in ONCE, at construction, and never re-parented. It
 * keeps its state -- the loaded preset, the edits, the caret -- across every
 * open and close, so a dialog reopened is the sheet as it was left. Closing
 * is not a cancel: an edit is applied when it is committed, exactly as it
 * was inline, and there is no separate OK to forget to press.
 */

import { el } from "../rails/dom.ts";
import type { ParameterView } from "./view.ts";

export interface ParameterDialogOptions {
  /** What this set of parameters is, as a heading. */
  title: string;
  /** One line under it: what these numbers reach, and when they take effect. */
  description?: string;
  view: ParameterView;
}

export interface ParameterDialog {
  readonly element: HTMLDialogElement;
  open(): void;
  close(): void;
  readonly isOpen: boolean;
}

export function createParameterDialog(options: ParameterDialogOptions): ParameterDialog {
  const element = document.createElement("dialog");
  element.className = "rails-dialog parameter-dialog";
  // Clicking the backdrop closes it. The dialog element itself is the
  // backdrop's hit target; a click on the body stops here.
  element.addEventListener("click", (event) => {
    if (event.target === element) element.close();
  });

  const close = el("button", {
    type: "button", class: "mini-button", on: { click: () => element.close() },
  }, "Close");

  element.append(el(
    "div",
    { class: "rails-dialog-body parameter-dialog-body" },
    el("h2", {}, options.title),
    options.description ? el("p", { class: "rails-dialog-stats" }, options.description) : null,
    options.view.root,
    el("div", { class: "rails-dialog-actions" }, close),
  ));

  return {
    element,
    get isOpen() {
      return element.open;
    },
    open() {
      if (!element.open) element.showModal();
      // The sheet is what the operator came to read, not the Close button.
      options.view.refresh();
    },
    close() {
      if (element.open) element.close();
    },
  };
}
