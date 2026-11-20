import { Widget, element } from "../ui/widget.ts";

/**
 * The left panel's lower half: what the selected tree node is, read
 * mostly, with a way into the tool that edits it.
 *
 * Deliberately not an editor. Editing happens in the tool cards over the
 * viewport (Move part, Rails welds, Weld presets), one editor per thing;
 * the inspector shows the summary and offers the "Edit…" that opens the
 * card. The exceptions are the Program name and the Workspace's cell
 * settings, which have no tool of their own and whose existing fields are
 * hosted here as they are.
 *
 * Each kind of selection has a pane; `show(kind)` reveals one. Panes are
 * plain hosts so the app can move its own elements in (the robot and
 * planner sections keep their handlers when re-parented).
 */
export type InspectorKind = "program" | "workspace" | "part" | "fixture" | "weld" | "none";

export class Inspector extends Widget<HTMLDivElement> {
  readonly panes: Record<InspectorKind, HTMLDivElement>;
  readonly #title: HTMLHeadingElement;
  readonly #kicker: HTMLParagraphElement;

  constructor() {
    super(element("div", "inspector"));
    this.el.setAttribute("aria-label", "Selection");
    const heading = element("div", "inspector-heading");
    this.#kicker = element("p", "eyebrow", "SELECTION");
    this.#title = element("h3", "", "Nothing selected");
    heading.append(this.#kicker, this.#title);
    this.el.append(heading);
    const body = element("div", "inspector-body");
    this.el.append(body);
    const pane = (kind: InspectorKind): HTMLDivElement => {
      const host = element("div", "inspector-pane");
      host.dataset.inspectorPane = kind;
      host.hidden = true;
      body.append(host);
      return host;
    };
    this.panes = {
      program: pane("program"),
      workspace: pane("workspace"),
      part: pane("part"),
      fixture: pane("fixture"),
      weld: pane("weld"),
      none: pane("none"),
    };
    this.panes.none.append(element("p", "inspector-empty", "Select something in the tree."));
  }

  show(kind: InspectorKind, title: string, kicker = "SELECTION"): void {
    this.#kicker.textContent = kicker;
    this.#title.textContent = title;
    for (const [name, host] of Object.entries(this.panes)) host.hidden = name !== kind;
  }
}

/** A label/value line for a summary pane. */
export function summaryRow(label: string, value: string): HTMLElement {
  const row = element("div", "inspector-row");
  row.append(element("span", "inspector-row-label", label), element("span", "inspector-row-value", value));
  return row;
}
