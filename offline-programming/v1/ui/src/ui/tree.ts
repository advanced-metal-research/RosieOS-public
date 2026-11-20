import type { IconNode } from "lucide";

import { iconElement } from "./icons.ts";
import { Signal } from "./signal.ts";
import { Widget, element } from "./widget.ts";

/**
 * A tree of what the document holds -- the left panel of every CAD tool.
 *
 * The tree is a view: `render(nodes)` redraws it from a fresh description
 * and keeps only what a view should remember, which node is selected and
 * which are folded. Selection is reported through `selected`, never
 * decided here; the app owns one selection and this is one of its views.
 *
 * A node can be `muted` -- an empty slot drawn in grey with a hint, the
 * tree's own way to say what to do next ("No part — Import a part") -- and
 * such a node may carry an `action` the hint runs.
 */
export interface TreeNode {
  id: string;
  label: string;
  /** Small text after the label: a count, a state. */
  sublabel?: string;
  /** Show an inline loading circle until this node's work completes. */
  busy?: boolean;
  icon?: IconNode | null;
  children?: TreeNode[];
  /** Drawn grey, as a placeholder; still selectable unless `selectable`
   *  is false. */
  muted?: boolean;
  selectable?: boolean;
  /** A hint link on a muted node, e.g. "Import a part". */
  hint?: { label: string; run: () => void };
  /** Folded by default on first render. */
  collapsed?: boolean;
}

export class TreeView extends Widget<HTMLDivElement> {
  /** Emitted with the chosen node's id when the operator clicks a node. */
  readonly selected = new Signal<string>();
  /** Emitted with the node's id on double-click. */
  readonly activated = new Signal<string>();
  #selectedId: string | null = null;
  readonly #folded = new Set<string>();
  #nodes: TreeNode[] = [];
  #renderKey = "";
  readonly #hintActions = new Map<string, () => void>();

  constructor(label = "Tree") {
    super(element("div", "tree"));
    this.el.setAttribute("role", "tree");
    this.el.setAttribute("aria-label", label);
  }

  get selectedId(): string | null {
    return this.#selectedId;
  }

  render(nodes: TreeNode[]): void {
    this.#nodes = nodes;
    this.#hintActions.clear();
    const rememberHints = (items: TreeNode[]): void => {
      for (const node of items) {
        if (node.hint) this.#hintActions.set(node.id, node.hint.run);
        if (node.children) rememberHints(node.children);
      }
    };
    rememberHints(nodes);
    // Machine status polls can refresh the same tree during a pointer press.
    // Preserve its DOM and focus until visible content actually changes. JSON
    // omits callbacks; the map above keeps their latest behavior independently.
    const key = JSON.stringify(nodes);
    if (key === this.#renderKey) return;
    this.#renderKey = key;
    this.el.replaceChildren();
    for (const node of nodes) this.el.append(this.#renderNode(node, 0));
  }

  /** Mark a node selected (or none). Does not emit `selected`. */
  select(id: string | null): void {
    this.#selectedId = id;
    for (const row of this.el.querySelectorAll<HTMLElement>(".tree-row")) {
      const active = row.dataset.nodeId === id;
      row.classList.toggle("is-selected", active);
      row.setAttribute("aria-selected", String(active));
    }
    if (id) this.#unfoldAncestors(id, this.#nodes, []);
  }

  #unfoldAncestors(id: string, nodes: TreeNode[], path: string[]): boolean {
    for (const node of nodes) {
      if (node.id === id) {
        for (const ancestor of path) this.#setFolded(ancestor, false);
        return true;
      }
      if (node.children && this.#unfoldAncestors(id, node.children, [...path, node.id])) return true;
    }
    return false;
  }

  #setFolded(id: string, folded: boolean): void {
    if (folded) this.#folded.add(id);
    else this.#folded.delete(id);
    const item = this.el.querySelector<HTMLElement>(`.tree-item[data-node-id="${cssEscape(id)}"]`);
    if (!item) return;
    item.classList.toggle("is-folded", folded);
    item.querySelector(".tree-row")?.setAttribute("aria-expanded", String(!folded));
  }

  #renderNode(node: TreeNode, depth: number): HTMLElement {
    const item = element("div", "tree-item");
    item.dataset.nodeId = node.id;
    const hasChildren = (node.children?.length ?? 0) > 0;
    const folded = this.#folded.has(node.id) || (node.collapsed === true && !this.#folded.has(`~${node.id}`));
    if (folded) this.#folded.add(node.id);
    item.classList.toggle("is-folded", folded);

    const row = element("div", "tree-row");
    row.dataset.nodeId = node.id;
    row.setAttribute("role", "treeitem");
    row.style.paddingLeft = `${8 + depth * 14}px`;
    if (hasChildren) row.setAttribute("aria-expanded", String(!folded));
    const selectable = node.selectable !== false;
    if (selectable) row.tabIndex = 0;
    row.classList.toggle("is-muted", node.muted === true);
    row.classList.toggle("is-selected", node.id === this.#selectedId);
    row.setAttribute("aria-selected", String(node.id === this.#selectedId));

    const twisty = element("span", "tree-twisty", hasChildren ? "▸" : "");
    if (hasChildren) {
      this.listen(twisty, "click", (event) => {
        event.stopPropagation();
        this.#setFolded(node.id, !item.classList.contains("is-folded"));
        // An explicit unfold outlives a `collapsed` default.
        this.#folded.add(`~${node.id}`);
      });
    }
    row.append(twisty);
    if (node.icon) row.append(iconElement(node.icon, 14));
    row.append(element("span", "tree-label", node.label));
    if (node.busy) {
      row.setAttribute("aria-busy", "true");
      const spinner = element("span", "tree-spinner");
      spinner.setAttribute("role", "status");
      spinner.setAttribute("aria-label", `Loading ${node.label}`);
      spinner.title = `Loading ${node.label}`;
      row.append(spinner);
    }
    if (node.sublabel) row.append(element("span", "tree-sublabel", node.sublabel));
    if (node.hint) {
      const hint = element("button", "tree-hint", node.hint.label);
      hint.type = "button";
      this.listen(hint, "click", (event) => {
        event.stopPropagation();
        this.#hintActions.get(node.id)?.();
      });
      row.append(hint);
    }
    if (selectable) {
      this.listen(row, "click", () => {
        this.select(node.id);
        this.selected.emit(node.id);
      });
      this.listen(row, "dblclick", () => this.activated.emit(node.id));
      this.listen(row, "keydown", (event) => {
        // Nested hint buttons own Enter/Space and their native activation.
        if (event.target !== row) return;
        if (event.key === "Enter" || event.key === " ") {
          event.preventDefault();
          this.select(node.id);
          this.selected.emit(node.id);
        }
      });
    }
    item.append(row);
    if (hasChildren) {
      const children = element("div", "tree-children");
      children.setAttribute("role", "group");
      for (const child of node.children!) children.append(this.#renderNode(child, depth + 1));
      item.append(children);
    }
    return item;
  }
}

function cssEscape(value: string): string {
  return typeof CSS !== "undefined" && CSS.escape ? CSS.escape(value) : value.replace(/["\\]/g, "\\$&");
}
