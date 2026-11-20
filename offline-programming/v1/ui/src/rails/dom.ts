// @ts-nocheck — verbatim port from weld_planner/v1/ui/src; that tree
// typechecks it under its own strict tsconfig (and scalar-function is
// pinned against python/weld_programmer/scalar.py by tests). Checking
// it again under this tree's noUncheckedIndexedAccess would force a
// fork full of `!`s that could silently drift from the original.
// Ported verbatim from weld_planner/v1/ui/src/dom.ts (the two trees
// typecheck under different tsconfigs, so rails carries its own copy).
/**
 * A twenty-line element builder, replacing `innerHTML = <template literal>`.
 *
 * Not a framework and not cosmetics. The old render path interpolated
 * server-supplied strings -- joint types, labels, filenames -- straight into
 * HTML, and the lists are about to get much bigger and carry operator-typed
 * text as well. `textContent` cannot be made to execute anything; a template
 * literal always can.
 *
 * The second reason is that panes now hold onto their elements. Rebuilding a
 * whole subtree from a string throws away focus, scroll position and the
 * caret, which an editor with thirty fields cannot afford.
 */

type Child = Node | string | number | null | undefined | false;

export interface Attrs {
  class?: string;
  id?: string;
  title?: string;
  hidden?: boolean;
  disabled?: boolean;
  /** `data-*`, written as `{ selected: "true" }`. */
  data?: Record<string, string | number | boolean | null | undefined>;
  /** Inline `on*` handlers, written without the prefix: `{ click: fn }`. */
  on?: Record<string, (event: Event) => void>;
  /** Anything else -- `type`, `value`, `min`, `step`, `for`, ... */
  [key: string]: unknown;
}

/** `el("li", {class: "seam"}, "1.5 mm")`. Children may be nested arrays. */
export function el<K extends keyof HTMLElementTagNameMap>(
  tag: K,
  attrs: Attrs = {},
  ...children: (Child | Child[])[]
): HTMLElementTagNameMap[K] {
  const node = document.createElement(tag);
  for (const [key, value] of Object.entries(attrs)) {
    if (value === undefined || value === null || value === false) continue;
    if (key === "data") {
      for (const [name, item] of Object.entries(value as Record<string, unknown>)) {
        if (item !== undefined && item !== null) node.dataset[name] = String(item);
      }
    } else if (key === "on") {
      for (const [name, handler] of Object.entries(value as Record<string, EventListener>)) {
        node.addEventListener(name, handler);
      }
    } else if (key === "class") {
      node.className = String(value);
    } else if (key in node) {
      (node as unknown as Record<string, unknown>)[key] = value;
    } else {
      node.setAttribute(key, String(value));
    }
  }
  append(node, children);
  return node;
}

export function append(node: HTMLElement, children: (Child | Child[])[]): void {
  for (const child of children.flat()) {
    if (child === null || child === undefined || child === false) continue;
    node.append(child instanceof Node ? child : document.createTextNode(String(child)));
  }
}

/** Replace a host's children in one go. */
export function replace(host: HTMLElement, ...children: (Child | Child[])[]): void {
  host.replaceChildren();
  append(host, children);
}

export const $ = <T extends HTMLElement>(id: string): T =>
  document.getElementById(id) as T;

/** Millimetres, the unit the UI shows everywhere. Geometry is metres on the
 *  wire and converted here, per SEAM-MODEL R6. */
export const mm = (metres: number, digits = 2): string =>
  `${(metres * 1000).toFixed(digits)} mm`;
