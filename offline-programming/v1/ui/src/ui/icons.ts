import { createElement, type IconNode } from "lucide";

/**
 * Lucide icons as inline SVG. Inline so the icon is in the bundle, not a
 * CDN fetch, and colours with `currentColor` like text does. Every icon is
 * decorative -- the button next to it carries the label -- so it is hidden
 * from assistive tech.
 */
export function iconElement(node: IconNode, size = 18): SVGElement {
  const svg = createElement(node, {
    width: size,
    height: size,
    "stroke-width": 1.75,
    "aria-hidden": "true",
    focusable: "false",
  });
  svg.classList.add("ui-icon");
  return svg;
}
