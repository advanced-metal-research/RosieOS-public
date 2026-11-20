/**
 * The base of every UI class here: one DOM element, a place to register
 * what must be undone when the widget goes away, and mount/unmount.
 *
 * Deliberately thin. The DOM already is a widget tree -- an element has a
 * parent, children, visibility, focus, events -- so this adds only what a
 * plain element lacks: ownership of listeners and signal connections, so
 * that disposing a toolbar disconnects its buttons from their actions
 * instead of leaving them updating a detached element forever.
 */
export class Widget<E extends HTMLElement = HTMLElement> {
  readonly el: E;
  #disposers: Array<() => void> = [];
  #disposed = false;

  constructor(el: E) {
    this.el = el;
  }

  get disposed(): boolean {
    return this.#disposed;
  }

  /** Register something to undo on dispose (a signal disconnect, a timer). */
  own(disposer: () => void): void {
    if (this.#disposed) {
      disposer();
      return;
    }
    this.#disposers.push(disposer);
  }

  /** addEventListener whose removal is owned by the widget. */
  listen<K extends keyof HTMLElementEventMap>(
    target: HTMLElement,
    type: K,
    handler: (event: HTMLElementEventMap[K]) => void,
    options?: AddEventListenerOptions,
  ): void;
  listen<K extends keyof WindowEventMap>(
    target: Window,
    type: K,
    handler: (event: WindowEventMap[K]) => void,
    options?: AddEventListenerOptions,
  ): void;
  listen<K extends keyof DocumentEventMap>(
    target: Document,
    type: K,
    handler: (event: DocumentEventMap[K]) => void,
    options?: AddEventListenerOptions,
  ): void;
  listen(
    target: EventTarget,
    type: string,
    handler: (event: never) => void,
    options?: AddEventListenerOptions,
  ): void {
    const listener = handler as EventListener;
    target.addEventListener(type, listener, options);
    this.own(() => target.removeEventListener(type, listener, options));
  }

  mount(host: HTMLElement | Widget, before: Node | null = null): this {
    const parent = host instanceof Widget ? host.el : host;
    parent.insertBefore(this.el, before);
    return this;
  }

  unmount(): void {
    this.el.parentNode?.removeChild(this.el);
  }

  setVisible(visible: boolean): void {
    this.el.hidden = !visible;
  }

  get visible(): boolean {
    return !this.el.hidden;
  }

  /** Undo every owned listener and connection, then leave the DOM. */
  dispose(): void {
    if (this.#disposed) return;
    this.#disposed = true;
    for (const disposer of this.#disposers.splice(0).reverse()) disposer();
    this.unmount();
  }
}

/** Create an element with a class list, in one expression. */
export function element<K extends keyof HTMLElementTagNameMap>(
  tag: K,
  className = "",
  text = "",
): HTMLElementTagNameMap[K] {
  const el = document.createElement(tag);
  if (className) el.className = className;
  if (text) el.textContent = text;
  return el;
}
