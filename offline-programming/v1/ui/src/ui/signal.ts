/**
 * A typed signal: the one piece of Qt's object model the DOM does not
 * already provide. `EventTarget` carries untyped `Event`s and needs a DOM
 * node; a `Signal<T>` carries a value of one type and works in a plain
 * object, so an `Action` can announce a state change to a toolbar button,
 * a menu item and a shortcut without any of them knowing the others.
 *
 * Slots run in connection order. A slot that throws does not stop the
 * others -- one broken view must not silence the rest -- and the error is
 * rethrown after the emit so it is not swallowed either.
 */
export type Slot<T> = (value: T) => void;

export class Signal<T = void> {
  private readonly slots = new Set<Slot<T>>();

  /** Connect a slot; the return value disconnects it. */
  connect(slot: Slot<T>): () => void {
    this.slots.add(slot);
    return () => this.disconnect(slot);
  }

  disconnect(slot: Slot<T>): void {
    this.slots.delete(slot);
  }

  get connections(): number {
    return this.slots.size;
  }

  emit(value: T): void {
    let failure: unknown = null;
    for (const slot of [...this.slots]) {
      try {
        slot(value);
      } catch (error) {
        failure ??= error;
      }
    }
    if (failure !== null) throw failure;
  }

  clear(): void {
    this.slots.clear();
  }
}
