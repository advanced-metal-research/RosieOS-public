import { type PerspectiveCamera } from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

/** Keep the workstation's orbit gestures, recovering when the browser loses
 * a release event so a stale drag cannot capture the next interaction. */
export class WorkstationOrbitControls extends OrbitControls {
  private readonly pane: HTMLElement;
  private readonly ownerWindow: Window | null;
  private readonly pointers = new Map<number, string>();
  private readonly beginPointer = (event: PointerEvent): void => {
    if (!this.enabled) return;
    // A new mouse press after a missed release must start a fresh gesture.
    if (this.pointers.has(event.pointerId)) this.cancelPointer(event.pointerId);
    this.pointers.set(event.pointerId, event.pointerType);
  };
  private readonly endPointer = (event: PointerEvent): void => {
    this.pointers.delete(event.pointerId);
  };
  private readonly lostCapture = (event: PointerEvent): void => {
    this.cancelPointer(event.pointerId);
  };
  private readonly checkButtons = (event: PointerEvent): void => {
    if (event.pointerType !== "touch" && event.buttons === 0) this.cancelPointer(event.pointerId);
  };
  private readonly preventNativeDrag = (event: DragEvent): void => {
    const target = event.target as Element | null;
    if (target?.nodeType === 1 && target.closest('input, textarea, [contenteditable="true"]')) return;
    // Selected labels or links can otherwise start HTML drag/drop after a
    // few pixels: the prohibited-drop cursor replaces orbiting and the
    // browser takes over the pointer stream. The viewport owns that drag.
    event.preventDefault();
  };
  private readonly cancelAll = (): void => {
    for (const id of [...this.pointers.keys()]) this.cancelPointer(id);
  };

  constructor(camera: PerspectiveCamera, pane: HTMLElement) {
    super(camera, pane);
    this.pane = pane;
    this.ownerWindow = pane.ownerDocument.defaultView;
    this.enableDamping = true;
    this.dampingFactor = 0.08;
    pane.addEventListener("dragstart", this.preventNativeDrag, true);
    pane.addEventListener("pointerdown", this.beginPointer, true);
    pane.addEventListener("pointerup", this.endPointer, true);
    pane.addEventListener("pointercancel", this.endPointer, true);
    pane.addEventListener("lostpointercapture", this.lostCapture, true);
    pane.addEventListener("pointermove", this.checkButtons, true);
    this.ownerWindow?.addEventListener("blur", this.cancelAll);
  }

  get interacting(): boolean { return this.pointers.size > 0; }

  private cancelPointer(id: number): void {
    const pointerType = this.pointers.get(id);
    if (pointerType === undefined) return;
    // Clear first: cancellation releases capture, which may synchronously
    // send lostpointercapture back here. Native Orbit cancellation resets
    // its pointer list/state and emits end; pointerup could commit a pick.
    this.pointers.delete(id);
    const EventClass = this.pane.ownerDocument.defaultView?.PointerEvent ?? PointerEvent;
    const ownRelease = Object.getOwnPropertyDescriptor(this.pane, "releasePointerCapture");
    const release = this.pane.releasePointerCapture;
    // Orbit releases capture before clearing its state. The pointer can
    // already be inactive after blur/lostcapture, so that browser call must
    // not abort native cancellation with NotFoundError. Scope the guard to
    // this synchronous dispatch and restore the element method immediately.
    this.pane.releasePointerCapture = (pointerId: number): void => {
      if (!this.pane.hasPointerCapture(pointerId)) return;
      try { release.call(this.pane, pointerId); }
      catch (error) {
        if (!(error instanceof Error) || error.name !== "NotFoundError") throw error;
      }
    };
    try {
      this.pane.dispatchEvent(new EventClass("pointercancel", { pointerId: id, pointerType, bubbles: true }));
    } finally {
      if (ownRelease) Object.defineProperty(this.pane, "releasePointerCapture", ownRelease);
      else Reflect.deleteProperty(this.pane, "releasePointerCapture");
    }
  }

  override dispose(): void {
    this.cancelAll();
    this.pane.removeEventListener("dragstart", this.preventNativeDrag, true);
    this.pane.removeEventListener("pointerdown", this.beginPointer, true);
    this.pane.removeEventListener("pointerup", this.endPointer, true);
    this.pane.removeEventListener("pointercancel", this.endPointer, true);
    this.pane.removeEventListener("lostpointercapture", this.lostCapture, true);
    this.pane.removeEventListener("pointermove", this.checkButtons, true);
    this.ownerWindow?.removeEventListener("blur", this.cancelAll);
    super.dispose();
  }
}
