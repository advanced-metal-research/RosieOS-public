export interface ResizableLayoutState {
  authorWidthPx: number;
  programWidthPx: number;
  spreadsheetHeightPx: number;
  cameraColumnPercent: number;
  cameraRowPercent: number;
}

export interface ResizableLayoutElements {
  workspace: HTMLElement;
  bottomWorkspace?: HTMLElement;
  jogResizer?: HTMLElement;
  /** The boundary inside the Jog pane between the Jog column and the
   *  telemetry plot; its share is written to the pane as `--jog-column`. */
  jogColumnResizer?: HTMLElement;
  jogPanel?: HTMLElement;
  stage: HTMLElement;
  authoringResizer: HTMLElement;
  /** Absent since the right-hand program panel was removed; the width is
   *  still stored so an older layout record reads back unchanged. */
  programResizer?: HTMLElement;
  spreadsheetResizer: HTMLElement;
  cameraColumnResizer: HTMLElement;
  cameraRowResizer: HTMLElement;
  /** The least height the dock needs right now (the Jog panel with its
   *  telemetry strip open asks for more than a bare spreadsheet); zero or
   *  absent means the stored height stands. Applied on every layout pass,
   *  never stored: a later drag takes over. */
  dockMinHeight?: () => number;
}

export interface ResizableLayoutHandle {
  /** Re-run the layout pass (after the dock's needs changed). */
  apply(): void;
  dispose(): void;
}

export const DEFAULT_RESIZABLE_LAYOUT: ResizableLayoutState = Object.freeze({
  authorWidthPx: 380,
  programWidthPx: 350,
  spreadsheetHeightPx: 360,
  cameraColumnPercent: 66.667,
  cameraRowPercent: 50,
});

const STORAGE_KEY = "offline-programming.v1.resizable-layout";
/** The Jog column's default share of the Jog pane, in percent. */
export const JOG_COLUMN_SHARE_DEFAULT = 42;

export function installResizableLayout(elements: ResizableLayoutElements): ResizableLayoutHandle {
  let state = readState();
  // Unverified layout default: Jog occupies 40 percent, bounded to 20–80 percent.
  let jogShare = 40;
  try { jogShare = finite(JSON.parse(localStorage.getItem("olp.jog.share") ?? "null"), 40); } catch { /* Optional persistence. */ }
  // Unverified layout default: the Jog column takes 42 percent of its pane
  // (the joints table's natural width beside a square plot at the default
  // dock size), bounded to 25–75 percent; the CSS floor keeps the table
  // usable below that.
  let jogColumnShare = JOG_COLUMN_SHARE_DEFAULT;
  try { jogColumnShare = finite(JSON.parse(localStorage.getItem("olp.jog.column") ?? "null"), JOG_COLUMN_SHARE_DEFAULT); } catch { /* Optional persistence. */ }
  const cleanups: Array<() => void> = [];

  const apply = (): void => {
    state = normalizeResizableLayout(
      state,
      elements.workspace.clientWidth,
      elements.workspace.clientHeight,
    );
    jogShare = clamp(jogShare, 20, 80);
    const dockMin = elements.dockMinHeight?.() ?? 0;
    const dockHeight = dockMin > 0
      ? clamp(Math.max(state.spreadsheetHeightPx, dockMin), 220, Math.max(220, elements.workspace.clientHeight - 330))
      : state.spreadsheetHeightPx;
    elements.bottomWorkspace?.style.setProperty("--jog-share", jogShare + "%");
    elements.jogResizer?.setAttribute("aria-valuenow", String(Math.round(jogShare)));
    elements.jogResizer?.setAttribute("aria-valuemin", "20");
    elements.jogResizer?.setAttribute("aria-valuemax", "80");
    jogColumnShare = clamp(jogColumnShare, 25, 75);
    elements.jogPanel?.style.setProperty("--jog-column", jogColumnShare + "%");
    elements.jogColumnResizer?.setAttribute("aria-valuenow", String(Math.round(jogColumnShare)));
    elements.jogColumnResizer?.setAttribute("aria-valuemin", "25");
    elements.jogColumnResizer?.setAttribute("aria-valuemax", "75");
    elements.workspace.style.setProperty("--author-width", `${state.authorWidthPx}px`);
    elements.workspace.style.setProperty("--program-width", `${state.programWidthPx}px`);
    elements.workspace.style.setProperty(
      "--spreadsheet-height",
      `${dockHeight}px`,
    );
    elements.stage.style.setProperty("--camera-column", `${state.cameraColumnPercent}%`);
    elements.stage.style.setProperty("--camera-row", `${state.cameraRowPercent}%`);
    elements.authoringResizer.setAttribute("aria-valuenow", String(Math.round(state.authorWidthPx)));
    elements.programResizer?.setAttribute("aria-valuenow", String(Math.round(state.programWidthPx)));
    elements.spreadsheetResizer.setAttribute(
      "aria-valuenow",
      String(Math.round(dockHeight)),
    );
    elements.spreadsheetResizer.setAttribute("aria-valuemin", "220");
    elements.spreadsheetResizer.setAttribute(
      "aria-valuemax",
      String(Math.max(220, elements.workspace.clientHeight - 330)),
    );
    elements.cameraColumnResizer.setAttribute(
      "aria-valuenow",
      String(Math.round(state.cameraColumnPercent)),
    );
    elements.cameraRowResizer.setAttribute(
      "aria-valuenow",
      String(Math.round(state.cameraRowPercent)),
    );
  };

  const save = (): void => {
    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify(state));
      if (elements.jogResizer) localStorage.setItem("olp.jog.share", JSON.stringify(jogShare));
      if (elements.jogColumnResizer) localStorage.setItem("olp.jog.column", JSON.stringify(jogColumnShare));
    } catch {
      // Layout persistence is optional; resizing still works in private mode.
    }
  };

  const bindDrag = (
    handle: HTMLElement,
    update: (event: PointerEvent) => void,
    resizingClass = "is-resizing-layout",
  ): void => {
    const pointerDown = (event: PointerEvent): void => {
      if (event.button !== 0) return;
      event.preventDefault();
      handle.setPointerCapture(event.pointerId);
      document.documentElement.classList.add(resizingClass);
      const pointerMove = (moveEvent: PointerEvent): void => {
        update(moveEvent);
        apply();
      };
      const pointerUp = (): void => {
        handle.removeEventListener("pointermove", pointerMove);
        handle.removeEventListener("pointerup", pointerUp);
        handle.removeEventListener("pointercancel", pointerUp);
        document.documentElement.classList.remove(resizingClass);
        save();
      };
      handle.addEventListener("pointermove", pointerMove);
      handle.addEventListener("pointerup", pointerUp);
      handle.addEventListener("pointercancel", pointerUp);
    };
    handle.addEventListener("pointerdown", pointerDown);
    cleanups.push(() => handle.removeEventListener("pointerdown", pointerDown));
  };

  bindDrag(elements.authoringResizer, (event) => {
    const bounds = elements.workspace.getBoundingClientRect();
    state.authorWidthPx = event.clientX - bounds.left;
  });
  if (elements.programResizer) {
    bindDrag(elements.programResizer, (event) => {
      const bounds = elements.workspace.getBoundingClientRect();
      state.programWidthPx = bounds.right - event.clientX;
    });
  }
  bindDrag(elements.spreadsheetResizer, (event) => {
    const bounds = elements.workspace.getBoundingClientRect();
    state.spreadsheetHeightPx = bounds.bottom - event.clientY;
  }, "is-resizing-spreadsheet");
  bindDrag(elements.cameraColumnResizer, (event) => {
    const bounds = elements.stage.getBoundingClientRect();
    state.cameraColumnPercent = ((event.clientX - bounds.left) / bounds.width) * 100;
  });
  bindDrag(elements.cameraRowResizer, (event) => {
    const bounds = elements.stage.getBoundingClientRect();
    state.cameraRowPercent = ((event.clientY - bounds.top) / bounds.height) * 100;
  });

  if (elements.jogResizer && elements.bottomWorkspace) {
    bindDrag(elements.jogResizer, event => {
      const bounds = elements.bottomWorkspace!.getBoundingClientRect();
      if (bounds.width > 0) jogShare = (bounds.right - event.clientX) / bounds.width * 100;
    });
  }
  if (elements.jogColumnResizer && elements.jogPanel) {
    bindDrag(elements.jogColumnResizer, event => {
      const bounds = elements.jogPanel!.getBoundingClientRect();
      if (bounds.width > 0) jogColumnShare = (event.clientX - bounds.left) / bounds.width * 100;
    });
  }

  const bindKeys = (
    handle: HTMLElement,
    update: (key: string) => boolean,
    reset: () => void,
  ): void => {
    const keyDown = (event: KeyboardEvent): void => {
      if (!update(event.key)) return;
      event.preventDefault();
      apply();
      save();
    };
    const doubleClick = (): void => {
      reset();
      apply();
      save();
    };
    handle.addEventListener("keydown", keyDown);
    handle.addEventListener("dblclick", doubleClick);
    cleanups.push(() => {
      handle.removeEventListener("keydown", keyDown);
      handle.removeEventListener("dblclick", doubleClick);
    });
  };

  bindKeys(
    elements.authoringResizer,
    (key) => adjust(key, "ArrowLeft", "ArrowRight", (delta) => {
      state.authorWidthPx += delta * 16;
    }),
    () => {
      state.authorWidthPx = DEFAULT_RESIZABLE_LAYOUT.authorWidthPx;
    },
  );
  if (elements.programResizer) {
    bindKeys(
      elements.programResizer,
      (key) => adjust(key, "ArrowRight", "ArrowLeft", (delta) => {
        state.programWidthPx += delta * 16;
      }),
      () => {
        state.programWidthPx = DEFAULT_RESIZABLE_LAYOUT.programWidthPx;
      },
    );
  }
  bindKeys(
    elements.spreadsheetResizer,
    (key) => adjust(key, "ArrowDown", "ArrowUp", (delta) => {
      state.spreadsheetHeightPx += delta * 16;
    }),
    () => {
      state.spreadsheetHeightPx = DEFAULT_RESIZABLE_LAYOUT.spreadsheetHeightPx;
    },
  );
  bindKeys(
    elements.cameraColumnResizer,
    (key) => adjust(key, "ArrowLeft", "ArrowRight", (delta) => {
      state.cameraColumnPercent += delta;
    }),
    () => {
      state.cameraColumnPercent = DEFAULT_RESIZABLE_LAYOUT.cameraColumnPercent;
    },
  );
  bindKeys(
    elements.cameraRowResizer,
    (key) => adjust(key, "ArrowUp", "ArrowDown", (delta) => {
      state.cameraRowPercent += delta;
    }),
    () => {
      state.cameraRowPercent = DEFAULT_RESIZABLE_LAYOUT.cameraRowPercent;
    },
  );

  if (elements.jogResizer) {
    bindKeys(elements.jogResizer,
      key => adjust(key, "ArrowRight", "ArrowLeft", delta => { jogShare += delta; }),
      () => { jogShare = 40; });
  }
  if (elements.jogColumnResizer) {
    bindKeys(elements.jogColumnResizer,
      key => adjust(key, "ArrowLeft", "ArrowRight", delta => { jogColumnShare += delta; }),
      () => { jogColumnShare = JOG_COLUMN_SHARE_DEFAULT; });
  }
  const resize = (): void => apply();
  window.addEventListener("resize", resize);
  cleanups.push(() => window.removeEventListener("resize", resize));
  apply();

  return {
    apply,
    dispose: () => {
      cleanups.forEach((cleanup) => cleanup());
      document.documentElement.classList.remove("is-resizing-layout");
      document.documentElement.classList.remove("is-resizing-spreadsheet");
    },
  };
}

export function normalizeResizableLayout(
  candidate: ResizableLayoutState,
  workspaceWidth: number,
  workspaceHeight: number,
): ResizableLayoutState {
  const width = Number.isFinite(workspaceWidth) ? workspaceWidth : 0;
  const height = Number.isFinite(workspaceHeight) ? workspaceHeight : 0;
  const sideBudget = Math.max(500, width - 440);
  const authorWidthPx = clamp(candidate.authorWidthPx, 380, Math.min(560, sideBudget - 260));
  const programWidthPx = clamp(
    candidate.programWidthPx,
    260,
    Math.min(560, sideBudget - authorWidthPx),
  );
  return {
    authorWidthPx,
    programWidthPx,
    spreadsheetHeightPx: clamp(candidate.spreadsheetHeightPx, 220, height - 330),
    cameraColumnPercent: clamp(candidate.cameraColumnPercent, 45, 82),
    cameraRowPercent: clamp(candidate.cameraRowPercent, 20, 80),
  };
}

function readState(): ResizableLayoutState {
  try {
    const parsed = JSON.parse(localStorage.getItem(STORAGE_KEY) ?? "") as Partial<ResizableLayoutState>;
    return {
      authorWidthPx: finite(parsed.authorWidthPx, DEFAULT_RESIZABLE_LAYOUT.authorWidthPx),
      programWidthPx: finite(parsed.programWidthPx, DEFAULT_RESIZABLE_LAYOUT.programWidthPx),
      spreadsheetHeightPx: finite(
        parsed.spreadsheetHeightPx,
        DEFAULT_RESIZABLE_LAYOUT.spreadsheetHeightPx,
      ),
      cameraColumnPercent: finite(
        parsed.cameraColumnPercent,
        DEFAULT_RESIZABLE_LAYOUT.cameraColumnPercent,
      ),
      cameraRowPercent: finite(parsed.cameraRowPercent, DEFAULT_RESIZABLE_LAYOUT.cameraRowPercent),
    };
  } catch {
    return { ...DEFAULT_RESIZABLE_LAYOUT };
  }
}

function adjust(
  key: string,
  decreaseKey: string,
  increaseKey: string,
  update: (delta: number) => void,
): boolean {
  if (key === decreaseKey) {
    update(-1);
    return true;
  }
  if (key === increaseKey) {
    update(1);
    return true;
  }
  return false;
}

function finite(value: unknown, fallback: number): number {
  return typeof value === "number" && Number.isFinite(value) ? value : fallback;
}

function clamp(value: number, minimum: number, maximum: number): number {
  return Math.min(Math.max(value, minimum), Math.max(minimum, maximum));
}
