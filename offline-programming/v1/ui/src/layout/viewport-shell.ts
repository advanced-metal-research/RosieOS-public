export interface ViewportShellSizingOptions {
  documentElement?: HTMLElement;
  shell: HTMLElement;
  window?: Window;
}

/**
 * Keep the desktop shell tied to the browser's actual layout viewport.
 *
 * Chromium can expose its pre-maximise dynamic viewport for the first layout
 * pass. A bare 100dvh then remains visibly short until the user resizes the
 * window. The root element's ResizeObserver and the short startup settling
 * sequence cover that browser transition without requiring user interaction.
 */
export function installViewportShellSizing(
  options: ViewportShellSizingOptions,
): () => void {
  const hostWindow = options.window ?? window;
  const documentElement = options.documentElement ?? document.documentElement;
  const { shell } = options;
  const animationFrames = new Set<number>();
  const timers = new Set<number>();

  const sync = (): void => {
    const height = viewportShellHeight(documentElement.clientHeight);
    shell.style.setProperty("--app-shell-height", `${height}px`);
  };

  const settle = (): void => {
    sync();
    const first = hostWindow.requestAnimationFrame(() => {
      animationFrames.delete(first);
      sync();
      const second = hostWindow.requestAnimationFrame(() => {
        animationFrames.delete(second);
        sync();
      });
      animationFrames.add(second);
    });
    animationFrames.add(first);
  };

  const delayedSettle = (): void => {
    settle();
  };
  for (const delayMs of [100, 300, 750]) {
    const timer = hostWindow.setTimeout(() => {
      timers.delete(timer);
      delayedSettle();
    }, delayMs);
    timers.add(timer);
  }

  hostWindow.addEventListener("resize", settle);
  hostWindow.visualViewport?.addEventListener("resize", settle);
  const resizeObserver = typeof ResizeObserver === "function"
    ? new ResizeObserver(settle)
    : null;
  resizeObserver?.observe(documentElement);
  settle();

  return () => {
    hostWindow.removeEventListener("resize", settle);
    hostWindow.visualViewport?.removeEventListener("resize", settle);
    resizeObserver?.disconnect();
    animationFrames.forEach((frame) => hostWindow.cancelAnimationFrame(frame));
    timers.forEach((timer) => hostWindow.clearTimeout(timer));
    animationFrames.clear();
    timers.clear();
  };
}

export function viewportShellHeight(clientHeight: number): number {
  if (!Number.isFinite(clientHeight)) return 1;
  return Math.max(1, Math.round(clientHeight));
}
