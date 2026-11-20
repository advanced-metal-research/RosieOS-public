import type { DenseCapabilities, DenseTelemetryWindow } from "./denseClient.ts";
import type { executionTcpErrors } from "./executedPath.ts";

export interface ExecutionCapture {
  schema: "olp-execution-capture-v1";
  modelId: string;
  robotDescriptionSha256: string;
  capturedAt: string;
  capabilities: DenseCapabilities;
  window: DenseTelemetryWindow;
}

/** A capture remains reviewable/exportable after the live ring rolls over. */
export class ExecutionPathPanel {
  readonly host: HTMLElement;
  private capture: ExecutionCapture | null = null;
  private summary: HTMLElement;
  private plot: SVGSVGElement;
  private exportButton: HTMLButtonElement;
  constructor(parent: HTMLElement, onImport: (capture: ExecutionCapture) => Promise<void>) {
    this.host = document.createElement("details");
    this.host.className = "execution-path-diagnostic";
    this.host.setAttribute("aria-label", "Executed TCP path analysis");
    this.host.innerHTML = `<summary>Executed TCP analysis</summary>
      <p data-summary>Capture the 1 kHz ring after a run to compare its recorded TCP path. Choose a window covering the run.</p>
      <p><span style="color:#a54e00">━ Encoder-derived TCP</span> · <span style="color:#9938a3">━ Commanded TCP</span> · the full plan remains in its existing colours.</p>
      <svg viewBox="0 0 620 150" role="img" aria-label="TCP tracking error in millimetres over capture time" style="display:none;width:100%;max-height:180px"></svg>
      <p>TCP tracking error compares feedback with the preceding submitted command. No fitted delay is removed. This is an encoder-based estimate, not a measurement of tool flex or calibration accuracy.</p>
      <button type="button" class="mini-button" data-export disabled>Export trace</button>
      <button type="button" class="mini-button" data-import>Import trace</button>
      <input type="file" accept="application/json,.json" hidden>`;
    parent.append(this.host);
    this.summary = this.host.querySelector("[data-summary]")!;
    this.plot = this.host.querySelector("svg")!;
    this.exportButton = this.host.querySelector("[data-export]")!;
    this.exportButton.addEventListener("click", () => {
      if (!this.capture) return;
      const url = URL.createObjectURL(new Blob([JSON.stringify(this.capture)], { type: "application/json" }));
      const link = document.createElement("a");
      link.href = url; link.download = `execution-${this.capture.modelId}-${this.capture.window.last_sequence}.json`;
      link.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
    });
    const input = this.host.querySelector<HTMLInputElement>("input")!;
    this.host.querySelector("[data-import]")!.addEventListener("click", () => input.click());
    input.addEventListener("change", async () => {
      const file = input.files?.[0]; input.value = "";
      if (!file) return;
      try {
        if (file.size > 160 * 1024 * 1024) throw new Error("Trace exceeds the 160 MB import limit");
        const data: unknown = JSON.parse(await file.text());
        if (!data || typeof data !== "object" || (data as ExecutionCapture).schema !== "olp-execution-capture-v1") throw new Error("Not an OLP execution trace");
        await onImport(data as ExecutionCapture);
      } catch (error) { this.message(error instanceof Error ? error.message : String(error)); }
    });
  }
  clear(): void {
    this.capture = null; this.exportButton.disabled = true; this.plot.style.display = "none";
    this.summary.textContent = "Capture the 1 kHz ring after a run to compare its recorded TCP path.";
  }
  message(message: string): void {
    this.host.setAttribute("open", "");
    this.summary.textContent = message;
  }
  show(capture: ExecutionCapture, errors: ReturnType<typeof executionTcpErrors>, omitted: number): void {
    this.host.setAttribute("open", "");
    this.capture = capture; this.exportButton.disabled = false;
    this.summary.textContent = `${capture.modelId} · captured window (${capture.window.seconds} s requested); may contain only part of the run. ` +
      (errors.count ? `TCP tracking error: peak ${errors.peakMM.toFixed(2)} mm · RMS ${errors.rmsMM!.toFixed(2)} mm · ${errors.count} paired samples.` : "No valid consecutive submitted commands for error analysis.") +
      ` ${omitted} invalid pose samples omitted; gaps are not connected.`;
    this.plot.replaceChildren(); this.plot.style.display = errors.count === 0 ? "none" : "block";
    const maxTime = Math.max(0.001, errors.points.at(-1)?.timeS ?? 0), maxError = Math.max(0.01, errors.peakMM);
    const ns = "http://www.w3.org/2000/svg";
    let path = "", open = false;
    for (const point of errors.points) {
      if (point.errorMM === null) { open = false; continue; }
      path += `${open ? "L" : "M"}${(45 + point.timeS / maxTime * 560).toFixed(2)},${(115 - point.errorMM / maxError * 95).toFixed(2)} `;
      open = true;
    }
    const line = document.createElementNS(ns, "path");
    line.setAttribute("d", path); line.setAttribute("fill", "none"); line.setAttribute("stroke", "#a54e00"); line.setAttribute("stroke-width", "1.5");
    this.plot.append(line);
    for (const [x, y, label] of [[4, 18, `${maxError.toFixed(2)} mm`], [4, 118, "0"], [45, 142, "0 s"], [525, 142, `${maxTime.toFixed(2)} s`]] as const) {
      const text = document.createElementNS(ns, "text"); text.setAttribute("x", String(x)); text.setAttribute("y", String(y)); text.setAttribute("font-size", "12"); text.textContent = label; this.plot.append(text);
    }
  }
}
