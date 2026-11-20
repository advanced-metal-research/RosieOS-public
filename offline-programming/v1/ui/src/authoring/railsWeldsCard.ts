import { PartFocusCard } from "./partFocusCard.ts";

/**
 * The Rails welds tool's dialog: the detected seams, each searched, then
 * accepted or trimmed, and under them the two parameter sets the search
 * runs with. The same ToolCard the Move part tool uses, so the two read
 * as tabs of one viewport toolbox and only one is open at a time.
 *
 * Three collapsible sections. Welds is open; Angle search and Fitted torch
 * start closed and carry their preset and how many values differ from it
 * in the header, so a wrong preset is noticed without opening anything --
 * and opening shows the few values worth changing here, with a way to the
 * full sheet for the rest. The rails mode draws into the three hosts;
 * this card is the frame. ✓ keeps what was decided and closes; ✕ puts
 * the decisions, searches and parameter edits back to how they stood
 * when the card opened -- the same contract as every tool card.
 */
export interface RailsWeldsCardCallbacks {
  onAccept: () => void;
  onCancel: () => void;
  /** Swing the camera back onto the part. */
  onCentreCamera: () => void;
  /** How faint the rest of the cell is drawn: opacity 1 solid, 0 gone. */
  onTransparency: (opacity: number) => void;
}

export class RailsWeldsCard extends PartFocusCard {
  readonly welds: HTMLDivElement;
  readonly search: HTMLDivElement;
  readonly torch: HTMLDivElement;

  constructor(callbacks: RailsWeldsCardCallbacks) {
    super({
      title: "Rails welds",
      className: "rails-welds-card",
      onAccept: callbacks.onAccept,
      onCancel: callbacks.onCancel,
      acceptTitle: "Keep these welds and close",
      cancelTitle: "Put the welds and parameters back to how they were when the card opened",
      onCentreCamera: callbacks.onCentreCamera,
      onTransparency: callbacks.onTransparency,
    });
    this.welds = this.addSection({ title: "Welds", key: "welds" });
    this.search = this.addSection({ title: "Angle search", key: "search", open: false });
    this.torch = this.addSection({ title: "Fitted torch", key: "torch", open: false });
  }

  setSummaries(summaries: { welds: string; search: string; torch: string }): void {
    this.setSectionSummary(this.welds, summaries.welds);
    this.setSectionSummary(this.search, summaries.search);
    this.setSectionSummary(this.torch, summaries.torch);
  }
}
