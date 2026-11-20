import { ToolCard } from "../ui/tool-card.ts";

/**
 * The Plan presets tool's dialog: the two parameter sets the weld planner
 * searches with -- the angle search and the fitted torch -- as collapsible
 * sections, beside Plan on the toolbar, so they can be read and trimmed
 * without opening the Rails welds tool. The rails mode draws the quick
 * views into the two hosts, the same views the Rails welds card shows.
 */
export class PlanPresetsCard extends ToolCard {
  readonly search: HTMLDivElement;
  readonly torch: HTMLDivElement;

  constructor(onClose: () => void) {
    super({ title: "Plan presets", className: "plan-presets-card", onClose });
    this.search = this.addSection({ title: "Angle search", key: "search" });
    this.torch = this.addSection({ title: "Fitted torch", key: "torch" });
  }

  setSummaries(summaries: { search: string; torch: string }): void {
    this.setSectionSummary(this.search, summaries.search);
    this.setSectionSummary(this.torch, summaries.torch);
  }
}
