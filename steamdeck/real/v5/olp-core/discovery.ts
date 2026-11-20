// Cells → ON THE NETWORK on the pendant: OLP's own discovered-cells reading
// (offline-programming/v1/ui/src/targets/discoveredCells.ts), so the Deck says
// what desktop OLP says and checks compatibility against the rt-control client
// this package bundles. Read-only: the Deck has no Add cell form, and nothing
// here connects.
import {
  discoveredRows, discoveryStatusText, validateDiscoveryResult,
} from "../../../../offline-programming/v1/ui/src/targets/discoveredCells.ts";

export interface DiscoveredCellLine {
  title: string;
  subtitle: string;
  badges: string;
  /** Incompatible or a differing pin: drawn as a warning. */
  flagged: boolean;
}

export interface DiscoveredCellsView {
  status: string;
  lines: DiscoveredCellLine[];
}

/** `body` is the route's JSON, or null with the client's error code. */
export function discoveredCells(body: unknown, error: string): DiscoveredCellsView {
  if (error) {
    return {
      status: error === "http_404"
        ? "This OLP server has no network search: it runs a build from before discovered-cells."
        : `Network search unavailable (${error}).`,
      lines: [],
    };
  }
  if (body === null || body === undefined) return { status: "Not searched yet.", lines: [] };
  let result;
  try {
    result = validateDiscoveryResult(body);
  } catch (failure) {
    return { status: failure instanceof Error ? failure.message : String(failure), lines: [] };
  }
  return {
    status: discoveryStatusText({ result, error: null, searching: false }),
    lines: discoveredRows(result).map((row) => ({
      title: row.title,
      subtitle: row.subtitle,
      badges: row.badges.map((badge) => badge.text).join(" · "),
      flagged: row.compatibility === "incompatible" || row.badges.some((badge) => badge.text === "Pin differs"),
    })),
  };
}
