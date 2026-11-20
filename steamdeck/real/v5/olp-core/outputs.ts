// The cell's outputs on the pendant's Run page, decided by OLP's own view
// (execution/cellOutputs.ts, the browser's outputs block under Run): per
// output what the program asks for, what rt-core commands and what the wired
// feedback reads back, and the time left on an ON. Stateless: every dense
// status poll redraws it.
import { cellOutputsView, normalizeCellOutputs, type CellOutputsView } from "../../../../offline-programming/v1/ui/src/execution/cellOutputs.ts";

/** One dense status body; what the Run page draws. */
export function cellOutputsFromStatus(body: unknown): CellOutputsView {
  return cellOutputsView(normalizeCellOutputs(body));
}
