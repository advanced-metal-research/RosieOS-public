// The live path-hold line on the pendant's jog page, decided by OLP's own
// view (execution/jogLive.ts, the browser jog pad's): the status poll's
// jog_live as speed against the speed asked for, the server's "limited by"
// sentence and the singularity proximity meter. Stateless: every dense
// status poll redraws it.
import { jogLiveView, normalizeJogLive, type JogLiveView } from "../../../../offline-programming/v1/ui/src/execution/jogLive.ts";

/** One dense status body; what the jog page draws. */
export function jogLiveFromStatus(body: unknown): JogLiveView {
  return jogLiveView(normalizeJogLive(body));
}
