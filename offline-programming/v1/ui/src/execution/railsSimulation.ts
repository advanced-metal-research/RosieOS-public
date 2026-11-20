/**
 * Simulate, for a rails program: the planned trajectory replayed in the
 * viewer, and the provenance that says which program produced it.
 *
 * This is the whole of the rails preview path. Plan is a separate button and
 * a separate trip -- it calls the weld planner, keeps the dense blob, and
 * stops. Nothing here plans, calls a server, or commands anything; it reads
 * bytes that are already on disk and poses a viewer from them.
 *
 * ## Why the blob rather than a re-plan
 *
 * The classic flow plans in the browser, hands its rows to the Motion Server
 * and replays what comes back, so what is watched is the canonical planner's
 * own answer. Rails used to borrow that: the weld planner's certified motion
 * was flattened to poses, handed to the Motion Server's spreadsheet planner,
 * and re-solved -- `line_tcp` rows have their J1..J6 discarded and IK run
 * again from the interpolated TCP pose, so what played was a second answer
 * that merely resembled the first. The rows that made that possible went
 * with `export/robot_v4.py`, and this replaces it with the bytes themselves.
 *
 * Nothing is lost by dropping the Tesseract discrete contact check with it.
 * That check samples waypoints, and the weld planner's own T2 verifier --
 * which every planned segment has already passed, `verify=true` on both
 * trajopt stages -- proves clearance over whole intervals with a Lipschitz
 * bound instead. Its docstring measured why sampling is not enough: 1.2-2.4
 * mm between consecutive samples on real plans, which passes clean through a
 * 3 mm clamp and reports two clear endpoints.
 *
 * ## Provenance
 *
 * A trajectory read back from the store can only be checked against whatever
 * program happens to be open, and after an edit that is a different program.
 * So the record carries `sourceProgramDigest` and `createdAt`, and this
 * reports both alongside whether the digest still matches -- a stale replay
 * is shown as stale rather than hidden, because watching last week's motion
 * and believing it is this program's is the failure worth preventing.
 */

import { denseSegmentNodeId } from "../program-v2/dense-cursor.ts";
import type { SimulationSample } from "../simulation/contracts.ts";
import type { DenseTrajectoryRecord, OfflineProgrammingProject } from "../domain/project.ts";
import { DenseSimulation, type DenseSimulationSnapshot, type PlaybackRate } from "./denseSimulation.ts";
import { decodeDenseTrajectory } from "./denseTrajectoryBlob.ts";
import { denseTrajectorySamples, torchOnFlags } from "./denseTrajectoryPlayback.ts";

/** What was loaded, and whether it is still this program's motion. */
export interface RailsTrajectoryProvenance {
  /** ISO timestamp the motion server stamped when it planned. */
  plannedAt: string;
  /** The `program.v2` digest that was planned. */
  programDigest: string;
  /** Whether that digest is still the open program's. False means the
   *  program was edited after this was planned. */
  current: boolean;
  totalSampleCount: number;
  totalDurationS: number;
}

export interface RailsSimulationSnapshot extends DenseSimulationSnapshot {
  provenance: RailsTrajectoryProvenance | null;
}

export interface RailsSimulationDeps {
  /** Pose the viewer from one sample. */
  show(sample: SimulationSample | null): void;
  onChange(snapshot: RailsSimulationSnapshot): void;
  /** Read the stored bytes for a digest. Null when the store has none --
   *  a project restored on another machine, or one whose store was cleared. */
  readTrajectory(digest: string): Promise<Uint8Array | null>;
  /** The open program's `program.v2` digest, for the freshness check. */
  programDigest(project: OfflineProgrammingProject): Promise<string>;
  now?(): number;
  schedule?(step: () => void): number;
  cancel?(handle: number): void;
}

export type RailsLoadOutcome =
  | { state: "loaded"; provenance: RailsTrajectoryProvenance }
  /** The program has never been planned. */
  | { state: "unplanned" }
  /** Planned, but the bytes are not in this browser's store. */
  | { state: "missing"; digest: string }
  /** The bytes are there and do not decode: a reader that has drifted from
   *  the format, or a truncated record. Never played -- half a motion is not
   *  a shorter motion. */
  | { state: "unreadable"; reason: string };

export interface RailsSimulation {
  /** Load the open project's planned trajectory, if it has one. Replaces
   *  whatever was loaded; a viewer showing two trajectories is showing two
   *  answers. */
  load(project: OfflineProgrammingProject): Promise<RailsLoadOutcome>;
  /** Forget the loaded trajectory: the program was edited, or a different
   *  project was opened. */
  clear(): void;
  /** Ask again whether the loaded trajectory is still this program's.
   *
   *  Cheap and worth doing after every authoring change: `current` is a
   *  digest comparison made once at load, and the program moves under it.
   *  The trajectory stays loaded either way -- an edited program's last plan
   *  is still worth watching, as long as nobody is told it is the current
   *  one. */
  recheck(project: OfflineProgrammingProject): Promise<void>;
  play(): void;
  pause(): void;
  step(direction: "forward" | "backward"): void;
  canStep(direction: "forward" | "backward"): boolean;
  /** Seek to a fraction of the whole replay, which is what a 0..1000 range
   *  input gives. */
  seekFraction(fraction: number): void;
  stop(): void;
  /** Trajectory seconds per wall second; see `DenseSimulation.setRate`. */
  setRate(rate: PlaybackRate): void;
  readonly loaded: boolean;
  readonly playing: boolean;
  readonly durationS: number;
  readonly provenance: RailsTrajectoryProvenance | null;
}

export function createRailsSimulation(deps: RailsSimulationDeps): RailsSimulation {
  let provenance: RailsTrajectoryProvenance | null = null;
  let moveBoundaries: number[] = [];
  let snapshot: DenseSimulationSnapshot = {
    phase: "idle",
    cursorS: 0,
    durationS: 0,
    sampleIndex: -1,
    torchOn: false,
    rate: 1,
  };

  const player = new DenseSimulation({
    now: deps.now ?? (() => performance.now()),
    schedule: deps.schedule ?? ((step) => requestAnimationFrame(step)),
    cancel: deps.cancel ?? ((handle) => cancelAnimationFrame(handle)),
    show: (sample) => deps.show(sample),
    onChange: (next) => {
      snapshot = next;
      deps.onChange({ ...next, provenance });
    },
  });

  function clear(): void {
    provenance = null;
    moveBoundaries = [];
    player.load([], []);
  }

  async function load(project: OfflineProgrammingProject): Promise<RailsLoadOutcome> {
    const record: DenseTrajectoryRecord | null = project.denseTrajectory;
    if (!record) {
      clear();
      return { state: "unplanned" };
    }
    const bytes = await deps.readTrajectory(record.trajectoryDigest);
    if (!bytes) {
      clear();
      return { state: "missing", digest: record.trajectoryDigest };
    }
    let samples: SimulationSample[];
    let torch: boolean[];
    try {
      const trajectory = decodeDenseTrajectory(bytes);
      samples = denseTrajectorySamples(project, trajectory);
      torch = torchOnFlags(trajectory);
      let elapsed = 0;
      moveBoundaries = [0];
      const owners = trajectory.segments.map(segment =>
        denseSegmentNodeId(project.nodes, segment.kind as "freespace" | "weld" | "dwell", segment.sourceId) ?? `segment:${segment.index}`);
      trajectory.segments.forEach((segment, index) => {
        elapsed += segment.samples.at(-1)?.tS ?? 0;
        if (owners[index] !== owners[index + 1] && elapsed > moveBoundaries.at(-1)!) moveBoundaries.push(elapsed);
      });
    } catch (error) {
      clear();
      return { state: "unreadable", reason: error instanceof Error ? error.message : String(error) };
    }
    // Asked of the open program, not read off the record: the record's own
    // digest is what WAS planned, and the question is whether that is still
    // what is authored.
    const open = await deps.programDigest(project);
    provenance = {
      plannedAt: record.createdAt,
      programDigest: record.sourceProgramDigest ?? "",
      current: Boolean(record.sourceProgramDigest) && record.sourceProgramDigest === open,
      totalSampleCount: record.totalSampleCount,
      totalDurationS: record.totalDurationS,
    };
    player.load(samples, torch);
    return { state: "loaded", provenance };
  }

  async function recheck(project: OfflineProgrammingProject): Promise<void> {
    if (!provenance) return;
    const open = await deps.programDigest(project);
    const current = provenance.programDigest !== "" && provenance.programDigest === open;
    if (current === provenance.current) return;
    provenance = { ...provenance, current };
    deps.onChange({ ...snapshot, provenance });
  }

  return {
    load,
    clear,
    recheck,
    play: () => player.play(),
    pause: () => player.pause(),
    step: direction => player.step(direction, moveBoundaries),
    canStep: direction => player.canStep(direction, moveBoundaries),
    seekFraction: (fraction) => {
      const clamped = Math.min(Math.max(fraction, 0), 1);
      player.seek(clamped * player.durationS);
    },
    stop: () => player.stop(),
    setRate: (rate) => player.setRate(rate),
    get loaded() {
      return provenance !== null && snapshot.phase !== "idle";
    },
    get playing() {
      return snapshot.phase === "playing";
    },
    get durationS() {
      return player.durationS;
    },
    get provenance() {
      return provenance;
    },
  };
}
