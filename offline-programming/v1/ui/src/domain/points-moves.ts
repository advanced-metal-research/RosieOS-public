import { reviseProject, type AuthoredProgramRow, type OfflineProgrammingProject, type RowMove, type Vec3 } from "./project.ts";
import { synchronizeTorchOrientations } from "./weld.ts";

export const MOVE_EXTENSION_KEY = "gradient_move_v1" as const;
export const DEFAULT_MOVE_TOLERANCE_M = 0.0001;
export interface GradientMoveMetadata { schemaVersion:1; moveId:string|null; role:"control"|"sample"; controlId:string|null; startControlId:string|null; endControlId:string|null; moveKind:"line"|"curve"|"joint"|null; curveKind:"three_point_arc"|null; centerM:Vec3|null; toleranceM:number; sampleIndex:number; sampleCount:number; samplesHash:string|null }
export interface MoveSegmentInput { startIndex:number; endIndex:number; kind:RowMove; centerM?:Vec3|null; moveId?:string; startControlId?:string; endControlId?:string; toleranceM?:number }
export interface ProgramControlPoint { id:string; number:number; row:AuthoredProgramRow; rowIndex:number }
export interface ProgramMove { id:string; number:number; kind:RowMove; start:ProgramControlPoint; end:ProgramControlPoint; rows:AuthoredProgramRow[]; rowIndices:number[]; sampleRows:AuthoredProgramRow[]; centerM:Vec3|null; toleranceM:number; samplesHash:string|null; modifiedExternally:boolean; externalReason:string|null; synthetic:boolean }
export type ProgramViewEntry={kind:"control";control:ProgramControlPoint}|{kind:"move";move:ProgramMove}|{kind:"raw";row:AuthoredProgramRow;rowIndex:number;reason?:string};
export interface ProgramView { controls:ProgramControlPoint[]; moves:ProgramMove[]; entries:ProgramViewEntry[]; visibleRowIds:string[]; moveById:Map<string,ProgramMove>; controlByRowId:Map<string,ProgramControlPoint> }

export function attachMoveMetadata(rows:AuthoredProgramRow[], segments:readonly MoveSegmentInput[], prefix="move"):void {
  const cps=new Map<number,string>();
  for(const s of segments){ assertSegment(rows,s); cps.set(s.startIndex,s.startControlId??cps.get(s.startIndex)??moveMetadata(rows[s.startIndex]!)?.controlId??cpId(rows[s.startIndex]!)); cps.set(s.endIndex,s.endControlId??cps.get(s.endIndex)??moveMetadata(rows[s.endIndex]!)?.controlId??cpId(rows[s.endIndex]!)); }
  for(const [i,id] of cps) if(!moveMetadata(rows[i]!)?.moveId) setMeta(rows[i]!,blankControl(id));
  segments.forEach((s,n)=>{const start=cps.get(s.startIndex)!,end=cps.get(s.endIndex)!,id=s.moveId??`${prefix}:${String(n+1).padStart(3,"0")}`,kind=metaKind(s.kind),center=kind==="curve"?copy3(s.centerM):null,count=s.endIndex-s.startIndex+1,tol=positive(s.toleranceM,DEFAULT_MOVE_TOLERANCE_M); for(let i=s.startIndex+1;i<=s.endIndex;i++) setMeta(rows[i]!,{schemaVersion:1,moveId:id,role:i===s.endIndex?"control":"sample",controlId:i===s.endIndex?end:null,startControlId:start,endControlId:end,moveKind:kind,curveKind:kind==="curve"?"three_point_arc":null,centerM:center,toleranceM:tol,sampleIndex:i-s.startIndex,sampleCount:count,samplesHash:null}); refreshMoveHash(rows,s.startIndex,s.endIndex);});
}

export function buildProgramView(project:OfflineProgrammingProject):ProgramView {
  const rows=project.rows, cpMap=new Map<string,ProgramControlPoint>(), controlByRowId=new Map<string,ProgramControlPoint>(); const controls:ProgramControlPoint[]=[];
  const cp=(i:number,id?:string|null)=>{const row=rows[i]!,old=cpMap.get(row.id);if(old)return old;const value={id:id??moveMetadata(row)?.controlId??cpId(row),number:controls.length+1,row,rowIndex:i};controls.push(value);cpMap.set(row.id,value);controlByRowId.set(row.id,value);return value};
  const moves:ProgramMove[]=[]; const seen=new Set<string>();
  rows.forEach((row,end)=>{const m=moveMetadata(row);if(!m?.moveId||m.role!=="control"||seen.has(m.moveId))return;seen.add(m.moveId);let start=end-m.sampleCount+1;if(start<0||moveMetadata(rows[start]!)?.controlId!==m.startControlId)start=rows.findIndex((r,i)=>i<end&&moveMetadata(r)?.controlId===m.startControlId);if(start<0||start>=end)start=Math.max(0,end-m.sampleCount+1);const idx=range(start,end),members=idx.map(i=>rows[i]!);const reason=validate(members,m);moves.push({id:m.moveId,number:0,kind:rowMove(m.moveKind),start:cp(start,m.startControlId),end:cp(end,m.endControlId??m.controlId),rows:members,rowIndices:idx,sampleRows:members.slice(1,-1),centerM:copy3(m.centerM),toleranceM:m.toleranceM,samplesHash:m.samplesHash,modifiedExternally:!!reason,externalReason:reason,synthetic:false});});
  // Orphaned metadata is a broken group: expose it.
  rows.forEach((row,i)=>{const m=moveMetadata(row);if(!m?.moveId||seen.has(m.moveId))return;seen.add(m.moveId);let a=i,b=i;while(a>0&&moveMetadata(rows[a-1]!)?.moveId===m.moveId)a--;while(b+1<rows.length&&moveMetadata(rows[b+1]!)?.moveId===m.moveId)b++;const start=Math.max(0,a-1),idx=range(start,b),members=idx.map(x=>rows[x]!);moves.push({id:m.moveId,number:0,kind:rowMove(m.moveKind),start:cp(start,m.startControlId),end:cp(b,m.endControlId),rows:members,rowIndices:idx,sampleRows:members.slice(1,-1),centerM:copy3(m.centerM),toleranceM:m.toleranceM,samplesHash:m.samplesHash,modifiedExternally:true,externalReason:"move destination/control row is missing",synthetic:false});});
  const claimed=new Set(moves.flatMap(m=>m.rowIndices));let previous:number|null=null;
  rows.forEach((row,i)=>{if(claimed.has(i)||!motion(row)){previous=null;return}cp(i);if(previous!==null){const start=cp(previous),end=cp(i);moves.push({id:`legacy:${start.row.id}:${end.row.id}`,number:0,kind:row.moveType,start,end,rows:[start.row,end.row],rowIndices:[previous,i],sampleRows:[],centerM:null,toleranceM:DEFAULT_MOVE_TOLERANCE_M,samplesHash:null,modifiedExternally:false,externalReason:null,synthetic:true});}previous=i;});
  moves.sort((a,b)=>a.rowIndices[0]!-b.rowIndices[0]!).forEach((m,i)=>m.number=i+1); const invalid=new Map<number,string>(),covered=new Set<number>();for(const m of moves)(m.modifiedExternally?m.rowIndices.forEach(i=>invalid.set(i,m.externalReason??"modified externally")):m.rowIndices.forEach(i=>covered.add(i)));
  // Dedupe key is the ROW identity for both control and raw entries: a CP
  // shared between an intact move and an externally-modified one must render
  // once (as its CP entry — order preference via o), never twice (review F2).
  const pos:Array<{p:number;o:number;k:string;e:ProgramViewEntry}>=[];for(const m of moves){if(m.modifiedExternally)continue;pos.push({p:m.start.rowIndex,o:0,k:`row:${m.start.row.id}`,e:{kind:"control",control:m.start}},{p:m.start.rowIndex,o:1,k:`m:${m.id}`,e:{kind:"move",move:m}},{p:m.end.rowIndex,o:0,k:`row:${m.end.row.id}`,e:{kind:"control",control:m.end}})}
  rows.forEach((row,i)=>{if(invalid.has(i))pos.push({p:i,o:2,k:`row:${row.id}`,e:{kind:"raw",row,rowIndex:i,reason:invalid.get(i)}});else if(!covered.has(i))pos.push(motion(row)?{p:i,o:0,k:`row:${row.id}`,e:{kind:"control",control:cp(i)}}:{p:i,o:2,k:`row:${row.id}`,e:{kind:"raw",row,rowIndex:i}})});const keys=new Set<string>();const entries=pos.sort((a,b)=>a.p-b.p||a.o-b.o).flatMap(x=>keys.has(x.k)?[]:(keys.add(x.k),[x.e]));
  controls.sort((a,b)=>a.rowIndex-b.rowIndex).forEach((c,i)=>c.number=i+1);return{controls,moves,entries,visibleRowIds:entries.flatMap(e=>e.kind==="control"?[e.control.row.id]:e.kind==="raw"?[e.row.id]:[]),moveById:new Map(moves.map(m=>[m.id,m])),controlByRowId};
}

/**
 * Reorder a control point one step along the authored CP sequence.
 *
 * This is what the spreadsheet Order column means for points: permute waypoints
 * in the path, not splice arbitrary wire rows. Shared CPs between consecutive
 * moves stay a single row. Interior samples on moves that lose their endpoints
 * are dropped (they would lie about the new geometry); CAD imports already have
 * empty sampleRows so they just swap.
 */
export function reorderAdjacentControlPoints(
  project: OfflineProgrammingProject,
  controlRowId: string,
  delta: -1 | 1,
): OfflineProgrammingProject {
  const view = buildProgramView(project);
  const controls = view.controls;
  const index = controls.findIndex((control) => control.row.id === controlRowId);
  if (index < 0) throw new Error(`control point ${controlRowId} was not found`);
  const other = index + delta;
  if (other < 0 || other >= controls.length) return project;

  const lo = Math.min(index, other);
  const hi = Math.max(index, other);
  const blocked = reorderBlockReason(view, project.rows, index, other);
  if (blocked) throw new Error(blocked);

  return reviseProject(project, "rows", (draft) => {
    const spans = controlOwnedSpans(view, draft.rows);
    // Samples tied to the old endpoints become false after a swap — keep only
    // the destination CP for the swapped spans and the move that used to leave
    // the upper CP (its samples assumed the previous start).
    const collapse = new Set<number>([lo, hi]);
    if (hi + 1 < spans.length) collapse.add(hi + 1);
    for (const spanIndex of collapse) {
      const span = spans[spanIndex]!;
      spans[spanIndex] = [span.at(-1)!];
    }

    const firstIdx = controls[0]!.rowIndex;
    const lastIdx = controls.at(-1)!.rowIndex;
    const prefix = draft.rows.slice(0, firstIdx);
    const suffix = draft.rows.slice(lastIdx + 1);
    const ordered = [...spans];
    [ordered[index], ordered[other]] = [ordered[other]!, ordered[index]!];
    const middle = ordered.flat().map((row) => {
      const copy = structuredClone(row);
      if (copy.extensions) delete copy.extensions[MOVE_EXTENSION_KEY];
      return copy;
    });
    draft.rows = [...prefix, ...middle, ...suffix];

    const segments: MoveSegmentInput[] = [];
    for (let cursor = 0; cursor < middle.length - 1; cursor += 1) {
      const start = middle[cursor]!;
      const end = middle[cursor + 1]!;
      const kind: RowMove = end.moveType === "CURVE" || end.moveType === "JOINT"
        ? end.moveType
        : "LINE";
      segments.push({
        startIndex: prefix.length + cursor,
        endIndex: prefix.length + cursor + 1,
        kind,
        centerM: kind === "CURVE" ? defaultCurveCenter(xyz(start), xyz(end)) : null,
        startControlId: cpId(start),
        endControlId: cpId(end),
        toleranceM: DEFAULT_MOVE_TOLERANCE_M,
      });
      end.moveType = kind;
    }
    if (segments.length > 0) attachMoveMetadata(draft.rows, segments, "reorder");
    draft.simulation.selectedRowId = controlRowId;
  });
}

export function canReorderControlPoint(
  project: OfflineProgrammingProject,
  controlRowId: string,
  delta: -1 | 1,
): boolean {
  try {
    const view = buildProgramView(project);
    const index = view.controls.findIndex((control) => control.row.id === controlRowId);
    if (index < 0) return false;
    const other = index + delta;
    if (other < 0 || other >= view.controls.length) return false;
    return reorderBlockReason(view, project.rows, index, other) === null;
  } catch {
    return false;
  }
}

/**
 * SAFETY GUARD (executed repro 2026-08-05): the rebuild below flattens every
 * CP span into 2-row segments, which deleted non-motion rows in the range and
 * re-segmented authored multi-sample moves that had nothing to do with the
 * swap. Until the rebuild is span-preserving, a reorder is only legal when it
 * cannot destroy anything: motion rows only, and no multi-sample span outside
 * the collapse set.
 */
function reorderBlockReason(
  view: ProgramView,
  rows: AuthoredProgramRow[],
  index: number,
  other: number,
): string | null {
  const controls = view.controls;
  const lo = Math.min(index, other);
  const hi = Math.max(index, other);
  for (const move of view.moves) {
    if (!move.modifiedExternally) continue;
    if (
      move.start.row.id === controls[index]!.row.id ||
      move.end.row.id === controls[index]!.row.id ||
      move.start.row.id === controls[other]!.row.id ||
      move.end.row.id === controls[other]!.row.id
    ) {
      return "cannot reorder a control point on an externally modified move";
    }
  }
  const firstIdx = controls[0]!.rowIndex;
  const lastIdx = controls.at(-1)!.rowIndex;
  for (let i = firstIdx; i <= lastIdx; i += 1) {
    if ((rows[i]!.rowType ?? "motion") !== "motion") {
      return "reorder is blocked: command/comment/IO rows sit between control points — move them out of the point range first";
    }
  }
  const collapse = new Set<number>([lo, hi]);
  if (hi + 1 < controls.length) collapse.add(hi + 1);
  const spans = controlOwnedSpans(view, rows);
  for (let i = 0; i < spans.length; i += 1) {
    if (!collapse.has(i) && spans[i]!.length > 1) {
      return "reorder is blocked: it would rebuild an unrelated multi-sample move and lose its authored samples";
    }
  }
  return null;
}

/** Rows owned by each CP: first CP is itself; later CPs own (samples… + destination). */
function controlOwnedSpans(
  view: ProgramView,
  rows: AuthoredProgramRow[],
): AuthoredProgramRow[][] {
  return view.controls.map((control, index) => {
    if (index === 0) return [rows[control.rowIndex]!];
    const previous = view.controls[index - 1]!;
    const incoming = view.moves.find((move) =>
      !move.modifiedExternally &&
      move.end.row.id === control.row.id &&
      move.start.row.id === previous.row.id
    );
    if (!incoming) return [rows[control.rowIndex]!];
    return incoming.rowIndices.slice(1).map((rowIndex) => rows[rowIndex]!);
  });
}

export function simplifyPolylineByChordDeviation(points:readonly (readonly [number,number,number])[],toleranceM=DEFAULT_MOVE_TOLERANCE_M):number[]{if(!points.length)return[];if(points.length===1)return[0];const out=[0],tol=positive(toleranceM,DEFAULT_MOVE_TOLERANCE_M);let start=0;while(start<points.length-1){let best=start+1;for(let end=start+2;end<points.length;end++){if(chordDeviation(points,start,end)<=tol+1e-12)best=end;else break}out.push(best);start=best}return out}
export function segmentSampledPath(points:readonly (readonly [number,number,number])[],toleranceM=DEFAULT_MOVE_TOLERANCE_M):Array<{startIndex:number;endIndex:number;kind:"LINE"|"CURVE";centerM:Vec3|null}>{const out=[];let start=0;const tol=positive(toleranceM,DEFAULT_MOVE_TOLERANCE_M);while(start<points.length-1){let line=start+1,curve=-1,center:Vec3|null=null;for(let end=start+2;end<points.length;end++){if(chordDeviation(points,start,end)<=tol+1e-12)line=end;const via=copy3(points[Math.floor((start+end)/2)]);if(via&&arcDeviation(points,start,end,via)<=tol+1e-12){curve=end;center=via}}if(curve>line&&chordDeviation(points,start,curve)>tol*.25){out.push({startIndex:start,endIndex:curve,kind:"CURVE" as const,centerM:center});start=curve}else{out.push({startIndex:start,endIndex:line,kind:"LINE" as const,centerM:null});start=line}}return out}

/**
 * Maximum chord length whose sagitta is no greater than `toleranceM`.
 * From e = R - sqrt(R² - (L/2)²), rearrange to
 * L = 2 * sqrt(2Re - e²).
 */
export function maximumChordLengthForSagitta(radiusM:number,toleranceM=DEFAULT_MOVE_TOLERANCE_M):number {
  const radius=positive(radiusM,Number.NaN),tolerance=positive(toleranceM,DEFAULT_MOVE_TOLERANCE_M);
  if(!Number.isFinite(radius))throw new Error("curve radius must be positive");
  if(tolerance>=radius)return radius*2;
  return 2*Math.sqrt(2*radius*tolerance-tolerance*tolerance);
}

/**
 * Select the actual wire control points. Dense CAD samples remain in seam
 * geometry; only these tolerance-bounded indices become Robot V4 rows.
 */
export function controlPointIndicesForPath(
  points:readonly (readonly [number,number,number])[],
  toleranceM=DEFAULT_MOVE_TOLERANCE_M,
  _curve=false,
):number[] {
  return simplifyPolylineByChordDeviation(points,toleranceM);
}

export function interpolateThreePointArc(control:readonly (readonly [number,number,number])[],count:number):Vec3[]{if(control.length<3)return control.map(p=>[...p] as Vec3);const p0=copy3(control[0])!,p1=copy3(control[1])!,p2=copy3(control[2])!,a=sub(p1,p0),b=sub(p2,p0);let normal=cross(a,b);if(dot(normal,normal)<=1e-10)return samplePolyline([p0,p1,p2],count);normal=normalize(normal);const u=normalize(a),v=normalize(cross(normal,u)),x1=dot(a,u),y1=dot(a,v),x2=dot(b,u),y2=dot(b,v),den=2*(x1*y2-y1*x2);if(Math.abs(den)<=1e-10)return samplePolyline([p0,p1,p2],count);const cx=(y2*(x1*x1+y1*y1)-y1*(x2*x2+y2*y2))/den,cy=(x1*(x2*x2+y2*y2)-x2*(x1*x1+y1*y1))/den,center=add(p0,add(scale(u,cx),scale(v,cy))),radius=distance(center,p0);if(radius<=1e-9)return samplePolyline([p0,p1,p2],count);const angle=(p:Vec3)=>{const r=sub(p,center);return Math.atan2(dot(r,v),dot(r,u))},a0=angle(p0),a1=angle(p1),a2=angle(p2),mid=positiveAngle(a1-a0),end=positiveAngle(a2-a0),total=mid<=end?end:end-Math.PI*2,n=Math.max(3,Math.trunc(count)),out:Vec3[]=[];for(let i=0;i<n;i++){const q=a0+total*i/(n-1);out.push(add(center,add(scale(u,Math.cos(q)*radius),scale(v,Math.sin(q)*radius))))}return dedupe(out)}

export function regenerateCurveMove(project:OfflineProgrammingProject,moveId:string,centerM:Vec3):OfflineProgrammingProject{const move=buildProgramView(project).moveById.get(moveId);if(!move)throw new Error(`move ${moveId} was not found`);if(move.externalReason?.includes("teach-modified"))throw new Error("teach-modified move rows cannot be silently regenerated");if(move.modifiedExternally)throw new Error("externally modified move must be resolved before regeneration");if(move.kind!=="CURVE")throw new Error("only CURVE moves have an editable center");if(move.rows.some(r=>r.sourceState==="teach_modified"))throw new Error("teach-modified move rows cannot be silently regenerated");return reviseProject(project,"rows",draft=>{if(move.rows.length>2){const points=interpolateThreePointArc([xyz(move.start.row),centerM,xyz(move.end.row)],move.rows.length);move.rowIndices.forEach((ri,pi)=>{const row=draft.rows[ri]!,p=points[pi]!;row.targetTcpPose.splice(0,3,...p);if(row.currentTcpPose)row.currentTcpPose.splice(0,3,...p)})}move.rowIndices.forEach(ri=>{const row=draft.rows[ri]!,m=moveMetadata(row);if(m?.moveId===moveId)setMeta(row,{...m,centerM:copy3(centerM),samplesHash:null});if(row.sourceState==="cad_derived")row.sourceState="manual";row.source="offline_programming"});synchronizeTorchOrientations(draft.rows,new Set(move.rows.flatMap(r=>r.seamId?[r.seamId]:[])));refreshMoveHash(draft.rows,move.start.rowIndex,move.end.rowIndex)})}

export function moveMetadata(row:AuthoredProgramRow):GradientMoveMetadata|null{const x=row.extensions?.[MOVE_EXTENSION_KEY];if(!record(x)||Number(x.schema_version)!==1||(x.role!=="control"&&x.role!=="sample"))return null;return{schemaVersion:1,moveId:text(x.move_id),role:x.role,controlId:text(x.control_id),startControlId:text(x.start_control_id),endControlId:text(x.end_control_id),moveKind:x.move_kind==="line"||x.move_kind==="curve"||x.move_kind==="joint"?x.move_kind:null,curveKind:x.curve_kind==="three_point_arc"?"three_point_arc":null,centerM:copy3(x.center_m),toleranceM:positive(x.tolerance_m,DEFAULT_MOVE_TOLERANCE_M),sampleIndex:integer(x.sample_index,0),sampleCount:Math.max(1,integer(x.sample_count,1)),samplesHash:text(x.samples_hash)}}
/**
 * Re-pin every move's sample hash after an in-app edit. `moveIds` restricts the
 * refresh to moves that reconciled cleanly beforehand: an externally modified
 * move must keep failing reconciliation until the user resolves it, rather than
 * being healed by an unrelated edit elsewhere in the program.
 */
export function refreshAllMoveHashes(rows:AuthoredProgramRow[],moveIds?:ReadonlySet<string>):void{rows.forEach((r,end)=>{const m=moveMetadata(r);if(!m?.moveId||m.role!=="control")return;if(moveIds&&!moveIds.has(m.moveId))return;const start=rows.findIndex(x=>moveMetadata(x)?.controlId===m.startControlId);if(start>=0&&start<end)refreshMoveHash(rows,start,end)})}

/** Materialize a hand-authored CURVE as a two-CP move with an editable via handle. */
export function ensureManualCurveMoves(rows:AuthoredProgramRow[]):void {
  let previousMotionIndex:number|null=null;
  rows.forEach((row,index)=>{
    if(!motion(row)){previousMotionIndex=null;return}
    if(row.moveType==="CURVE"&&!moveMetadata(row)?.moveId&&previousMotionIndex!==null){
      const start=xyz(rows[previousMotionIndex]!),end=xyz(row),center=defaultCurveCenter(start,end);
      attachMoveMetadata(rows,[{
        startIndex:previousMotionIndex,
        endIndex:index,
        kind:"CURVE",
        centerM:center,
        moveId:`manual:${rows[previousMotionIndex]!.id}:${row.id}`,
      }],"manual");
    }
    previousMotionIndex=index;
  });
}

function validate(rows:AuthoredProgramRow[],end:GradientMoveMetadata):string|null{if(rows.length<2||end.sampleCount!==rows.length)return"sample count or start control changed externally";if(rows.some(r=>r.sourceState==="teach_modified"))return"move contains a teach-modified row";for(let i=1;i<rows.length;i++){const m=moveMetadata(rows[i]!);if(!m||m.moveId!==end.moveId)return"sample metadata was removed or changed externally";if(m.sampleIndex!==i||m.sampleCount!==rows.length)return"sample ordering changed externally";if(m.startControlId!==end.startControlId||m.endControlId!==end.endControlId)return"control identity changed externally";if((i===rows.length-1)!==(m.role==="control"))return"sample/control role changed externally"}if(!end.samplesHash)return"sample hash is missing";return rowsHash(rows)===end.samplesHash?null:"sample rows were modified externally"}
function refreshMoveHash(rows:AuthoredProgramRow[],start:number,end:number){const hash=rowsHash(rows.slice(start,end+1));for(let i=start+1;i<=end;i++){const m=moveMetadata(rows[i]!);if(m?.moveId)setMeta(rows[i]!,{...m,samplesHash:hash})}}
function rowsHash(rows:readonly AuthoredProgramRow[]):string{const baseline=rows[0]!;const clean=rows.map((r,index)=>{const x=structuredClone(r) as unknown as Record<string,unknown>;
  // sourceState/source stay excluded: our own edits legitimately flip
  // cad_derived->manual and provenance is not physical behaviour. revPiIOSlot
  // IS physical behaviour (which IO fires) — hash it NORMALIZED: null becomes
  // 0 on the wire and reads back as 0, so hashing raw null would false-flag
  // every catalog round-trip (review F1, 2026-08-04).
  for(const k of ["rowRevision","rowHash","plannerStatus","validationState","collisionCheckState","limitCheckState","plannedPathRevision","plannedSampleCount","currentTcpPose","currentJointsRad","revPiIOPending","sourceState","source","fixtureId"])delete x[k];
  x.revPiIOSlot=(x.revPiIOSlot as number|null|undefined)??0;
  const e=record(x.extensions)?x.extensions:{};delete e.__wire;delete e[MOVE_EXTENSION_KEY];if(index===0){for(const key of ["id","label","moveType","phase","seamId","passId","passRole","targetTcpPose","enabled","command","rowType"])delete x[key];}return x});return`fnv1a64:${fnv(stable(clean))}`}
function setMeta(row:AuthoredProgramRow,m:GradientMoveMetadata){row.extensions={...(row.extensions??{}),[MOVE_EXTENSION_KEY]:{schema_version:1,move_id:m.moveId,role:m.role,control_id:m.controlId,start_control_id:m.startControlId,end_control_id:m.endControlId,move_kind:m.moveKind,curve_kind:m.curveKind,center_m:m.centerM?[...m.centerM]:null,tolerance_m:m.toleranceM,sample_index:m.sampleIndex,sample_count:m.sampleCount,samples_hash:m.samplesHash}}}
function blankControl(id:string):GradientMoveMetadata{return{schemaVersion:1,moveId:null,role:"control",controlId:id,startControlId:null,endControlId:id,moveKind:null,curveKind:null,centerM:null,toleranceM:DEFAULT_MOVE_TOLERANCE_M,sampleIndex:0,sampleCount:1,samplesHash:null}}
function chordDeviation(points:readonly (readonly number[])[],a:number,b:number){const p=points[a]!,q=points[b]!,d=sub(q,p),l=dot(d,d);let max=0;for(let i=a+1;i<b;i++){const r=points[i]!;const t=l<=1e-24?0:Math.max(0,Math.min(1,dot(sub(r,p),d)/l));max=Math.max(max,distance(r,add(p,scale(d,t))))}return max}
function arcDeviation(points:readonly (readonly [number,number,number])[],a:number,b:number,via:Vec3){const fit=interpolateThreePointArc([points[a]!,via,points[b]!],b-a+1);if(fit.length!==b-a+1)return Infinity;return Math.max(...fit.map((p,i)=>distance(p,points[a+i]!)))}
function samplePolyline(points:Vec3[],count:number){const lengths=[0];for(let i=1;i<points.length;i++)lengths.push(lengths[i-1]!+distance(points[i-1]!,points[i]!));const total=lengths.at(-1)!;if(total<=1e-9)return[points[0]!,points.at(-1)!];const out:Vec3[]=[],n=Math.max(2,Math.trunc(count));for(let i=0;i<n;i++){const target=total*i/(n-1);let r=1;while(r<lengths.length-1&&lengths[r]!<target)r++;const l=r-1,span=lengths[r]!-lengths[l]!,t=span<=1e-12?0:(target-lengths[l]!)/span;out.push(add(points[l]!,scale(sub(points[r]!,points[l]!),t)))}return dedupe(out)}
function dedupe(points:Vec3[]){const out=points.length?[points[0]!]:[];for(const p of points.slice(1))if(distance(out.at(-1)!,p)>1e-9)out.push(p);return out}
function assertSegment(rows:readonly AuthoredProgramRow[],s:MoveSegmentInput){if(!Number.isInteger(s.startIndex)||!Number.isInteger(s.endIndex)||s.startIndex<0||s.endIndex>=rows.length||s.endIndex<=s.startIndex)throw new Error("move segment must reference at least two ordered rows")}
function range(a:number,b:number){return Array.from({length:b-a+1},(_,i)=>a+i)} function motion(r:AuthoredProgramRow){return(r.rowType??"motion")==="motion"} function cpId(r:AuthoredProgramRow){return`cp:${r.id.replace(/[^a-zA-Z0-9_.:-]+/g,"-")}`} function metaKind(m:RowMove){return m==="CURVE"?"curve" as const:m==="JOINT"?"joint" as const:"line" as const} function rowMove(m:GradientMoveMetadata["moveKind"]){return m==="curve"?"CURVE":m==="joint"?"JOINT":"LINE"} function xyz(r:AuthoredProgramRow):Vec3{return[r.targetTcpPose[0],r.targetTcpPose[1],r.targetTcpPose[2]]}
function defaultCurveCenter(start:Vec3,end:Vec3):Vec3{const chord=sub(end,start),length=Math.hypot(...chord),mid=scale(add(start,end),.5);if(length<=1e-9)return[mid[0],mid[1]+.01,mid[2]];const reference=Math.abs(chord[2]/length)<.9?[0,0,1] as Vec3:[0,1,0] as Vec3,side=normalize(cross(chord,reference));return add(mid,scale(side,Math.max(.005,length*.2)))}
function copy3(v:unknown):Vec3|null{if(!Array.isArray(v)||v.length<3)return null;const x=[Number(v[0]),Number(v[1]),Number(v[2])] as Vec3;return x.every(Number.isFinite)?x:null}function text(v:unknown){return typeof v==="string"&&v.trim()?v.trim():null}function positive(v:unknown,f:number){const n=Number(v);return Number.isFinite(n)&&n>0?n:f}function integer(v:unknown,f:number){const n=Number(v);return Number.isInteger(n)?n:f}function record(v:unknown):v is Record<string,unknown>{return typeof v==="object"&&v!==null&&!Array.isArray(v)}
function sub(a:readonly number[],b:readonly number[]):Vec3{return[a[0]!-b[0]!,a[1]!-b[1]!,a[2]!-b[2]!]}function add(a:readonly number[],b:readonly number[]):Vec3{return[a[0]!+b[0]!,a[1]!+b[1]!,a[2]!+b[2]!]}function scale(a:readonly number[],s:number):Vec3{return[a[0]!*s,a[1]!*s,a[2]!*s]}function dot(a:readonly number[],b:readonly number[]){return a[0]!*b[0]!+a[1]!*b[1]!+a[2]!*b[2]!}function cross(a:readonly number[],b:readonly number[]):Vec3{return[a[1]!*b[2]!-a[2]!*b[1]!,a[2]!*b[0]!-a[0]!*b[2]!,a[0]!*b[1]!-a[1]!*b[0]!]}function normalize(a:Vec3){const l=Math.hypot(...a);return l<=1e-12?[0,0,0] as Vec3:scale(a,1/l)}function distance(a:readonly number[],b:readonly number[]){return Math.hypot(a[0]!-b[0]!,a[1]!-b[1]!,a[2]!-b[2]!)}function positiveAngle(v:number){while(v<0)v+=Math.PI*2;while(v>=Math.PI*2)v-=Math.PI*2;return v}
function stable(v:unknown):string{if(Array.isArray(v))return`[${v.map(stable).join(",")}]`;if(record(v))return`{${Object.keys(v).sort().map(k=>`${JSON.stringify(k)}:${stable(v[k])}`).join(",")}}`;return JSON.stringify(v)}function fnv(v:string){let h=0xcbf29ce484222325n;for(const b of new TextEncoder().encode(v)){h^=BigInt(b);h=BigInt.asUintN(64,h*0x100000001b3n)}return h.toString(16).padStart(16,"0")}
