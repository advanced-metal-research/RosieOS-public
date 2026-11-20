import assert from "node:assert/strict";
import test from "node:test";
import { seedRobotCatalogue } from "../../../../offline-programming/v1/ui/test/support/robot-catalogue.ts";
import * as session from "./session.ts";
import * as trace from "./trace.ts";
seedRobotCatalogue();
const names = Array.from({length: 9}, (_, i) => `J${i+1}`);
const pose = (x: number) => ({frame_id: "world" as const, xyz_m: [x,.7,.9] as [number,number,number], rpy_rad: [Math.PI,.6,3.1] as [number,number,number], joint_names: names, joint_values_rad: names.map((_,i)=>i<6?.1*i:0)});
function setup() {
 trace.traceForget(); session.newProject("Trace", "rosie_1400_v3");
 for (let i=1;i<=3;i++) session.record("new",pose(i),{source:"machine",observed_at:"2026-10-16T12:00:00Z",model_id:"rosie_1400_v3",robot_description_sha256:session.view().description!,cell_id:"local-simulation",pose:pose(i)},`p${i}`);
 session.select("p1");
}
const result = {dense:{trajectory_digest:"sha256:d",plan_id:"trace-plan",program_id:"p",program_digest:"sha256:p",manifest_revision:1,plan_revision:1,
 segments:[{index:0,kind:"freespace",source_id:"p1"},{index:1,kind:"weld",source_id:"p2"},{index:2,kind:"freespace",source_id:"p3"}]},dense_blob_base64:"kept geometry"};
function ready() { const k=trace.traceKey();trace.tracePrepared(k,result);trace.traceReady(k); }
function readyAfterEdit() {
 const edited={dense:{...result.dense,segments:session.currentProject().nodes.filter(n=>n.enabled!==false&&!["io","dwell"].includes(n.op))
  .map((n,index)=>({index,kind:"freespace",source_id:n.id}))}};
 const k=trace.traceKey();trace.tracePrepared(k,edited);trace.traceReady(k);
}
test("empty authoring drafts have a stable Trace key and no executable path",async()=>{
 session.newProject("Empty", "rosie_1400_v3");
 const before=JSON.stringify(session.currentProject());
 const key=trace.traceKey();assert.equal(trace.traceKey(),key);
 await assert.rejects(trace.tracePlanRequest(),/Record a waypoint/);
 assert.equal(JSON.stringify(session.currentProject()),before);
});
test("deleting the final waypoint invalidates a prepared path without breaking refresh",async()=>{
 setup();ready();const key=trace.traceKey();
 for(const id of ["p1","p2","p3"]) session.remove(id);
 assert.notEqual(trace.traceKey(),key);
 assert.equal(trace.traceCurrent(),false);
 assert.throws(()=>trace.traceRequest(1),/preparing/);
 await assert.rejects(trace.tracePlanRequest(),/Record a waypoint/);
});
test("prepares the complete authored path without measured-start capture or edits",async()=>{
 setup();const before=JSON.stringify(session.currentProject());const r=await trace.tracePlanRequest();
 assert.ok(r.planning);assert.equal((r.planning.program as any).execution_mode,"dry_run");assert.equal(r.planning.plan_from_program_start,true);assert.equal(r.planning.require_measured_start,false);
 assert.deepEqual((r.planning.program as any).nodes.map((n: any)=>n.id),["p1","p2","p3"]);
 assert.equal(JSON.stringify(session.currentProject()),before);
});
test("all subsequent directions use the identical program digest and correct segment",()=>{
 setup();ready();const f=trace.traceRequest(1);assert.deepEqual(f,{allow_entry:true,endpoint_only:false,linear_speed:.4,angular_speed:Math.PI/6,time_scale:1,trajectory_digest:"sha256:d",first_segment:1,last_segment:1,reverse:false,target:"p2"});
 // A release before the destination never advances selection or replans.
 assert.deepEqual(trace.traceRequest(1),f);assert.equal(trace.traceRequest(-1).target,"p1");assert.equal(session.view().selected,"p1");
 trace.traceReached("p2");const back=trace.traceRequest(-1);assert.equal(back.first_segment,1);assert.equal(back.reverse,true);
 trace.traceReached("p1");assert.deepEqual(trace.traceRequest(1),f);
 trace.traceReached("p2");const next=trace.traceRequest(1);assert.equal(next.first_segment,2);assert.equal(next.target,"p3");assert.equal(next.trajectory_digest,f.trajectory_digest);
 trace.traceReached("p3");assert.equal(trace.traceRequest(-1).target,"p2");
});
test("program edits invalidate pending and prepared paths; selection only resets the cursor",()=>{
 setup();const k=trace.traceKey();ready();session.edit("p2","speed_mm_s",45);
 assert.throws(()=>trace.traceRequest(1),/preparing/);assert.throws(()=>trace.tracePrepared(k,result),/changed/);
 ready();session.select("p2");assert.equal(trace.traceRequest(-1).target,"p1");
 trace.traceForget();assert.equal(trace.traceRequest(1).target,"p3");
});
test("speed edit, Undo and Redo reprepare without skipping the unfinished leg's start",()=>{
 setup();session.edit("p2","motion","linear");session.select("p3");
 const via={...pose(2.5),xyz_m:[2.5,.8,.9] as [number,number,number]};
 session.record("via",via,{source:"machine",observed_at:"2026-10-27T00:00:00Z",model_id:"rosie_1400_v3",
  robot_description_sha256:session.view().description!,cell_id:"local-simulation",pose:via},"unused");
 session.select("p2");ready(); // Mixed MoveJ / MoveL / MoveC program.
 assert.equal(trace.traceRequest(1).target,"p3"); // Stopped between P2 and P3; no traceReached.
 assert.equal(trace.traceRequest(-1).target,"p2");
 for(const change of [()=>session.edit("p2","speed_mm_s",45),session.undo,session.redo,()=>session.setSpeedScale(50)]) {
  const oldKey=trace.traceKey();change();
  assert.throws(()=>trace.traceRequest(-1),/preparing/);
  assert.throws(()=>trace.tracePrepared(oldKey,result),/changed/);
  const key=trace.traceKey();trace.tracePrepared(key,result);
  assert.throws(()=>trace.traceRequest(-1),/preparing/); // New native preparation is still mandatory.
  trace.traceReady(key);
  const back=trace.traceRequest(-1),forward=trace.traceRequest(1);
  assert.equal(back.target,"p2");assert.equal(forward.target,"p3");
  assert.equal(back.first_segment,2);assert.equal(back.last_segment,2);
 }
 trace.traceReached("p2");assert.equal(trace.traceRequest(-1).target,"p1");
});
test("reprepared unfinished legs use the new trajectory's indices and digest",()=>{
 setup();session.select("p2");ready();trace.traceRequest(1);
 session.edit("p2","speed_mm_s",45);
 const next={dense:{...result.dense,trajectory_digest:"sha256:replanned",segments:[
  {index:0,kind:"freespace",source_id:"p1"},{index:1,kind:"freespace",source_id:"p1"},
  {index:2,kind:"freespace",source_id:"p2"},{index:3,kind:"freespace",source_id:"p2"},
  {index:4,kind:"freespace",source_id:"p3"},{index:5,kind:"freespace",source_id:"p3"}]}};
 const key=trace.traceKey();trace.tracePrepared(key,next);trace.traceReady(key);
 const back=trace.traceRequest(-1);
 assert.equal(back.target,"p2");assert.equal(back.first_segment,4);assert.equal(back.last_segment,5);
 assert.equal(back.trajectory_digest,"sha256:replanned");
});
test("changed geometry, reteach, calibration and program identity never inherit an old unfinished leg",()=>{
 const changes=[
  ()=>session.edit("p2","target.xyz_m.0",2100),
  ()=>session.edit("p2","motion","linear"),
  ()=>session.record("reteach",pose(2.1),{source:"machine",observed_at:"2026-10-27T00:00:00Z",model_id:"rosie_1400_v3",
   robot_description_sha256:session.view().description!,cell_id:"local-simulation",pose:pose(2.1)},"unused"),
  ()=>{const p=session.currentProject();session.commitProject({...p,robot:{...p.robot,cell:{...p.robot.cell,machinePlanningCalibrationSha256:"changed"} as any}});},
  ()=>{const saved=JSON.parse(session.exportProject());saved.project.identity.id="another-program";session.openProject(JSON.stringify(saved));session.select("p2");},
 ];
 for(const change of changes) {
  setup();session.select("p2");ready();trace.traceRequest(1);change();readyAfterEdit();
  assert.equal(trace.traceRequest(-1).target,"p1");
 }
});
test("arc geometry and intermediate program rows survive complete-path preparation",async()=>{
 setup();session.select("p2");session.record("via",pose(1.5),{source:"machine",observed_at:"2026-10-16T12:00:00Z",model_id:"rosie_1400_v3",robot_description_sha256:session.view().description!,cell_id:"local-simulation",pose:pose(1.5)},"unused");
 session.addEvent("io");const r=await trace.tracePlanRequest();const node=(r.planning!.program as any).nodes.find((n: any)=>n.id==="p2") as any;
 assert.equal(node.motion,"circular");assert.deepEqual(node.via,pose(1.5));assert.deepEqual(node.target,pose(2));
 assert.ok((r.planning!.program as any).nodes.some((n: any)=>n.op==="io"));
 ready();session.select("p2");assert.equal(trace.traceRequest(-1).reverse,true); // no backwards replan
});
test("waypoint boundaries stop range selection and preparation grants no motion",()=>{
 setup();const k=trace.traceKey();trace.tracePrepared(k,result);assert.throws(()=>trace.traceRequest(1),/preparing/);
 trace.traceReady(k);assert.throws(()=>trace.traceRequest(-1),/First waypoint/);session.select("p3");assert.throws(()=>trace.traceRequest(1),/Last waypoint/);
});

test("Jog base speed retimes the same path in both directions",()=>{
 setup();session.edit("p2","motion","linear");session.edit("p2","speed_mm_s",50);session.select("p1");ready();
 const forward=trace.traceRequest(1,200);assert.equal(forward.time_scale,4);
 const reverse=trace.traceRequest(-1,200);assert.equal(reverse.time_scale,4);
 assert.equal(reverse.trajectory_digest,forward.trajectory_digest);
 assert.equal(reverse.first_segment,forward.first_segment);
});
test("a current production plan is reused without a planner request",async()=>{
 setup();const {computeProgramV2Digest}=await import("../../../../offline-programming/v1/ui/src/contracts/robot-v4-program-v2.ts");
 const {toProgramV2Document}=await import("../../../../offline-programming/v1/ui/src/program-v2/from-project.ts");
 const p=session.currentProject();const digest=await computeProgramV2Digest(toProgramV2Document(p));
 session.commitProject({...p,denseTrajectory:{schema:"test",trajectoryDigest:"sha256:production",requestSha256:"x",planId:"actual",programId:"p",programDigest:"sha256:p",manifestRevision:1,planRevision:2,createdAt:"2026-10-16",dtS:.01,totalSampleCount:100,totalDurationS:1,bytes:100,sourceProjectRevision:p.identity.revision,sourceProgramDigest:digest,segments:result.dense.segments.map(s=>({...s,sourceId:s.source_id,kind:s.kind as "weld"|"freespace",sampleCount:10,durationS:.1}))}});
 const r=await trace.tracePlanRequest();assert.equal(r.planning,undefined);assert.equal(r.load?.trajectory_digest,"sha256:production");
 const k=trace.traceKey();trace.tracePrepared(k,r);trace.traceReady(k);assert.equal(trace.traceRequest(1).trajectory_digest,"sha256:production");
 session.edit("p2","speed_mm_s",60);assert.ok((await trace.tracePlanRequest()).planning);
});

// Panasonic OM0105045E29 pp.44–45: off a point, back returns to that
// point; off a partial leg, forward/back retain its two endpoints.
test("jogging off a taught point returns there before tracing farther backward",()=>{
 setup(); ready(); session.select("p2"); trace.traceForget(true);
 const back=trace.traceRequest(-1,200,20);
 assert.equal(back.target,"p2");assert.equal(back.first_segment,1);assert.equal(back.endpoint_only,true);
 assert.equal(back.reverse,false);assert.equal(back.linear_speed,.2);
 assert.equal(back.angular_speed,20*Math.PI/180);
 trace.traceReached("p2");assert.equal(trace.traceRequest(-1).target,"p1");
 setup(); ready(); trace.traceForget(true);
 assert.equal(trace.traceRequest(-1).target,"p1"); // return to first point is allowed
});
test("jogging off a partially traced weld retains forward and backward endpoints",()=>{
 setup();ready();trace.traceRequest(1);trace.traceForget(true);
 assert.equal(trace.traceRequest(-1).target,"p1");
 assert.equal(trace.traceRequest(1).target,"p2");
 assert.equal(trace.traceRequest(1).endpoint_only,false);
});
test("speed edits preserve a jog departure before its first Trace request",()=>{
 for(const undo of [false,true]) for(const direction of [-1,1]) {
  setup();ready();session.select("p2");trace.traceForget(true);
  session.edit("p2","speed_mm_s",45);if(undo)session.undo();readyAfterEdit();
  const request=trace.traceRequest(direction);
  assert.equal(request.target,direction<0?"p2":"p3");
  assert.equal(request.endpoint_only,direction<0);
  if(direction<0){trace.traceReached("p2");assert.equal(trace.traceRequest(-1).target,"p1");}
 }
});
test("geometry changes and another program clear an old jog departure",()=>{
 for(const otherProgram of [false,true]) {
  setup();ready();session.select("p2");trace.traceForget(true);
  if(otherProgram){const saved=JSON.parse(session.exportProject());saved.project.identity.id="other-departure";
   session.openProject(JSON.stringify(saved));session.select("p2");}
  else session.edit("p2","target.xyz_m.0",2100);
  readyAfterEdit();assert.equal(trace.traceRequest(-1).target,"p1");
 }
});

// Panasonic OM0105045E29 section 6.15, printed pp.44–45: deleting P4
// while at P4 OR stopped between P4/P5 leaves forward=P5, backward=P3.
// Highlighting the surviving next row is not evidence of arriving there.
test("deleting the current point preserves both neighbouring Trace endpoints",()=>{
 for(const partway of [false,true]) {
  setup();ready();session.select("p2");
  if(partway) assert.equal(trace.traceRequest(1).target,"p3");
  session.remove("p2");readyAfterEdit();
  assert.equal(session.view().selected,"p3");
  const forward=trace.traceRequest(1),backward=trace.traceRequest(-1);
  assert.equal(forward.target,"p3");assert.equal(backward.target,"p1");
  assert.equal(forward.first_segment,1);assert.equal(forward.last_segment,1);
  assert.equal(backward.first_segment,1);assert.equal(backward.last_segment,1);
  assert.equal(backward.reverse,true);assert.equal(forward.endpoint_only,false);
  assert.equal(session.view().selected,"p3"); // Requests alone never report arrival.
  trace.traceReached("p1");assert.equal(session.view().selected,"p1");
  assert.equal(trace.traceRequest(1).target,"p3");
 }
});
test("deleting the first or last point approaches the remaining endpoint once",()=>{
 setup();ready();session.select("p1");session.remove("p1");readyAfterEdit();
 assert.throws(()=>trace.traceRequest(-1),/First waypoint/);
 const forward=trace.traceRequest(1);assert.equal(forward.target,"p2");assert.equal(forward.endpoint_only,true);
 assert.equal(forward.reverse,false);assert.equal(forward.first_segment,0);
 trace.traceReached("p2");assert.equal(trace.traceRequest(1).target,"p3");

 setup();ready();session.select("p3");session.remove("p3");readyAfterEdit();
 assert.throws(()=>trace.traceRequest(1),/Last waypoint/);
 const backward=trace.traceRequest(-1);assert.equal(backward.target,"p2");assert.equal(backward.endpoint_only,true);
 assert.equal(backward.reverse,false);assert.equal(backward.first_segment,1);
 trace.traceReached("p2");assert.equal(trace.traceRequest(-1).target,"p1");
});
test("deletion cursor survives speed edits and plan metadata without implying arrival",()=>{
 setup();session.select("p2");session.remove("p2");session.edit("p3","speed_mm_s",40);readyAfterEdit();
 assert.equal(trace.traceRequest(1).target,"p3");assert.equal(trace.traceRequest(-1).target,"p1");
 session.setSpeedScale(50);readyAfterEdit();assert.equal(trace.traceRequest(1).target,"p3");
 trace.traceReached("p3");assert.equal(session.traceSelectionGap(),null);
});
test("deletion cursor skips non-motion rows and survives plan metadata",()=>{
 setup();session.select("p2");session.addEvent("io");session.select("p2");
 session.remove("p2");assert.notEqual(session.view().selected,"p3");
 // Recording/replacing a plan keeps the authoring revision and must not
 // turn the successor's table highlight into a physical arrival.
 session.commitProject(structuredClone(session.currentProject()));readyAfterEdit();
 assert.equal(trace.traceRequest(1).target,"p3");assert.equal(trace.traceRequest(-1).target,"p1");
 trace.traceReached("p3");assert.equal(session.traceSelectionGap(),null);
});
test("explicit selection discards a deletion cursor",()=>{
 setup();session.select("p2");session.remove("p2");readyAfterEdit();trace.traceRequest(-1);
 session.select("p3");assert.equal(session.traceSelectionGap(),null);
 assert.throws(()=>trace.traceRequest(1),/Last waypoint/);
 assert.equal(trace.traceRequest(-1).target,"p1");
});
test("repeated and nonselected deletions retain the original physical gap",()=>{
 for(const deletion of ["p1","p3","p4"]){
  setup();session.select("p3");session.record("new",pose(4),{source:"machine",observed_at:"2026-10-27T00:00:00Z",
   model_id:"rosie_1400_v3",robot_description_sha256:session.view().description!,cell_id:"local-simulation",pose:pose(4)},"p4");
  session.select("p2");session.remove("p2");session.remove(deletion);readyAfterEdit();
  assert.equal(trace.traceRequest(1).target,deletion==="p3"?"p4":"p3");
  if(deletion==="p1") assert.throws(()=>trace.traceRequest(-1),/First waypoint/);
  else assert.equal(trace.traceRequest(-1).target,"p1");
  session.undo();readyAfterEdit();
  assert.equal(trace.traceRequest(1).target,"p3");
  assert.equal(trace.traceRequest(-1).target,"p1");
 }
});
test("Undo and Redo of a speed edit cannot erase a deletion gap",()=>{
 setup();session.select("p2");session.remove("p2");session.edit("p3","speed_mm_s",45);
 for(const change of [session.undo,session.redo]){
  change();readyAfterEdit();assert.equal(session.view().selected,"p3");
  assert.equal(trace.traceRequest(1).target,"p3");assert.equal(trace.traceRequest(-1).target,"p1");
 }
});
test("Undo deletion restores its source but returns there before claiming physical arrival",()=>{
 for(const partway of [false,true]){
  setup();ready();session.select("p2");
  if(partway) assert.equal(trace.traceRequest(1).target,"p3");
  session.remove("p2");session.undo();readyAfterEdit();
  assert.equal(session.view().selected,"p2");
  const back=trace.traceRequest(-1);
  assert.equal(back.target,"p2");assert.equal(back.endpoint_only,true);assert.equal(back.reverse,false);
  assert.equal(trace.traceRequest(1).target,"p3");
  trace.traceReached("p2");assert.equal(session.traceSelectionGap(),null);
  assert.equal(trace.traceRequest(-1).target,"p1");
 }
});
test("Redo deletion retains the gap both before and after a confirmed return",()=>{
 for(const reached of [false,true]){
  setup();session.select("p2");session.remove("p2");session.undo();readyAfterEdit();
  if(reached){trace.traceRequest(-1);trace.traceReached("p2");}
  session.redo();readyAfterEdit();assert.equal(session.view().selected,"p3");
  assert.equal(trace.traceRequest(1).target,"p3");assert.equal(trace.traceRequest(-1).target,"p1");
 }
});
test("editing the surviving pose keeps the deletion endpoints and requires new preparation",()=>{
 setup();session.select("p2");session.remove("p2");readyAfterEdit();
 assert.equal(trace.traceRequest(1).target,"p3");
 session.edit("p3","target.xyz_m.0",3200);
 assert.throws(()=>trace.traceRequest(1),/preparing/);readyAfterEdit();
 assert.equal(trace.traceRequest(1).target,"p3");assert.equal(trace.traceRequest(-1).target,"p1");
});
test("adding or reteaching from a point or partial leg uses the edited neighbours",()=>{
 for(const partway of [false,true]) for(const mode of ["new","reteach"] as const) {
  setup();ready();session.select("p2");
  if(partway) trace.traceRequest(1);
  const captured=pose(2.5);
  session.record(mode,captured,{source:"machine",observed_at:"2026-10-27T00:00:00Z",model_id:"rosie_1400_v3",
   robot_description_sha256:session.view().description!,cell_id:"local-simulation",pose:captured},"added");
  readyAfterEdit();
  assert.equal(trace.traceRequest(1).target,"p3");
  // An unfinished forward request is still on the outgoing leg. Resolve
  // backward independently from the newly taught point, as in the manual.
  trace.traceForget();readyAfterEdit();
  assert.equal(trace.traceRequest(-1).target,mode==="new"?"p2":"p1");
 }
});

test("Trace plans weld geometry with outputs disabled without editing the production program",async()=>{
 setup();session.edit("p1","weld_transition","start");session.edit("p2","motion","linear");session.edit("p2","weld_transition","stop");
 const before=JSON.stringify(session.currentProject());const r=await trace.tracePlanRequest();
 const doc=r.planning!.program as any;
 assert.equal(doc.execution_mode,"dry_run");
 assert.equal(doc.nodes[0].weld_transition,"start");assert.equal(doc.nodes[1].weld_transition,"stop");
 assert.equal(JSON.stringify(session.currentProject()),before);
 assert.equal(trace.tracePrepared(trace.traceKey(),result).dry_run,true);
});
