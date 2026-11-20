// The TCP OLP stores for a captured pose, computed by OLP's own libraries
// (urdf-loader and three.js, as workstation-renderer.ts tcpPoseFor does), for
// seeded joint values on each robot description. The C++ capture chain must
// reproduce every pose: tests/capture_check.cpp.
// usage, from offline-programming/v1/ui so OLP's node_modules resolve:
//   ROBOTS=<robot_description/robots> OUT=<out.json> node --input-type=module < <this>
import { readFileSync, writeFileSync } from "node:fs";
import { Window } from "happy-dom";
import * as THREE from "three";
import URDFLoader from "urdf-loader";

// urdf-loader parses with DOMParser and checks the result against Document.
const window = new Window();
for (const name of ["DOMParser", "Document", "XMLDocument", "Element", "Node"]) globalThis[name] = window[name];
const root = new URL(`file://${process.env.ROBOTS}/`);
let seed = 20260928;
const random = () => ((seed = (seed * 1103515245 + 12345) % 2147483648) / 2147483648);
const cases = [];
for (const model of ["rosie_1400_v3", "rosie_1420_v1"]) {
  const urdf = readFileSync(new URL(`${model}/robot.urdf`, root), "utf8");
  const loader = new URDFLoader();
  loader.parseVisual = false; loader.parseCollision = false;
  const robot = loader.parse(urdf);
  const names = Object.values(robot.joints).filter((joint) => joint.jointType !== "fixed").map((joint) => joint.name);
  for (let i = 0; i < 12; ++i) {
    const joints = Object.fromEntries(names.map((name) => [name, i === 0 ? 0 : (random() * 2 - 1) * Math.PI]));
    for (const [name, value] of Object.entries(joints)) {
      robot.joints[name].ignoreLimits = true;
      robot.joints[name].setJointValue(value);
    }
    const frame = robot.links.tool0;
    frame.updateWorldMatrix(true, false);
    const position = new THREE.Vector3().setFromMatrixPosition(frame.matrixWorld);
    const rotation = new THREE.Euler().setFromRotationMatrix(frame.matrixWorld, "ZYX");
    cases.push({ model, joints, pose: [position.x, position.y, position.z, rotation.x, rotation.y, rotation.z] });
  }
}
writeFileSync(process.env.OUT, `${JSON.stringify({ frame: "tool0", cases }, null, 1)}\n`);
console.log(`${cases.length} capture poses from urdf-loader`);
