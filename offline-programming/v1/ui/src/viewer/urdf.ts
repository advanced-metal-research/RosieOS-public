import * as THREE from "three";
import { ColladaLoader } from "three/addons/loaders/ColladaLoader.js";
import URDFLoader, { type URDFRobot } from "urdf-loader";
import type { RobotModelDescriptor } from "./models.ts";
import { disposeMaterialResources } from "./resource-disposal.ts";

export interface UrdfLoadResult {
  robot: URDFRobot;
  assetErrors: string[];
}

export async function loadUrdfRobot<TModelId extends string>(
  descriptor: RobotModelDescriptor<TModelId>,
  baseURL: string,
): Promise<UrdfLoadResult> {
  const manager = new THREE.LoadingManager();
  const loader = new URDFLoader(manager);
  const assetErrors: string[] = [];
  loader.packages = Object.fromEntries(
    Object.entries(descriptor.packageMap).map(([name, path]) => [
      name,
      resolveURL(path, baseURL),
    ]),
  );

  const defaultMeshLoader = loader.loadMeshCb.bind(loader);
  loader.loadMeshCb = (meshURL, loadingManager, material, done) => {
    if (!meshURL.toLowerCase().endsWith(".dae")) {
      defaultMeshLoader(meshURL, loadingManager, material, done);
      return;
    }
    new ColladaLoader(loadingManager).load(
      meshURL,
      (collada) => {
        prepareCollada(collada.scene, meshURL);
        done(collada.scene);
      },
      undefined,
      (error) => {
        const message = `${meshURL}: ${error instanceof Error ? error.message : String(error)}`;
        assetErrors.push(message);
        done(new THREE.Object3D(), error instanceof Error ? error : new Error(String(error)));
      },
    );
  };

  return new Promise<UrdfLoadResult>((resolve, reject) => {
    let robot: URDFRobot | null = null;
    let managerComplete = false;
    let rejected = false;

    const finish = (): void => {
      if (!rejected && managerComplete && robot) resolve({ robot, assetErrors });
    };
    manager.onLoad = () => {
      managerComplete = true;
      finish();
    };
    manager.onError = (url) => {
      if (!assetErrors.some((message) => message.startsWith(url))) {
        assetErrors.push(`${url}: failed to load`);
      }
    };
    loader.load(
      resolveURL(descriptor.urdfURL, baseURL),
      (loaded) => {
        robot = loaded;
        finish();
      },
      undefined,
      (error) => {
        rejected = true;
        reject(error instanceof Error ? error : new Error(String(error)));
      },
    );
  });
}

export function resolveURL(path: string, baseURL: string): string {
  return new URL(path, ensureTrailingSlash(baseURL)).href;
}

function prepareCollada(root: THREE.Object3D, meshURL: string): void {
  root.rotation.set(0, 0, 0);
  root.up.set(0, 0, 1);
  const color = meshColor(meshURL);
  root.traverse((object) => {
    object.up.set(0, 0, 1);
    const mesh = object as THREE.Mesh;
    if (!mesh.isMesh) return;
    if (Array.isArray(mesh.material)) {
      mesh.material.forEach((material) => disposeMaterialResources(material));
    } else {
      disposeMaterialResources(mesh.material);
    }
    mesh.material = new THREE.MeshStandardMaterial({
      color,
      emissive: color,
      emissiveIntensity: 0.035,
      metalness: 0.08,
      roughness: 0.62,
      side: THREE.DoubleSide,
    });
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    mesh.geometry.computeVertexNormals();
  });
}

function meshColor(meshURL: string): number {
  const filename = meshURL.split("/").at(-1)?.toLowerCase() ?? "";
  if (/floor|pedestal|base/.test(filename)) return 0x697985;
  if (/positioner|h_frame/.test(filename)) return 0x8297a8;
  if (/end_effector|tool/.test(filename)) return 0xe3e9ed;
  const palette = [0x5c646b, 0x4a5259, 0x6b737a, 0x3f474e];
  let hash = 0;
  for (const character of filename) hash = ((hash * 31) + character.charCodeAt(0)) >>> 0;
  return palette[hash % palette.length]!;
}

function ensureTrailingSlash(value: string): string {
  return value.endsWith("/") ? value : `${value}/`;
}
