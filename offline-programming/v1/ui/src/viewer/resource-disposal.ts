import type { Material, Object3D } from "three";

interface Disposable {
  dispose(): void;
}

interface TextureLike extends Disposable {
  isTexture?: boolean;
}

interface ResourceNode {
  geometry?: Disposable;
  material?: Material | Material[];
}

export function disposeObjectTree(root: Object3D): void {
  const disposed = new Set<Disposable>();
  root.traverse((object) => {
    const node = object as Object3D & ResourceNode;
    disposeOnce(node.geometry, disposed);
    const materials = Array.isArray(node.material) ? node.material : [node.material];
    materials.forEach((material) => disposeMaterialResources(material, disposed));
  });
}

export function disposeMaterialResources(
  material: Material | undefined,
  disposed: Set<Disposable> = new Set(),
): void {
  if (!material) return;
  for (const value of Object.values(material)) {
    const texture = value as Partial<TextureLike> | null;
    if (texture && texture.isTexture === true && typeof texture.dispose === "function") {
      disposeOnce(texture as TextureLike, disposed);
    }
  }
  disposeOnce(material, disposed);
}

function disposeOnce(resource: Disposable | undefined, disposed: Set<Disposable>): void {
  if (!resource || disposed.has(resource)) return;
  disposed.add(resource);
  resource.dispose();
}
