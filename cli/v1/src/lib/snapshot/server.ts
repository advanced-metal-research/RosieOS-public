// A static file server for the built snapshot page, on 127.0.0.1 and a port
// the kernel picks: never a fixed port, so it cannot collide with the
// owner's Vite on :5189 or a second snapshot run, and never on an interface
// another host can reach.
import { createReadStream } from "node:fs";
import { stat } from "node:fs/promises";
import { createServer, type Server } from "node:http";
import type { AddressInfo } from "node:net";
import path from "node:path";

/** The robot assets are URDF + Collada; a wrong type makes the loader's
 *  fetch succeed and its parser fail, which reads as a broken robot. */
const MIME: Record<string, string> = {
  ".html": "text/html; charset=utf-8",
  ".js": "text/javascript; charset=utf-8",
  ".mjs": "text/javascript; charset=utf-8",
  ".css": "text/css; charset=utf-8",
  ".json": "application/json; charset=utf-8",
  ".webmanifest": "application/manifest+json",
  ".map": "application/json; charset=utf-8",
  ".png": "image/png",
  ".jpg": "image/jpeg",
  ".jpeg": "image/jpeg",
  ".svg": "image/svg+xml",
  ".ico": "image/x-icon",
  ".wasm": "application/wasm",
  ".urdf": "application/xml; charset=utf-8",
  ".srdf": "application/xml; charset=utf-8",
  ".xml": "application/xml; charset=utf-8",
  ".dae": "model/vnd.collada+xml",
  ".stl": "model/stl",
  ".obj": "model/obj",
  ".glb": "model/gltf-binary",
  ".gltf": "model/gltf+json",
  ".step": "model/step",
  ".stp": "model/step",
  ".md": "text/markdown; charset=utf-8",
};

export interface StaticServer {
  /** http://127.0.0.1:<port>/ */
  url: string;
  close(): Promise<void>;
}

export function contentType(file: string): string {
  return MIME[path.extname(file).toLowerCase()] ?? "application/octet-stream";
}

export async function startStaticServer(root: string): Promise<StaticServer> {
  const base = path.resolve(root);
  const server: Server = createServer(async (request, response) => {
    try {
      if (request.method !== "GET" && request.method !== "HEAD") {
        response.writeHead(405).end();
        return;
      }
      const pathname = decodeURIComponent(new URL(request.url ?? "/", "http://localhost").pathname);
      // The browser asks for one unprompted; a 404 would be a console error.
      if (pathname === "/favicon.ico") {
        response.writeHead(204).end();
        return;
      }
      const file = path.resolve(base, `.${pathname.endsWith("/") ? `${pathname}index.html` : pathname}`);
      if (file !== base && !file.startsWith(base + path.sep)) {
        response.writeHead(403).end();
        return;
      }
      const info = await stat(file).catch(() => null);
      if (!info?.isFile()) {
        response.writeHead(404, { "Content-Type": "text/plain; charset=utf-8" }).end(`not found: ${pathname}`);
        return;
      }
      response.writeHead(200, { "Content-Type": contentType(file), "Content-Length": info.size, "Cache-Control": "no-store" });
      if (request.method === "HEAD") {
        response.end();
        return;
      }
      createReadStream(file).on("error", () => response.destroy()).pipe(response);
    } catch {
      if (!response.headersSent) response.writeHead(500);
      response.end();
    }
  });
  await new Promise<void>((resolve, reject) => {
    server.once("error", reject);
    server.listen(0, "127.0.0.1", () => resolve());
  });
  const { port } = server.address() as AddressInfo;
  return {
    url: `http://127.0.0.1:${port}/`,
    close: () => new Promise<void>((resolve) => {
      server.closeAllConnections();
      server.close(() => resolve());
    }),
  };
}
