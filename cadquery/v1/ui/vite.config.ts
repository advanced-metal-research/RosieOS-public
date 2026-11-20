import path from "node:path";
import { spawn } from "node:child_process";
import { createReadStream, readFileSync, statSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { defineConfig, type Plugin } from "vite";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const cadqueryRoot = path.resolve(__dirname, "..");
const repoRoot = path.resolve(cadqueryRoot, "..", "..");

export default defineConfig({
  base: "/cadquery/v1/ui/",
  server: {
    host: "127.0.0.1",
    port: 5184,
    allowedHosts: true,
  },
  build: {
    outDir: "dist",
    emptyOutDir: true,
    chunkSizeWarningLimit: 900,
    rollupOptions: {
      input: {
        main: path.resolve(__dirname, "index.html"),
        parts: path.resolve(__dirname, "parts.html"),
        seam: path.resolve(__dirname, "seam.html"),
        edgeGeometry: path.resolve(__dirname, "edge-geometry.html"),
      },
      output: {
        manualChunks(id: string) {
          if (id.includes("node_modules/three/")) {
            return "three";
          }
          if (id.includes("node_modules/monaco-editor/")) {
            return "monaco";
          }
          return undefined;
        },
      },
    },
  },
  plugins: [htmlRoutePlugin(["parts", "seam", "edge-geometry"]), cadqueryRobotAssetPlugin(), cadqueryTopologyPlugin()],
});

function htmlRoutePlugin(routeNames: string[]): Plugin {
  return {
    name: "cadquery-html-routes",
    configureServer(server) {
      server.middlewares.use((req, _res, next) => {
        const url = req.url ?? "/";
        const [rawPathname, query = ""] = url.split("?", 2);
        const pathname = rawPathname.replace(/\/+$/, "");
        const match = routeNames.find((routeName) => pathname.endsWith(`/${routeName}`));
        if (match) {
          req.url = `${pathname}.html${query ? `?${query}` : ""}`;
        }
        next();
      });
    },
  };
}

function cadqueryRobotAssetPlugin(): Plugin {
  const robotAssetRoot = path.join(repoRoot, "robot_description", "robots", "rosie_1400_v3");
  return {
    name: "cadquery-robot-asset-routes",
    configureServer(server) {
      server.middlewares.use((req, res, next) => {
        const url = req.url ?? "/";
        const pathname = decodeURIComponent(url.split("?", 1)[0]);
        const robotNeedle = "/robot/rosie_1400_v3/";
        const robotIndex = pathname.indexOf(robotNeedle);
        if (robotIndex >= 0) {
          serveStaticFile(path.join(robotAssetRoot, pathname.slice(robotIndex + robotNeedle.length)), res, next);
          return;
        }
        next();
      });
    },
  };
}

function serveStaticFile(filePath: string, res: { statusCode: number; setHeader: (name: string, value: string) => void; end: (value?: string) => void }, next: () => void): void {
  try {
    const stat = statSync(filePath);
    if (!stat.isFile()) {
      next();
      return;
    }
    res.statusCode = 200;
    res.setHeader("Content-Type", contentTypeForPath(filePath));
    createReadStream(filePath).pipe(res as NodeJS.WritableStream);
  } catch {
    next();
  }
}

function contentTypeForPath(filePath: string): string {
  const ext = path.extname(filePath).toLowerCase();
  if (ext === ".urdf" || ext === ".dae" || ext === ".xml") return "application/xml; charset=utf-8";
  if (ext === ".json") return "application/json; charset=utf-8";
  return "application/octet-stream";
}

function cadqueryTopologyPlugin(): Plugin {
  return {
    name: "cadquery-seam-api",
    configureServer(server) {
      server.middlewares.use(async (req, res, next) => {
        const url = req.url ?? "/";
        const pathname = url.split("?", 1)[0].replace(/\/+$/, "");
        if (pathname !== "/api/cadquery/topology") {
          next();
          return;
        }
        if (req.method !== "POST") {
          writeJSON(res, 405, { error: "POST required" });
          return;
        }
        try {
          const body = await readRequestBody(req);
          const result = await runCadqueryTopologyExtractor(body);
          res.statusCode = 200;
          res.setHeader("Content-Type", "application/json; charset=utf-8");
          res.end(result);
        } catch (error) {
          const message = error instanceof Error ? error.message : String(error);
          server.config.logger.error(`cadquery topology extraction failed: ${message}`);
          writeJSON(res, 500, { error: message });
        }
      });
    },
  };
}

function readRequestBody(req: NodeJS.ReadableStream): Promise<string> {
  return new Promise((resolve, reject) => {
    const chunks: Buffer[] = [];
    req.on("data", (chunk: Buffer | string) => {
      chunks.push(Buffer.isBuffer(chunk) ? chunk : Buffer.from(chunk));
    });
    req.on("end", () => resolve(Buffer.concat(chunks).toString("utf8")));
    req.on("error", reject);
  });
}

function runCadqueryTopologyExtractor(stdinPayload: string, operation?: string): Promise<string> {
  const explicitPython = process.env.CADQUERY_TOPOLOGY_PYTHON?.trim();
  const command = explicitPython || process.env.CADQUERY_PIXI?.trim() || "pixi";
  const args = explicitPython
    ? [path.join("python", "extract_weld_edges.py"), "--stdin"]
    : ["run", "python", path.join("python", "extract_weld_edges.py"), "--stdin"];
  const payload = normalizeTopologyRequest(stdinPayload, operation);

  return new Promise((resolve, reject) => {
    const child = spawn(command, args, {
      cwd: cadqueryRoot,
      stdio: ["pipe", "pipe", "pipe"],
      windowsHide: true,
    });
    const stdout: Buffer[] = [];
    const stderr: Buffer[] = [];
    const timeout = setTimeout(() => {
      child.kill();
      reject(new Error("Geometry topology extraction timed out after 90s."));
    }, 90_000);

    child.stdout.on("data", (chunk: Buffer) => stdout.push(chunk));
    child.stderr.on("data", (chunk: Buffer) => stderr.push(chunk));
    child.on("error", (error) => {
      clearTimeout(timeout);
      reject(error);
    });
    child.on("close", (code) => {
      clearTimeout(timeout);
      const out = Buffer.concat(stdout).toString("utf8").trim();
      const err = Buffer.concat(stderr).toString("utf8").trim();
      if (code !== 0) {
        reject(new Error(err || `extract_weld_edges.py exited with ${code}`));
        return;
      }
      if (!out) {
        reject(new Error("extract_weld_edges.py returned no JSON."));
        return;
      }
      resolve(out);
    });
    child.stdin.end(payload);
  });
}

function normalizeTopologyRequest(stdinPayload: string, operation?: string): string {
  const request = JSON.parse(stdinPayload || "{}") as Record<string, unknown>;
  if (operation) {
    request.operation = operation;
  }
  if (request.example === "dog_box") {
    const stepPath = path.join(cadqueryRoot, "step", "dog_box.step");
    request.filename = "dog_box.step";
    request.step_base64 = readFileSync(stepPath).toString("base64");
  }
  return JSON.stringify(request);
}

function writeJSON(res: { statusCode: number; setHeader: (name: string, value: string) => void; end: (value: string) => void }, statusCode: number, body: unknown): void {
  res.statusCode = statusCode;
  res.setHeader("Content-Type", "application/json; charset=utf-8");
  res.end(JSON.stringify(body));
}
