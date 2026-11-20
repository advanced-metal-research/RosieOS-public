const CACHE_NAME = "offline-programming-v1-pwa-v5";
const APP_BASE = new URL("./", self.registration.scope).pathname;
// Precache only the manifest. Never precache the HTML app shell alone —
// hashed JS/CSS are not in this list, so a shell-only offline fallback paints
// a blank white page with the correct tab title.
const APP_SHELL = [
  `${APP_BASE}manifest.webmanifest`,
];

self.addEventListener("install", (event) => {
  event.waitUntil(
    caches.open(CACHE_NAME)
      .then((cache) => cache.addAll(APP_SHELL))
      .then(() => self.skipWaiting()),
  );
});

self.addEventListener("activate", (event) => {
  event.waitUntil(
    caches.keys()
      .then((keys) => Promise.all(keys.filter((key) => key !== CACHE_NAME).map((key) => caches.delete(key))))
      .then(() => self.clients.claim()),
  );
});

self.addEventListener("fetch", (event) => {
  const request = event.request;
  if (request.method !== "GET") return;
  const url = new URL(request.url);
  if (url.origin !== self.location.origin || url.pathname.startsWith("/api/")) return;
  if (request.mode === "navigate") {
    event.respondWith(networkFirstNavigate(request));
    return;
  }
  // Only content-hashed bundles are safe to pin forever. Stable-named files
  // (boot-recovery.js, manifest, robot assets) must be able to change, or a
  // single bad cache entry wedges the profile with no way back.
  event.respondWith(isImmutableAsset(url) ? cacheFirst(request) : staleWhileRevalidate(event));
});

function isImmutableAsset(url) {
  return url.pathname.startsWith(`${APP_BASE}assets/`);
}

async function networkFirstNavigate(request) {
  const cache = await caches.open(CACHE_NAME);
  try {
    const response = await fetch(request);
    if (response.ok) await cache.put(request, response.clone());
    return response;
  } catch {
    // Prefer an explicit offline page over a JS-less cached shell (WSoD).
    return offlineShellResponse();
  }
}

function offlineShellResponse() {
  return new Response(
    `<!doctype html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>Offline Programming V1</title>
  <style>
    html, body {
      margin: 0;
      min-height: 100%;
      background: #090b0e;
      color: #edf1f5;
      font: 14px/1.45 ui-sans-serif, system-ui, sans-serif;
    }
    main { max-width: 36rem; padding: 2.5rem 1.5rem; }
    h1 { font-size: 1.15rem; font-weight: 600; margin: 0 0 0.75rem; }
    p { margin: 0 0 0.75rem; color: #8994a1; }
    code { color: #ff7a1a; }
  </style>
</head>
<body>
  <main>
    <h1>Offline Programming server unreachable</h1>
    <p>The UI shell cannot boot without the local Go serve process and its hashed assets.</p>
    <p>Start <code>./rosie.sh offline-programming v1 serve</code> (or your usual <code>go run . serve --listen …</code>) and reload this tab.</p>
  </main>
</body>
</html>`,
    {
      status: 503,
      headers: { "Content-Type": "text/html; charset=utf-8" },
    },
  );
}

async function cacheFirst(request) {
  const cache = await caches.open(CACHE_NAME);
  const cached = await cache.match(request);
  if (cached) return cached;
  const response = await fetch(request);
  if (response.ok) await cache.put(request, response.clone());
  return response;
}

async function staleWhileRevalidate(event) {
  const request = event.request;
  const cache = await caches.open(CACHE_NAME);
  const cached = await cache.match(request);
  const network = fetch(request)
    .then(async (response) => {
      if (response.ok) await cache.put(request, response.clone());
      return response;
    })
    .catch(() => null);
  if (cached) {
    event.waitUntil(network);
    return cached;
  }
  const response = await network;
  if (response) return response;
  throw new Error(`offline and uncached: ${request.url}`);
}
