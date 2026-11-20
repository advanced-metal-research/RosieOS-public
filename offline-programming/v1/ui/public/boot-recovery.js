// Stable-named recovery script loaded before the hashed app bundle.
//
// A browser profile can end up wedged in ways the app bundle cannot repair,
// because the bundle itself is what fails to run: a stale service-worker cache
// pinning an old asset, or a storage layer that never settles. This file keeps
// a fixed URL so a fresh copy always reaches the page, and reloads once after
// clearing worker + cache state when boot stops making progress.
(function () {
  "use strict";

  var STALL_MS = 12000;
  var RECOVERY_FLAG = "olp:boot-recovery-attempted";
  var timer = null;
  var finished = false;

  function purge(includeStorage) {
    var jobs = [];

    if (navigator.serviceWorker && navigator.serviceWorker.getRegistrations) {
      jobs.push(
        navigator.serviceWorker
          .getRegistrations()
          .then(function (registrations) {
            return Promise.all(
              registrations.map(function (registration) {
                return registration.unregister();
              }),
            );
          })
          .catch(function () {}),
      );
    }

    if (self.caches && caches.keys) {
      jobs.push(
        caches
          .keys()
          .then(function (keys) {
            return Promise.all(
              keys.map(function (key) {
                return caches.delete(key);
              }),
            );
          })
          .catch(function () {}),
      );
    }

    if (includeStorage && self.indexedDB && indexedDB.deleteDatabase) {
      jobs.push(
        new Promise(function (resolve) {
          var request = indexedDB.deleteDatabase("offline-programming-v1");
          // `blocked` means another tab holds the connection; do not hang on it.
          request.onsuccess = resolve;
          request.onerror = resolve;
          request.onblocked = resolve;
          setTimeout(resolve, 2000);
        }),
      );
    }

    return Promise.all(jobs);
  }

  function banner(text) {
    var node = document.createElement("div");
    node.setAttribute("role", "status");
    node.style.cssText =
      "position:fixed;inset:0;z-index:99999;display:flex;align-items:center;" +
      "justify-content:center;background:#090b0e;color:#edf1f5;padding:2rem;" +
      "font:14px/1.45 ui-sans-serif,system-ui,sans-serif;text-align:center";
    node.textContent = text;
    (document.body || document.documentElement).appendChild(node);
  }

  function recover() {
    if (finished) return;
    finished = true;

    if (sessionStorage.getItem(RECOVERY_FLAG)) {
      banner(
        "Offline Programming could not finish loading, and clearing the cached " +
          "app did not help. Open DevTools \u2192 Application \u2192 Clear site data, " +
          "or reload with ?reset=all to also drop the local project store.",
      );
      return;
    }

    sessionStorage.setItem(RECOVERY_FLAG, String(Date.now()));
    banner("Clearing a stale cached copy of the app and reloading\u2026");
    purge(false).then(function () {
      location.reload();
    });
  }

  function arm() {
    if (finished) return;
    clearTimeout(timer);
    timer = setTimeout(recover, STALL_MS);
  }

  window.__olpBootProgress = arm;

  window.__olpBootDone = function () {
    finished = true;
    clearTimeout(timer);
    sessionStorage.removeItem(RECOVERY_FLAG);
  };

  var params = new URLSearchParams(location.search);
  var reset = params.get("reset");
  if (reset) {
    finished = true;
    banner("Resetting local app state\u2026");
    purge(reset === "all").then(function () {
      params.delete("reset");
      var query = params.toString();
      location.replace(location.pathname + (query ? "?" + query : ""));
    });
    return;
  }

  arm();
})();
