#include "olp_core.hpp"
#include <QFile>
#include <QTimer>

namespace {
// What OLP's modules assume of a current browser and Qt 5.15's engine lacks:
// ES2019-2023 built-ins (esbuild lowers syntax, not library calls), and two
// browser globals. structuredClone only ever copies JSON-shaped program data
// here; node ids need uniqueness, not cryptographic strength. Each is defined
// only where missing, non-enumerable, with the standard's behaviour.
constexpr const char *prelude = R"(
var globalThis = this;
(function () {
  function define(target, name, value) {
    if (!(name in target)) Object.defineProperty(target, name, { value: value, writable: true, configurable: true });
  }
  function at(index) {
    var n = Math.trunc(index) || 0;
    if (n < 0) n += this.length;
    return n < 0 || n >= this.length ? undefined : this[n];
  }
  function flat(depth) {
    var limit = depth === undefined ? 1 : Math.trunc(depth) || 0, out = [];
    (function walk(list, level) {
      for (var i = 0; i < list.length; i++) {
        if (!(i in list)) continue;
        if (Array.isArray(list[i]) && level < limit) walk(list[i], level + 1); else out.push(list[i]);
      }
    })(this, 0);
    return out;
  }
  define(Object, "fromEntries", function (entries) {
    var out = {};
    for (var entry of entries) out[entry[0]] = entry[1];
    return out;
  });
  define(Object, "hasOwn", function (object, key) { return Object.prototype.hasOwnProperty.call(object, key); });
  define(Array.prototype, "at", at);
  define(String.prototype, "at", at);
  define(Array.prototype, "flat", flat);
  define(Array.prototype, "flatMap", function (fn, self) { return flat.call(this.map(fn, self), 1); });
  define(Array.prototype, "findLastIndex", function (fn, self) {
    for (var i = this.length - 1; i >= 0; i--) if (fn.call(self, this[i], i, this)) return i;
    return -1;
  });
  define(Array.prototype, "findLast", function (fn, self) {
    var i = this.findLastIndex(fn, self);
    return i < 0 ? undefined : this[i];
  });
  define(String.prototype, "replaceAll", function (search, replacement) {
    if (search instanceof RegExp) return this.replace(search, replacement);
    return this.split(String(search)).join(typeof replacement === "function" ? replacement(search) : replacement);
  });
  define(String.prototype, "trimStart", function () { return this.replace(/^\s+/, ""); });
  define(String.prototype, "trimEnd", function () { return this.replace(/\s+$/, ""); });
  if (typeof TextEncoder === "undefined") {
    globalThis.TextEncoder = function TextEncoder() {};
    globalThis.TextEncoder.prototype.encode = function (text) {
      var utf8 = unescape(encodeURIComponent(String(text === undefined ? "" : text))), bytes = new Uint8Array(utf8.length);
      for (var i = 0; i < utf8.length; i++) bytes[i] = utf8.charCodeAt(i);
      return bytes;
    };
  }
  if (typeof TextDecoder === "undefined") {
    globalThis.TextDecoder = function TextDecoder() {};
    globalThis.TextDecoder.prototype.decode = function (bytes) {
      var view = bytes instanceof Uint8Array ? bytes : new Uint8Array(bytes || []), binary = "";
      var chars = [];
      for (var i = 0; i < view.length; i++) chars.push(String.fromCharCode(view[i]));
      binary = chars.join("");
      return decodeURIComponent(escape(binary));
    };
  }
  var alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  if (typeof atob === "undefined") {
    globalThis.atob = function (text) {
      var clean = String(text).replace(/[\s=]+/g, ""), out = [], bits = 0, value = 0;
      for (var i = 0; i < clean.length; i++) {
        var index = alphabet.indexOf(clean.charAt(i));
        if (index < 0) throw new Error("atob: the string is not valid base64");
        value = ((value << 6) | index) & 0xffffff;
        bits += 6;
        if (bits >= 8) { bits -= 8; out.push(String.fromCharCode((value >> bits) & 255)); }
      }
      return out.join("");
    };
  }
  // SHA-256 (FIPS 180-4) for crypto.subtle.digest: OLP digests a program's
  // canonical JSON, and the digest must equal the browser's byte for byte.
  var K = [0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2];
  function sha256(bytes) {
    var length = bytes.length, padded = new Uint8Array(((length + 9 + 63) >> 6) << 6);
    padded.set(bytes);
    padded[length] = 0x80;
    var bitsHigh = Math.floor(length / 0x20000000), bitsLow = (length << 3) >>> 0, end = padded.length;
    padded[end - 8] = bitsHigh >>> 24; padded[end - 7] = bitsHigh >>> 16; padded[end - 6] = bitsHigh >>> 8; padded[end - 5] = bitsHigh;
    padded[end - 4] = bitsLow >>> 24; padded[end - 3] = bitsLow >>> 16; padded[end - 2] = bitsLow >>> 8; padded[end - 1] = bitsLow;
    var h = [0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19], w = new Array(64);
    function rotr(x, n) { return (x >>> n) | (x << (32 - n)); }
    for (var offset = 0; offset < end; offset += 64) {
      for (var t = 0; t < 16; t++) {
        var j = offset + t * 4;
        w[t] = ((padded[j] << 24) | (padded[j + 1] << 16) | (padded[j + 2] << 8) | padded[j + 3]) | 0;
      }
      for (t = 16; t < 64; t++) {
        var s0 = rotr(w[t - 15], 7) ^ rotr(w[t - 15], 18) ^ (w[t - 15] >>> 3);
        var s1 = rotr(w[t - 2], 17) ^ rotr(w[t - 2], 19) ^ (w[t - 2] >>> 10);
        w[t] = (w[t - 16] + s0 + w[t - 7] + s1) | 0;
      }
      var a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], k = h[7];
      for (t = 0; t < 64; t++) {
        var t1 = (k + (rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25)) + ((e & f) ^ (~e & g)) + K[t] + w[t]) | 0;
        var t2 = ((rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22)) + ((a & b) ^ (a & c) ^ (b & c))) | 0;
        k = g; g = f; f = e; e = (d + t1) | 0; d = c; c = b; b = a; a = (t1 + t2) | 0;
      }
      h[0] = (h[0] + a) | 0; h[1] = (h[1] + b) | 0; h[2] = (h[2] + c) | 0; h[3] = (h[3] + d) | 0;
      h[4] = (h[4] + e) | 0; h[5] = (h[5] + f) | 0; h[6] = (h[6] + g) | 0; h[7] = (h[7] + k) | 0;
    }
    var out = new Uint8Array(32);
    for (var i = 0; i < 8; i++) { out[i * 4] = h[i] >>> 24; out[i * 4 + 1] = h[i] >>> 16; out[i * 4 + 2] = h[i] >>> 8; out[i * 4 + 3] = h[i]; }
    return out.buffer;
  }
  globalThis.__sha256 = sha256;
  // Replaced, not filled in: Qt 5.15's own WeakMap and WeakSet segfault the
  // engine in a later lookup once the collector has reclaimed a key
  // (a Steam Deck on Qt 5.15.17, 2026-10-11; reproduced on 5.15.3; a Map
  // with the same traffic does not), and esbuild lowers every #private field
  // to a WeakMap and every #private method to a WeakSet. Each store keeps
  // its entry on the key, under a symbol only it knows, so the entry dies
  // with the key and the engine's weak tables are never used. A key that
  // cannot take a property (frozen first) is held strongly instead.
  var weakSerial = 0;
  function weakInit(store, kind) {
    Object.defineProperty(store, "__slot", { value: Symbol(kind + " " + ++weakSerial) });
    Object.defineProperty(store, "__fixed", { value: [] });
  }
  function weakSlot(store, key, kind, create) {
    if (key === null || (typeof key !== "object" && typeof key !== "function")) {
      if (create) throw new TypeError("Invalid value used as " + kind + " key");
      return undefined;
    }
    if (Object.prototype.hasOwnProperty.call(key, store.__slot)) return key[store.__slot];
    for (var i = 0; i < store.__fixed.length; i++) if (store.__fixed[i].key === key) return store.__fixed[i];
    if (!create) return undefined;
    var slot = { key: undefined, live: false, value: undefined };
    if (Object.isExtensible(key)) Object.defineProperty(key, store.__slot, { value: slot });
    else { slot.key = key; store.__fixed.push(slot); }
    return slot;
  }
  function weakDelete(store, key, kind) {
    var slot = weakSlot(store, key, kind, false);
    if (!slot || !slot.live) return false;
    slot.live = false;
    slot.value = undefined;
    var fixed = store.__fixed.indexOf(slot);
    if (fixed >= 0) store.__fixed.splice(fixed, 1);
    return true;
  }
  function WeakMap(entries) {
    if (!(this instanceof WeakMap)) throw new TypeError("Constructor WeakMap requires 'new'");
    weakInit(this, "WeakMap");
    if (entries !== undefined && entries !== null) for (var entry of entries) this.set(entry[0], entry[1]);
  }
  WeakMap.prototype.get = function (key) {
    var slot = weakSlot(this, key, "WeakMap", false);
    return slot && slot.live ? slot.value : undefined;
  };
  WeakMap.prototype.has = function (key) { var slot = weakSlot(this, key, "WeakMap", false); return !!(slot && slot.live); };
  WeakMap.prototype.set = function (key, value) {
    var slot = weakSlot(this, key, "WeakMap", true);
    slot.live = true;
    slot.value = value;
    return this;
  };
  WeakMap.prototype["delete"] = function (key) { return weakDelete(this, key, "WeakMap"); };
  function WeakSet(values) {
    if (!(this instanceof WeakSet)) throw new TypeError("Constructor WeakSet requires 'new'");
    weakInit(this, "WeakSet");
    if (values !== undefined && values !== null) for (var value of values) this.add(value);
  }
  WeakSet.prototype.add = function (key) { weakSlot(this, key, "WeakSet", true).live = true; return this; };
  WeakSet.prototype.has = function (key) { var slot = weakSlot(this, key, "WeakSet", false); return !!(slot && slot.live); };
  WeakSet.prototype["delete"] = function (key) { return weakDelete(this, key, "WeakSet"); };
  globalThis.WeakMap = WeakMap;
  globalThis.WeakSet = WeakSet;
})();
globalThis.structuredClone = function (value) {
  return value === undefined ? undefined : JSON.parse(JSON.stringify(value));
};
globalThis.crypto = {
  randomUUID: function () {
    return "xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx".replace(/[xy]/g, function (c) {
      var r = Math.random() * 16 | 0;
      return (c === "x" ? r : (r & 3 | 8)).toString(16);
    });
  },
  subtle: {
    digest: function (algorithm, data) {
      var name = typeof algorithm === "string" ? algorithm : algorithm && algorithm.name;
      if (String(name).toUpperCase() !== "SHA-256") return Promise.reject(new Error("crypto.subtle.digest: only SHA-256"));
      var bytes = data instanceof Uint8Array ? data : new Uint8Array(data instanceof ArrayBuffer ? data : data.buffer);
      return Promise.resolve(globalThis.__sha256(bytes));
    }
  }
};
// An async call's outcome, read back by OlpCore::callAsync.
globalThis.__settled = {};
globalThis.__settle = function (value, id) {
  Promise.resolve(value).then(function (v) { globalThis.__settled[id] = { ok: true, value: v === undefined ? null : v }; },
    function (e) { globalThis.__settled[id] = { ok: false, error: String(e && e.message || e) }; });
};
)";

QString describe(const QJSValue &error) {
  const QString message = error.property("message").toString();
  return message.isEmpty() ? error.toString() : message;
}
} // namespace

OlpCore::OlpCore(const QString &bundlePath) {
  engine.installExtensions(QJSEngine::ConsoleExtension);
  QFile file(bundlePath);
  if (!file.open(QIODevice::ReadOnly))
    throw std::runtime_error(("OLP core bundle missing: " + bundlePath).toStdString());
  for (const auto &[source, name] : {std::pair{QString(prelude), QString("prelude")},
                                     std::pair{QString::fromUtf8(file.readAll()), bundlePath}}) {
    const QJSValue result = engine.evaluate(source, name);
    if (result.isError())
      throw std::runtime_error(QString("%1 line %2: %3").arg(name)
          .arg(result.property("lineNumber").toInt()).arg(describe(result)).toStdString());
  }
  api = engine.globalObject().property("olp");
  if (!api.isObject())
    throw std::runtime_error("OLP core bundle defines no olp object");
}

QJsonValue OlpCore::call(const QString &function, const QJsonArray &args) {
  static const bool trace = qEnvironmentVariableIsSet("OLP_CORE_TRACE");
  if (trace)
    qInfo("olp.%s(%s)", qPrintable(function), qPrintable(QString::number(args.size())));
  QJSValueList values;
  for (const auto &arg : args)
    values << engine.toScriptValue(arg.toVariant());
  return invoke(function, values);
}

QJsonValue OlpCore::callBytes(const QString &function, const QByteArray &bytes, const QJsonArray &args) {
  // These functions only read the buffer; they never return it or expose it
  // to an editor. Exact native byte equality preserves the JS cache's source
  // identity across path/start/preview without a per-byte JavaScript loop.
  if (function != "plannedPathRuns" && function != "planStart" && function != "previewPath")
    throw OlpError("Binary input is only supported for trajectory readers");
  constexpr int maxKeptBytes = 2 * 1024 * 1024;
  QJSValue input;
  if (bytes.size() <= maxKeptBytes) {
    if (!binaryValue.isObject() || binaryBytes != bytes) {
      // Own storage even if a caller supplied QByteArray::fromRawData.
      binaryBytes = QByteArray(bytes.constData(), bytes.size());
      binaryValue = engine.toScriptValue(binaryBytes);
    }
    input = binaryValue;
  } else {
    binaryBytes.clear(); binaryValue = QJSValue();
    input = engine.toScriptValue(bytes);
  }
  QJSValueList values{input};
  for (const auto &arg : args) values << engine.toScriptValue(arg.toVariant());
  return invoke(function, values);
}

QJsonValue OlpCore::invoke(const QString &function, const QJSValueList &values) {
  QJSValue target = api.property(function);
  if (!target.isCallable())
    throw OlpError(("OLP core has no " + function).toStdString());
  const QJSValue result = target.call(values);
  if (result.isError()) {
    // OLP's refusals are Errors with a message; anything else is an engine
    // fault in the bundle, reported with where it happened.
    const QString message = result.property("message").toString();
    if (result.property("name").toString() == "Error" && !message.isEmpty())
      throw OlpError(message.toStdString());
    throw OlpError(QString("%1: %2 at bundle line %3\n%4").arg(function, describe(result))
        .arg(result.property("lineNumber").toInt()).arg(result.property("stack").toString()).toStdString());
  }
  return QJsonValue::fromVariant(result.toVariant());
}

void OlpCore::callAsync(const QString &function, const QJsonArray &args, AsyncDone done) {
  QJSValue target = api.property(function);
  if (!target.isCallable()) {
    done(std::nullopt, "OLP core has no " + function);
    return;
  }
  QJSValueList values;
  for (const auto &arg : args) values << engine.toScriptValue(arg.toVariant());
  const QJSValue result = target.call(values);
  if (result.isError()) {
    done(std::nullopt, describe(result));
    return;
  }
  // The engine runs promise reactions from the event loop, so the outcome is
  // collected on a timer rather than inside this call.
  const QString id = QString::number(++asyncSequence);
  engine.globalObject().property("__settle").call({result, id});
  auto *timer = new QTimer;
  timer->setInterval(2);
  QObject::connect(timer, &QTimer::timeout, timer, [this, timer, id, done] {
    QJSValue settled = engine.globalObject().property("__settled");
    const QJSValue outcome = settled.property(id);
    if (outcome.isUndefined()) return;
    settled.deleteProperty(id);
    timer->deleteLater();
    if (outcome.property("ok").toBool()) done(QJsonValue::fromVariant(outcome.property("value").toVariant()), {});
    else done(std::nullopt, outcome.property("error").toString());
  });
  timer->start();
}
