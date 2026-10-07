// Offline cache for the builder's own static files only. The ROM and the
// generated DAT are never cached (they are not fetched over the network).
const CACHE = "ssb64-builder-v3";
const FILES = [
  "./",
  "index.html",
  "style.css",
  "app.js",
  "builder.worker.js",
  "app.webmanifest",
  "core/build.js",
  "core/layout.js",
  "core/n64reloc.js",
  "core/pack.js",
  "core/rom.js",
  "core/spu.js",
  "core/vpk0.js",
  "data/ssb64-us.manifest.json.gz",
];

self.addEventListener("install", (e) => {
  e.waitUntil(caches.open(CACHE).then((c) => c.addAll(FILES)).then(() => self.skipWaiting()));
});

self.addEventListener("activate", (e) => {
  e.waitUntil(
    caches.keys()
      .then((keys) => Promise.all(keys.filter((k) => k !== CACHE).map((k) => caches.delete(k))))
      .then(() => self.clients.claim()),
  );
});

// Network first so a new release (code + manifest) is always picked up
// together; the cache is only the offline fallback.
self.addEventListener("fetch", (e) => {
  if (e.request.method !== "GET" || new URL(e.request.url).origin !== self.location.origin) return;
  e.respondWith(
    fetch(e.request)
      .then((res) => {
        if (res.ok) {
          const copy = res.clone();
          caches.open(CACHE).then((c) => c.put(e.request, copy));
        }
        return res;
      })
      .catch(() => caches.match(e.request).then((hit) => hit || Response.error())),
  );
});
