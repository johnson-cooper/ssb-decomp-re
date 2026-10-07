// Runs the whole conversion off the main thread. The ROM arrives as a
// transferred ArrayBuffer and never leaves this worker except as SSB64.DAT.
import { verifyRom } from "./core/rom.js";
import { buildDat, BuildError, expectedDatSha256 } from "./core/build.js";

const MANIFEST_URL = new URL("./data/ssb64-us.manifest.json.gz", import.meta.url);
const WEIGHTS = [
  ["Loading conversion data", 0.02],
  ["Processing relocations", 0.08],
  ["Converting audio samples", 0.6],
  ["Building asset pack", 0.25],
  ["Verifying output", 0.05],
];

let rom = null;

async function loadManifest() {
  const res = await fetch(MANIFEST_URL);
  if (!res.ok) throw new BuildError("MANIFEST", `could not load conversion data (${res.status})`);
  const text = await new Response(res.body.pipeThrough(new DecompressionStream("gzip"))).text();
  return JSON.parse(text);
}

function report(stage, fraction) {
  let done = 0;
  for (const [name, w] of WEIGHTS) {
    if (name === stage) {
      postMessage({ type: "progress", stage, percent: Math.min(100, Math.round((done + w * fraction) * 100)) });
      return;
    }
    done += w;
  }
  postMessage({ type: "progress", stage, percent: stage === "Complete" ? 100 : Math.round(done * 100) });
}

function fail(err) {
  const code = err instanceof BuildError ? err.code : err instanceof RangeError ? "OUT_OF_MEMORY" : "CONVERSION";
  postMessage({ type: "error", code, detail: err && err.stack ? String(err.stack) : String(err) });
}

self.onmessage = async (e) => {
  const msg = e.data;
  try {
    if (msg.type === "verify") {
      rom = null;
      const bytes = new Uint8Array(msg.rom);
      const info = await verifyRom(bytes);
      rom = bytes;
      postMessage({ type: "verified", info });
    } else if (msg.type === "build") {
      if (!rom) throw new BuildError("NO_ROM", "no verified ROM loaded");
      report("Loading conversion data", 0);
      const manifest = await loadManifest();
      const dat = await buildDat(rom, manifest, report);
      postMessage({ type: "done", dat: dat.buffer, size: dat.length, sha256: expectedDatSha256(manifest) }, [dat.buffer]);
    }
  } catch (err) {
    fail(err);
  }
};
