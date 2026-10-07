// ROM -> SSB64.DAT, driven by the conversion manifest.
// Mirrors ps2/tools/build_assets.py main(); see README.md for the mapping.
import { readRelocTable, relocFileBytes, externIds, walkRelocs } from "./n64reloc.js";
import { applyOps } from "./layout.js";
import { layoutPack, writePackHeader } from "./pack.js";
import { buildSpuSamples, SPU_SAMPLES_VROM } from "./spu.js";

const BOOT_SIZE = 0x1000;

async function sha256Hex(bytes) {
  const d = new Uint8Array(await crypto.subtle.digest("SHA-256", bytes));
  return Array.from(d, (x) => x.toString(16).padStart(2, "0")).join("");
}

export class BuildError extends Error {
  constructor(code, message) {
    super(message);
    this.code = code;
  }
}

/*
 * The browser conversion manifest predates the final real-hardware DAT fixes
 * in ps2/tools/build_assets.py.  Keep the site builder byte-for-byte aligned
 * with those fixes without requiring users to run a local compiler.
 *
 * swap16 entries undo stale TLUT classification that had byte-swapped CI
 * texels after runtime MObj display lists.  swap32halves fixes Pikachu's two
 * accessory costume material-animation streams, which were extracted as u16
 * palette-looking blocks but are consumed as AObjEvent32 scripts.
 *
 * These ranges were audited against the local hardware-validated DAT builder
 * using a verified SSB64 US 1.0 ROM.  The browser DAT intentionally still
 * differs from the local compiler-built pack in the pre-existing fid 199/200
 * .text regions that the browser manifest zeroes; those are unrelated to
 * fighter assets and these hardware fixes.
 */
const LEGACY_MANIFEST_OUTPUT_SHA256 = "baad2cdc52e039353863fdda2ba41c2a753aacd7d53a7b7eebe45ba9137a6366";
export const HARDWARE_FIXED_OUTPUT_SHA256 = "151b492025ac780c5114b8a842b157c73ed56208e7ddfff1731ad74882eb809e";

const HARDWARE_DAT_POST_OPS = new Map([
  [86,  [["swap16", 0x1240, 0x1254]]],
  [105, [["swap16", 0x192e, 0x1940], ["swap16", 0x2540, 0x2560]]],
  [317, [["swap16", 0xc0f8, 0xc118], ["swap16", 0xc37a, 0xc394], ["swap16", 0xca78, 0xca96],
         ["swap16", 0xccd8, 0xccec], ["swap16", 0xcf88, 0xcfa8], ["swap16", 0xd20e, 0xd21e],
         ["swap16", 0xd288, 0xd2a8]]],
  [320, [["swap16", 0xb812, 0xb81a], ["swap16", 0xbada, 0xbaf0], ["swap16", 0xbfd0, 0xbfe8],
         ["swap16", 0xc8e8, 0xc8fa], ["swap16", 0xcedc, 0xcef4]]],
  [330, [["swap16", 0x67d2, 0x67f0], ["swap16", 0x697a, 0x698e]]],
  [332, [["swap16", 0xb522, 0xb534], ["swap16", 0xbe40, 0xbe60], ["swap16", 0xc000, 0xc018],
         ["swap16", 0xc130, 0xc142]]],
  [338, [["swap16", 0x9532, 0x9534], ["swap16", 0x9bd0, 0x9bf0], ["swap16", 0x9e82, 0x9e9c]]],
  [341, [["swap32halves", 0x6504, 0x6548], ["swap16", 0x80f8, 0x8100]]],
]);

function applyPostOps(dst, ops) {
  if (!ops) return;
  for (const op of ops) {
    const kind = Array.isArray(op) ? op[0] : op.kind;
    const start = Array.isArray(op) ? op[1] : op.start;
    const end = Array.isArray(op) ? op[2] : op.end;
    const step = kind === "swap16" ? 2 : kind === "swap32halves" ? 4 : 0;
    if (!step || !Number.isInteger(start) || !Number.isInteger(end) ||
        start < 0 || end <= start || end > dst.length ||
        (start % step) !== 0 || ((end - start) % step) !== 0) {
      throw new BuildError("MANIFEST", `invalid DAT post-op ${kind} [${start}, ${end})`);
    }
    if (kind === "swap16") {
      for (let o = start; o < end; o += 2) {
        const a = dst[o];
        dst[o] = dst[o + 1];
        dst[o + 1] = a;
      }
    } else {
      for (let o = start; o < end; o += 4) {
        const a = dst[o], b = dst[o + 1];
        dst[o] = dst[o + 2];
        dst[o + 1] = dst[o + 3];
        dst[o + 2] = a;
        dst[o + 3] = b;
      }
    }
  }
}

export function expectedDatSha256(manifest) {
  return manifest.output.sha256 === LEGACY_MANIFEST_OUTPUT_SHA256
    ? HARDWARE_FIXED_OUTPUT_SHA256
    : manifest.output.sha256;
}

/**
 * rom: Uint8Array (.z64, verified). manifest: parsed manifest object.
 * progress(stage, fraction): optional. Returns Uint8Array (the DAT).
 */
export async function buildDat(rom, manifest, progress = () => {}) {
  if (manifest.format !== 1) throw new BuildError("MANIFEST", `unsupported manifest format ${manifest.format}`);
  const { count, vrom: relocVrom } = manifest.relocTable;
  const layouts = manifest.layouts;
  const table64 = readRelocTable(rom);
  let scratch = new Uint8Array(0x40000);
  const fileBytes = (fid) => {
    const need = table64[fid].dsize * 4;
    if (scratch.length < need) scratch = new Uint8Array(need);
    return relocFileBytes(rom, table64, fid, scratch);
  };

  // ---- 1. sizes and relocations ----
  progress("Processing relocations", 0);
  const files = [];
  for (let fid = 0; fid < count; fid++) {
    const m = manifest.relocFiles[fid];
    const src = fileBytes(fid);
    const relocs = walkRelocs(fid, src, table64[fid], externIds(rom, table64, fid));
    files.push({
      size: m.size !== undefined ? m.size : table64[fid].dsize * 4,
      intern: relocs.filter((r) => !r.ext).sort((a, b) => a.slot - b.slot),
      extern: relocs.filter((r) => r.ext).sort((a, b) => a.slot - b.slot),
    });
    if (fid % 64 === 0) progress("Processing relocations", fid / count);
  }

  // ---- 2. SPU samples (size needed for the layout) ----
  progress("Converting audio samples", 0);
  const spu = buildSpuSamples(rom, (i, n) => { if (i % 8 === 0) progress("Converting audio samples", i / n); });

  // ---- 3. region layout ----
  const tableBytes = 12 * (count + 1);
  const hi = relocVrom + tableBytes;
  const regions = [];
  const boot = { vrom: 0, size: BOOT_SIZE, resident: true };
  const tableRegion = { vrom: relocVrom, size: tableBytes, resident: true };
  regions.push(boot, tableRegion);
  let bodyOff = 0;
  for (const f of files) {
    f.dataOffset = bodyOff;
    f.region = { vrom: hi + bodyOff, size: f.size, resident: false };
    regions.push(f.region);
    bodyOff += f.size;
    if (f.extern.length) {
      f.extRegion = { vrom: hi + bodyOff, size: 2 * f.extern.length, resident: true };
      regions.push(f.extRegion);
      bodyOff += 2 * f.extern.length;
    }
  }
  const romRegions = manifest.regions.map((r) => ({ vrom: r.vrom, size: r.size, resident: false, ops: r.ops }));
  regions.push(...romRegions);
  const spuRegion = { vrom: SPU_SAMPLES_VROM, size: spu.length, resident: false };
  regions.push(spuRegion);

  const layout = layoutPack(regions);
  if (layout.totalSize !== manifest.output.size) {
    throw new BuildError("OUTPUT_SIZE", `layout gives ${layout.totalSize} bytes, expected ${manifest.output.size}`);
  }
  let out;
  try {
    out = new Uint8Array(layout.totalSize);
  } catch (e) {
    throw new BuildError("OUT_OF_MEMORY", String(e));
  }
  writePackHeader(out, layout, relocVrom, count);
  const view = (r) => out.subarray(r.fileOffset, r.fileOffset + r.size);

  // ---- 4. fill ----
  progress("Building asset pack", 0);
  view(boot).set(rom.subarray(0, BOOT_SIZE));

  const tv = new DataView(out.buffer, tableRegion.fileOffset, tableBytes);
  for (let fid = 0; fid < count; fid++) {
    const f = files[fid];
    const dst = view(f.region);
    const fm = manifest.relocFiles[fid];
    applyOps(fileBytes(fid), dst, fm.ops, layouts);

    // Newer manifests may carry their own postOps.  The currently deployed
    // legacy manifest does not, so layer the hardware-validated corrections
    // here until that manifest is regenerated.
    const postOps = fm.postOps || (
      manifest.output.sha256 === LEGACY_MANIFEST_OUTPUT_SHA256
        ? HARDWARE_DAT_POST_OPS.get(fid)
        : null
    );
    applyPostOps(dst, postOps);

    const dv = new DataView(dst.buffer, dst.byteOffset, dst.byteLength);
    const chain = (list) => {
      for (let i = 0; i < list.length; i++) {
        const r = list[i];
        if (r.slot % 4 || r.target % 4 || r.slot / 4 >= 0xffff || r.target / 4 >= 0xffff) {
          throw new BuildError("CONVERSION", `relocData ${fid}: pointer slot not word addressable`);
        }
        dv.setUint16(r.slot, i + 1 < list.length ? list[i + 1].slot / 4 : 0xffff, true);
        dv.setUint16(r.slot + 2, r.target / 4, true);
      }
      return list.length ? list[0].slot / 4 : 0xffff;
    };
    const internFirst = chain(f.intern);
    const externFirst = chain(f.extern);
    if (f.extRegion) {
      const ev = new DataView(out.buffer, f.extRegion.fileOffset, f.extRegion.size);
      f.extern.forEach((r, i) => ev.setUint16(2 * i, r.fid, true));
    }
    const words = f.size / 4;
    tv.setUint32(12 * fid, (f.dataOffset * 2) >>> 0, true); // (data_offset << 1) | is_compressed=0
    tv.setUint16(12 * fid + 4, internFirst, true);
    tv.setUint16(12 * fid + 6, words, true);
    tv.setUint16(12 * fid + 8, externFirst, true);
    tv.setUint16(12 * fid + 10, words, true);
    if (fid % 64 === 0) progress("Building asset pack", fid / count);
  }
  tv.setUint32(12 * count, (bodyOff * 2) >>> 0, true);
  tv.setUint16(12 * count + 4, 0xffff, true);
  tv.setUint16(12 * count + 6, 0, true);
  tv.setUint16(12 * count + 8, 0xffff, true);
  tv.setUint16(12 * count + 10, 0, true);

  for (const r of romRegions) applyOps(rom.subarray(r.vrom, r.vrom + r.size), view(r), r.ops, layouts);
  view(spuRegion).set(spu);

  // ---- 5. verify ----
  progress("Verifying output", 0);
  const got = await sha256Hex(out);
  const expected = expectedDatSha256(manifest);
  if (got !== expected) {
    const spuOk = (await sha256Hex(spu)) === manifest.output.spuSha256;
    throw new BuildError("OUTPUT_HASH", `output sha256 ${got} != ${expected} (audio samples ${spuOk ? "ok" : "differ"})`);
  }
  progress("Complete", 1);
  return out;
}

