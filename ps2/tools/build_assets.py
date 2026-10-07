#!/usr/bin/env python3
"""Build SSB64.DAT, the PS2 asset pack, from the user's own ROM extraction.

Stage 2 of the PS2 asset pipeline (stage 1 = ps2/tools/n64prep.mk, which runs
the decomp's own extraction). Everything here uses the typed decomp sources:

  * relocData: every file's master .c is compiled with the PS2Build EE
    compiler (so Gfx/Vtx/u16/u32/f32 data comes out little-endian), then the
    object's R_MIPS_32 relocations are re-encoded into the game's own
    reloc-chain format. src/lb/lbreloc.c therefore runs unmodified on PS2.
    Each file's size is checked against the original ROM table.
  * particle banks: compiled the same way (file-relative offsets, no relocs).
  * palettes: normalised to native u16 everywhere (sprite LUTs, particle
    palettes and mistyped TLUT sources are byte-swapped); texel data stays in
    the original N64 byte order. See PS2_PORT.md "Asset pipeline".

Output: ps2/build/bin/SSB64.DAT (format: ps2/include/ps2/assetpack.h).

Usage (repo root):  python3 ps2/tools/build_assets.py [--version us] [-j N]
"""
import argparse
import os
import re
import shutil
import struct
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Virtual ROM address of the (re-encoded, uncompressed) relocData table.
# Must match lLBRelocTableAddr in ps2/ps2_link.x.
RELOC_VROM = 0x20000000
SPU_SAMPLES_VROM = 0x28000000  # keep in sync with PS2_SPU_SAMPLES_VROM (ps2/include/ps2/assetpack.h)
PACK_VERSION = 1
REGION_RESIDENT = 1

# N64 ROM layout (US): relocData table and particle bank segments.
ROM_RELOC_TABLE = {"us": 0x1AC870}
PARTICLE_BANKS = [
    # name, rom start, rom end
    ("efcommon_scb", 0xAC7340, 0xAC9DE0), ("efcommon_txb", 0xAC9DE0, 0xB16C80),
    ("particles_unk0_scb", 0xB16C80, 0xB17060), ("particles_unk0_txb", 0xB17060, 0xB174A0),
    ("particles_unk1_scb", 0xB174A0, 0xB176A0), ("particles_unk1_txb", 0xB176A0, 0xB19700),
    ("particles_unk2_scb", 0xB19700, 0xB19850), ("particles_unk2_txb", 0xB19850, 0xB1BCA0),
    ("itcommon_scb", 0xB1BCA0, 0xB1BDE0), ("itcommon_txb", 0xB1BDE0, 0xB1E640),
    ("grpupupu_scb", 0xB1E640, 0xB1E7E0), ("grpupupu_txb", 0xB1E7E0, 0xB1F960),
    ("grhyrule_scb", 0xB1F960, 0xB1FC80), ("grhyrule_txb", 0xB1FC80, 0xB22980),
    ("gryoster_scb", 0xB22980, 0xB22A00), ("gryoster_txb", 0xB22A00, 0xB22C30),
    ("mntitle_scb", 0xB22C30, 0xB22D40), ("mntitle_txb", 0xB22D40, 0xB277B0),
]

# Audio segments (US ROM). "c" = compiled from the decomp's generated C
# source (native endianness), "raw" = sample bytes (ADPCM, byte stream),
# "fgm_*" = FGM engine blobs whose word fields are swapped here.
AUDIO_REGIONS = [
    ("S1_music_sbk", 0xB277B0, 0xB4E5C0, "c", "build/{v}/src/audio/S1_music_sbk.c"),
    ("B1_sounds1_ctl", 0xB4E5C0, 0xB54CE0, "c", "build/{v}/src/audio/B1_sounds1_ctl.c"),
    ("B1_sounds1_tbl", 0xB54CE0, 0xC6B650, "raw", "build/{v}/src/audio/B1_sounds1.tbl.bin"),
    ("B1_sounds2_ctl", 0xC6B650, 0xC7B1F0, "c", "build/{v}/src/audio/B1_sounds2_ctl.c"),
    ("B1_sounds2_tbl", 0xC7B1F0, 0xF573D0, "raw", "build/{v}/src/audio/B1_sounds2.tbl.bin"),
    ("fgm_unk", 0xF573D0, 0xF57BF0, "fgm_unk", "build/{v}/src/audio/fgm.unk.bin"),
    ("fgm_tbl", 0xF57BF0, 0xF5A9C0, "fgm_pkg", "build/{v}/src/audio/fgm.tbl.bin"),
    ("fgm_ucd", 0xF5A9C0, 0xF5F4E0, "fgm_pkg", "build/{v}/src/audio/fgm.ucd.bin"),
]

DEFINES = ["-DF3DEX_GBI_2", "-D_MIPS_SZLONG=32", "-DNDEBUG", "-DN_MICRO", "-D_FINALROM",
           "-DPLATFORM_PS2", "-DNON_MATCHING"]


def log(msg):
    print(msg, flush=True)


# --------------------------------------------------------------------------
# Toolchain
# --------------------------------------------------------------------------

def find_ee_gcc():
    ps2build = shutil.which("ps2build")
    if not ps2build:
        sys.exit("ps2build not found in PATH (install the PS2Build SDK from https://ps2.techwritescode.dev/)")
    root = os.path.dirname(os.path.realpath(ps2build))
    for name in ("mips64r5900el-ps2-elf-gcc.exe", "mips64r5900el-ps2-elf-gcc"):
        p = os.path.join(root, "toolchain", "ee", "bin", name)
        if os.path.exists(p):
            return p
    sys.exit("EE gcc not found under %s/toolchain/ee/bin" % root)


# --------------------------------------------------------------------------
# Minimal ELF32 little-endian object reader
# --------------------------------------------------------------------------

class Elf:
    def __init__(self, path):
        with open(path, "rb") as f:
            self.b = f.read()
        b = self.b
        if b[:4] != b"\x7fELF" or b[4] != 1 or b[5] != 1:
            raise ValueError("%s: not an ELF32 LE object" % path)
        (self.shoff,) = struct.unpack_from("<I", b, 0x20)
        self.shentsize, self.shnum, self.shstrndx = struct.unpack_from("<HHH", b, 0x2E)
        self.sections = []
        for i in range(self.shnum):
            o = self.shoff + i * self.shentsize
            name, typ, flags, addr, off, size, link, info, align, entsize = struct.unpack_from("<10I", b, o)
            self.sections.append(dict(name_off=name, type=typ, off=off, size=size, link=link, info=info,
                                      entsize=entsize, index=i))
        shstr = self.sections[self.shstrndx]
        for s in self.sections:
            s["name"] = self._str(shstr, s["name_off"])
        self.by_name = {s["name"]: s for s in self.sections}
        self.symbols = []
        symtab = next((s for s in self.sections if s["type"] == 2), None)
        if symtab:
            strtab = self.sections[symtab["link"]]
            for i in range(symtab["size"] // 16):
                name, value, size, info, other, shndx = struct.unpack_from("<IIIBBH", b, symtab["off"] + i * 16)
                self.symbols.append(dict(name=self._str(strtab, name), value=value, size=size,
                                         bind=info >> 4, type=info & 15, shndx=shndx))

    def _str(self, sec, off):
        start = sec["off"] + off
        end = self.b.index(b"\0", start)
        return self.b[start:end].decode("ascii", "replace")

    def data(self, name):
        s = self.by_name.get(name)
        if not s or s["type"] == 8:  # SHT_NOBITS
            return b""
        return self.b[s["off"]:s["off"] + s["size"]]

    def size(self, name):
        s = self.by_name.get(name)
        return s["size"] if s else 0

    def rels(self, target_name):
        """R_MIPS_* REL entries applying to section `target_name`."""
        tgt = self.by_name.get(target_name)
        out = []
        if not tgt:
            return out
        for s in self.sections:
            if s["type"] == 9 and s["info"] == tgt["index"]:  # SHT_REL
                for i in range(s["size"] // 8):
                    off, info = struct.unpack_from("<II", self.b, s["off"] + i * 8)
                    out.append((off, info >> 8, info & 0xFF, None))
            elif s["type"] == 4 and s["info"] == tgt["index"]:  # SHT_RELA
                for i in range(s["size"] // 12):
                    off, info, addend = struct.unpack_from("<IIi", self.b, s["off"] + i * 12)
                    out.append((off, info >> 8, info & 0xFF, addend))
        return out


# --------------------------------------------------------------------------
# Compilation
# --------------------------------------------------------------------------

def compile_one(gcc, src, obj, includes, region):
    if os.path.exists(obj) and os.path.getmtime(obj) >= os.path.getmtime(src):
        return None
    os.makedirs(os.path.dirname(obj), exist_ok=True)
    # 32-bit o32 code model: identical struct layout to the EE ABI for the
    # decomp's data types, but char arrays keep 4-byte alignment like IDO
    # (the R5900's 64-bit word would pad u8 arrays to 8 and shift offsets).
    cmd = [gcc, "-c", "-O0", "-G0", "-w", "-std=gnu99", "-fpermissive",
           "-mabi=32", "-march=mips2", "-mno-abicalls",
           # Keep the exact N64 data layout: declaration order, unused statics
           # (PAD arrays), zero-filled arrays in .data, no -O2 array alignment.
           "-fno-common", "-fno-toplevel-reorder", "-fno-zero-initialized-in-bss",
           "-fno-data-sections", "-fno-function-sections", "-fno-builtin", "-DREGION_" + region.upper()] + DEFINES
    for inc in includes:
        cmd += ["-I", inc]
    cmd += ["-o", obj, src]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        return "%s:\n%s" % (src, r.stderr[-4000:])
    return None


def compile_all(gcc, jobs, items, includes, region):
    errors = []
    with ThreadPoolExecutor(max_workers=jobs) as ex:
        for err in ex.map(lambda it: compile_one(gcc, it[0], it[1], includes, region), items):
            if err:
                errors.append(err)
    if errors:
        for e in errors[:10]:
            log(e)
        sys.exit("%d compile errors" % len(errors))


# --------------------------------------------------------------------------
# Source-level type map (for palette normalisation)
# --------------------------------------------------------------------------

DECL_RE = re.compile(r"^\s*(?:static\s+)?(?:const\s+)?([A-Za-z_]\w*)\s+([A-Za-z_]\w*)\s*(?:\[[^\]]*\])*\s*=",
                     re.M)


def source_types(path):
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            text = f.read()
    except OSError:
        return {}
    return {m.group(2): m.group(1) for m in DECL_RE.finditer(text)}


# --------------------------------------------------------------------------
# relocData
# --------------------------------------------------------------------------

class RelocFile:
    pass


def read_rom_reloc_table(rom, base, count):
    entries = []
    for i in range(count + 1):
        w0, ri, cs, re_, ds = struct.unpack_from(">IHHHH", rom, base + i * 12)
        entries.append(dict(compressed=w0 >> 31, data_offset=w0 & 0x7FFFFFFF, words=ds))
    return entries


INC_BLOCK_RE = re.compile(r"^\s*(u16|s16)\s+(\w+)\s*\[[^\]]*\]\s*=\s*\{\s*#include\s*<[^>]*\.palette\.inc\.c>", re.M)
SCRIPT_CAST_RE = re.compile(r"\(\s*AObjEvent32\s*\*\s*\)\s*&?\s*(\w+)")


def fix_mistyped_scripts(files, stats):
    """u16 blocks included as ".palette" data but used as AObjEvent32 scripts.

    On the N64 a u16 array and a u32 script are the same bytes; compiled
    natively the two halfwords of every script word end up swapped. Only
    blocks whose *include* says palette while the source uses them as an
    AObjEvent32 pointer are touched (genuine u16 figatree data, which is
    also cast to AObjEvent32*, comes from inline macros, not palette
    includes)."""
    palettes = {}
    texts = {}
    for rf in files:
        try:
            with open(rf.master, encoding="utf-8", errors="replace") as f:
                texts[rf.fid] = f.read()
        except OSError:
            continue
        for m in INC_BLOCK_RE.finditer(texts[rf.fid]):
            palettes[m.group(2)] = rf
    used = set()
    for text in texts.values():
        for m in SCRIPT_CAST_RE.finditer(text):
            if m.group(1) in palettes:
                used.add(m.group(1))
    for name in sorted(used):
        rf = palettes[name]
        for start, size, sym in rf.sym_list:
            if sym == name:
                for o in range(start, start + size - 3, 4):
                    rf.blob[o:o + 4] = rf.blob[o + 2:o + 4] + rf.blob[o:o + 2]
                stats["mistyped_scripts"] += 1
                break


def fix_animjoint_files(root, files, stats):
    """Fighter animations the motion tables flag FTANIM_FLAG_ANIMJOINT.

    Those files are AObjEvent32 scripts (parsed by gcParseDObjAnimJoint),
    but the decomp types them like figatree data, as u16 arrays. On the N64
    that is the same bytes; compiled natively, every 32-bit event would have
    its halfwords swapped, so swap them back in the script arrays."""
    ids = set()
    rx = re.compile(r"&(ll\w+FileID)\s*,[^,]*,\s*([^}]*)\}")
    for dirpath, _, names in os.walk(os.path.join(root, "src")):
        if "relocData" in dirpath:
            continue
        for n in names:
            if not n.endswith(".c"):
                continue
            with open(os.path.join(dirpath, n), encoding="utf-8", errors="replace") as f:
                for m in rx.finditer(f.read()):
                    if "FTANIM_FLAG_ANIMJOINT" in m.group(2):
                        ids.add(m.group(1))
    fids = set()
    with open(os.path.join(root, "symbols", "reloc_data_symbols.us.txt"), encoding="utf-8") as f:
        for line in f:
            m = re.match(r"\s*(ll\w+FileID)\s*=\s*(0x[0-9A-Fa-f]+|\d+)\s*;", line)
            if m and m.group(1) in ids:
                fids.add(int(m.group(2), 0))
    for rf in files:
        if rf.fid not in fids:
            continue
        for start, size, sym in rf.sym_list:
            if rf.types.get(sym) not in ("u16", "s16"):
                continue
            for o in range(start, start + size - 3, 4):
                rf.blob[o:o + 4] = rf.blob[o + 2:o + 4] + rf.blob[o:o + 2]
        stats["animjoint_files"] += 1


def load_reloc(fid, master, obj):
    e = Elf(obj)
    rf = RelocFile()
    rf.fid = fid
    rf.master = master
    text = e.data(".text")
    data = e.data(".data")
    rf.text_size = len(text)
    rf.blob = bytearray(text + data)
    rf.problems = []
    for sec in (".rodata", ".bss", ".sdata", ".sbss"):
        if e.size(sec):
            rf.problems.append("%s has %d bytes of %s (not part of the N64 layout)" % (master, e.size(sec), sec))
    base_of = {}
    for s in e.sections:
        if s["name"] == ".text":
            base_of[s["index"]] = 0
        elif s["name"] == ".data":
            base_of[s["index"]] = rf.text_size
    rf.base_of = base_of
    rf.elf = e
    # Defined symbols -> offset in blob
    rf.syms = {}
    rf.sym_list = []
    for s in e.symbols:
        if s["shndx"] in base_of and s["type"] in (0, 1, 2) and s["name"]:
            off = base_of[s["shndx"]] + s["value"]
            if s["bind"] == 1:  # global
                rf.syms[s["name"]] = off
            rf.sym_list.append((off, s["size"], s["name"]))
    rf.sym_list.sort()
    rf.types = source_types(master)
    return rf


def symbol_at(rf, off):
    """(name, start) of the defined symbol containing blob offset `off`."""
    best = None
    for start, size, name in rf.sym_list:
        if start <= off < start + max(size, 1):
            best = (name, start)
        elif start > off:
            break
    return best


def resolve_relocs(rf, index):
    """Returns list of (slot_off, kind, target_fid, target_off)."""
    out = []
    e = rf.elf
    for secname, secbase in ((".data", rf.text_size), (".text", 0)):
        for off, symi, typ, rela_addend in e.rels(secname):
            if secname == ".text":
                rf.problems.append("%s: .text relocation type %d ignored" % (rf.master, typ))
                continue
            if typ != 2:  # R_MIPS_32
                raise ValueError("%s: unsupported relocation type %d" % (rf.master, typ))
            slot = secbase + off
            if rela_addend is None:
                (addend,) = struct.unpack_from("<I", rf.blob, slot)
            else:
                addend = rela_addend & 0xFFFFFFFF
            sym = e.symbols[symi]
            if sym["type"] == 3:  # STT_SECTION
                if sym["shndx"] not in rf.base_of:
                    raise ValueError("%s: reloc against unexpected section" % rf.master)
                out.append((slot, "intern", rf.fid, rf.base_of[sym["shndx"]] + addend))
            elif sym["shndx"] in rf.base_of:
                out.append((slot, "intern", rf.fid, rf.base_of[sym["shndx"]] + sym["value"] + addend))
            elif sym["shndx"] == 0:
                tgt = index.get(sym["name"])
                if tgt is None:
                    raise ValueError("%s: unresolved extern symbol %s" % (rf.master, sym["name"]))
                tfid, toff = tgt
                if tfid == rf.fid:
                    out.append((slot, "intern", rf.fid, toff + addend))
                else:
                    out.append((slot, "extern", tfid, toff + addend))
            else:
                raise ValueError("%s: reloc against symbol %s in section %d" % (rf.master, sym["name"], sym["shndx"]))
    out.sort()
    return out


# --------------------------------------------------------------------------
# Palette normalisation: every palette ends up as native (LE) u16.
# --------------------------------------------------------------------------

def swap16_range(blob, start, count, done, key):
    for i in range(count):
        o = start + i * 2
        if o + 2 > len(blob) or (key, o) in done:
            continue
        done.add((key, o))
        blob[o], blob[o + 1] = blob[o + 1], blob[o]


def normalise_palettes(files, relocs_by_fid, stats):
    done = set()
    by_fid = {rf.fid: rf for rf in files}

    def target_type(tfid, toff):
        rf = by_fid.get(tfid)
        if not rf:
            return None
        hit = symbol_at(rf, toff)
        if not hit:
            return None
        return rf.types.get(hit[0])

    for rf in files:
        slots = {r[0]: r for r in relocs_by_fid[rf.fid]}
        # (a) display lists: SETTIMG ... LOADTLUT
        for start, size, name in rf.sym_list:
            if rf.types.get(name) != "Gfx":
                continue
            timg = None
            timg_maybe_replaced = False
            for o in range(start, start + size - 7, 8):
                w0, w1 = struct.unpack_from("<II", rf.blob, o)
                op = w0 >> 24
                if op == 0xFD:
                    timg = slots.get(o + 4)
                    timg_maybe_replaced = False
                elif op == 0xDE and (o + 4) not in slots:
                    # A DL in a segment is built at runtime (MObj lists) and
                    # usually sets the texel image itself: a following texel
                    # load is not known to read the image set above.
                    timg_maybe_replaced = True
                elif op in (0xF3, 0xF4) and timg is not None and not timg_maybe_replaced:
                    # Texel load: texture data must stay in the original N64
                    # byte order. Blocks the decomp typed as u16/u32 were
                    # compiled natively and need their elements swapped back.
                    _, _, tfid, toff = timg
                    ttype = target_type(tfid, toff)
                    if ttype in ("u16", "s16", "u32", "s32"):
                        trf = by_fid[tfid]
                        hit = symbol_at(trf, toff)
                        key = ("tex", tfid, hit[1] if hit else toff)
                        if hit is not None and key not in done:
                            done.add(key)
                            sz = next(z for st, z, nm in trf.sym_list if st == hit[1] and nm == hit[0])
                            w = 2 if ttype in ("u16", "s16") else 4
                            for q in range(hit[1], hit[1] + sz - (w - 1), w):
                                trf.blob[q:q + w] = trf.blob[q:q + w][::-1]
                            stats["texels_restored"] += 1
                elif op == 0xF0 and timg is not None and not timg_maybe_replaced:
                    # Like texel loads above, a TLUT load after an unrelocated
                    # segment DL may consume a texture image selected by that
                    # runtime MObj list.  Do not reinterpret the last static
                    # SETTIMG target as a palette: doing so byte-swaps CI texels
                    # in the generated DAT and produces localized fighter
                    # texture corruption.
                    count = ((w1 >> 14) & 0x3FF) + 1
                    _, _, tfid, toff = timg
                    ttype = target_type(tfid, toff)
                    if ttype not in ("u16", "s16"):
                        hit = symbol_at(by_fid[tfid], toff)
                        if hit is not None:
                            sz = next(z for st, z, nm in by_fid[tfid].sym_list if st == hit[1] and nm == hit[0])
                            count = min(count, (hit[1] + sz - toff) // 2)
                        swap16_range(by_fid[tfid].blob, toff, count, done, tfid)
                        stats["tlut_swapped"] += 1
        # (b) libultra Sprite: LUT at +32, nTLUT at +30 (native layout)
        for start, size, name in rf.sym_list:
            if rf.types.get(name) != "Sprite":
                continue
            lut = slots.get(start + 32)
            if lut is None:
                continue
            (ntlut,) = struct.unpack_from("<h", rf.blob, start + 30)
            bmsiz = rf.blob[start + 49]
            _, _, tfid, toff = lut
            if ntlut <= 0 or target_type(tfid, toff) in ("u16", "s16"):
                continue
            # nTLUT is often 256 even for CI4 sprites; only 16 entries exist.
            entries = min(ntlut, 16 if bmsiz == 0 else 256)
            hit = symbol_at(by_fid[tfid], toff)
            if hit is not None:
                sym_start = hit[1]
                sym_size = next(sz for st, sz, nm in by_fid[tfid].sym_list if st == sym_start and nm == hit[0])
                entries = min(entries, (sym_start + sym_size - toff) // 2)
            swap16_range(by_fid[tfid].blob, toff, entries, done, tfid)
            stats["sprite_luts_swapped"] += 1


def normalise_particle_txb(blob, name):
    """Byte-swap CI palettes in a particle texture bank (see lbParticleSetupBankID)."""
    swapped = 0
    (ntex,) = struct.unpack_from("<i", blob, 0)
    for t in range(ntex):
        (toff,) = struct.unpack_from("<I", blob, 4 + t * 4)
        count, fmt, siz, width, height, flags = struct.unpack_from("<6i", blob, toff)
        data_off = toff + 24
        if fmt != 2:  # G_IM_FMT_CI
            continue
        entries = 16 if siz == 0 else 256
        npal = 1 if (flags & 1) else count
        for p in range(npal):
            (poff,) = struct.unpack_from("<I", blob, data_off + (count + p) * 4)
            for i in range(entries):
                o = poff + i * 2
                if o + 2 <= len(blob):
                    blob[o], blob[o + 1] = blob[o + 1], blob[o]
            swapped += 1
    return swapped


def swap_words(blob, offsets):
    for o in offsets:
        if o + 4 <= len(blob):
            blob[o:o + 4] = blob[o:o + 4][::-1]


def swap_seq_headers(blob):
    """S1_music_sbk: the ALSeqFile table is typed C (native), but every
    compressed-MIDI sequence starts with an ALCMidiHdr (16 s32 track offsets
    + s32 division) stored as raw bytes; n_alCSeqNew reads it as words."""
    (revision, count) = struct.unpack_from("<hh", blob, 0)
    for i in range(count):
        off, length = struct.unpack_from("<Ii", blob, 4 + 8 * i)
        if length >= 68:
            swap_words(blob, range(off, off + 68, 4))


def swap_fgm_pkg(blob):
    """fgm.tbl / fgm.ucd: {s32 count; u32 offsets[count]; u8 scripts...}"""
    (count,) = struct.unpack_from(">i", blob, 0)
    swap_words(blob, range(0, 4 + 4 * count, 4))


def swap_fgm_unk(blob):
    """fgm.unk: {s32 count; {u8 x4; f32 x3}[count]} (ALWhatever8009EE0C_3)"""
    (count,) = struct.unpack_from(">i", blob, 0)
    offs = [0]
    for i in range(count):
        base = 4 + 16 * i
        offs += [base + 4, base + 8, base + 12]
    swap_words(blob, offs)


# --------------------------------------------------------------------------
# Pack writer
# --------------------------------------------------------------------------

def write_pack(path, regions, reloc_count):
    """regions: list of (vrom, bytes, resident). Resident blobs go first."""
    regions = sorted(regions, key=lambda r: r[0])
    for a, b in zip(regions, regions[1:]):
        if a[0] + len(a[1]) > b[0]:
            sys.exit("overlapping regions at 0x%08X / 0x%08X" % (a[0], b[0]))
    header_size = 64
    table_off = header_size
    table_size = 16 * len(regions)
    pos = (table_off + table_size + 63) & ~63
    resident_start = pos
    layout = []
    for vrom, blob, res in regions:
        if res:
            layout.append((vrom, len(blob), pos, REGION_RESIDENT))
            pos += len(blob)
    resident_bytes = pos - resident_start
    pos = (pos + 63) & ~63
    for vrom, blob, res in regions:
        if not res:
            layout.append((vrom, len(blob), pos, 0))
            pos = (pos + len(blob) + 15) & ~15
    total = pos
    layout.sort(key=lambda r: r[0])

    out = bytearray(total)
    struct.pack_into("<8sIIIIIII3I16s", out, 0, b"SSB64PS2", PACK_VERSION, len(layout), table_off,
                     resident_bytes, total, RELOC_VROM, reloc_count, resident_start, 0, 0,
                     b"ssb64-ps2-pack")
    blobs = {r[0]: r[1] for r in regions}
    for i, (vrom, size, off, flags) in enumerate(layout):
        struct.pack_into("<IIII", out, table_off + i * 16, vrom, size, off, flags)
        out[off:off + size] = blobs[vrom]
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as f:
        f.write(out)
    return total, resident_bytes, len(layout)


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--version", default="us")
    ap.add_argument("-j", "--jobs", type=int, default=os.cpu_count() or 4)
    ap.add_argument("--rom", default=None)
    ap.add_argument("-o", "--output", default=os.path.join(ROOT, "ps2", "build", "bin", "SSB64.DAT"))
    args = ap.parse_args()
    v = args.version
    os.chdir(ROOT)

    rom_path = args.rom or "baserom.%s.z64" % v
    with open(rom_path, "rb") as f:
        rom = f.read()
    masters_txt = os.path.join("build", v, "ps2_reloc_masters.txt")
    if not os.path.exists(masters_txt):
        sys.exit("%s missing: run stage 1 first (make -f ps2/tools/n64prep.mk ps2-prep)" % masters_txt)

    gcc = find_ee_gcc()
    log("EE compiler: %s" % gcc)
    includes = ["include", "src", "src/relocData", "build/%s/src/relocData" % v, "build/%s/src" % v]
    objdir = os.path.join("build", "ps2assets", v)

    # ---- relocData ----
    masters = []
    with open(masters_txt) as f:
        for tok_line in f:
            parts = tok_line.split()
            if len(parts) == 2:
                masters.append((int(parts[0]), parts[1]))
    masters.sort()
    count = len(masters)
    log("relocData: compiling %d files (-j%d)..." % (count, args.jobs))
    compile_all(gcc, args.jobs, [(m, os.path.join(objdir, "reloc", "%d.o" % fid)) for fid, m in masters], includes, v)

    files = [load_reloc(fid, m, os.path.join(objdir, "reloc", "%d.o" % fid)) for fid, m in masters]
    index = {}
    for rf in files:
        for name, off in rf.syms.items():
            if name not in index:
                index[name] = (rf.fid, off)

    rom_table = read_rom_reloc_table(rom, ROM_RELOC_TABLE[v], count)
    size_mismatch = 0
    for rf in files:
        while len(rf.blob) % 4:
            rf.blob.append(0)
        want = rom_table[rf.fid]["words"] * 4
        if 0 < want - len(rf.blob) < 16:
            rf.blob += bytes(want - len(rf.blob))  # IDO rounds sections up to 16 bytes
        if len(rf.blob) != want:
            size_mismatch += 1
            if size_mismatch <= 20:
                log("  size mismatch fid %d (%s): native %d, ROM %d" % (rf.fid, rf.master, len(rf.blob), want))
    if size_mismatch:
        log("WARNING: %d relocData files differ in size from the ROM" % size_mismatch)

    relocs_by_fid = {rf.fid: resolve_relocs(rf, index) for rf in files}
    stats = {"tlut_swapped": 0, "sprite_luts_swapped": 0, "mistyped_scripts": 0, "texels_restored": 0, "animjoint_files": 0}
    fix_mistyped_scripts(files, stats)
    fix_animjoint_files(ROOT, files, stats)
    normalise_palettes(files, relocs_by_fid, stats)

    # Encode chains + table
    table = bytearray(12 * (count + 1))
    body = bytearray()
    ext_regions = []
    data_regions = []
    problems = 0
    for rf in files:
        relocs = relocs_by_fid[rf.fid]
        blob = rf.blob
        intern = [r for r in relocs if r[1] == "intern"]
        extern = [r for r in relocs if r[1] == "extern"]
        for kind_list in (intern, extern):
            for slot, kind, tfid, toff in kind_list:
                if slot % 4 or toff % 4 or slot // 4 >= 0xFFFF or toff // 4 >= 0xFFFF:
                    raise ValueError("fid %d: pointer slot 0x%X -> 0x%X not word addressable" % (rf.fid, slot, toff))

        def chain(lst):
            first = 0xFFFF
            for i, (slot, kind, tfid, toff) in enumerate(lst):
                nxt = lst[i + 1][0] // 4 if i + 1 < len(lst) else 0xFFFF
                struct.pack_into("<HH", blob, slot, nxt, toff // 4)
                if i == 0:
                    first = slot // 4
            return first

        intern_first = chain(intern)
        extern_first = chain(extern)
        ext_ids = b"".join(struct.pack("<H", r[2]) for r in extern)
        data_off = len(body)
        words = len(blob) // 4
        struct.pack_into("<IHHHH", table, rf.fid * 12, (data_off << 1) | 0, intern_first, words, extern_first, words)
        data_regions.append((data_off, bytes(blob)))
        body += blob
        if ext_ids:
            ext_regions.append((len(body), ext_ids))
            body += ext_ids
        for p in rf.problems:
            problems += 1
            if problems <= 10:
                log("  note: " + p)
    struct.pack_into("<IHHHH", table, count * 12, len(body) << 1, 0xFFFF, 0, 0xFFFF, 0)

    hi = RELOC_VROM + len(table)
    regions = [(RELOC_VROM, bytes(table), True)]
    for off, blob in data_regions:
        regions.append((hi + off, blob, False))
    for off, blob in ext_regions:
        regions.append((hi + off, blob, True))

    # ---- particle banks ----
    log("particles: compiling %d banks..." % len(PARTICLE_BANKS))
    items = [(os.path.join("src", "particles", n + ".c"), os.path.join(objdir, "particles", n + ".o"))
             for n, _, _ in PARTICLE_BANKS]
    compile_all(gcc, args.jobs, items, includes, v)
    for (name, lo, hi_rom), (_, obj) in zip(PARTICLE_BANKS, items):
        e = Elf(obj)
        blob = bytearray(e.data(".rodata") + e.data(".data"))
        if e.rels(".rodata") or e.rels(".data"):
            sys.exit("particle bank %s unexpectedly has relocations" % name)
        if 0 < (hi_rom - lo) - len(blob) < 16:
            blob += bytes((hi_rom - lo) - len(blob))  # trailing section alignment
        if len(blob) != hi_rom - lo:
            sys.exit("particle bank %s: native size %d != ROM size %d" % (name, len(blob), hi_rom - lo))
        if name.endswith("_txb"):
            stats.setdefault("particle_palettes_swapped", 0)
            stats["particle_palettes_swapped"] += normalise_particle_txb(blob, name)
        regions.append((lo, bytes(blob), False))

    # ---- audio ----
    log("audio: %d regions..." % len(AUDIO_REGIONS))
    csrc = [(src.format(v=v), os.path.join(objdir, "audio", name + ".o"))
            for name, lo, hi_rom, kind, src in AUDIO_REGIONS if kind == "c"]
    compile_all(gcc, args.jobs, csrc, includes, v)
    for name, lo, hi_rom, kind, src in AUDIO_REGIONS:
        src = src.format(v=v)
        if kind == "c":
            e = Elf(os.path.join(objdir, "audio", name + ".o"))
            if e.rels(".rodata") or e.rels(".data"):
                sys.exit("audio %s unexpectedly has relocations" % name)
            blob = bytearray(e.data(".rodata") + e.data(".data"))
            if name.endswith("_sbk"):
                swap_seq_headers(blob)
        else:
            with open(src, "rb") as f:
                blob = bytearray(f.read())
            if kind == "fgm_unk":
                swap_fgm_unk(blob)
            elif kind == "fgm_pkg":
                swap_fgm_pkg(blob)
        rom_size = hi_rom - lo
        if 0 < rom_size - len(blob) < 16:
            blob += bytes(rom_size - len(blob))
        if len(blob) > rom_size:
            sys.exit("audio %s: native size %d exceeds ROM segment %d" % (name, len(blob), rom_size))
        if len(blob) != rom_size:
            log("  note: audio %s is %d bytes (ROM segment %d)" % (name, len(blob), rom_size))
        regions.append((lo, bytes(blob), False))

    # ---- SPU2 samples (PS-ADPCM, see spu_samples.py) ----
    import spu_samples
    regions.append((SPU_SAMPLES_VROM, spu_samples.build(rom, args.jobs, log), False))

    # ---- raw ROM header / boot area (read by harmless boot-time code) ----
    regions.append((0, rom[:0x1000], True))

    total, resident, nregions = write_pack(args.output, regions, count)
    log("palettes normalised: %s" % stats)
    log("wrote %s: %d regions, %.1f MiB total, %d KiB resident" % (
        args.output, nregions, total / 1048576.0, resident // 1024))
    return 0


if __name__ == "__main__":
    sys.exit(main())
