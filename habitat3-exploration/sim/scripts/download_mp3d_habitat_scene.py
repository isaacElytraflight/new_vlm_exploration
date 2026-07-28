#!/usr/bin/env python3
"""Extract one MP3D habitat scene from the remote mp3d_habitat.zip via HTTP Range.

Confirms you already have Matterport access (download_mp.py). Downloads only the
byte ranges for the requested house instead of the full ~16GB archive.

Usage:
  python download_mp3d_habitat_scene.py [--scene JmbYfDe2QKZ] [--out DIR]
"""

from __future__ import annotations

import argparse
import io
import struct
import sys
import urllib.request
import zipfile
from pathlib import Path

HABITAT_ZIP_URL = "https://kaldir.vc.cit.tum.de/matterport/v1/tasks/mp3d_habitat.zip"
TOS_URL = "http://kaldir.vc.cit.tum.de/matterport/MP_TOS.pdf"
CONFIG_URL = "http://dl.fbaipublicfiles.com/habitat/mp3d/config_v1/mp3d.scene_dataset_config.json"

EOCD_SIG = b"PK\x05\x06"
ZIP64_LOCATOR_SIG = b"PK\x06\x07"
ZIP64_EOCD_SIG = b"PK\x06\x06"


def http_range(url: str, start: int, end: int | None = None) -> bytes:
    """Inclusive byte range [start, end]. end=None → to EOF."""
    if end is None:
        spec = f"bytes={start}-"
    else:
        spec = f"bytes={start}-{end}"
    req = urllib.request.Request(url, headers={"Range": spec})
    with urllib.request.urlopen(req, timeout=120) as resp:
        return resp.read()


def http_get(url: str) -> bytes:
    with urllib.request.urlopen(url, timeout=120) as resp:
        return resp.read()


def content_length(url: str) -> int:
    req = urllib.request.Request(url, method="HEAD")
    with urllib.request.urlopen(req, timeout=60) as resp:
        return int(resp.headers["Content-Length"])


def find_eocd(tail: bytes) -> int:
    """Return offset of EOCD signature within tail bytes."""
    idx = tail.rfind(EOCD_SIG)
    if idx < 0:
        raise RuntimeError("EOCD not found in zip tail")
    return idx


def parse_central_directory(url: str, size: int) -> zipfile.ZipFile:
    """Fetch zip central directory via Range and open as ZipFile."""
    # Last 64KiB is enough for EOCD + typical CD; MP3D habitat may need more.
    # Fetch last 8 MiB to be safe for a large central directory.
    tail_len = min(size, 8 * 1024 * 1024)
    tail = http_range(url, size - tail_len, size - 1)
    eocd_rel = find_eocd(tail)
    eocd = tail[eocd_rel : eocd_rel + 22]
    (
        _sig,
        _disk,
        _cd_disk,
        _disk_entries,
        _total_entries,
        cd_size,
        cd_offset,
        _comment_len,
    ) = struct.unpack("<IHHHHIIH", eocd)

    # ZIP64 if offsets are maxed out.
    if cd_size == 0xFFFFFFFF or cd_offset == 0xFFFFFFFF:
        # ZIP64 end-of-central-directory locator sits immediately before EOCD.
        loc_rel = eocd_rel - 20
        if loc_rel < 0 or tail[loc_rel : loc_rel + 4] != ZIP64_LOCATOR_SIG:
            raise RuntimeError("ZIP64 locator missing")
        _lsig, _disk, zip64_eocd_offset, _disks = struct.unpack(
            "<IIQI", tail[loc_rel : loc_rel + 20]
        )
        # Fetch ZIP64 EOCD record (variable size; first 56 bytes has what we need).
        z64 = http_range(url, zip64_eocd_offset, zip64_eocd_offset + 55)
        if z64[:4] != ZIP64_EOCD_SIG:
            raise RuntimeError("ZIP64 EOCD signature mismatch")
        (
            _zsig,
            _zsize,
            _ver_made,
            _ver_need,
            _disk,
            _cd_disk,
            _disk_entries,
            _total_entries,
            cd_size,
            cd_offset,
        ) = struct.unpack("<IQHHIIQQQQ", z64[:56])

    cd_bytes = http_range(url, cd_offset, cd_offset + cd_size - 1)
    # Build a minimal zip in memory: empty locals + CD + EOCD pointing at CD.
    buf = io.BytesIO()
    buf.write(cd_bytes)
    # Standard EOCD (works when cd fits in 32-bit; for ZIP64 ZipFile still
    # parses CD records which carry 64-bit sizes in extra fields).
    if cd_size >= 0xFFFFFFFF or cd_offset >= 0xFFFFFFFF or len(cd_bytes) >= 0xFFFFFFFF:
        # Append ZIP64 EOCD + locator + EOCD with 0xFFFFFFFF markers.
        zip64_eocd = struct.pack(
            "<IQHHIIQQQQ",
            0x06064B50,
            44,  # size of rest of record
            45,
            45,
            0,
            0,
            0,  # disk entries unknown — ZipFile recounts from CD
            0,
            len(cd_bytes),
            0,  # CD at start of our buffer
        )
        buf.write(zip64_eocd)
        locator = struct.pack(
            "<IIQI",
            0x07064B50,
            0,
            len(cd_bytes),  # offset of ZIP64 EOCD
            1,
        )
        buf.write(locator)
        eocd_out = struct.pack(
            "<IHHHHIIH",
            0x06054B50,
            0,
            0,
            0xFFFF,
            0xFFFF,
            0xFFFFFFFF,
            0xFFFFFFFF,
            0,
        )
        buf.write(eocd_out)
    else:
        eocd_out = struct.pack(
            "<IHHHHIIH",
            0x06054B50,
            0,
            0,
            0,
            0,
            len(cd_bytes),
            0,
            0,
        )
        buf.write(eocd_out)

    buf.seek(0)
    # ZipFile needs file headers for extraction — we only use namelist + getinfo
    # for offsets, then fetch local headers ourselves.
    return zipfile.ZipFile(buf), cd_offset


def local_header_data_offset(url: str, local_header_offset: int) -> tuple[int, int]:
    """Return (data_start, compressed_size) from a local file header."""
    header = http_range(url, local_header_offset, local_header_offset + 29)
    if header[:4] != b"PK\x03\x04":
        raise RuntimeError("Bad local file header")
    (
        _sig,
        _ver,
        _flags,
        method,
        _time,
        _date,
        _crc,
        comp_size,
        _uncomp,
        name_len,
        extra_len,
    ) = struct.unpack("<IHHHHHIIIHH", header)
    data_start = local_header_offset + 30 + name_len + extra_len
    # ZIP64 extra may override comp_size == 0xFFFFFFFF — read extra if needed.
    if comp_size == 0xFFFFFFFF:
        extra = http_range(
            url,
            local_header_offset + 30 + name_len,
            local_header_offset + 30 + name_len + extra_len - 1,
        )
        # Parse ZIP64 extra field (id 1)
        pos = 0
        while pos + 4 <= len(extra):
            eid, esize = struct.unpack("<HH", extra[pos : pos + 4])
            payload = extra[pos + 4 : pos + 4 + esize]
            if eid == 1:
                # order: uncomp, comp, offset, disk — present fields only
                vals = []
                off = 0
                while off + 8 <= len(payload):
                    vals.append(struct.unpack("<Q", payload[off : off + 8])[0])
                    off += 8
                if len(vals) >= 2:
                    comp_size = vals[1]
                elif len(vals) == 1:
                    # only uncomp size present
                    pass
                break
            pos += 4 + esize
    return data_start, comp_size, method


def extract_scene(url: str, scene_id: str, out_dir: Path) -> list[Path]:
    size = content_length(url)
    print(f"Remote zip size: {size / 1e9:.2f} GB", flush=True)
    zf, _cd_abs = parse_central_directory(url, size)

    # Match members whose path contains the scene id.
    members = [zi for zi in zf.infolist() if scene_id in zi.filename.replace("\\", "/")]
    if not members:
        # Show a few names for debugging.
        sample = [zi.filename for zi in zf.infolist()[:20]]
        raise RuntimeError(
            f"No zip members matched {scene_id!r}. Sample entries: {sample}"
        )

    print(f"Found {len(members)} members for {scene_id}:", flush=True)
    for zi in members:
        print(f"  {zi.filename} ({zi.file_size} bytes)", flush=True)

    out_dir.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []

    for zi in members:
        if zi.is_dir() or zi.filename.endswith("/"):
            rel = zi.filename.replace("\\", "/")
            if f"/{scene_id}/" in f"/{rel}":
                sub = rel[rel.index(scene_id) :]
            else:
                sub = f"{scene_id}/"
            dest = out_dir / sub
            dest.mkdir(parents=True, exist_ok=True)
            print(f"Skipping directory entry {rel}", flush=True)
            continue

        # Destination: flatten to out_dir / basename (keep folder if nested).
        rel = zi.filename.replace("\\", "/")
        # Prefer …/SCENE_ID/file under out_dir
        if f"/{scene_id}/" in f"/{rel}":
            sub = rel[rel.index(scene_id) :]
        else:
            sub = f"{scene_id}/{Path(rel).name}"
        dest = out_dir / sub
        dest.parent.mkdir(parents=True, exist_ok=True)

        if dest.is_file() and dest.stat().st_size == zi.file_size and zi.file_size > 0:
            print(f"Already present {dest} ({zi.file_size} bytes) — skip", flush=True)
            written.append(dest)
            continue

        data_start, comp_size, method = local_header_data_offset(url, zi.header_offset)
        if comp_size <= 0 and zi.file_size == 0:
            dest.write_bytes(b"")
            written.append(dest)
            continue
        print(f"Fetching {rel} ({comp_size} compressed bytes)…", flush=True)
        payload = http_range(url, data_start, data_start + comp_size - 1)

        if method == zipfile.ZIP_STORED:
            data = payload
        elif method == zipfile.ZIP_DEFLATED:
            import zlib

            data = zlib.decompress(payload, -15)
        else:
            raise RuntimeError(f"Unsupported compression method {method} for {rel}")

        dest.write_bytes(data)
        written.append(dest)
        print(f"  -> {dest} ({len(data)} bytes)", flush=True)

    return written


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", default="JmbYfDe2QKZ")
    parser.add_argument(
        "--out",
        default="",
        help="Output root (default: habitat3-exploration/sim/data/scene_datasets/mp3d)",
    )
    parser.add_argument(
        "--i-agree-to-mp-tos",
        action="store_true",
        help=f"Confirm agreement to Matterport ToS ({TOS_URL})",
    )
    args = parser.parse_args()
    if not args.i_agree_to_mp_tos:
        print(
            "Refusing to download without --i-agree-to-mp-tos "
            f"(see {TOS_URL}). You already have download_mp.py from Matterport.",
            file=sys.stderr,
        )
        return 2

    repo_default = (
        Path(__file__).resolve().parents[1]
        / "data"
        / "scene_datasets"
        / "mp3d"
    )
    out_root = Path(args.out) if args.out else repo_default
    out_root.mkdir(parents=True, exist_ok=True)

    config_path = out_root / "mp3d.scene_dataset_config.json"
    if not config_path.is_file():
        print(f"Fetching {CONFIG_URL}", flush=True)
        config_path.write_bytes(http_get(CONFIG_URL))

    written = extract_scene(HABITAT_ZIP_URL, args.scene, out_root)
    glb = out_root / args.scene / f"{args.scene}.glb"
    if not glb.is_file():
        # Sometimes nested differently
        matches = list(out_root.rglob(f"{args.scene}.glb"))
        if matches:
            glb = matches[0]
    if not glb.is_file():
        print("ERROR: .glb not found after extract", file=sys.stderr)
        for p in written:
            print(f"  wrote {p}", file=sys.stderr)
        return 1

    print(f"OK: {glb}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
