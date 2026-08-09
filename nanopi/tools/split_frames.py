#!/usr/bin/env python3
"""
Mode2 数据拆分工具 —— 把 MJPEG/Z16 大包拆成单帧，提取时间戳。

    输入:  episode_dir/
              head_cam/
                  index_000000.bin  ...  index_0000NN.bin
                  rgb_000000.mjpg   ...  rgb_0000NN.mjpg
                  depth_000000.z16  ...  depth_0000NN.z16
              left_cam/   ...
              right_cam/  ...

    输出 (每个 camera 目录下):
        rgb/              - 000000.jpg, 000001.jpg, ...
        depth/            - 000000.z16, 000001.z16, ...
        timestamps.txt    - frame_index  rgb_ts_ms  depth_ts_ms  host_ns  ...

用法:
    python split_frames.py <episode_dir>              # 拆分单个 episode
    python split_frames.py <episode_dir> --cleanup     # 拆分后删除原始大包
    python split_frames.py <episode_dir> --verify      # 拆分 + 校验
    python split_frames.py <dir_with_multiple_episodes>  # 批量处理
"""

import argparse
import shutil
import struct
import sys
import time
from pathlib import Path

# ── Struct layouts (mode2_capture.cpp, #pragma pack(1)) ──

INDEX_HEADER_FMT = "<8sIIIIIIIIIIfqQ32s"       # 100 bytes
INDEX_RECORD_FMT = "<QQQqddqqIIQQIIII"          # 104 bytes

INDEX_HEADER_SIZE = struct.calcsize(INDEX_HEADER_FMT)
INDEX_RECORD_SIZE = struct.calcsize(INDEX_RECORD_FMT)


def parse_index_header(data: bytes) -> dict:
    f = struct.unpack_from(INDEX_HEADER_FMT, data, 0)
    return {
        "magic": f[0], "version": f[1], "header_bytes": f[2],
        "record_bytes": f[3], "width": f[4], "height": f[5],
        "fps": f[6], "chunk_id": f[7], "session_id": f[8],
        "rgb_fourcc": f[9], "depth_fourcc": f[10], "depth_scale": f[11],
        "created_unix_ns": f[12], "record_count": f[13],
    }


def parse_index_record(data: bytes) -> dict:
    f = struct.unpack_from(INDEX_RECORD_FMT, data, 0)
    return {
        "sequence": f[0], "rgb_frame_number": f[1],
        "depth_frame_number": f[2], "host_receive_unix_ns": f[3],
        "rgb_rs_timestamp_ms": f[4], "depth_rs_timestamp_ms": f[5],
        "rgb_sensor_timestamp": f[6], "depth_sensor_timestamp": f[7],
        "rgb_timestamp_domain": f[8], "depth_timestamp_domain": f[9],
        "rgb_offset": f[10], "depth_offset": f[11],
        "rgb_bytes": f[12], "depth_bytes": f[13], "flags": f[14],
    }


TS_HEADER = (
    "# frame_index  rgb_rs_timestamp_ms  depth_rs_timestamp_ms  "
    "host_receive_unix_ns  rgb_frame_number  depth_frame_number  "
    "rgb_bytes  depth_bytes\n"
)


def fmt_ns(ns: int) -> str:
    """Format nanosecond timestamp as human-readable for logging."""
    sec = ns / 1_000_000_000.0
    return f"{sec:.3f}s"


# ── Per-camera processing ──

def process_camera(cam_dir: Path, *, cleanup: bool = False,
                   progress: bool = True) -> dict:
    """
    Split a single camera directory. Returns stats dict.
    """
    cam_name = cam_dir.name
    index_files = sorted(cam_dir.glob("index_*.bin"))
    if not index_files:
        return {"camera": cam_name, "error": "no index files"}

    rgb_dir = cam_dir / "rgb"
    depth_dir = cam_dir / "depth"
    rgb_dir.mkdir(exist_ok=True)
    depth_dir.mkdir(exist_ok=True)

    ts_lines = []
    total_frames = 0
    chunk_files_to_cleanup = []

    for idx_path in index_files:
        chunk_id = int(idx_path.stem.split("_")[1])
        rgb_chunk = cam_dir / f"rgb_{chunk_id:06d}.mjpg"
        depth_chunk = cam_dir / f"depth_{chunk_id:06d}.z16"

        raw = idx_path.read_bytes()
        header = parse_index_header(raw[:INDEX_HEADER_SIZE])
        record_count = int(header["record_count"])
        if record_count == 0:
            record_count = (len(raw) - INDEX_HEADER_SIZE) // INDEX_RECORD_SIZE

        if progress:
            print(f"  {idx_path.name}: {record_count} frames", end="", flush=True)

        rgb_f = open(rgb_chunk, "rb") if rgb_chunk.exists() else None
        depth_f = open(depth_chunk, "rb") if depth_chunk.exists() else None

        t0 = time.time()
        for i in range(record_count):
            offset = INDEX_HEADER_SIZE + i * INDEX_RECORD_SIZE
            rec = parse_index_record(raw[offset : offset + INDEX_RECORD_SIZE])

            fname = f"{total_frames:06d}"

            if rgb_f and rec["rgb_bytes"] > 0:
                rgb_f.seek(rec["rgb_offset"])
                (rgb_dir / f"{fname}.jpg").write_bytes(rgb_f.read(rec["rgb_bytes"]))

            if depth_f and rec["depth_bytes"] > 0:
                depth_f.seek(rec["depth_offset"])
                (depth_dir / f"{fname}.z16").write_bytes(depth_f.read(rec["depth_bytes"]))

            ts_lines.append(
                f"{total_frames}  {rec['rgb_rs_timestamp_ms']:.6f}  "
                f"{rec['depth_rs_timestamp_ms']:.6f}  "
                f"{rec['host_receive_unix_ns']}  "
                f"{rec['rgb_frame_number']}  {rec['depth_frame_number']}  "
                f"{rec['rgb_bytes']}  {rec['depth_bytes']}"
            )
            total_frames += 1

        if rgb_f:
            rgb_f.close()
        if depth_f:
            depth_f.close()

        # Track original files for potential cleanup
        chunk_files_to_cleanup.extend([idx_path, rgb_chunk, depth_chunk])

        if progress:
            elapsed = time.time() - t0
            rate = record_count / elapsed if elapsed > 0 else 0
            print(f" ({rate:.0f} fps)", flush=True)

    # Write timestamps.txt
    ts_path = cam_dir / "timestamps.txt"
    ts_path.write_text(TS_HEADER + "\n".join(ts_lines) + "\n")

    rgb_count = len(list(rgb_dir.iterdir()))
    depth_count = len(list(depth_dir.iterdir()))

    # ── Cleanup ──
    cleaned = 0
    if cleanup:
        for f in chunk_files_to_cleanup:
            if f.exists():
                f.unlink()
                cleaned += 1

    return {
        "camera": cam_name, "frames": total_frames,
        "rgb_files": rgb_count, "depth_files": depth_count,
        "ts_lines": len(ts_lines), "cleaned": cleaned,
    }


# ── Validation ──

def verify_camera(cam_dir: Path) -> list[str]:
    """Return list of issues found (empty = all good)."""
    issues = []
    rgb_dir = cam_dir / "rgb"
    depth_dir = cam_dir / "depth"
    ts_file = cam_dir / "timestamps.txt"

    if not ts_file.exists():
        issues.append("timestamps.txt missing")

    rgb_files = sorted(rgb_dir.iterdir()) if rgb_dir.exists() else []
    depth_files = sorted(depth_dir.iterdir()) if depth_dir.exists() else []

    # Check JPEG headers
    bad_jpgs = []
    for f in rgb_files[:5] + rgb_files[-5:]:  # spot-check first/last 5
        try:
            head = f.read_bytes()[:2]
            tail = f.read_bytes()[-2:]
            if head != b"\xff\xd8" or tail != b"\xff\xd9":
                bad_jpgs.append(f.name)
        except Exception:
            bad_jpgs.append(f.name)
    if bad_jpgs:
        issues.append(f"corrupt JPEGs: {bad_jpgs}")

    # Check depth file sizes
    bad_depths = []
    for f in depth_files[:5] + depth_files[-5:]:
        sz = f.stat().st_size
        if sz != 1280 * 720 * 2:
            bad_depths.append(f"{f.name} ({sz} bytes)")
    if bad_depths:
        issues.append(f"wrong-size depth files: {bad_depths}")

    # Check timestamp line count
    if ts_file.exists():
        lines = [l for l in ts_file.read_text().splitlines() if not l.startswith("#")]
        expected = len(rgb_files) or len(depth_files)
        if expected and len(lines) != expected:
            issues.append(
                f"timestamps.txt has {len(lines)} data lines, "
                f"expected {expected}"
            )

    return issues


# ── Episode discovery ──

def find_episodes(root: Path) -> list[Path]:
    """
    If root itself contains camera dirs, return [root].
    Otherwise, scan for subdirectories that contain camera dirs.
    """
    if any(d.is_dir() and any(d.glob("index_*.bin"))
           for d in root.iterdir()):
        return [root]
    return sorted(
        p for p in root.iterdir()
        if p.is_dir() and any(
            sub.is_dir() and any(sub.glob("index_*.bin"))
            for sub in p.iterdir()
        )
    )


def find_camera_dirs(episode_dir: Path) -> list[Path]:
    return sorted(
        d for d in episode_dir.iterdir()
        if d.is_dir() and any(d.glob("index_*.bin"))
    )


# ── Main ──

def main():
    parser = argparse.ArgumentParser(
        description="Split Mode2 MJPEG/Z16 chunks into per-frame files + timestamps"
    )
    parser.add_argument(
        "path", type=Path,
        help="Episode directory, or a parent directory containing multiple episodes"
    )
    parser.add_argument(
        "--cleanup", action="store_true",
        help="Delete original index_*.bin, rgb_*.mjpg, depth_*.z16 after splitting"
    )
    parser.add_argument(
        "--verify", action="store_true",
        help="Run validation after splitting"
    )
    parser.add_argument(
        "--force", action="store_true",
        help="Re-process even if rgb/ already exists"
    )
    args = parser.parse_args()

    root = args.path.resolve()
    if not root.exists():
        print(f"Error: {root} not found")
        sys.exit(1)

    episodes = find_episodes(root)
    if not episodes:
        print(f"No episodes found in {root}")
        sys.exit(1)

    print(f"Found {len(episodes)} episode(s):")
    for ep in episodes:
        cams = find_camera_dirs(ep)
        print(f"  {ep.name}/  ({len(cams)} camera(s): {', '.join(d.name for d in cams)})")
    print()

    if args.cleanup:
        print("WARNING: --cleanup enabled: original chunk files will be DELETED after splitting")
        print()

    all_ok = True
    for ep in episodes:
        print(f"{'='*60}")
        print(f"Episode: {ep.name}")
        print(f"{'='*60}")

        for cam_dir in find_camera_dirs(ep):
            if not args.force and (cam_dir / "rgb").exists() and list((cam_dir / "rgb").iterdir()):
                print(f"  [{cam_dir.name}] SKIP: rgb/ already populated (use --force to redo)")
                continue

            t0 = time.time()
            stat = process_camera(cam_dir, cleanup=args.cleanup)
            elapsed = time.time() - t0

            if "error" in stat:
                print(f"  [{stat['camera']}] ERROR: {stat['error']}")
                all_ok = False
                continue

            print(f"  [{stat['camera']}] {stat['frames']} frames in {elapsed:.1f}s "
                  f"-> rgb/{stat['rgb_files']}, depth/{stat['depth_files']}, "
                  f"timestamps.txt ({stat['ts_lines']} lines)", end="")
            if stat["cleaned"]:
                print(f", cleaned {stat['cleaned']} original files", end="")
            print()

            if args.verify:
                issues = verify_camera(cam_dir)
                if issues:
                    print(f"    [!] Issues found:")
                    for issue in issues:
                        print(f"       - {issue}")
                    all_ok = False
                else:
                    print(f"    [OK] Verification passed")

        # Episode-level summary
        if args.verify:
            for cam_dir in find_camera_dirs(ep):
                pass  # already verified above

    print(f"\n{'='*60}")
    print("All done." if all_ok else "Done with warnings (see above).")


if __name__ == "__main__":
    main()
