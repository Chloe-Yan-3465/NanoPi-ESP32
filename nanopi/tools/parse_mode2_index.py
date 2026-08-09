#!/usr/bin/env python3
"""Export RSDIDX3 index_*.bin files to readable timestamp tables.

Python 3.10+ is required; only the standard library is used.

Examples:
    python3 parse_mode2_index.py /data/ep_xxx/head_cam
    python3 parse_mode2_index.py /data/ep_xxx/head_cam --format csv
    python3 parse_mode2_index.py /data/ep_xxx/head_cam -o timestamps.txt
    python3 parse_mode2_index.py /data/ep_xxx --recursive
"""

from __future__ import annotations

import argparse
import csv
import json
import struct
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable


HEADER_STRUCT = struct.Struct("<8s10IfqQ32s")
RECORD_STRUCT = struct.Struct("<QQQqddqqIIQQIIII")
EXPECTED_MAGIC = b"RSDIDX3"

TIMESTAMP_DOMAINS = {
    0: "hardware_clock",
    1: "system_time",
    2: "global_time",
}

OUTPUT_COLUMNS = [
    "camera_path",
    "source_index",
    "chunk_id",
    "record_in_chunk",
    "sequence",
    "rgb_frame_number",
    "depth_frame_number",
    "host_receive_unix_ns",
    "host_receive_iso_utc",
    "host_delta_ms",
    "rgb_rs_timestamp_ms",
    "rgb_rs_iso_utc",
    "depth_rs_timestamp_ms",
    "depth_rs_iso_utc",
    "depth_minus_rgb_ms",
    "host_minus_rgb_rs_ms",
    "host_minus_depth_rs_ms",
    "rgb_sensor_timestamp",
    "depth_sensor_timestamp",
    "rgb_timestamp_domain",
    "rgb_timestamp_domain_name",
    "depth_timestamp_domain",
    "depth_timestamp_domain_name",
    "rgb_offset",
    "depth_offset",
    "rgb_bytes",
    "depth_bytes",
    "flags",
    "has_rgb_sensor_timestamp",
    "has_depth_sensor_timestamp",
]


@dataclass
class ParsedIndex:
    path: Path
    header: dict[str, Any]
    records: list[dict[str, Any]]
    warnings: list[str]


def fourcc(value: int) -> str:
    return struct.pack("<I", value).decode("ascii", errors="replace").rstrip("\x00")


def unix_ns_to_iso(value: int) -> str:
    if value <= 0:
        return ""
    seconds, nanoseconds = divmod(value, 1_000_000_000)
    try:
        prefix = datetime.fromtimestamp(seconds, timezone.utc).strftime(
            "%Y-%m-%dT%H:%M:%S"
        )
    except (OverflowError, OSError, ValueError):
        return ""
    return f"{prefix}.{nanoseconds:09d}Z"


def rs_ms_to_iso(value_ms: float, domain: int) -> str:
    # Hardware-clock timestamps are relative to the device and are not UTC.
    if domain not in (1, 2) or value_ms < 946_684_800_000.0:
        return ""
    return unix_ns_to_iso(round(value_ms * 1_000_000.0))


def parse_header(raw: bytes, path: Path) -> dict[str, Any]:
    if len(raw) != HEADER_STRUCT.size:
        raise ValueError(f"{path}: truncated index header")

    values = HEADER_STRUCT.unpack(raw)
    magic = values[0].rstrip(b"\x00")
    if magic != EXPECTED_MAGIC:
        raise ValueError(f"{path}: bad magic {magic!r}; expected {EXPECTED_MAGIC!r}")

    (
        _magic,
        version,
        header_bytes,
        record_bytes,
        width,
        height,
        fps,
        chunk_id,
        session_id,
        rgb_fourcc,
        depth_fourcc,
        depth_scale,
        created_unix_ns,
        record_count,
        _reserved,
    ) = values

    if header_bytes != HEADER_STRUCT.size:
        raise ValueError(
            f"{path}: header_bytes={header_bytes}, expected {HEADER_STRUCT.size}"
        )
    if record_bytes != RECORD_STRUCT.size:
        raise ValueError(
            f"{path}: record_bytes={record_bytes}, expected {RECORD_STRUCT.size}"
        )

    return {
        "version": version,
        "header_bytes": header_bytes,
        "record_bytes": record_bytes,
        "width": width,
        "height": height,
        "fps": fps,
        "chunk_id": chunk_id,
        "session_id": session_id,
        "rgb_fourcc": fourcc(rgb_fourcc),
        "depth_fourcc": fourcc(depth_fourcc),
        "depth_scale": depth_scale,
        "created_unix_ns": created_unix_ns,
        "created_iso_utc": unix_ns_to_iso(created_unix_ns),
        "record_count": record_count,
    }


def parse_record(raw: bytes, header: dict[str, Any], record_index: int) -> dict[str, Any]:
    if len(raw) != RECORD_STRUCT.size:
        raise ValueError("truncated index record")

    (
        sequence,
        rgb_frame_number,
        depth_frame_number,
        host_receive_unix_ns,
        rgb_rs_timestamp_ms,
        depth_rs_timestamp_ms,
        rgb_sensor_timestamp,
        depth_sensor_timestamp,
        rgb_timestamp_domain,
        depth_timestamp_domain,
        rgb_offset,
        depth_offset,
        rgb_bytes,
        depth_bytes,
        flags,
        _reserved,
    ) = RECORD_STRUCT.unpack(raw)

    return {
        "chunk_id": header["chunk_id"],
        "record_in_chunk": record_index,
        "sequence": sequence,
        "rgb_frame_number": rgb_frame_number,
        "depth_frame_number": depth_frame_number,
        "host_receive_unix_ns": host_receive_unix_ns,
        "host_receive_iso_utc": unix_ns_to_iso(host_receive_unix_ns),
        "host_delta_ms": "",
        "rgb_rs_timestamp_ms": f"{rgb_rs_timestamp_ms:.6f}",
        "rgb_rs_iso_utc": rs_ms_to_iso(
            rgb_rs_timestamp_ms, rgb_timestamp_domain
        ),
        "depth_rs_timestamp_ms": f"{depth_rs_timestamp_ms:.6f}",
        "depth_rs_iso_utc": rs_ms_to_iso(
            depth_rs_timestamp_ms, depth_timestamp_domain
        ),
        "depth_minus_rgb_ms": f"{depth_rs_timestamp_ms - rgb_rs_timestamp_ms:.6f}",
        "host_minus_rgb_rs_ms": f"{host_receive_unix_ns / 1e6 - rgb_rs_timestamp_ms:.6f}",
        "host_minus_depth_rs_ms": f"{host_receive_unix_ns / 1e6 - depth_rs_timestamp_ms:.6f}",
        "rgb_sensor_timestamp": rgb_sensor_timestamp,
        "depth_sensor_timestamp": depth_sensor_timestamp,
        "rgb_timestamp_domain": rgb_timestamp_domain,
        "rgb_timestamp_domain_name": TIMESTAMP_DOMAINS.get(
            rgb_timestamp_domain, f"unknown_{rgb_timestamp_domain}"
        ),
        "depth_timestamp_domain": depth_timestamp_domain,
        "depth_timestamp_domain_name": TIMESTAMP_DOMAINS.get(
            depth_timestamp_domain, f"unknown_{depth_timestamp_domain}"
        ),
        "rgb_offset": rgb_offset,
        "depth_offset": depth_offset,
        "rgb_bytes": rgb_bytes,
        "depth_bytes": depth_bytes,
        "flags": flags,
        "has_rgb_sensor_timestamp": int(bool(flags & 1)),
        "has_depth_sensor_timestamp": int(bool(flags & 2)),
    }


def companion_path(index_path: Path, prefix: str, extension: str) -> Path:
    suffix = index_path.stem.removeprefix("index_")
    return index_path.with_name(f"{prefix}_{suffix}.{extension}")


def validate_payloads(parsed: ParsedIndex) -> None:
    if not parsed.records:
        return

    last = parsed.records[-1]
    expected_rgb_size = int(last["rgb_offset"]) + int(last["rgb_bytes"])
    expected_depth_size = int(last["depth_offset"]) + int(last["depth_bytes"])

    rgb_path = companion_path(parsed.path, "rgb", "mjpg")
    if not rgb_path.exists():
        parsed.warnings.append(f"missing RGB payload: {rgb_path.name}")
    elif rgb_path.stat().st_size != expected_rgb_size:
        parsed.warnings.append(
            f"{rgb_path.name}: size={rgb_path.stat().st_size}, "
            f"indexed_end={expected_rgb_size}"
        )

    depth_path = companion_path(parsed.path, "depth", "z16")
    if expected_depth_size == 0:
        return
    if not depth_path.exists():
        parsed.warnings.append(f"missing depth payload: {depth_path.name}")
    elif depth_path.stat().st_size != expected_depth_size:
        parsed.warnings.append(
            f"{depth_path.name}: size={depth_path.stat().st_size}, "
            f"indexed_end={expected_depth_size}"
        )


def read_index(path: Path) -> ParsedIndex:
    file_size = path.stat().st_size
    if file_size < HEADER_STRUCT.size:
        raise ValueError(f"{path}: file is smaller than the index header")

    warnings: list[str] = []
    with path.open("rb") as stream:
        header = parse_header(stream.read(HEADER_STRUCT.size), path)
        payload_bytes = file_size - HEADER_STRUCT.size
        complete_records, trailing_bytes = divmod(payload_bytes, RECORD_STRUCT.size)
        if trailing_bytes:
            warnings.append(f"index has {trailing_bytes} trailing byte(s)")

        declared_count = int(header["record_count"])
        if declared_count == 0 and complete_records > 0:
            warnings.append(
                "header record_count is zero; parsing complete records from file size"
            )
            count = complete_records
        elif declared_count > complete_records:
            raise ValueError(
                f"{path}: header declares {declared_count} records, "
                f"but only {complete_records} are complete"
            )
        else:
            count = declared_count
            if complete_records > declared_count:
                warnings.append(
                    f"file contains {complete_records - declared_count} extra complete record(s)"
                )

        records = [
            parse_record(stream.read(RECORD_STRUCT.size), header, index)
            for index in range(count)
        ]

    parsed = ParsedIndex(path=path, header=header, records=records, warnings=warnings)
    validate_payloads(parsed)
    return parsed


def find_index_files(path: Path, recursive: bool) -> list[Path]:
    if path.is_file():
        return [path]
    if not path.is_dir():
        raise ValueError(f"input does not exist: {path}")

    direct = sorted(path.glob("index_*.bin"))
    if direct:
        return direct
    if recursive:
        return sorted(path.rglob("index_*.bin"))
    raise ValueError(
        f"no index_*.bin files in {path}; pass --recursive for ep_xxx/camera_name layout"
    )


def add_cross_record_fields(records: list[dict[str, Any]]) -> list[str]:
    previous_by_camera: dict[str, dict[str, Any]] = {}
    warnings: list[str] = []
    for record in records:
        camera = str(record["camera_path"])
        host_ns = int(record["host_receive_unix_ns"])
        previous = previous_by_camera.get(camera)
        if previous is not None:
            previous_host_ns = int(previous["host_receive_unix_ns"])
            record["host_delta_ms"] = f"{(host_ns - previous_host_ns) / 1e6:.6f}"
            for field in ("sequence", "rgb_frame_number", "depth_frame_number"):
                if int(record[field]) != int(previous[field]) + 1:
                    warnings.append(
                        f"{camera}: {field} gap between {previous[field]} and {record[field]}"
                    )
        previous_by_camera[camera] = record
    return warnings


def write_table(
    output: Path,
    records: list[dict[str, Any]],
    parsed_files: list[ParsedIndex],
    output_format: str,
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)

    if output_format == "jsonl":
        with output.open("w", encoding="utf-8", newline="\n") as stream:
            for record in records:
                stream.write(json.dumps(record, ensure_ascii=False) + "\n")
        return

    delimiter = "\t" if output_format == "tsv" else ","
    with output.open("w", encoding="utf-8", newline="") as stream:
        if output_format == "tsv":
            stream.write("# RSDIDX3 timestamp export\n")
            stream.write(f"# index_files={len(parsed_files)} frames={len(records)}\n")
            for parsed in parsed_files:
                h = parsed.header
                stream.write(
                    f"# {parsed.path.name}: chunk={h['chunk_id']} "
                    f"records={len(parsed.records)} size={h['width']}x{h['height']} "
                    f"fps={h['fps']} rgb={h['rgb_fourcc']} "
                    f"depth={h['depth_fourcc']} depth_scale={h['depth_scale']:.9g}\n"
                )
                for warning in parsed.warnings:
                    stream.write(f"# WARNING {parsed.path.name}: {warning}\n")

        writer = csv.DictWriter(
            stream,
            fieldnames=OUTPUT_COLUMNS,
            delimiter=delimiter,
            extrasaction="ignore",
        )
        writer.writeheader()
        writer.writerows(records)


def output_extension(output_format: str) -> str:
    return {"tsv": ".txt", "csv": ".csv", "jsonl": ".jsonl"}[output_format]


def export_group(
    index_files: list[Path], output: Path, output_format: str, root: Path
) -> tuple[int, list[str]]:
    parsed_files = [read_index(path) for path in index_files]
    parsed_files.sort(
        key=lambda item: (
            str(item.path.parent),
            int(item.header["chunk_id"]),
            item.path.name,
        )
    )

    records: list[dict[str, Any]] = []
    warnings: list[str] = []
    for parsed in parsed_files:
        try:
            camera_path = str(parsed.path.parent.relative_to(root))
        except ValueError:
            camera_path = str(parsed.path.parent)
        if camera_path == ".":
            camera_path = parsed.path.parent.name
        for record in parsed.records:
            record["camera_path"] = camera_path
            record["source_index"] = parsed.path.name
            records.append(record)
        warnings.extend(f"{parsed.path.name}: {item}" for item in parsed.warnings)

    warnings.extend(add_cross_record_fields(records))
    write_table(output, records, parsed_files, output_format)
    print(f"[OK] {output}: {len(records)} frame(s), {len(parsed_files)} chunk(s)")
    for warning in warnings:
        print(f"[WARN] {warning}", file=sys.stderr)
    return len(records), warnings


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export mode2 RSDIDX3 binary indexes to TXT/CSV/JSONL."
    )
    parser.add_argument("input", type=Path, help="index file, camera directory, or episode")
    parser.add_argument("-o", "--output", type=Path, help="explicit combined output path")
    parser.add_argument(
        "--format",
        choices=("tsv", "csv", "jsonl"),
        default="tsv",
        help="output format; default tsv produces timestamps.txt",
    )
    parser.add_argument(
        "--recursive",
        action="store_true",
        help="find camera subdirectories below an ep_xxx directory",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="return a nonzero exit code when validation warnings are found",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    input_path = args.input.resolve()
    try:
        index_files = find_index_files(input_path, args.recursive)
    except ValueError as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        return 2

    root = input_path if input_path.is_dir() else input_path.parent
    try:
        if args.output is not None:
            _, warnings = export_group(
                index_files, args.output.resolve(), args.format, root
            )
            return 3 if args.strict and warnings else 0

        groups: dict[Path, list[Path]] = {}
        for index_path in index_files:
            groups.setdefault(index_path.parent, []).append(index_path)

        all_warnings: list[str] = []
        for directory, group_files in sorted(groups.items(), key=lambda item: str(item[0])):
            output = directory / f"timestamps{output_extension(args.format)}"
            _, warnings = export_group(group_files, output, args.format, root)
            all_warnings.extend(warnings)
        return 3 if args.strict and all_warnings else 0
    except (OSError, ValueError, struct.error) as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
