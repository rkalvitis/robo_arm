#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Organizes a flat AirDrop dump of Pro Reeflex focus stacks into one
folder per stack, with a manifest for pose matching.

Reeflex saves each focus-stacked shot as N focus-bracketed DNGs
(default 11, ~2 s apart) followed by ONE composite JPG whose EXIF
timestamp is copied from the LAST DNG. This script:

1. reads DateTimeOriginal + SubSecTimeOriginal (millisecond
   precision) from every DNG/JPG - no external dependencies;
2. sorts by time and splits into bursts wherever the gap between
   consecutive files exceeds --gap-seconds (frames inside a stack
   are ~2 s apart; repositioning takes much longer);
3. a burst of exactly --stack-size DNGs + one trailing JPG is a
   stack; anything else (clock shots, ring-marker photos, aborted
   stacks) goes to unassigned/ and is listed in the manifest;
4. if the number of stacks equals --stops x --rotations, each stack
   is provisionally labeled (stop, rotation) in shooting order:
   all rotations of stop 0 first, then stop 1, ... Retakes break
   this order - prune them and re-run, or rely on the OptiTrack
   timestamp matching, which needs no ordering at all;
5. RENAMES every file self-describingly, reading the per-frame
   focus position from the Apple MakerNote (recorded automatically
   by the camera - no manual notes needed):
       s03_r120_f07_fp181.dng   stop 3, rotation 120 deg, frame 7
                                of the sweep, focus position 181
       s03_r120_stacked.jpg     the Reeflex composite of that stack
   (unlabeled runs use stack017_... prefixes instead);
6. writes manifest.csv: one row per stack with old->new names,
   the focus sweep, and t_first / t_last / t_mid (use t_mid to
   look up the pose - the whole stack shares one pose since robot
   and turntable are at rest), and warns if any stack's focus
   sweep differs from the session's dominant one (anchor touched);
7. with --move or --copy, places files into OUT_DIR (per-stack
   folders, or all in one folder with --flat); the default is a
   DRY RUN that only prints the plan and writes the manifest.

Usage:
  python3 organize_stacks.py DUMP_DIR OUT_DIR                # dry run
  python3 organize_stacks.py DUMP_DIR OUT_DIR --move
  python3 organize_stacks.py DUMP_DIR OUT_DIR --copy \\
      --stops 9 --rotations 36 --rot-step 10

Keep only the DNGs in the optitrack pipeline's photos/ folder: the
composite JPG duplicates the last DNG's timestamp and would show up
as a phantom extra photo in frames.csv.
"""

import argparse
import csv
import datetime
import os
import shutil
import struct
import sys

DNG_EXTENSIONS = {".dng"}
JPG_EXTENSIONS = {".jpg", ".jpeg"}

TAG_EXIF_IFD = 0x8769
TAG_DATETIME_ORIGINAL = 0x9003
TAG_SUBSEC_ORIGINAL = 0x9291
TAG_MAKERNOTE = 0x927C
APPLE_TAG_FOCUS_POSITION = 0x002F


def parse_tiff_exif(data):
    """DateTimeOriginal/SubSecTimeOriginal from a TIFF blob (DNG or
    the Exif payload of a JPG APP1 segment)."""

    if data[:2] == b"II":
        endian = "<"
    elif data[:2] == b"MM":
        endian = ">"
    else:
        return {}

    def u16(offset):
        return struct.unpack(endian + "H", data[offset:offset + 2])[0]

    def u32(offset):
        return struct.unpack(endian + "I", data[offset:offset + 4])[0]

    def read_ifd(offset):
        entries = {}
        count = u16(offset)
        for index in range(count):
            entry = offset + 2 + 12 * index
            tag = u16(entry)
            value_count = u32(entry + 4)
            entries[tag] = (value_count, entry + 8)
        return entries

    def ascii_value(value_count, value_offset):
        if value_count <= 4:
            raw = data[value_offset:value_offset + value_count]
        else:
            start = u32(value_offset)
            raw = data[start:start + value_count]
        return raw.split(b"\0")[0].decode("ascii", "replace").strip()

    result = {}
    ifd0 = read_ifd(u32(4))
    if TAG_EXIF_IFD not in ifd0:
        return result
    exif_ifd = read_ifd(u32(ifd0[TAG_EXIF_IFD][1]))
    if TAG_DATETIME_ORIGINAL in exif_ifd:
        result["datetime"] = ascii_value(*exif_ifd[TAG_DATETIME_ORIGINAL])
    if TAG_SUBSEC_ORIGINAL in exif_ifd:
        result["subsec"] = ascii_value(*exif_ifd[TAG_SUBSEC_ORIGINAL])

    # Lens focus position of the frame, from the Apple MakerNote
    # (arbitrary actuator units; monotonic with focus distance).
    # Reeflex focus stacks sweep it frame by frame - recording it
    # per frame identifies the bracket without any manual notes.
    if TAG_MAKERNOTE in exif_ifd:
        count, value_offset = exif_ifd[TAG_MAKERNOTE]
        start = u32(value_offset) if count > 4 else value_offset
        note = data[start:start + count]
        if note.startswith(b"Apple iOS\x00"):
            note_endian = ">" if note[12:14] == b"MM" else "<"

            def n16(offset):
                return struct.unpack(note_endian + "H",
                                     note[offset:offset + 2])[0]

            def n32(offset):
                return struct.unpack(note_endian + "I",
                                     note[offset:offset + 4])[0]

            for index in range(n16(14)):
                entry = 16 + 12 * index
                if n16(entry) == APPLE_TAG_FOCUS_POSITION:
                    result["focus"] = n32(entry + 8)
                    break
    return result


def read_exif(path):
    """Returns (capture datetime with milliseconds, focus position);
    either may be None."""

    with open(path, "rb") as handle:
        head = handle.read(4 * 1024 * 1024)

    extension = os.path.splitext(path)[1].lower()

    if extension in JPG_EXTENSIONS:
        tags = {}
        offset = 2
        while offset + 4 < len(head) and head[offset] == 0xFF:
            marker = head[offset + 1]
            size = struct.unpack(">H", head[offset + 2:offset + 4])[0]
            if marker == 0xE1 and head[offset + 4:offset + 10] == b"Exif\0\0":
                tags = parse_tiff_exif(head[offset + 10:offset + 2 + size])
                break
            if marker == 0xDA:      # start of scan - no EXIF found
                break
            offset += 2 + size
    else:
        tags = parse_tiff_exif(head)

    if "datetime" not in tags:
        return None, None

    stamp = datetime.datetime.strptime(tags["datetime"],
                                       "%Y:%m:%d %H:%M:%S")
    subsec = tags.get("subsec", "")
    if subsec.isdigit():
        stamp += datetime.timedelta(
            seconds=int(subsec) / (10.0 ** len(subsec))
        )
    return stamp, tags.get("focus")


def collect(source_dir):
    """All DNG/JPG files with timestamps and focus positions,
    sorted by (time, name)."""

    records = []
    for name in sorted(os.listdir(source_dir)):
        extension = os.path.splitext(name)[1].lower()
        if extension not in DNG_EXTENSIONS | JPG_EXTENSIONS:
            continue
        path = os.path.join(source_dir, name)
        stamp, focus = read_exif(path)
        if stamp is None:
            print("WARNING: no EXIF timestamp in %s - skipped" % name)
            continue
        records.append((stamp, name, extension in DNG_EXTENSIONS,
                        focus))
    records.sort(key=lambda record: (record[0], record[1]))
    return records


def split_bursts(records, gap_seconds):
    bursts = []
    current = []
    for record in records:
        if current and (record[0] - current[-1][0]).total_seconds() \
                > gap_seconds:
            bursts.append(current)
            current = []
        current.append(record)
    if current:
        bursts.append(current)
    return bursts


def classify(bursts, stack_size):
    """Splits bursts into stacks and unassigned files."""

    stacks = []
    unassigned = []
    for burst in bursts:
        dngs = [record for record in burst if record[2]]
        jpgs = [record for record in burst if not record[2]]
        is_stack = (
            len(dngs) == stack_size
            and len(jpgs) <= 1
            and (not jpgs or jpgs[0][0] >= dngs[-1][0])
        )
        if is_stack:
            stacks.append((dngs, jpgs[0] if jpgs else None))
            if not jpgs:
                print("WARNING: stack starting %s has no composite "
                      "JPG (stacking off or failed?)" % dngs[0][1])
        else:
            unassigned.extend(burst)
            print("note: burst of %d DNG + %d JPG starting %s -> "
                  "unassigned (clock shot / marker / aborted stack)"
                  % (len(dngs), len(jpgs), burst[0][1]))
    return stacks, unassigned


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("source", help="flat folder with the AirDropped files")
    parser.add_argument("output", help="destination folder")
    parser.add_argument("--stack-size", type=int, default=11,
                        help="focus frames per stack (default 11)")
    parser.add_argument("--gap-seconds", type=float, default=10.0,
                        help="time gap that separates stacks (default 10)")
    parser.add_argument("--stops", type=int, default=9,
                        help="camera positions incl. the start pose "
                             "(default 9 = start + 8 arc stops)")
    parser.add_argument("--rotations", type=int, default=36,
                        help="turntable positions per stop (default 36)")
    parser.add_argument("--rot-step", type=float, default=10.0,
                        help="degrees per turntable step (default 10)")
    parser.add_argument("--flat", action="store_true",
                        help="all renamed files directly in OUT_DIR "
                             "(no per-stack folders) - convenient "
                             "for the optitrack photos/ contract")
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--move", action="store_true",
                        help="move files into the output structure")
    action.add_argument("--copy", action="store_true",
                        help="copy files instead of moving")
    arguments = parser.parse_args()

    records = collect(arguments.source)
    if not records:
        print("no DNG/JPG files found in %s" % arguments.source)
        return 1
    print("%d files, %s .. %s" % (
        len(records), records[0][0], records[-1][0]))

    bursts = split_bursts(records, arguments.gap_seconds)
    stacks, unassigned = classify(bursts, arguments.stack_size)

    expected = arguments.stops * arguments.rotations
    labeled = len(stacks) == expected
    print("%d stacks found (%d expected for %d stops x %d rotations)"
          % (len(stacks), expected, arguments.stops,
             arguments.rotations))
    if not labeled:
        print("stack count does not match - folders get sequential "
              "names only; prune retakes and re-run, or rely on "
              "timestamp matching.")

    dry_run = not (arguments.move or arguments.copy)
    if dry_run:
        print("DRY RUN - no files are touched (use --move or --copy).")
    transfer = shutil.copy2 if arguments.copy else shutil.move

    # Focus-sweep consistency: with the Reeflex anchor locked, every
    # stack sweeps the same focus positions - a deviating stack
    # means the focus was touched mid-session.
    sweep_counts = {}
    for dngs, _jpg in stacks:
        sweep = tuple(record[3] for record in dngs)
        sweep_counts[sweep] = sweep_counts.get(sweep, 0) + 1
    dominant_sweep = (max(sweep_counts, key=sweep_counts.get)
                      if sweep_counts else ())
    if len(sweep_counts) > 1:
        print("WARNING: %d different focus sweeps found - the "
              "anchor focus changed during the session!"
              % len(sweep_counts))

    os.makedirs(arguments.output, exist_ok=True)
    manifest_path = os.path.join(arguments.output, "manifest.csv")
    with open(manifest_path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "stack_id", "stop", "rot_deg", "folder", "n_frames",
            "t_first", "t_last", "t_mid", "duration_s",
            "focus_positions", "composite", "composite_original",
            "frames", "frames_original"])

        for index, (dngs, jpg) in enumerate(stacks):
            if labeled:
                stop = index // arguments.rotations
                rotation = (index % arguments.rotations) \
                    * arguments.rot_step
                prefix = "s%02d_r%03d" % (stop, rotation)
                stop_text, rot_text = str(stop), "%g" % rotation
            else:
                prefix = "stack%03d" % index
                stop_text, rot_text = "", ""

            sweep = tuple(record[3] for record in dngs)
            if sweep != dominant_sweep:
                print("WARNING: %s focus sweep %s differs from the "
                      "session's dominant sweep" % (prefix, sweep))

            # Self-describing names: stack prefix, frame number in
            # the sequence, Apple focus position of that frame.
            frame_names = [
                "%s_f%02d_fp%s.dng" % (
                    prefix, frame_index,
                    record[3] if record[3] is not None else "na")
                for frame_index, record in enumerate(dngs)]
            composite_name = "%s_stacked.jpg" % prefix if jpg else ""

            t_first, t_last = dngs[0][0], dngs[-1][0]
            t_mid = t_first + (t_last - t_first) / 2
            writer.writerow([
                index, stop_text, rot_text,
                "." if arguments.flat else prefix, len(dngs),
                t_first.isoformat(), t_last.isoformat(),
                t_mid.isoformat(),
                "%.3f" % (t_last - t_first).total_seconds(),
                ";".join(str(value) for value in sweep),
                composite_name, jpg[1] if jpg else "",
                ";".join(frame_names),
                ";".join(record[1] for record in dngs)])

            if dry_run:
                continue
            destination = (arguments.output if arguments.flat
                           else os.path.join(arguments.output, prefix))
            os.makedirs(destination, exist_ok=True)
            for record, new_name in zip(dngs, frame_names):
                transfer(os.path.join(arguments.source, record[1]),
                         os.path.join(destination, new_name))
            if jpg:
                transfer(os.path.join(arguments.source, jpg[1]),
                         os.path.join(destination, composite_name))

        for record in unassigned:
            writer.writerow([
                "", "", "", "unassigned", "", record[0].isoformat(),
                "", "", "", "", "", "", "", record[1]])
            if dry_run:
                continue
            destination = os.path.join(arguments.output, "unassigned")
            os.makedirs(destination, exist_ok=True)
            transfer(os.path.join(arguments.source, record[1]),
                     os.path.join(destination, record[1]))

    print("manifest written: %s" % manifest_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
