#!/usr/bin/env python3
"""Verify or restore losslessly archived ARM6 logs; no ROS or hardware access."""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import tempfile


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        while block := stream.read(1024 * 1024):
            result.update(block)
    return result.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='verify archives without extracting')
    args = parser.parse_args()
    root = Path(__file__).resolve().parent.parent
    data = json.loads((root / 'docs/validation_log_archives.json').read_text())
    if data['format'] != 1 or data['compression'] != 'gzip':
        raise ValueError('unsupported archive manifest')
    for entry in data['entries']:
        archive, original = [root / entry[key] for key in ('archive', 'original')]
        for path in (archive, original):
            if not path.resolve().is_relative_to(root / 'docs') or path.is_symlink():
                raise ValueError('archive paths must stay inside docs without symlinks')
        if archive.stat().st_size != entry['archive_bytes'] or digest(archive) != entry['archive_sha256']:
            raise ValueError(f'archive checksum mismatch: {archive}')
        if original.exists():
            if original.stat().st_size != entry['original_bytes'] or digest(original) != entry['sha256']:
                raise ValueError(f'existing file differs; refusing overwrite: {original}')
            print(f'Existing original verified: {entry["original"]}')
            continue
        temp_path = None
        try:
            out = None
            if not args.check:
                out = tempfile.NamedTemporaryFile(dir=original.parent, delete=False)
                temp_path = Path(out.name)
            try:
                h, size = hashlib.sha256(), 0
                with gzip.open(archive, 'rb') as source:
                    while block := source.read(1024 * 1024):
                        h.update(block)
                        size += len(block)
                        if size > entry['original_bytes']:
                            raise ValueError('expanded size exceeds manifest')
                        if out is not None:
                            out.write(block)
                if size != entry['original_bytes'] or h.hexdigest() != entry['sha256']:
                    raise ValueError(f'expanded checksum mismatch: {archive}')
            finally:
                if out is not None:
                    out.close()
            if temp_path is not None:
                # Hard-link creation is atomic and refuses to overwrite an existing file.
                os.link(temp_path, original)
            print(f'{"Verified" if args.check else "Restored"}: {entry["original"]}')
        finally:
            if temp_path is not None:
                temp_path.unlink(missing_ok=True)


if __name__ == '__main__':
    main()
