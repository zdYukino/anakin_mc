#!/usr/bin/env python3
"""Install the pinned official native ORT into this repository, or check an override."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import sys
import tempfile
import urllib.request
import zipfile

REPO = Path(__file__).resolve().parents[1]
MANIFEST = json.loads((REPO / "docs/onnxruntime-provenance.json").read_text())
DEFAULT_ROOT = REPO / ".deps/onnxruntime-linux-x64-1.20.1"


def digest(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def validate(root, managed=False):
    required = (
        "include/onnxruntime_c_api.h", "include/onnxruntime_cxx_api.h",
        "lib/libonnxruntime.so", "lib/libonnxruntime.so.1",
        "lib/libonnxruntime_providers_shared.so", "VERSION_NUMBER",
    )
    for relative in required:
        if not (root / relative).is_file():
            raise ValueError(f"Missing ORT file: {root / relative}")
    if (root / "VERSION_NUMBER").read_text().strip() != MANIFEST["version"]:
        raise ValueError(f"ORT must be version {MANIFEST['version']}: {root}")
    for library in ("libonnxruntime.so", "libonnxruntime_providers_shared.so"):
        with (root / "lib" / library).open("rb") as stream:
            if stream.read(5) != b"\x7fELF\x02":
                raise ValueError(f"ORT library is not a 64-bit ELF: {root / 'lib' / library}")
    if (root / "lib/libonnxruntime.so.1").resolve() != (root / "lib/libonnxruntime.so").resolve():
        raise ValueError(f"ORT SONAME alias does not select the validated library: {root}")
    if managed:
        record = json.loads((root / "provenance.json").read_text())
        if record.get("archive_sha256") != MANIFEST["archive_sha256"]:
            raise ValueError("Installed ORT provenance differs from the pinned archive")
        checksums = record.get("files_sha256", {})
        if not checksums or not set(required[:-1]) - {"lib/libonnxruntime.so.1"} <= checksums.keys():
            raise ValueError("Installed ORT checksum record is incomplete")
        for relative, checksum in checksums.items():
            path = root / relative
            if root.resolve() not in path.resolve().parents or digest(path) != checksum:
                raise ValueError(f"Installed ORT checksum mismatch: {path}")


def extract(archive, target):
    with zipfile.ZipFile(archive) as package:
        names = package.namelist()
        headers = [name for name in names
                   if Path(name).parent.as_posix() == "build/native/include" and name.endswith(".h")]
        if not headers:
            raise ValueError("ORT archive has no native headers")
        mapping = {name: f"include/{Path(name).name}" for name in headers}
        mapping.update({
            f"runtimes/linux-x64/native/{name}": f"lib/{name}"
            for name in ("libonnxruntime.so", "libonnxruntime_providers_shared.so")
        })
        mapping.update({name: name for name in ("LICENSE", "ThirdPartyNotices.txt", "Microsoft.ML.OnnxRuntime.nuspec")})
        checksums = {}
        for member, relative in mapping.items():
            if names.count(member) != 1:
                raise ValueError(f"Missing or duplicate ORT archive member: {member}")
            destination = target / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            with package.open(member) as source, destination.open("wb") as output:
                shutil.copyfileobj(source, output)
            checksums[relative] = digest(destination)
    (target / "lib/libonnxruntime.so.1").symlink_to("libonnxruntime.so")
    (target / "VERSION_NUMBER").write_text(MANIFEST["version"] + "\n")
    record = dict(MANIFEST, files_sha256=checksums)
    (target / "provenance.json").write_text(json.dumps(record, indent=2) + "\n")
    validate(target, managed=True)


def prepare(archive=None, check=False):
    external = os.environ.get("ONNXRUNTIME_ROOT")
    if external:
        root = Path(external).resolve()
        validate(root)
        print(f"Using explicit ONNXRUNTIME_ROOT: {root}")
        return root
    root = DEFAULT_ROOT
    if check:
        validate(root, managed=True)
        print(f"Verified ORT: {root}")
        return root
    if platform.system() != "Linux" or platform.machine() != "x86_64":
        raise ValueError("The pinned deployment supports Linux x86_64")
    root.parent.mkdir(parents=True, exist_ok=True)
    with (root.parent / "ort.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if root.exists():
            validate(root, managed=True)
            print(f"Verified ORT already installed: {root}")
            return root
        cache = root.parent / "cache"
        cache.mkdir(exist_ok=True)
        selected = Path(archive).resolve() if archive else cache / "microsoft.ml.onnxruntime.1.20.1.nupkg"
        if not selected.is_file():
            if archive:
                raise ValueError(f"Missing local ORT archive: {selected}")
            with tempfile.TemporaryDirectory(prefix="download-", dir=cache) as directory:
                temporary = Path(directory) / "ort.nupkg"
                print(f"Downloading {MANIFEST['url']}", flush=True)
                with urllib.request.urlopen(MANIFEST["url"], timeout=60) as source, temporary.open("wb") as output:
                    shutil.copyfileobj(source, output)
                if digest(temporary) != MANIFEST["archive_sha256"]:
                    raise ValueError("Downloaded ORT archive SHA256 mismatch")
                temporary.replace(selected)
        if digest(selected) != MANIFEST["archive_sha256"]:
            raise ValueError(f"ORT archive SHA256 mismatch: {selected}")
        with tempfile.TemporaryDirectory(prefix="extract-", dir=root.parent) as directory:
            staging = Path(directory) / "native"
            staging.mkdir()
            extract(selected, staging)
            staging.rename(root)
    print(f"Installed verified ORT: {root}")
    return root


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, help="reuse a local official NuGet archive; still verify SHA256")
    parser.add_argument("--check", action="store_true", help="validate the selected installation without downloading")
    args = parser.parse_args()
    try:
        prepare(args.archive, args.check)
    except (OSError, ValueError, zipfile.BadZipFile) as error:
        print(f"ORT setup failed: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
