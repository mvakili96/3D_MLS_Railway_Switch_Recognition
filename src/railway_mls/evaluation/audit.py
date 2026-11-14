"""Independent fingerprints and computational dispositions for the 2018 reference."""

from hashlib import sha256
from importlib.resources import files
import json
from pathlib import Path

from .io import file_fingerprint


def reference_manifest():
    return json.loads(files(__package__).joinpath("reference.json").read_text())


def source_fingerprint():
    source = reference_manifest()["files"]
    return sha256(json.dumps(source, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def evaluation_audit(repo="."):
    manifest = reference_manifest()
    errors = []
    for entry in manifest["files"]:
        path = Path(repo) / entry["path"]
        if not path.is_file():
            errors.append({"path": entry["path"], "status": "missing"})
        elif file_fingerprint(path)["sha256"] != entry["sha256"]:
            errors.append({"path": entry["path"], "status": "drift"})
    return {"status": "ok" if not errors else "unverified", "errors": errors,
            "profile": "source2018", "source_fingerprint": source_fingerprint(),
            "file_count": len(manifest["files"]), "computational_scope": manifest["scope"],
            "native_windows_application_executed": False,
            "full_scene_python_accuracy_validated": False}
