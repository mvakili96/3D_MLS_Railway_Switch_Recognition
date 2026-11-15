"""Admission checks and paging for backend-owned allocations, not algorithm changes."""

import mmap
import os
from pathlib import Path
import shutil
import sys

try:
    import resource
except ImportError:
    resource = None


class SceneCacheError(ValueError):
    status = "cache_error"


class ResourceLimitError(SceneCacheError):
    status = "resource_limited"


def resident_bytes():
    try:
        pages = int(Path("/proc/self/statm").read_text().split()[1])
        return pages*os.sysconf("SC_PAGE_SIZE")
    except (OSError, ValueError, IndexError, AttributeError):
        if resource is None:
            raise ResourceLimitError("resident memory accounting is unavailable on this platform")
        peak = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        return int(peak if sys.platform == "darwin" else peak*1024)


class MemoryBudget:
    """Conservative allocation estimates plus RSS checks at backend boundaries."""

    def __init__(self, limit):
        self.limit = int(limit)
        self.observed_peak = 0

    def require(self, additional=0, *, location="backend", reclaim=None):
        used = resident_bytes()
        self.observed_peak = max(self.observed_peak, used)
        if used+additional > self.limit and reclaim is not None:
            reclaim()
            used = resident_bytes()
        if used+additional > self.limit:
            raise ResourceLimitError(f"{location}: resident {used} plus estimated allocation "
                                     f"{additional} exceeds memory budget {self.limit}")


def check_disk(directory, requested, limit):
    if requested > limit:
        raise ResourceLimitError(f"estimated cache/state bytes {requested} exceed disk budget {limit}")
    parent = Path(directory).resolve()
    while not parent.exists():
        parent = parent.parent
    if requested > shutil.disk_usage(parent).free:
        raise ResourceLimitError(f"insufficient free disk space for {requested} bytes")


def release_pages(array, *, writable=False):
    mapping = getattr(array, "_mmap", None)
    if mapping is None:
        return
    if writable:
        array.flush()
    if hasattr(mapping, "madvise") and hasattr(mmap, "MADV_DONTNEED"):
        mapping.madvise(mmap.MADV_DONTNEED)


def close_mapping(array, *, writable=False):
    mapping = getattr(array, "_mmap", None)
    if mapping is not None and not mapping.closed:
        if writable:
            array.flush()
        mapping.close()
