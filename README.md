# Railway track and switch recognition from 3D MLS

This repository preserves the existing C++ implementation of railway tracking
and switch-region recognition from 3D mobile laser scanning (MLS) data. A Python
port is planned. The current unpacked C++ code is the source of truth: preserve
its computational behavior and document proposed corrections separately.

## Repository contents

- `ADE code/PCLProject/PCLProject/PCLProject/`: original C++/CLI application,
  railway algorithm, geometry/state models, readers, exporters, and UI resources.
  The small `Jama/` and `etc/` numerical helpers are retained because the source
  uses them directly.
- `ADE code/PCLProject/PCLProject/Visualizer.sln` and project configuration:
  original Windows project/build references.
- `Multi-Railway_Track_and_Switch_Region_Recognition_Using_Mobile_Laser_Scanning_Data.pdf`:
  reference paper.
- [PYTHON_PORT_PLAN.md](PYTHON_PORT_PLAN.md): source audit, behavior differences,
  and staged implementation plan.

The original application uses Windows Forms/C++/CLI, PCL, VTK, Boost and LAS
libraries. Project files contain machine-specific paths. Retaining them documents
the existing setup; it does not provide a portable build or a runnable Python
implementation yet.

## What stays local

The root `.gitignore` excludes datasets and extracted data, dependency installation
bundles, the alternate project ZIP, compiled libraries/executables, build folders,
IDE caches and user settings, local credentials, and agent state. These files
remain on disk and are not part of the initial commit.

Keep future raw data under `data/` or `datasets/`, and generated results under
`outputs/` or `results/`. Common point-cloud, tabular, array and database formats
are also ignored by extension. Source files, dependency declarations and small
UI images are intentionally tracked. Do not use `git add -f` for local datasets.

The source tree contains its own pre-existing `.git` metadata. That history is
preserved locally and excluded from the outer repository; the outer repository
tracks the selected source files directly, without a submodule. Run this project's
Git commands from the repository root, because commands inside the nested source
repository refer to its separate history.

## Development direction

Start with the source-to-Python behavior specification, then port numerical
components, tracking, and switch recognition in stages. Include computational
orchestration that currently lives in the GUI. Keep inactive source paths inactive
and preserve defaults, templates, ordering, precision and reproducible quirks.

Synthetic data can support implementation and component checks before database
integration. Real-data experiments and comparisons with the original implementation
will follow when inputs and reference results are available.
