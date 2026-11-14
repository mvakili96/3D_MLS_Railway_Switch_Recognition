# Ground-truth storage clues — 2026-11-14

**Subsequent update:** the user supplied a separate `evaluation data/` package.
Its Vancouver manually edited LAS, saved models and implemented evaluator are
now identified in [the evaluation package audit](EVALUATION_PACKAGE_AUDIT.md).
The negative search finding below concerns the earlier ADE code bundle and
its companion artifacts; it does not describe the newly supplied package.

No explicit ground-truth directory, annotation filename or ground-truth-loading
workflow was found in the checked application sources and companion artifacts.
This is a search result for the supplied material, not proof that annotations
are absent from the lab's database.

## Concrete identifiers for a database search

| Observed identifier | Evidence | Interpretation |
| --- | --- | --- |
| `E:\Theles Project\Vancouver data\Thales_11_20Las\` | Local `Vancouver/Thales_11_20Las/delete.txt.config`, lines 1–2. Backslashes are doubled in the text file. | A historical raw/profile-data location. Search the parent `Theles Project` and sibling folders for related annotation work. It is not identified as a ground-truth directory. The spelling **Theles** is present in the original metadata. |
| `GeoICT/Thales-Railway-Tracking` | Bundled native Git commit `625f4783994d423e37b472f4bcbb2adaaef6133d`, whose merge message names `https://gitlab.com/GeoICT/Thales-Railway-Tracking.git`. | An original project/repository identifier, useful for locating related project archives or directories. No ground-truth location is given. |
| `TerraScan` and manually classified track points | Paper, Section IV-B, PDF page 6. | The paper identifies the annotation workflow. Related classified point-cloud copies or annotation projects may be stored separately from this tracing application. This is an inference, not an observed directory name. |

Suggested search terms, rather than filenames observed in code: `Vancouver`
combined with `classified`, `annotated`, `labelled`, `labeled`, `ground_truth`,
`rail head` or `track head`. Search both `Theles` and `Thales` spellings. For the
paper's accuracy assessment, the sought reference is manually classified
rail-head points; switch-type evaluation also requires its reference labels.

## Checked scope

- Current native C/C++ sources, GUI resources, project settings and relevant
  companion text files, including comments and file-selection/export handlers.
- `ADE code/VS2017/PCLProject.zip`: 938 file entries, including the older
  source variants. Every one of its 417 Git object-store files matches the
  bundled unpacked repository by CRC; it does not add a separate historical
  object set to search.
- The original nested Git repository: 39 reachable commits; 272 stored text
  blobs inspected, including 41 without a reachable filename mapping.
- Printable ASCII and UTF-16LE strings in six application EXE/PDB artifacts
  and four Visual Studio browse/cache databases. Large precompiled-header
  caches and unrelated dependency binaries were not exhaustively searched.
- Existing Vancouver sidecar metadata and the paper's annotation description.

Search evidence is local under `outputs/ground-truth-search/`, including
`scan_history.py`, `history_hits.json`, `binary_strings.json` and
`project_cache_strings.json`. No native source, archive or historical Git
repository was modified; no remote repository was contacted.

## Findings that avoid misleading search leads

**Runtime file selection.** The GUI selects LAS/LAZ/PTS inputs through file
dialogs. No ground-truth-specific initial directory or importer was found.
The LAS reader copies XYZ/intensity into the tracing cloud and does not load
the LAS classification field as reference annotations. See
[LASFileReader.cpp:18](<../ADE code/PCLProject/PCLProject/PCLProject/LASFileReader.cpp>)
and the input handlers in `MainForm.cpp`.

**`*_with_track_points.csv` is an output naming rule.**
`MainForm.cpp:157,180` constructs the name for saving computed center/rail
vectors; `TXTFileReader.cpp:191–283` writes those algorithm results. The suffix
is not evidence of manual annotations. The supplied
`ML_23_Line_2_with_track_points.csv` also has stored coordinate bounds different
from the Vancouver tiles; its relationship to the present scene is unverified.

**Validation names do not locate a reference dataset.**
`RailwayExtraction.cpp:987` contains only a commented
`ValidateTurnoutType(m_TrkStks)` call, with no implementation found in the
checked source. `ValidateLoopCross`, at line 2342, checks relative spacing of
detected objects without reading an annotation file.

**FLANN's `ground_truth.h` is unrelated to railway annotations.**
The hits in Visual Studio caches resolve to
`.../FLANN/include/flann/nn/ground_truth.h`. Its `find_nearest` and
`compute_ground_truth` functions compute exact nearest-neighbor indices for
generic search data; they do not identify or read this study's labels.

The absence of annotation/evaluation input paths is consistent with the paper's
manual preparation and evaluation occurring separately from the shared tracing
application. Confirm that interpretation with the provider. Request the
TerraScan-classified Vancouver rail-head points for paths 1–9 and the reference
types/region index for the additional 100 multi-track regions. Their actual
filenames, storage folders and schemas remain unknown. **LIM-012 remains open.**
