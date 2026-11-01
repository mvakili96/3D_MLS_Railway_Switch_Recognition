# Stack signatures and template recognition

This increment implements the source's 16-integer stack signature, all 20 active
templates, label assignment, `RecognitionTurnoutType`, and its tracking postpass.
It also exposes the inactive `TemplateMatching` helper separately, without enabling
that alternative in tracking. Toe detection/refinement is available in the
[toe increment](PYTHON_TOES.md). Branch seeds, switch boxes and
bidirectional orchestration remain later work. The current C++ remains authoritative.

## Run and use

```bash
.venv/bin/python -m railway_mls recognition-demo
.venv/bin/python -m railway_mls tracking-demo
.venv/bin/python -m unittest discover -s tests -v
```

`recognition-demo` constructs controlled stacks from all 20 source patterns. It
reports 16 unique exact matches and four ambiguities, with score 1. A one-bit
change to the type-0 pattern retains type 0 with score 0.9375. This is a component
example starting with side tags, not a claim to detect 20 regions from raw MLS.
The default tracking demo now runs the postpass at its natural stop; its single
rail scene has no completed turnout stacks to classify.

```python
from railway_mls import (
    BinaryString, define_templates, recognize_stacks, recognize_turnout,
)
from railway_mls.synthetic import signature_stack

catalogue = define_templates()
observed = BinaryString(bits=catalogue.all[0].bits)
match = recognize_turnout(observed)  # Mutates observed: type 0, score 1.
stacks = [signature_stack(catalogue.all[2].bits)]
batch = recognize_stacks(stacks)    # Mutates stack.binary: type 999, score 1.
```

`define_templates()` returns independent all/begin/end value copies through
`TemplateCatalogue`. Each call starts fresh. `recognize_turnout(observed,
previous=None, *, templates=None)` mutates the observed binary state and returns
a diagnostic snapshot. It reports template IDs, unary counts, context scores,
combined scores, all best indices, previous state and status. Omitting previous
uses the first-stack source context (`type=-1`, `position_label=1`).

`recognize_stacks(stacks, *, main_track=True)` mutates only eligible stack binary
state and returns one report per input stack. Each report preserves its index,
segment count, output snapshot, optional detailed match and status. It does not
touch segment geometry, side tags, point identities or labels.

## Source mapping

Paths are relative to `ADE code/PCLProject/PCLProject/PCLProject/`.

| Source | Python | Behavior |
| --- | --- | --- |
| `LIDARPT.h:1078–1154` | `models.BinaryString` | Constructor defaults, bit assignment and independent label assignment |
| `LIDARPT.h:1157–1167` | `TrackStack.binary` | Recognition state stored independently from stack geometry |
| `RailwayExtraction.cpp:2001–2112` | `define_templates` | All active literals, original IDs/order and begin/end copies |
| `RailwayExtraction.cpp:930–952` | `stack_signature` | First/second/penultimate/last side-tag rows |
| `RailwayExtraction.cpp:924–985` | `recognize_stacks` | Main-track and short-stack gates, immediate previous-stack context |
| `RailwayExtraction.cpp:2789–2870` | `recognize_turnout` | Unary weight 1, normalized scores, explicit 999 ambiguity and labels |
| `RailwayExtraction.cpp:2884–2929` | `template_matching` | Standalone inactive raw-count matcher, first tied ID and unchanged labels |

## Signature and gates

Only completed stacks with **more than four** segments are recognized. The four
sampled rows are indices `0`, `1`, `N-2`, `N-1`. Each row is ordered:

```text
track2.left_tag, track2.right_tag, track1.left_tag, track1.right_tag
```

Middle segments do not affect the signature. Main-window presence bits are not
added; the active signature contains 16 side-tag integers. Read stored tags,
not point-member counts. Values are not coerced to boolean: a nonbinary integer
retains the source's integer-equality behavior.

`BinaryString.set_bits` overwrites exactly 16 integers without resetting type,
score or labels. `set_labels(type_id)` updates only labels, without setting type.
Direct construction can represent the native vector's other lengths for standalone
helpers. A template longer than the observation is rejected at the Python boundary
because the source would read out of bounds; extra observed entries are ignored
by scoring and retained in output. The scoring denominator remains 16 even for a
custom shorter template. The batch always builds the original 16-entry signature.

A skipped short stack retains all previous binary state. Previous context is the
**immediate predecessor**, even when that stack was skipped; it is not the most
recently recognized stack. `main_track=False` leaves every stack unchanged.
The tracker does not automatically recognize its terminal active/unflushed stack.

## Catalogue and labels

The active patterns below retain source order. Spaces separate the four signature
rows. Commented-out alternative patterns remain inactive.

| Type | Pattern |
| --- | --- |
| 0 | `0100 0100 0101 0101` |
| 1 | `0100 0100 1010 1010` |
| 2 | `0010 0011 0011 0010` |
| 3 | `0100 1100 1100 0100` |
| 4 | `0010 0010 0101 0101` |
| 5 | `0010 0010 1010 1010` |
| 6 | `0101 0101 0100 0100` |
| 7 | `1010 1010 0100 0100` |
| 8 | `0010 0011 0011 0010` |
| 9 | `0100 1100 1100 0100` |
| 10 | `0101 0101 0010 0010` |
| 11 | `1010 1010 0010 0010` |
| 12 | `0110 0110 1010 1010` |
| 13 | `0110 0110 0101 0101` |
| 14 | `0101 0101 0110 0110` |
| 15 | `1010 1010 0110 0110` |
| 16 | `0010 1110 0010 0010` |
| 17 | `0101 0111 0100 0100` |
| 18 | `0100 0100 0111 0100` |
| 19 | `0010 0010 1110 0010` |

Begin IDs are `0,1,2,3,4,5,12,13,16,17`; end IDs are the remaining ten, in their
original order. `DefineTemplates` sets type and bits **without calling SetLables**.
Consequently all catalogue labels remain -1. Deriving catalogue labels from their
types would change context diagnostics and is not done in this port.

Result labels use the source mapping: position `0=start, 1=end`; direction
`0=left, 1=right`; turnout `0=merge, 1=split`; toe `0=open, 1=close, 2=both,
-1=other`. The source's else branches assign unknown/999 types the labels
`(end, right, split, other)`, rather than resetting all fields to -1. Ambiguity must
therefore be read from type/status, not inferred from those labels.

## Scoring, ties and the inactive alternative

Unary score counts **all equal integers**, including matching zeros, then divides
by 16. Context calculation awards one point when template position differs from
previous position. If previous position is start, matching direction and matching
merge/split each add one. The active weight is exactly **1** for unary, so context
has **zero influence**. No parameter enables a paper-aligned contextual variant.

The maximum starts at zero. If more than one template has that score, the active
method assigns **type 999** and preserves the maximum score. Otherwise it assigns
the sole winner's original type. It then applies the source label mapping.
Patterns 2/8 and 3/9 are identical: an exact observation of any of those four
templates returns 999 with score 1. Neither deduplication nor previous context
resolves the tie. An all-nonbinary observation can have score 0 for every source
template and still return 999; no recognition rejection threshold is introduced.
The later source toe code's separate score threshold is not applied here.

An explicitly empty custom catalogue resets type to -1, retains the incoming
score and updates labels through the unknown-type else branches. This is a defined
native path, not a fabricated match. Builtin recognition always uses 20 entries.

The separately callable `template_matching` preserves its distinct source rules:
raw match count rather than normalized score, first nonnegative tied type, and no
label update. Its `Type < 0` assignment guard can overwrite a negative winning
type with a later tied candidate; with all negative types it keeps the last one.
An empty catalogue retains score and labels while resetting type. This helper
is not used by tracking, and begin/end-only matching calls remain inactive.

## Tracking integration and verification

After a natural count, gauge, movement or no-support stop, the tracker runs the
recognition postpass once on its completed stacks. Main-track gating is preserved.
`TrackingResult.recognition` exposes the batch and returned completed stack
snapshots retain their binary metadata. A diagnostic step limit defers the postpass
until continuation reaches a natural stop. Undefined/nonfinite paths stop before
unsafe source operations and do not trigger automatic recognition. Empty-layer
invocations return immediately. The [toe increment](PYTHON_TOES.md) now follows
recognition with its own source gates and missing-reference diagnostics. Branch
work remains pending.

At completion of this increment, the suite passed **198 test methods**, including **64** comparing original
C++ extracts. Nine new native comparison methods check every catalogue field and
subset, all source label mappings, all 20 exact patterns and all 320 single-bit
flips, random/noisy/nonbinary inputs, context diagnostics, the raw helper, empty
catalogues, defined variable-length vectors, all twenty stack patterns and short
stack/main-track gates. Integer and score comparisons are exact for these routines.

The harness uses the actual original `BinaryString` and `TrackStack` definitions,
template literals, scoring functions and extracted signature/postpass code through
the point before toe detection. Diagnostic assignments only copy existing local
scores. Integration tests verify completion, deferred recognition, value snapshots,
and non-main gating. Original source files remain unchanged and the full Windows
application is not built.

Toe detection/refinement is now implemented in the [toe increment](PYTHON_TOES.md).
Next implement branch integration, followed by bidirectional orchestration and
remaining source features. Real-data accuracy,
performance and reproduction of the paper's evaluation remain unverified.
