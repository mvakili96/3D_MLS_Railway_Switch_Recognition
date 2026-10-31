"""Source stack signatures and template recognition, with context weight fixed at zero."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, replace

from .models import BinaryString, TrackStack


# DefineTemplates' active literals, in source order. Identical 2/8 and 3/9 entries
# are intentional; commented alternative literals are not enabled.
_TEMPLATE_ROWS = (
    ("0100", "0100", "0101", "0101"),
    ("0100", "0100", "1010", "1010"),
    ("0010", "0011", "0011", "0010"),
    ("0100", "1100", "1100", "0100"),
    ("0010", "0010", "0101", "0101"),
    ("0010", "0010", "1010", "1010"),
    ("0101", "0101", "0100", "0100"),
    ("1010", "1010", "0100", "0100"),
    ("0010", "0011", "0011", "0010"),
    ("0100", "1100", "1100", "0100"),
    ("0101", "0101", "0010", "0010"),
    ("1010", "1010", "0010", "0010"),
    ("0110", "0110", "1010", "1010"),
    ("0110", "0110", "0101", "0101"),
    ("0101", "0101", "0110", "0110"),
    ("1010", "1010", "0110", "0110"),
    ("0010", "1110", "0010", "0010"),
    ("0101", "0111", "0100", "0100"),
    ("0100", "0100", "0111", "0100"),
    ("0010", "0010", "1110", "0010"),
)
_BEGIN_TYPES = (0, 1, 2, 3, 4, 5, 12, 13, 16, 17)


@dataclass(frozen=True)
class TemplateCatalogue:
    all: tuple[BinaryString, ...]
    begin: tuple[BinaryString, ...]
    end: tuple[BinaryString, ...]


def define_templates() -> TemplateCatalogue:
    """Fresh value copies of all 20 templates and their begin/end subsets.

    DefineTemplates sets Type and bits, but never calls SetLables. Template labels
    remain -1; deriving them here would change the source context diagnostics.
    """
    templates = tuple(
        BinaryString(type=i, bits=tuple(int(bit) for row in rows for bit in row))
        for i, rows in enumerate(_TEMPLATE_ROWS)
    )
    return TemplateCatalogue(
        templates,
        tuple(replace(template) for template in templates if template.type in _BEGIN_TYPES),
        tuple(replace(template) for template in templates if template.type not in _BEGIN_TYPES),
    )


@dataclass(frozen=True)
class TemplateRecognition:
    binary: BinaryString
    template_types: tuple[int, ...]
    unary_scores: tuple[int, ...]
    context_scores: tuple[int, ...]
    scores: tuple[float, ...]
    best_indices: tuple[int, ...]
    previous: BinaryString | None
    status: str


def _unary(observed: BinaryString, templates: Sequence[BinaryString]) -> tuple[int, ...]:
    if any(len(template.bits) > len(observed.bits) for template in templates):
        raise ValueError(
            "an observation shorter than a template would cause a native out-of-bounds read"
        )
    return tuple(
        sum(value == observed.bits[i] for i, value in enumerate(template.bits))
        for template in templates
    )


def recognize_turnout(
    observed: BinaryString,
    previous: BinaryString | None = None,
    *,
    templates: Sequence[BinaryString] | None = None,
) -> TemplateRecognition:
    """RecognitionTurnoutType mutates observed; unary weight stays exactly one."""
    catalogue = tuple(templates) if templates is not None else define_templates().all
    prior = previous if previous is not None else BinaryString(position_label=1)
    unary = _unary(observed, catalogue)
    context = tuple(
        int(template.position_label != prior.position_label)
        + (
            int(template.direction_label == prior.direction_label)
            + int(template.turnout_label == prior.turnout_label)
            if prior.position_label == 0
            else 0
        )
        for template in catalogue
    )
    scores = tuple(count / 16 for count in unary)
    maximum = max((0.0, *scores))
    best = tuple(i for i, score in enumerate(scores) if score == maximum)
    observed.type = -1
    if len(best) > 1:
        observed.type, observed.score = 999, maximum
    elif best:
        observed.type, observed.score = catalogue[best[0]].type, scores[best[0]]
    # An empty catalogue resets Type but retains the incoming Score.
    observed.set_labels(observed.type)
    return TemplateRecognition(
        replace(observed),
        tuple(template.type for template in catalogue),
        unary,
        context,
        scores,
        best,
        replace(prior),
        "ambiguous" if len(best) > 1 else "matched" if best else "no_templates",
    )


def template_matching(
    observed: BinaryString,
    *,
    templates: Sequence[BinaryString] | None = None,
) -> TemplateRecognition:
    """The separately callable, inactive TemplateMatching helper.

    Raw match counts (0..16 for builtins), first nonnegative tied Type, and no
    label updates. This helper is not substituted for the active recognizer.
    """
    catalogue = tuple(templates) if templates is not None else define_templates().all
    unary = _unary(observed, catalogue)
    maximum = max((0, *unary))
    best = tuple(i for i, score in enumerate(unary) if score == maximum)
    observed.type = -1
    for i in best:
        if observed.type < 0:
            observed.type, observed.score = catalogue[i].type, float(unary[i])
    return TemplateRecognition(
        replace(observed),
        tuple(template.type for template in catalogue),
        unary,
        (),
        tuple(map(float, unary)),
        best,
        None,
        "matched" if best else "no_templates",
    )


def stack_signature(stack: TrackStack) -> tuple[int, ...]:
    """First two/last two rows, each track2 left/right then track1 left/right."""
    segments = stack.track_segments
    if len(segments) <= 4:
        raise ValueError("source recognition requires more than four stack segments")
    return tuple(
        tag
        for i in (0, 1, len(segments) - 2, len(segments) - 1)
        for tag in (
            segments[i].track2.left_tag,
            segments[i].track2.right_tag,
            segments[i].track1.left_tag,
            segments[i].track1.right_tag,
        )
    )


@dataclass(frozen=True)
class StackRecognition:
    index: int
    segment_count: int
    binary: BinaryString
    match: TemplateRecognition | None
    status: str


@dataclass(frozen=True)
class StackRecognitionBatch:
    entries: tuple[StackRecognition, ...]
    templates: TemplateCatalogue
    main_track: bool


def recognize_stacks(
    stacks: Sequence[TrackStack], *, main_track: bool = True
) -> StackRecognitionBatch:
    """Trackingrailway's recognition postpass, without toe/branch computation.

    Updates supplied stack binary state in place. A short stack retains ALL
    existing metadata. The previous stack is the immediate predecessor, even if
    it was skipped, rather than the most recently recognized stack.
    """
    catalogue = define_templates()
    entries = []
    for i, stack in enumerate(stacks):
        if not main_track or len(stack.track_segments) <= 4:
            entries.append(
                StackRecognition(
                    i,
                    len(stack.track_segments),
                    replace(stack.binary),
                    None,
                    "main_track_disabled" if not main_track else "too_short",
                )
            )
            continue
        stack.binary.set_bits(stack_signature(stack))
        previous = BinaryString(position_label=1) if i == 0 else replace(stacks[i - 1].binary)
        match = recognize_turnout(stack.binary, previous, templates=catalogue.all)
        entries.append(
            StackRecognition(
                i, len(stack.track_segments), replace(stack.binary), match, match.status
            )
        )
    return StackRecognitionBatch(tuple(entries), catalogue, main_track)
