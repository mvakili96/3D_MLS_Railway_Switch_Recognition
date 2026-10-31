"""Explicit C-rand draws for reproducible legacy line fitting."""

from collections.abc import Iterable
from operator import index
from typing import Protocol


class RandSource(Protocol):
    def rand(self) -> int:
        """Return a nonnegative raw C-rand value, before modulo reduction."""


class MsvcRand:
    """Microsoft CRT integer generator, with explicit owned state (default seed 1).

    The original Windows application uses rand() without an explicit srand() call
    in the project source. Share one instance across successive fits/extractions;
    constructing another instance starts a new sequence, not the original process.
    """

    def __init__(self, seed: int = 1) -> None:
        self.state = index(seed) & 0xFFFFFFFF
        self.draw_count = 0

    def rand(self) -> int:
        self.state = (self.state * 214013 + 2531011) & 0xFFFFFFFF
        self.draw_count += 1
        return (self.state >> 16) & 0x7FFF


class RecordedRand:
    """Replay raw draws for cross-language comparisons and saved experiments."""

    def __init__(self, draws: Iterable[int]) -> None:
        self._draws = iter(draws)
        self.draw_count = 0

    def rand(self) -> int:
        try:
            value = index(next(self._draws))
        except StopIteration as error:
            raise RuntimeError("recorded random stream exhausted") from error
        if value < 0:
            raise ValueError("raw random draws must be nonnegative")
        self.draw_count += 1
        return value
