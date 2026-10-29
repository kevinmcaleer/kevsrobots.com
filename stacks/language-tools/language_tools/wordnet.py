"""Definitions and synonyms for Look Up and the thesaurus, from a local WordNet.

Reads the WordNet database files as they ship (index.noun, data.noun and so
on) without loading them: the index files are sorted, so a word is found by
binary search, and its senses are read straight from the data files by byte
offset. That keeps memory flat on a Raspberry Pi and needs no network.
"""

from __future__ import annotations

import mmap
import re
import threading
from pathlib import Path

POS = (("noun", "noun"), ("verb", "verb"), ("adj", "adjective"), ("adv", "adverb"))

# WordNet's "morphy" rules: how an inflected form maps back to its base form.
# Irregular forms (went, mice) need the exception lists, which this copy lacks.
DETACH = {
    "noun": [
        ("s", ""),
        ("ses", "s"),
        ("xes", "x"),
        ("zes", "z"),
        ("ches", "ch"),
        ("shes", "sh"),
        ("men", "man"),
        ("ies", "y"),
    ],
    "verb": [
        ("s", ""),
        ("ies", "y"),
        ("es", "e"),
        ("es", ""),
        ("ed", "e"),
        ("ed", ""),
        ("ing", "e"),
        ("ing", ""),
    ],
    "adj": [("er", ""), ("est", ""), ("er", "e"), ("est", "e"), ("ier", "y"), ("iest", "y")],
    "adv": [],
}

MAX_SENSES = 8
MAX_WORD = 64


def normalise(word: str) -> str:
    """The form WordNet indexes: lower case, underscores for spaces, plain apostrophes."""
    word = word.strip().lower().replace("’", "'")
    return re.sub(r"\s+", "_", word)


def base_forms(word: str, pos: str) -> list[str]:
    """The word itself, then each base form the morphy rules allow, in order."""
    forms = [word]
    for suffix, ending in DETACH[pos]:
        if word.endswith(suffix) and len(word) > len(suffix) + 1:
            stem = word[: len(word) - len(suffix)]
            candidates = [stem + ending]
            # running -> run, bigger -> big
            if not ending and len(stem) > 2 and stem[-1] == stem[-2] and stem[-1] not in "aeiouls":
                candidates.append(stem[:-1])
            forms.extend(c for c in candidates if c not in forms)
    return forms


class _File:
    """One WordNet file, memory-mapped on first use."""

    def __init__(self, path: Path):
        self.path = path
        self._map: mmap.mmap | None = None
        self._lock = threading.Lock()

    @property
    def map(self) -> mmap.mmap:
        if self._map is None:
            with self._lock:
                if self._map is None:
                    with self.path.open("rb") as fh:
                        self._map = mmap.mmap(fh.fileno(), 0, access=mmap.ACCESS_READ)
        return self._map

    def line_at(self, offset: int) -> bytes:
        data = self.map
        end = data.find(b"\n", offset)
        return data[offset : end if end >= 0 else len(data)]

    def find(self, key: str) -> bytes | None:
        """The index line for ``key``, by binary search over the sorted lines."""
        data = self.map
        target = key.encode("utf-8") + b" "
        lo, hi = 0, len(data)
        while lo < hi:
            mid = (lo + hi) // 2
            start = data.rfind(b"\n", 0, mid) + 1
            line = self.line_at(start)
            # The licence at the top of each file is indented; it sorts first.
            if line.startswith(b"  ") or line < target:
                lo = data.find(b"\n", mid)
                if lo < 0:
                    return None
                lo += 1
            else:
                hi = start
        if lo >= len(data):
            return None
        line = self.line_at(lo)
        return line if line.startswith(target) else None


class WordNet:
    def __init__(self, directory: Path):
        self.directory = directory
        self._index = {pos: _File(directory / f"index.{pos}") for pos, _ in POS}
        self._data = {pos: _File(directory / f"data.{pos}") for pos, _ in POS}

    @classmethod
    def open(cls, directory: Path | None) -> WordNet | None:
        """A reader for ``directory``, or None when it doesn't hold WordNet."""
        if directory is None:
            return None
        names = [f"{kind}.{pos}" for pos, _ in POS for kind in ("index", "data")]
        if not all((directory / name).is_file() for name in names):
            return None
        return cls(directory)

    def define(self, word: str) -> dict | None:
        """Every sense of ``word`` (or its base form), grouped by part of speech."""
        key = normalise(word)
        if not key or len(key) > MAX_WORD:
            return None
        entries = []
        lemma = None
        for pos, label in POS:
            for form in base_forms(key, pos):
                line = self._index[pos].find(form)
                if line is None:
                    continue
                senses = [self._sense(pos, offset, form) for offset in _offsets(line)[:MAX_SENSES]]
                entries.append({"pos": label, "lemma": form.replace("_", " "), "senses": senses})
                lemma = lemma or form
                break
        if not entries:
            return None
        return {"word": word.strip(), "lemma": (lemma or key).replace("_", " "), "entries": entries}

    def _sense(self, pos: str, offset: int, lemma: str) -> dict:
        line = self._data[pos].line_at(offset).decode("utf-8", "replace")
        head, _, gloss = line.partition(" | ")
        fields = head.split()
        count = int(fields[3], 16)
        words = [fields[4 + 2 * i] for i in range(count)]
        synonyms = []
        for w in words:
            # Adjectives can carry a position marker, e.g. "galore(ip)".
            w = re.sub(r"\([a-z]+\)$", "", w)
            if w.lower() != lemma and w.replace("_", " ") not in synonyms:
                synonyms.append(w.replace("_", " "))
        definition, examples = _split_gloss(gloss.strip())
        return {"definition": definition, "examples": examples, "synonyms": synonyms[:8]}


def _offsets(index_line: bytes) -> list[int]:
    fields = index_line.decode("utf-8", "replace").split()
    # lemma pos synset_cnt p_cnt [ptr_symbol...] sense_cnt tagsense_cnt offsets...
    synsets = int(fields[2])
    pointers = int(fields[3])
    return [int(o) for o in fields[4 + pointers + 2 : 4 + pointers + 2 + synsets]]


def _split_gloss(gloss: str) -> tuple[str, list[str]]:
    """'a domestic dog; "the dog barked"' -> ('a domestic dog', ['the dog barked'])."""
    first = gloss.find('"')
    if first < 0:
        return gloss.rstrip("; "), []
    definition = gloss[:first].rstrip("; ").strip()
    examples = [e.strip() for e in re.findall(r'"([^"]+)"', gloss[first:])]
    return definition, examples
