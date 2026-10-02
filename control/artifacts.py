"""Generated text artefacts and the staleness check used by CI.

Everything the repository derives from a pipeline run (results.json, the
README results block, the firmware parameter header, the golden vectors) is
built here as text. `stale()` compares those texts with the committed files,
number by number with a small tolerance, so a different BLAS or libm on the
CI runner cannot cause false alarms while any real change still does.
"""

from __future__ import annotations

import json
import math
import re
from pathlib import Path

from . import golden, identification as ident, report
from .motor import MotorParams
from .pipeline import ROOT, PipelineOutput, run

RESULTS = Path("docs/results.json")
HEADER = Path("firmware/include/controller_params.h")
GOLDEN_DIR = Path("tests/golden")
README = Path("README.md")
DATASET = Path("data/step_response_synthetic.csv")

_NUM = re.compile(r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?|[-+]?(?:inf|Infinity|nan|NaN)")


def dataset_text() -> str:
    return ident.step_csv_text(ident.make_synthetic_step(MotorParams()))


def build(out: PipelineOutput, root: Path = ROOT) -> dict[Path, str]:
    """Relative path -> text for every generated text artefact."""
    files = {
        RESULTS: json.dumps(out.summary, indent=2, allow_nan=False) + "\n",
        HEADER: report.firmware_header(out),
        README: report.replace_block((root / README).read_text(encoding="utf-8"),
                                     report.results_markdown(out.summary)),
    }
    for name, text in golden.generate(out).items():
        files[GOLDEN_DIR / name] = text
    return files


def _as_float(tok: str) -> float:
    return float(tok.replace("Infinity", "inf").replace("NaN", "nan"))


def numerically_equal(a: str, b: str, rtol: float = 1e-4, atol: float = 1e-6) -> bool:
    """Same text apart from numbers, and the numbers agree within tolerance."""
    if _NUM.sub("#", a) != _NUM.sub("#", b):
        return False
    for x, y in zip(_NUM.findall(a), _NUM.findall(b)):
        fx, fy = _as_float(x), _as_float(y)
        if math.isnan(fx) and math.isnan(fy):
            continue
        if not (fx == fy or abs(fx - fy) <= atol + rtol * max(abs(fx), abs(fy))):
            return False
    return True


def stale(root: Path = ROOT, out: PipelineOutput | None = None) -> list[str]:
    """Paths whose committed content differs from a fresh pipeline run.

    `out` may be passed in to reuse a run (it must come from root's dataset).
    """
    bad = []
    on_disk = (root / DATASET).read_text(encoding="utf-8")
    if not numerically_equal(dataset_text(), on_disk):
        bad.append(str(DATASET).replace("\\", "/"))
    if out is None:
        out = run(root / DATASET)
    for rel, text in build(out, root).items():
        path = root / rel
        if not path.exists() or not numerically_equal(text, path.read_text(encoding="utf-8")):
            bad.append(str(rel).replace("\\", "/"))
    return bad
