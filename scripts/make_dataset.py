"""Write the synthetic step-response dataset used by the identification step.

The data are simulated from the "true" MotorParams with white sensor noise and
stand in for a lab measurement; the CSV header says so explicitly.

    python scripts/make_dataset.py
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from control import identification as ident  # noqa: E402
from control.artifacts import DATASET  # noqa: E402
from control.motor import MotorParams  # noqa: E402


def main() -> None:
    data = ident.make_synthetic_step(MotorParams())
    ident.save_step_csv(data, ROOT / DATASET)
    print(f"wrote {DATASET.as_posix()} ({len(data.t)} samples)")


if __name__ == "__main__":
    main()
