"""Run identification -> design -> verification and write every derived file.

    python scripts/run_pipeline.py            # regenerate artefacts and plots
    python scripts/run_pipeline.py --check    # exit 1 if committed artefacts are stale

Written files:
    docs/results.json                    all numbers quoted in the README
    README.md (generated results block)  tables built from results.json
    firmware/include/controller_params.h gains for the ESP32 firmware
    tests/golden/*.csv                   reference vectors for the C unit tests
    docs/img/*.png                       figures
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from control import artifacts, plots  # noqa: E402
from control.pipeline import run  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true",
                    help="compare with the committed files instead of writing them")
    ap.add_argument("--no-plots", action="store_true", help="skip the figures")
    args = ap.parse_args()

    t0 = time.perf_counter()
    out = run(ROOT / artifacts.DATASET)
    print(f"pipeline finished in {time.perf_counter() - t0:.1f} s")

    if args.check:
        bad = artifacts.stale(ROOT, out)
        for path in bad:
            print(f"stale: {path}")
        print("artefacts up to date" if not bad else
              "run 'python scripts/run_pipeline.py' and commit the result")
        return 1 if bad else 0

    for rel, text in artifacts.build(out, ROOT).items():
        path = ROOT / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8", newline="\n")
        print(f"wrote {rel.as_posix()}")
    if not args.no_plots:
        for path in plots.write_all(out, ROOT / "docs" / "img"):
            print(f"wrote {path.relative_to(ROOT).as_posix()} ({path.stat().st_size // 1024} KB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
