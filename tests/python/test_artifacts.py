"""Generated files (results.json, README tables, firmware header, golden
vectors) must be what the current code produces."""

import json
import re

import pytest

from control import artifacts
from control.pipeline import ROOT


def test_committed_artifacts_are_up_to_date(pipeline):
    stale = artifacts.stale(ROOT, pipeline)
    assert not stale, f"run 'python scripts/run_pipeline.py': stale {stale}"


def test_results_json_is_strict_json():
    def reject(constant):
        raise ValueError(f"non-standard JSON constant {constant}")

    summary = json.loads((ROOT / artifacts.RESULTS).read_text(encoding="utf-8"),
                         parse_constant=reject)
    assert "synthetic" in summary["note"]


def test_firmware_header_carries_the_designed_gains(pipeline):
    text = (ROOT / artifacts.HEADER).read_text(encoding="utf-8")
    defines = {m.group(1): float(m.group(2).rstrip("f"))
               for m in re.finditer(r"#define\s+(\w+)\s+([-+0-9.eE]+f?)", text)}
    cfg = pipeline.designs.servo
    expected = {"SPEED_KP": cfg.speed.kp, "SPEED_KI": cfg.speed.ki, "SPEED_TT": cfg.speed.tt,
                "POS_KP": cfg.pos.kp, "POS_KD": cfg.pos.kd, "POS_TF": cfg.pos.tf,
                "SSC_K0": cfg.ssc.k[0], "SSC_K1": cfg.ssc.k[1], "SSC_KI": cfg.ssc.ki,
                "SSC_L0": cfg.ssc.l[0], "SSC_L1": cfg.ssc.l[1], "CTRL_TS_S": cfg.ts,
                "CTRL_RAD_PER_COUNT": cfg.rad_per_count}
    for name, value in expected.items():
        assert defines[name] == pytest.approx(value, rel=1e-8), name


def test_figures_render_and_stay_small(pipeline, tmp_path):
    from control import plots

    paths = plots.write_all(pipeline, tmp_path)
    assert {p.name for p in paths} == set(plots.FIGURES)
    for p in paths:
        assert p.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"
        assert p.stat().st_size < 300 * 1024


def test_numerically_equal_tolerates_last_digit_noise_only():
    assert artifacts.numerically_equal("x = 1.00000, y = [2, 3]", "x = 1.000001, y = [2, 3]")
    assert not artifacts.numerically_equal("x = 1.0", "x = 1.01")
    assert not artifacts.numerically_equal("x = 1.0", "y = 1.0")
    assert artifacts.numerically_equal("u_min -inf", "u_min -inf")
