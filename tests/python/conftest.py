"""Shared fixtures: one full pipeline run per test session (about 3 s)."""

from __future__ import annotations

import pytest

from control.motor import DriveParams, MotorParams
from control.pipeline import PipelineOutput, run


@pytest.fixture(scope="session")
def pipeline() -> PipelineOutput:
    return run()


@pytest.fixture(scope="session")
def summary(pipeline: PipelineOutput) -> dict:
    return pipeline.summary


@pytest.fixture(scope="session")
def true_plant() -> MotorParams:
    return MotorParams()


@pytest.fixture(scope="session")
def drive() -> DriveParams:
    return DriveParams()
