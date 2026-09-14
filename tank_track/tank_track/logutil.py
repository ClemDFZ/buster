"""Minimal phase logger (vendored from pantilt_slave logging_utils)."""
from __future__ import annotations

import logging

LOG = logging.getLogger("tank_track")


def phase(phase_name: str, state: str, message: str) -> None:
    LOG.info("[%s] [%s] %s", phase_name, state, message)
