"""Shared current deployment values; no devices or heavy imports."""
import json
from pathlib import Path

PROFILE = Path(__file__).with_suffix('.json')


def load_profile():
    return json.loads(PROFILE.read_text())
