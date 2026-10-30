"""Shared model selection for FK/IK, dynamics and simulation."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT/"configs/active_robot_model.json"


def selected_urdf(*, collision=False):
    if not CONFIG.exists():
        return ROOT/"models"/("x2_ultra_simple_collision.urdf" if collision else "x2_ultra.urdf")
    selection = json.loads(CONFIG.read_text(encoding="utf-8"))
    key = "collision_urdf" if collision else "dynamics_urdf"
    if key not in selection:
        raise ValueError("This sharing copy contains a dynamics-only URDF; no collision model is bundled")
    path = (ROOT/selection[key]).resolve()
    if not path.is_file():
        raise FileNotFoundError("Selected robot model is missing: "+str(path))
    return path
