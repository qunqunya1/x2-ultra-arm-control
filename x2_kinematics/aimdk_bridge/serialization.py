"""JSON serialization for panel diagnostics."""
import json
import math


def diagnostic_json(value):
    """Keep singularity diagnostics valid JSON; nonfinite metrics become null."""
    def finite(item):
        if isinstance(item, dict):
            return {key: finite(val) for key, val in item.items()}
        if isinstance(item, (list, tuple)):
            return [finite(val) for val in item]
        return None if isinstance(item, float) and not math.isfinite(item) else item
    return json.dumps(finite(value), ensure_ascii=False, allow_nan=False)
