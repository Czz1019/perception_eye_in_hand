"""Select the newest image with its exact-time transform available."""
from tf2_ros import TransformException


def newest_ready(entries, resolve):
    for entry in reversed(entries):
        try:
            result = resolve(entry)
        except TransformException:
            continue
        if result is not None:
            return entry, result
    return None, None
