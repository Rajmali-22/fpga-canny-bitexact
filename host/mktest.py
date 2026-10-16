"""Shim: mktest.py moved to model/. Kept so host/ scripts and old commands still work."""
import os
import runpy
import sys

_MODEL = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "model")
if __name__ == "__main__":
    runpy.run_path(os.path.join(_MODEL, "mktest.py"), run_name="__main__")
else:
    sys.path.insert(0, _MODEL)
    del sys.modules[__name__]
    import mktest as _real  # noqa: E402  (resolves to model/mktest.py: host/ copy was just unloaded)
    sys.modules[__name__] = _real
