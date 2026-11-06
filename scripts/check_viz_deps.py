#!/usr/bin/env python3
for mod in ["matplotlib", "open3d", "scipy", "PIL"]:
    try:
        m = __import__(mod)
        print(f"[OK] {mod}: {getattr(m, '__version__', 'installed')}")
    except ImportError:
        print(f"[MISSING] {mod}")
