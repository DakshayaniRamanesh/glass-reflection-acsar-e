#!/usr/bin/env python3
import os
from pathlib import Path

search_roots = ["/mnt/d/go2_mapping", "/mnt/d/glass-research", "/mnt/d/projects"]
for r in search_roots:
    if not os.path.exists(r): continue
    for p in Path(r).rglob("*"):
        if p.is_file() and any(k in p.name.lower() for k in ["launch", "fast_lio", "fastlio", "hesai", "utlidar"]):
            print(p)
