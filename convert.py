
"""Grid conversion and project maintenance helpers."""
from __future__ import annotations
import compileall
from pathlib import Path

def compile_grid(root: str | Path = ".") -> bool:
    """Compile all Grid Python modules to local __pycache__ bytecode."""
    return compileall.compile_dir(str(root), force=True, quiet=1)

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Compile Grid Python sources.")
    parser.add_argument("--root", default=".")
    args = parser.parse_args()
    raise SystemExit(0 if compile_grid(args.root) else 1)
