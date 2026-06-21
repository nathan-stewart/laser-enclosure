#!/usr/bin/env python3
import json
import sys

from .core import run_halctl, HalCtlError


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]

    try:
        result = run_halctl(argv, source="cli")
    except HalCtlError as e:
        print(f"halctl: {e}", file=sys.stderr)
        return 2

    if result is not None:
        print(json.dumps(result, indent=2, sort_keys=True))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())