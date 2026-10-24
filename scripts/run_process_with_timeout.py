#!/usr/bin/env python3
"""Run one command with a hard wall-clock timeout and forward its output."""

from __future__ import annotations

import argparse
import subprocess
import sys


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, required=True)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        parser.error("a command is required after --")
    try:
        completed = subprocess.run(command, capture_output=True, text=True, timeout=args.timeout)
    except subprocess.TimeoutExpired as error:
        if error.stdout:
            sys.stdout.write(error.stdout if isinstance(error.stdout, str) else error.stdout.decode())
        if error.stderr:
            sys.stderr.write(error.stderr if isinstance(error.stderr, str) else error.stderr.decode())
        return 124
    sys.stdout.write(completed.stdout)
    sys.stderr.write(completed.stderr)
    return completed.returncode


if __name__ == "__main__":
    raise SystemExit(main())
