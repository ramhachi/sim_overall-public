# fsae-sim Python tools

Python tooling for the FSAE vehicle dynamics simulator.

## Requirements

- Python 3.10 or newer
- The C++ core built with CMake (see the repository `README.md`)

The package uses Python 3.10 syntax such as `X | None` and built-in generic
types, so Python 3.9 and older are not supported.

## Install

From the repository root:

```bash
cd python
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ".[dev]"
```

On Windows use `python -m venv .venv` and `.venv\Scripts\activate.bat`.

The repository setup scripts run the same editable install. They stop with a
clear error when the interpreter is older than Python 3.10.
