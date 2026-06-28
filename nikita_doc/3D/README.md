# CadQuery Hexagon with Holes

A parametric CAD model generator using CadQuery that creates a hexagon with an inner hexagon hole and four circular corner holes.

## Features

- **Outer hexagon**: 60mm diameter (circumscribed)
- **Inner hexagon**: 30mm diameter, rotated 30° for visual interest
- **Corner holes**: Four 3mm circular holes positioned at the corners of an imaginary 20mm square
- **Material thickness**: 1mm
- **Export formats**: DXF (for laser cutting) and STEP (for CAD software)

## Setup

1. Create and activate a virtual environment:
   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   ```

2. Install dependencies:

   **Recommended (single environment with viewer support):**
   ```bash
   python -m pip install --upgrade pip
   pip install "cadquery-ocp[vtk]" build123d ocp_vscode ezdxf
   ```

   Notes:
   - Installing `VTK` alone is not sufficient when `cadquery-ocp-novtk` is active.
   - If backend prints `VTK not installed`, remove `cadquery-ocp-novtk` and reinstall the command above.

   **Alternative for CadQuery-only usage:**
   ```bash
   python -m pip install --upgrade pip
   pip install cadquery ocp_vscode ezdxf
   ```

## Usage

Run either implementation to generate the model and export files:

**CadQuery implementation:**
```bash
python test.py
```

**build123d implementation:**
```bash
python test_build123d.py
```

This will create:

**CadQuery (test.py):**
- `hex_with_hole.dxf` - For laser cutters and 2D workflows
- `hex_with_hole.step` - For CAD software and 3D workflows

**build123d (test_build123d.py):**
- `hex_with_hole.step` - For CAD software and 3D workflows

## Visualization

When run inside VS Code with the OCP CAD Viewer extension, the model is automatically displayed in the viewer.

Important runtime notes:
- `python -m ocp_vscode --backend --port 3939` starts only the measurement backend.
- The VS Code OCP CAD Viewer frontend must already be running and connected on that port.
- If scripts print `Cannot access viewer config. Is the viewer running?`, open/start the OCP CAD Viewer panel in VS Code first.

Quick verification:
```bash
python -c "from OCP.IVtk import *; print('IVtk import ok')"
python test_build123d.py
```

Expected:
- The first command prints `IVtk import ok` (no `VTK not installed`).
- The second command exports files and opens the model in the VS Code viewer when the viewer panel is active.

## Implementations

This project includes two different CAD implementations:

### test.py (CadQuery)
Uses the [CadQuery](https://cadquery.readthedocs.io/) library, a Python API for automated CAD development. CadQuery uses a fluent interface for model building with method chaining.

### test_build123d.py (build123d)
Uses the [build123d](https://build123d.readthedocs.io/) library, a Python library for 3D modeling with a more modern, intuitive API. build123d uses context managers and a declarative approach for model building.

Both implementations produce identical output files (DXF and STEP).

## Parameters

Modify these constants in `test.py` to customize the design:

- `OUTER_DIAMETER`: Outer hexagon diameter (mm)
- `INNER_DIAMETER`: Inner hexagon diameter (mm)
- `THICKNESS`: Material thickness (mm)
- `ROTATION_DEGREES`: Inner hexagon rotation (degrees)
- `SQUARE_SIZE`: Reference square size for corner hole positions (mm)
- `HOLE_RADIUS`: Radius of corner holes (mm)
