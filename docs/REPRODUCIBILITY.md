# Reproducibility record

This record describes the repository's intended Windows reproduction path. It
is based on the checked-in `PythonAPI/setup_carla37.ps1` script and does not
claim that the complete CARLA experiment suite has been re-run as part of this
documentation cleanup.

## Runtime contract

- Operating system: Windows 11 (with a compatible GPU and driver).
- Simulator: the bundled CARLA 0.9.14 Windows distribution.
- Simulation timing: synchronous control steps of 0.05 s (20 Hz).
- CARLA Python runtime: Conda environment `carla37`, using Python 3.7.
- Packaged CARLA wheel: `PythonAPI/carla/dist/carla-0.9.14-cp37-cp37m-win_amd64.whl`.

The wheel path is repository-relative and is installed by the setup script.
The CARLA runtime environment is intentionally separate from the Python 3.8+
development environment used for the project unit tests. The tests should not
be run under Python 3.7; they exercise project contracts without replacing
closed-loop CARLA execution.

## Pins recorded by the setup script

The script requests the following packages from conda-forge:

### Conda request

```text
python=3.7
openssl=1.1.1
libdeflate=1.14
libtiff=4.4
pip
numpy=1.21
scipy=1.7
matplotlib=3.5
pillow=9.2
networkx=2.6
shapely=1.8
future
distro
```

### Pip request

```text
pygame==2.6.1
```

After the conda operation, the script installs this Pygame pin with pip and
then installs the packaged CARLA wheel at
`PythonAPI/carla/dist/carla-0.9.14-cp37-cp37m-win_amd64.whl` with pip. The
setup script is the source of truth for this list. This project does not add a
`requirements.txt` or lock file in this cleanup.

## Commands

From the repository root, create or refresh the CARLA runtime environment:

```powershell
powershell -ExecutionPolicy Bypass -File .\PythonAPI\setup_carla37.ps1
```

Start the bundled server in one terminal:

```powershell
.\CarlaUE4.exe
```

With the server listening on its default `localhost:2000`, run a strict fixed-
speed comparison from a second terminal:

```powershell
.\PythonAPI\my_control_project\scripts\run_my_control.ps1 `
  --speed-planner-mode off `
  --speed-planner-limit-profile global `
  --planner-mode frenet `
  --planner-fallback error `
  --target-speed 70 `
  --route-shape true_straight `
  --error-provider ground_truth `
  --controllers pid lqr mpc
```

An adaptive-speed example is:

```powershell
.\PythonAPI\my_control_project\scripts\run_my_control.ps1 `
  --speed-planner-mode adaptive `
  --target-speed 90 `
  --route-shape s_curve `
  --map-name Town04_Opt `
  --seed 37321498 `
  --controllers pid lqr mpc
```

Run the project unit tests from the separate Python 3.8+ development
environment:

```powershell
python -m unittest discover -s .\PythonAPI\my_control_project\tests -v
```

Generated run evidence belongs under `PythonAPI/my_control_project/log/` and
is ignored by Git. No claim is made here that all dissertation-scale CARLA
runs or reported result tables have been reproduced by these commands alone.
