# Third-party CARLA boundary

CARLA 0.9.14 is third-party infrastructure bundled with this repository. The
CARLA server, maps, assets, physics, actors, sensors, and upstream Python API
are not author-original research code. The repository's custom evaluation
layer is under `PythonAPI/my_control_project/`; that location does not make
the bundled simulator or upstream components original.

## Bundled paths

The packaged distribution includes, among other components:

- `CarlaUE4.exe`
- `CarlaUE4/`
- `Engine/`
- `HDMaps/`
- `Co-Simulation/`
- upstream `PythonAPI/` content, except for the repository's custom
  `PythonAPI/my_control_project/` layer
- `Plugins/`
- root `CHANGELOG`, `Dockerfile`, and `Tools/`, which are retained as
  distribution/build/support material associated with the packaged CARLA tree
  where their individual provenance permits

The setup script and wheel under `PythonAPI/carla/` are repository assets used
by the current reproduction path; this note does not make a blanket originality
or licensing claim about every file in that tree. Components under `Plugins/`
must be checked against their own notices and licences.

Security scanners may flag the TestPilot public debug signing keys under
`Plugins/testpilot/`; these are upstream development materials retained with
that third-party plugin, not author or production secrets.

Upstream references are [CARLA on GitHub](https://github.com/carla-simulator/carla)
and the [CARLA project site](https://carla.org/).

The bundled distribution is retained for compatibility with the current
reproduction commands. Its presence does not represent the bundled CARLA
infrastructure as author-original work.

## License boundary

The root [`LICENSE`](../LICENSE) is the CVC/CARLA MIT license text associated
with the bundled CARLA provenance. It must not be read as automatically
licensing author research code under MIT. The independent licence for the
author-original research layer was not decided by this cleanup and remains
unresolved here.
