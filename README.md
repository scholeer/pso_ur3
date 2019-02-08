# PSO based point-to-point optimization for UR3 robotic arm


Main python script for trajectory optimization is in file [optimize_trajectory.py](https://github.com/scholeer/pso_ur3/optimize_trajectory.py)


Link to the paper ...

## Dependencies

- [pyswarm](https://pythonhosted.org/pyswarm/)
- [bezier](https://github.com/dhermes/bezier)
- [python-urx](https://github.com/SintefManufacturing/python-urx)


## Issue with Instant currents information in [python-urx](https://github.com/SintefManufacturing/python-urx)

We modified source code of [python-urx](https://github.com/SintefManufacturing/python-urx) library to obtain information
about instant UR3 currents which is being sent in c structures from UR3 contoller.
Alternatively the voltage was aproximately constant ~46? V, so we can use it for energy/power computation.
