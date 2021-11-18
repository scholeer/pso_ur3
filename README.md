# PSO based point-to-point optimization for UR3 robotic arm

Main python script for trajectory optimization is in file [optimize_trajectory.py](https://github.com/scholeer/pso_ur3/optimize_trajectory.py)

Link to the [paper](https://www.mdpi.com/2076-3417/10/22/8241)

## Dependencies

- [pyswarm](https://pythonhosted.org/pyswarm/)
- [bezier](https://github.com/dhermes/bezier)
- [python-urx](https://github.com/SintefManufacturing/python-urx)

## NOTE: Issue with Instant currents information in python-urx

We had to modified the source code of [python-urx](https://github.com/SintefManufacturing/python-urx) library to obtain information
about instant currents.
