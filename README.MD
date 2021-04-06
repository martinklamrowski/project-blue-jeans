# PYROBO
[![forthebadge](https://forthebadge.com/images/badges/made-with-python.svg)](https://forthebadge.com)
[![forthebadge](https://forthebadge.com/images/badges/built-with-science.svg)](https://forthebadge.com)
[![forthebadge](https://forthebadge.com/images/badges/not-a-bug-a-feature.svg)](https://forthebadge.com)

![Python](https://img.shields.io/badge/python-v3.7+-blue.svg)
[![CoppeliaSim](https://img.shields.io/badge/CoppeliaSim-v4.1-red.svg)](https://www.coppeliarobotics.com/helpFiles/en/versionInfo.htm#coppeliaSim4.1.0)
## SYSC4805 Project Code - Team Blue Jeans

![an image has mysteriously gone missing, please notify the relevant authorities if you find it](https://github.com/SYSC4805-Winter-2021/project-blue-jeans/blob/master/misc/concept.png)

### Required

 - CoppeliaSim to open the scene.
    - coppelia_artifacts/maze.ttt

Note: Coppelia port set in scene is 8008.

### Package Dependencies

```bash
$ pip install msgpack
$ pip install colorama
$ pip install keyboard
$ pip install numpy
```

### Usage

```bash
$ git clone https://github.com/SYSC4805-Winter-2021/project-blue-jeans
```

```bash
$ python main.py -p <port> -d <maze_length> <maze_width>
```

Example to run simulation in 18x18 maze: <code>python main.py -p 8008 -d 18 18</code>

### Contributors

[@Angela](https://github.com/angiebyun),
[@Tarun](https://github.com/TarunAK),
[@Martin](https://github.com/martinklamrowski),
[@Matt](https://github.com/mwesleyjames)
