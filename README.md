### BVH Tree
Super-early release. No selection criteria, no cuda, no triangle intersection, nothing.

But GUESS WHAT? PYTHON BINDINGS ARE ALREADY HERE! See `tests/test.py`

![no image?](https://i.imgur.com/yh6rj9C.png)
![no image?](https://i.imgur.com/sVzMaJX.png)

## Requirements
* `sudo pacman -S assimp glm` or similar with other package managers
* `pip install -r requirements.txt`

## Installation

To install as python package, run
```
pip install .
```

To build and run as C++ program, run
```
make release
make run
```