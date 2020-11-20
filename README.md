# Bubble ball code Description
This is a code implementation of our hierarchical algorithm in **Learning How to Solve “Bubble Ball”**. 
[Website](https://sites.google.com/berkeley.edu/bubble-ball/home),

## Tips
* To use 2 python scripts for public, you can go to the 'BubbleBall Challenge' repository.
* This is for actual implementation to play this game.
* To test my code, you should have bubble-ball execution file of Linux version from Nay Games.
* It includes a python package named `functions` and `main.py` in the `bubble-ball-ubuntu-18.04-x86` directory.

## Prerequisite
* Required to have **automate.json, movableObjects.json, bb.log, movableObjects.json, levels directory** in the same folder as this code. (Usually, the directory name is `bubble-ball-ubuntu-18.04-x86-64-x.x` (I cannot upload this here.))
* You should have a bubble-ball game execution file in the same folder.

## How to use 
* You can decide on the level what you want in the `main.py`.
1. Locate the `functions/` and `main.py` in the `bubble-ball-ubuntu-18.04-x86` directory.
2. `main.py` can import any module from `functions` package. 
3. If you wanna check each function module such as local_region algorithm, you can run it separately.

## Structure
.
+-- main.py
+-- functions
|   +-- localregion.py
|   +-- log2matrix.py
|   +-- parsing_movableobjects_levels.py
|   +-- path_planning.py
|   +-- utils.py
|   +-- physics
|   |	+-- obj_class.py
|   |	+-- simple_models.py
|   |	+-- common
|   |	|   +-- const.py
|   +-- planning_algo
|   |	+-- prm.py
|   |	+-- utils.py



