# How to compile the MatLab wrapper for MuJoCo
The MuJoCo software and license can be obtained from http://www.mujoco.org/index.html.

The wrapper is needed when calling MuJoCo from MatLab.

## Ingredients
- Windows x64
- Matlab 2018 and later
- MuJoCo 200 Windows x64 version

## Path
- The MuJoCo license (key) file is in the folder `./keys`.
- The wrapper source file is `mexstep.c`.
- The MuJoCo .dll files are in the parent folder `workspace` to be used by MatLab.
- The rest MuJoCo files are in the `mujoco` folder.

## Compile the MatLab wrapper
1. Change the path and filename of the MuJoCo license file in `mexstep.c` and make sure they match your own license file.
2. Make sure the MatLab C/C++ compiler is properly installed by running `mex -setup` in MatLab command window.
3. Make sure the MatLab current path is the parent folder `workspace`.
4. Compile the wrapper by running `mex ./mujoco/mexstep.c ./mujoco/mujoco200.lib ./mujoco/mujoco200nogl.lib` in MatLab command window.
5. The generated `mexstep.mexw64` file will be used by MatLab. There is no need to re-compile it unless changes are made to `mexstep.c`.