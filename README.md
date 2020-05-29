# iolqr_mujoco200
iolqr implementation on MuJoCo 200 Windows x64 version

------
The algorithm takes input-output data to fit an arma model. The arma model is then used to constructed a linear time-varying system. A time-varying lqr is wrapped around the system to generate feedback control. 

The algorithm is coded in Matlab. MuJoCo is called via a Matlab wrapper to simulate the true system for data and result verification.

----

## Requirements
- Windows x64
- Matlab 2018 and later
- MuJoCo 200 Windows x64 version

## The Matlab wrapper
1. The wrapper is coded in file `mexstep.c`
2. Make sure the Matlab C/C++ compiler is properly installed by running `mex -setup` in Matlab.
3. Compile the wrapper running `mex mexstep.c mujoco200.lib mujoco200nogl.lib` in Matlab.
4. The `mexstep.mexw64` file generated can be used without re-compiling until further modification to `mexstep.c`.

## Algorithm implementation
- `iomatch_mujoco_tense.m` applies the algorithm to the tensegrity structures: dbar3d, t1d1_3d and t2d1_3d.
- `iomatch_mujoco.m` applies the algorithm to the classic robotic models: pendulum, cartpole, 3-link swimmer and 6-link swimmer.