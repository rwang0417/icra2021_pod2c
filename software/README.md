# d2c_mujoco200
d2c implementation on MuJoCo 200 Windows x64 version

This is a sorted version as the ICRA2021 supplementary material. The full version is in this repo: https://github.com/rwang0417/d2c_mujoco200

--------
D2C stands for Decoupled Data-based Control. It's a model-free optimal control algorithm proposed to solve finite-horizon control problems for stochastic discrete systems.

The algorithm has three steps:
1. Open-loop training using the first order gradient descent method
2. System identification along the nominal trajectory using the least square method
3. LQR feedback gain calculation for each discretized step
   
The code for each of the above steps is in a separate file.

The D2C algorithm has advantages in the following aspects over reinforcement learning(compared with DDPG)
1. Training efficiency
2. Reproducibility
   
D2C also has comparable performance with DDPG in robustness.

The D2C algorithm has advantages in the following aspects over the model-based method(compared with analytical shape control)
1. Robustness
2. Energy efficiency
3. Model-free

The details of the D2C algorithm can be found in <https://arxiv.org/abs/1904.08361>.

## Setup

Operating system: Windows 64-bit

Software Platform: Visual Studio 2017, Matlab 2019a, Python 3

License Requirement: MuJoCo license from <https://www.roboti.us/license.html>.

For openloop, sysid2d, sysid3d and test, set up a Visual Studio project for each of them and generate the executable files.

For the Matlab wrapper, first set up the c compiler by running `mex setup` in Matlab. Then compile mexstep.c by `mex mexstep.c mujoco200.lib mujoco200nogl.lib`.


## Workflow

1. Write the MuJoCo model in the .xml file and put needed files in the workspace folder. Subfolders in `Tensegrity\data\` can be used as examples.
2. Write model-dependent parameters into funclib.cpp. Generate executable files: openloop.exe, sysid2d.exe and test2d.exe and put them into the workspace folder.
3. Open a command window in the workspace folder and run the D2C algorithm
   1. openloop: `openloop modelname.xml iteration_number [modeltype] [thread_number]` Modeltype is usually the same as modelname. If not, it needs to be specified for the code to identify the model.
   2. sysid2d: `sysid2d modelname.xml noise_level rollout_number [modeltype] [thread_number] [sysmode]` The standard deviation of perturbation is set as noise_level * Umax, where Umax is the maximum nominal control value from step 1. If sysmode is set to "top", the code will generate the linearized system at the top position (pendulum, cartpole) and the thread number will be set to 1. The output file is "lnr_top.txt" for top position and "lnr.txt" otherwise. Run toplqr.m in Matlab to get the stablizer geedback gain at the top.
   3. sysid3d: `sysid3d modelname.xml noise_level rollout_number [modeltype] [thread_number]` Same as sysid2d.
   4. LQR gain: Run tvlqr.m in Matlab. Make sure the model dependent parameters align with those in funclib.cpp.
4. Test the result by running `test modelname.xml [modeltype] [mode] [noise_level]` in the command window. The options for mode are listed here:
   - modeltest: simulate the model with no control input and display. Some model parameters will be printed on the command window.
   - policy_compare: generate data for the D2C open-loop policy and closed-loop policy comparison under different noise level. Also outputs the episodic energy data for the D2C Closed-loop policy.
   - performance_test: generate data of the distance from the target with the D2C closed-loop and open-loop policy applied under different noise level.
   - nfinal: print the positions for all nodes. Copy-paste the sequence EXCEPT the last 3 values to shape_control.m to run analytical shape control.
5. Run the model-based shape control algorithm(shape_control.m) in Matlab. Make sure the Matlab wrapper is re-compiled or the .mexw64 file is in the workspace folder.
6. Make plots by the functions in dataprocess.py using the data generated from the above steps.
