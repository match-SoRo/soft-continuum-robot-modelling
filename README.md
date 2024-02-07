## Soft continuum robot modelling

This repository provides functions to simulate the statics of a soft continuum robot (pneumatic) including mechanical contact 
Developed in MATLAB 2021b

### Usage

under /scripts:
run Startup_Cosserat_Toolbox
run simulate_and_visualize_contact_env

to speed up simulation uncomment generate_code(robot, objects, bc, optimizer, 'sim', 'mex') (line 81) in simulate_and_visualize_contact_env.m (only once) and use simulate_robot_mex instead of simulate robot (line 98/99).

### Citation 

M. Wiese, R. Berthold, M. Wangenheim and A. Raatz, "Describing and Analyzing Mechanical Contact for Continuum Robots Using a Shooting-Based Cosserat Rod Implementation," in IEEE Robotics and Automation Letters, vol. 9, no. 2, pp. 1668-1675, Feb. 2024, doi: 10.1109/LRA.2023.3346272 