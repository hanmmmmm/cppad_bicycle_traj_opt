# cppad_bicycle_traj_opt
Using CppAD and bicycle vehicle kinematic model for trajectory optimization and obstacle avoidance




---- 

**GIF: Case 1: Reach Target Pose, without obstacles**

<a id="demogif1" href="https://github.com/hanmmmmm/cppad_bicycle_traj_opt/blob/main/media/traj_demo2_no_obs.gif">
    <img src="https://github.com/hanmmmmm/cppad_bicycle_traj_opt/blob/main/media/traj_demo2_no_obs.gif" alt="gif 1" title="case 1" width="600"/>
</a>

----

**GIF: Case 2: Reach Target Pose, with obstacles**

<a id="demogif2" href="https://github.com/hanmmmmm/cppad_bicycle_traj_opt/blob/main/media/traj_demo4_obs.gif">
    <img src="https://github.com/hanmmmmm/cppad_bicycle_traj_opt/blob/main/media/traj_demo4_obs.gif" alt="gif 2" title="case 2" width="600"/>
</a>

----

**GIF: Case 3: Reach Target Pose, with obstacles**

<a id="demogif4" href="https://github.com/hanmmmmm/cppad_bicycle_traj_opt/blob/main/media/traj_demo5_obs.gif">
    <img src="https://github.com/hanmmmmm/cppad_bicycle_traj_opt/blob/main/media/traj_demo5_obs.gif" alt="gif 3" title="case 3" width="600"/>
</a>

----

![](https://github.com/hanmmmmm/cppad_bicycle_traj_opt/blob/main/media/states_definitions.png)

![](https://github.com/hanmmmmm/cppad_bicycle_traj_opt/blob/main/media/object_function.png)

![](https://github.com/hanmmmmm/cppad_bicycle_traj_opt/blob/main/media/constraints.png)


**Cost function for obstacle distance**

x is the distance between a state pose and an obstacle; 
y is the cost.

![](https://github.com/hanmmmmm/cppad_bicycle_traj_opt/blob/main/media/Screenshot%20from%202024-09-10%2023-30-49.png)



