# AAE_Notebook_032_3dDrone

## 3d Control Architecture

![3d Control Architecture](/images/3d_control_architecture.png)

On the left, we have a trajectory -- z_t, z_dot_t, x_t, x_dot_t, y_t, y_dot_t, and psi_t. For now, let's assume that a trajectory contains target values for x, y, z, yaw angle, and the associated velocities. (It could, also, contain acceleration_target.)

This trajectory gets split into three parts: Altitude (z), Lateral Position (x and y), and yaw (psi). 

The z trajectory is handled by the altitude controller. The role of the altitude controller, here, is identical to what it was in the 2D case -- it sets the collective thrust. 

The x and y trajectory are first handled by the lateral position controller. This is almost identical to the y position controller from the 2D case. The only difference is that this controller outputs acceleration targets rather than angular targets; but, this was just a design decision so that we could keep all of the more complicated, angular control logic in the roll-pitch controller.

The yaw trajectory is handled by the yaw controller. 

### Path 1: Altitude

![Path One](/images/path1.png)

**Altitude Controller:** 2nd order system => PD Controller at least. I-term useful but can be safely ignored.

The altitude controller is responsible for ensuring that the vehicle stays close to the commanded z position and velocity by computing a target thrust value. The inputs to the altitude controller are everything related to the target and actual z (z_t, z_dot_t, z, and z_dot) plus the vehicle's estimated current attitude (phi, theta, and psi) so that it may adjust its thrust if it's tilted. The output to the altitude controller (u1) gets sent to the roll-pitch controller since the current commanded thrust is to be shared in the x, y, and z directions and the portion that points in the x and y direction will determine the acceleration in those directions. 

### Path 2: Lateral Position

![Path Two](/images/path2.png)

**Lateral Controller:** 2nd order system => PD Controller at least. I-term useful but can be safely ignored.

Controlling lateral position is the most involved control loop. The position controller is pretty straight-forward; it's just a PD controller on the x and y trajectories, similar to what we saw for y in the 2d case. It generates and acceleration command in the x and y directions (z_dot_dot_c and y_dot_dot_c) which is sent to the roll-pitch controller. (The same cascaded structure we saw in 2d.)

The roll-pitch controller is the most interesting of all the control blocks. Its job is to take a thrust command (u1) as well as the desired x and y accelerations (z_dot_dot_c and y_dot_dot_c), the attitude (phi, theta, psi) and pqr and output a target roll and pitch rate, p command (p_c), and q command (q_c). These commanded p and q values get sent to the body-rate commands. 

The body rate controller is just a P controller that converts p, q, and r commands into three rotational moment commands. (The r commands come from the yaw controller.)

### Path 3: Yaw

![Path Three](/images/path3.png)

**Yaw Controller:** 1st order system (like the body rate and roll-pitch controllers) => P Controller

As you can see, yaw is the least interesting of these loops. That's because it controls through the reactive moment command -- and that command only applies to yaw. So yaw can almost be treated in isolation. In practice, if yaw does not matter in particular application, we often just try to keep the rate to 0 and do not care the exact yaw angle. By doing this, we ensure that all available thrust and differential thrust is used for the translational motion.


***   ***   ***   ***   ***   ***   ***   ***   ***

Read sections 3 and 4 of the paper [Feed-Forward Parameter Identification](http://www.dynsyslab.org/wp-content/papercite-data/pdf/schoellig-acc12.pdf). As you read, pay attention to equations 2, 4, 5, and 6.

***   ***   ***   ***   ***   ***   ***   ***   ***


## Understanding Attitude Control Equations

If we lump together the 1st order systems -- roll-pitch controller, yaw controller, and body rate controller, based on inputs and outputs,  we note that they're controlling 2nd order systems. For example, yaw goes in as input and some moment (u4) is produced as output. Since that moment controls the 2nd derivative of yaw, that makes the whole thing a 2nd order system. 

Likewise, together, the roll-pitch and body-rate controllers make up a 2nd order system. The roll-pitch controller handles the first half of that system.

![Roll-Pitch Controller](/images/roll_pitch_controller.png)

The job of the roll-pitch controller is to take commanded x, y accelerations (x_dot_dot_c and y_dot_dot_c), a thrust (u1), and the vehicle's attitude (phi, theta, psi) and produce a roll-rate command (p_c) and pitch-rate command (q_c)

In equation 2 from the above paper, we start to see our 'control knobs' for x and y emerging. We can see that the x acceleration is the product of the total commanded thrust (c) multiplied by b^x -- wherein b^x is the element in the rotation matrix that maps x in the body frame to x in the world frame, R13. (b^y = R23)

How would we command the value of these matrix elements; how do we compute matrix elements into p and q commands? As seen in equation 6 from the above paper, we see how to do so...

![Equation Six](/images/equation_six.png)
