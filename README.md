# Model Predictive Control Project

---

### Project description and objectives
The goal of this project is to design an MPC-controller to control a vehicle in a virtual environment.
See https://github.com/udacity/CarND-MPC-Project for the original project repository.


### Pipeline of the project
To achieve the goal of the project, we adopt the following pipeline:

Step 1 - Modeling the vehicle as a bicycle model and analyze the input/output/state/objective function as well as constraint
Step 2 - Tuning parameters in the MPC. More specifically, the parameters include (i) the cost function, (ii) the prediction horizon.

Hereafter, we discuss the above two steps in more details.

The model under consideration is described in the following figure. The state consists of 6 entries: (i) (x,y) represent the location of the vehicle, (ii) phi denotes the facing angle of between the vehicle and the x-axis, (iii) v denotes the speed of vehicle (note that this is a scalar quantity), (iv) cte stands for cross track error, which can be expressed as the difference between the center of the road and current position of the vehicle, and (v) ephi is the orientation error. The evolution of the dynamics of discretized version of this model is shown in the following figure.

![alt_text](https://github.com/Ximingchen/Udacity-Model-Predictive-Control/blob/master/images/model.png)

The objective of this project is to follow a polynomial in the frame that captures the shape of the road, i.e., building a controller to track the middle lane. In our case, the control inputs are the steering angles delta as well as the throttle (i.e., 2-dimensional input).

In order to leverage the MPC framework, see https://en.wikipedia.org/wiki/Model_predictive_control for more details, one has to describe the objective function as well as the constraint.

In this project, we use the following objective function
![alt text](https://github.com/Ximingchen/Udacity-Model-Predictive-Control/blob/master/images/objective.png)

Essentially, we minimize the accumulated predicted cross track error and orienation error over the H horizons. Moreover, we penalize using large and constant steering/throttling. Finally, we ensure that the control gives us ``smooth'' trajectories by penalizing any adrupt change in steering angles as well as hard push of the throttle or gas paddle. 

![alt text](https://github.com/Ximingchen/Udacity-Model-Predictive-Control/blob/master/images/badperformance.png)




