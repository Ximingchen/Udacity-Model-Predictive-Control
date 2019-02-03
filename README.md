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

The constraint of this model are: (i) the constraints imposed by the dynamical update equation in the first figure, and (ii) the limits of steering angle and throttle inputs. We set the steering angle to be within [-25, 25] degrees, and enforce that the throttle input to be within [-1, 1].

The horizon is set to be 10 and the discretization step is set to be 0.1 second. The main reason behind selecting these two parameter is two-fold. First, on one hand, we want to ensure that the discritized dynamical model captures the real continuous model well, subsequently, we should not use large discretization period. On the other hand, setting very small discritization period may lead to computation difficulties. Secondly, if we use 0.1 seconds as our period for sampling a discrete version of the system, using horizon 10 represents that we want to predict into the incoming second. Balancing the tradeoff between the need for fast computation and the prediction power, we decided that 10 is appropriate.

As described in the second figure, we associate weight with different cost. In our experiement, we use trail-and-error to obtain the weight. Intuitively, since our goal leans toward staying in the lane, we do not penalize how much power we use our control inputs. Therefore, large weights associating with the cte as well as orientation error are used.

### Other implementation details
* The reference lanes are calculated using third-order polynomial, which is powerful enough to describe the geometry of the lanes. To obtain the polynomial, we transform the global coordinates (x,y) into local coordinate in the view of the vehicle, that is, we let the moving direction of the car to be the x-axis. 

* Initially, our model performs badly despite a set of parameters are chosen carefully -- see the figure below. 
![alt text](https://github.com/Ximingchen/Udacity-Model-Predictive-Control/blob/master/images/badperformance.png)

After examining our code, we found that latency is a huge issue here. More specifically, let s0 be the initial state of the vehicle. The MPC gives us a sequence of control inputs u0, ..., uH assuming the vehicle is at s0.  However, when we try to implement the first control input returned given by MPC, a certain amount has passed, thus the current state is no longer at s0. Subsequently, implementing u0 leads to instability of the system. To overcome this trouble, given the current state s0, we first manually (using program) calculates the state s(L), where L is the amount of latency of the simulator. We then run MPC assuming we are at the initial state s(L). Here, we used the heuristic that the time to calculate the control inputs is approximately L. 

* The optimization problem is solved by IPOPT: https://projects.coin-or.org/Ipopt/.




