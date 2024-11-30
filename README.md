The goal of this lab is to control a thruster-propelled Autonomous Underwater Vehicle (AUV).
The AUV should be able to follow a trajectory defined with a sequence of waypoints around an off-shore wind platform for inspection purposes, using an extended Kalman filter to estimate its attitude when ground truth is absent

![top](https://github.com/user-attachments/assets/9ea8ae56-0ee3-42d0-879d-2afdaa9773c2)

waypoint positioning around the wind platform

# Exercise 1

In the first part of the exercise, we used the ground truth pose of the robot obtained from the Gazebo simulator. Given a pre-defined list of waypoints, we developed the *trackWayPoint()* function to ensure that the robot cycles through them sequentially.

The function worked by:

1. Computing the robot's **current distance** (both linear and angular) from the current waypoint.
2. Checking if this distance was below a predefined threshold.
3. Switching to the next waypoint when the threshold conditions were satisfied.
4. Passing the updated waypoint to the control algorithm.
5. When the last waypoint has been reached, the AUV starts from the beginning

## Linear Distance Computation

The **linear distance** was straightforward to calculate as the Euclidean distance between the robot's current position  $(x_r, y_r)$ and the waypoint $(x_w, y_w)$

$$
d_{linear}=\sqrt{(x_w - x_r)^2 + (y_w - y_r)^2}
$$

## Angular Error via Quaternions

The **angular error**, however, was less intuitive, as the orientation of the robot was represented using quaternions, while the waypoint orientation was described only by the yaw angle.

 Quaternions are a four-dimensional representation of rotations and are not directly comparable using simple arithmetic.

We recall that a quaternion is represented as:

$$
q = \begin{bmatrix} w & x & y & z \end{bmatrix},
$$

where w is the scalar part, and  x y z represent the vector part.

To compute the angular difference between the robot's current orientation  $\theta$ and the target waypoint orientation , we followed these steps:

1. Find the current yaw angle of the robot with respect to the absolute reference frame from its corresponding quaternion:

$$
\theta= 2 \arctan2(z,w),
$$

2. At this point the angular error is just the difference between the target and the robot yaw:

$$
\theta_{error}=|\theta_r-\theta_t|
$$


# Exercise 2

In this exercise, we evaluated the performance of the AUV on the same task, but this time no ground truth was available, similarly to real-world scenarios. This meant using the available sensors to perform state estimation through a Kalman Filter. We did experiments using different sets of sensors, comparing the overall accuracy of position estimation.

To model the sensor we added their description to the *ekf.yaml* parameters file. Each sensor was modelled as a 15 boolean-list, each element could be set to true or false if the corresponding sensor had the capability to measure that quantity. In fact out of 15 booleans 6 were for the linear and angular positions, 6 for the velocities, and 3 for the linear accelerations.

## 1. Single IMU

Inertial measurement units are module comprising mainly three kinds of sensors: 

- Gyroscope: Measures angular velocity along three axes.
- Accelerometer: Measures linear acceleration.
- Magnetometer: Measures magnetic fields, enabling it to act as a compass under controlled conditions.

IMUs, as proprioceptive sensors, measure changes relative to the robot's body frame rather than the environment, which leads to drift over time and growing uncertainty in pose estimation.

This IMU was modelled inside the *ekf.yaml ***** by setting to true the values corresponding to the angular position from the magnetometer and velocities from the gyroscope and the linear accelerations from the accelerometer.
![imu1](https://github.com/user-attachments/assets/58cd0325-6cea-4a77-9b9c-b6906be4bf7a)
![imu11](https://github.com/user-attachments/assets/a2a6ed01-af3c-43f4-9f2f-2f694620ed41)


## 2. Dual IMUs

Adding a second IMU introduces redundancy, theoretically improving reliability by averaging or cross-verifying sensor data,  slowed the growth of uncertainty, but the Kalman filter still suffered from drift over time, as both IMUs are proprioceptive sensors, and their inherent bias and drift were not significantly mitigated.
![Screenshot_2024-11-15_11-55-08](https://github.com/user-attachments/assets/459a6593-e1b6-4120-ad75-2b68793f1a13)
![Screenshot_2024-11-15_11-55-32](https://github.com/user-attachments/assets/6f14ab90-5cf5-41bd-b617-38a100b24fbf)



## 3. Dual IMUs + Depth Sensor

The addition of a **depth sensor** drastically changed the behavior of the uncertainty. Unlike IMUs, a depth sensor is **exteroceptive**, providing measurements relative to an absolute reference frame (sea surface).
The depth sensor allows a precise vertical position reading, providing exteroceptive data that allows the Kalman filter to periodically correct the estimated pose, effectively reducing uncertainty and preventing unbounded drift along the z-axis of the AUV

The depth sensor was modelled in the parameters by setting to true the z component of the position.
![Screenshot_2024-11-15_11-58-21](https://github.com/user-attachments/assets/49e4ff71-8774-463e-afd8-5b9c14e2211f)
![Screenshot_2024-11-15_11-58-43](https://github.com/user-attachments/assets/baf6f293-e279-4191-92ad-a10b073a7f68)


## 4. Dual IMUs + Depth Sensor + USBL

Adding an **Ultra-Short Baseline (USBL)** acoustic positioning system further improved state estimation. The USBL provides absolute position estimates in the global reference frame (e.g., relative to a base station).

This was reflected in the *ekf.yaml* file, which is set to true for the position components.

The combination of IMUs, a depth sensor, and USBL significantly reduced uncertainty in both horizontal and vertical position estimates by providing periodic corrections to the robot’s global position, mitigating IMU drift and compensating for the depth sensor's one-dimensional measurements.

![f![full1](https://github.com/user-attachments/assets/517fa478-10ba-42ad-b64e-f2e62fbcf169)
![full2](https://github.com/user-attachments/assets/d1eeef9b-3eee-488f-b02e-1531a239c4d9)

## Comparison of Setups



| Configuration | Advantages  | Challenges | Uncertainty |
| --- | --- | --- | --- |
| Single IMU | Simple setup | Drift over time | High (grows rapidly) |
| Dual IMUs | Redundancy, slower drift | Still lacks exteroceptive data | High (grows slower) |
| Dual IMUs + Depth Sensor | Vertical constraint via absolute reference | Depth-only correction | Moderate (bounded on one axis) |
| Dual IMUs + Depth Sensor + USBL | Global position corrections | USBL requires line-of-sight | Low (minimal uncertainty) |

# Measurement noise

## **Measurement Noise**

Measurement noise plays a critical role in determining the performance of the Kalman filter. The noise characteristics of the sensors directly impact the accuracy and reliability of state estimation. Below is a breakdown of how measurement noise influences the results observed in the exercise:

### **Key Observations**

1. **IMU Noise**:
    - Gyroscope measurements are affected by **bias drift** and high-frequency noise.
    - Accelerometers are sensitive to **vibrations** and **gravitational alignment errors**.
2. **Depth Sensor Noise**:
    - Exhibits relatively low noise as it provides a direct measurement of depth, but environmental factors like water turbulence and sensor resolution may introduce some noise.
3. **USBL Noise**:
    - Noise in USBL data can arise from **multipath reflections**, **signal attenuation**, or **external disturbances** in underwater communication.
    - This noise typically has a significant impact on horizontal position estimation but is less critical for depth.
    
    We experiment by changing the values for measurement noise on the different sensors in the *noise.yaml* file
    
![noise1_4usbl](https://github.com/user-attachments/assets/f8c74029-69de-4f90-a404-08a5daac9f52)

The first experiment involved increasing measurement noise on the USBL readings to 4.0 m, the resulting uncertainty was greater but still not enough to affect the robot motion in a significant way.

![noise21_4usbl_1d_3rpy](https://github.com/user-attachments/assets/6b552af4-1cc7-4617-b063-d8365cfb2d7d)

For the second experiment, in addition to the noise on the USBL, we increased the noise on depth measures to 1m. The AUV could complete the task, but this time the motion was less smooth.

# Conclusion

The integration of proprioceptive (IMUs) and exteroceptive (depth and USBL) sensors dramatically improved the state estimation performance as opposed to methods relying only on dead-reckoning navigation. The presence of measurement noise in limited amounts didn’t affect in a significant way the performance of the system, thanks to the combination of sensors and the proven efficacy of the Kalman filter.

## Further improvement

The simulation environment used for this lab exercise didn’t present any external disturbance, such as waves or sea currents. In real-world scenarios, these phenomena would greatly affect the state estimation capability of the Kalman filter, if not addressed correctly.

In this exercise, we didn’t tune the Kalman filter, but correctly setting the measurement and state noise could make the difference in more complex scenarios.

DONDERO Enrico  
BUA ODETTI Emanuele  
