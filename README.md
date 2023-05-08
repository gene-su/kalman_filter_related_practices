# kalman_filter_related_practices
## kalman_filter
state $x$:

$$
x=
\begin{bmatrix} 
p_x \\
p_y \\
v_x \\
v_y 
\end{bmatrix}
$$

state transition matrix $F$:

$$
F=
\begin{bmatrix}
1 & 0 & \Delta t & 0 \\
0 & 1 & 0 & \Delta t \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

process noise covariance matrix $Q$:

$$
Q=
\begin{bmatrix}
\frac{\Delta t^4}{4}\sigma^2_{a_x} & 0 & \frac{\Delta t^3}{2}\sigma^2_{a_x} & 0 \\
0 & \frac{\Delta t^4}{4}\sigma^2_{a_y} & 0 & \frac{\Delta t^3}{2}\sigma^2_{a_y} \\
\frac{\Delta t^3}{2}\sigma^2_{a_x} & 0 & \Delta t^2\sigma^2_{a_x} & 0 \\
0 & \frac{\Delta t^3}{2}\sigma^2_{a_y} & 0 & \Delta t^2\sigma^2_{a_y}
\end{bmatrix}
$$

measurement $y$:

$$
y=
\begin{bmatrix}
p_x \\ 
p_y 
\end{bmatrix}
$$

measurement matrix $H$:

$$
H=
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0
\end{bmatrix}
$$

measurement noise covariance matrix $R$:

$$
R=
\begin{bmatrix}
\sigma^2_{p_x} & 0 \\
0 & \sigma^2_{p_y}
\end{bmatrix}
$$

## unscented_kalman_filter
This is a project for the Sensor Fusion course on Udacity. Tracking cars using both lidar and radar measurements. The lidar measurements provide the car's pose in terms of x and y coordinates, while the radar measurements provide the car's range, bearing, and radial velocity.

![image](images/unscented_kalman_filter.gif)