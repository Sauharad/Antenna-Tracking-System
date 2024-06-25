# Antenna-Tracking-System
Problem: To design an Antenna Tracking System using MATLAB code, given the equations of motion of the UAV with respect to time.

Solution Explanation: The length of the antenna, and its initial orientation is assumed to be along the x-axis. Hence, the inital coordinates of the antenna are [length;0;0]. 

The user is then asked to input the inital time final time and size of time intervals under which the system is to operate.

The equations of motion of the UAV are as follows:

x(t)=0.3*t^2
y(t)=3*t + 4
z(t)=0.05*t^3 + 7

From the above equations, the position matrices X,Y,Z of the UAV are constructed.

There are three user defined functions in the code, direction(), elevation(), and rotate().
The direction() function calculates the azimuth of the UAV at a given instant (spherical coordinates), given the coordinates. The standard formula for this is: phi = atan(y/x). Due to the properties of the inverse trigonometric functions, addition/ subtraction of pi is necessary depending upon the sign of x and y.
The elevation() function calculates the elevation of the UAV at a given instant, using the formula theta = atan(z/sqrt(x^2 + y^2). This value is then subtracted from pi/2 to obtain the value of elevation in the conventional sense, as elevation is calculated downwards from z axis in traditional spherical coordinates.

Now, after obtaining the azimuth and elevation of the UAV, our purpose is to rotate the antenna so that it points in the direction of the UAV.
The rotate() functions uses properties of Quarternions(4-Dimensional mathematical structures) to accomplish this task. It takes three inputs: the angle of rotation, the axis of rotation (given as a unit vector, ai + bj + ck , and the point which is to be rotated. To rotate a point about an arbitrary axis at an arbitrary angle, the normal 3D rotation matrices are not sufficient, and hence this concept has to be used. The function constructs a rotation matrice using four values: q0 = cos(theta/2),
q1 = a*sin(theta/2), q2=b*sin(theta/2), q3=c*sin(theta/2). From these four values, the required rotation matrix is constructed.

This function is first used to calculate the coordinates of the antenna tip after rotation by azimuth angle. The axis of rotation in this case is simply the z axis, or the unit vector k. The resultant of this rotation now has to be elevated. For this, the axis of rotation will be a unit vector which is perpendicular to the position vector of our previous resultant point. After accomplishing both of these rotations, we obtain the required coordinates of the tip of the antenna. 

Finally, the coordinates of the UAV, the azimuth and elevation of the antenna, and the corresponding coordinates of the antenna tip are displayed on the screen. Also,
two scatter plots have been created.The first one is of the position of UAV with time, and the second of is of the corresponding position of antenna, at the different instants of time.
