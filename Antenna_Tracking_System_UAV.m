% Inputting initial position of tip of antenna
l=input('Enter length of antenna: ');
az=input('Enter initial azimuth of antenna (rad): ');
el=input('Enter initial elevation of antenna (rad): ');
a_x_i=l*sin(1.5707-el)*cos(az);
a_y_i=l*sin(1.5707-el)*sin(az);
a_z_i=l*cos(1.5707-el);
antenna_pos_initial=[a_x_i;a_y_i;a_z_i];

% Inputting required time interval
t1=input("Enter starting time: ");
t2=input("Enter final time: ");
tt=input("Enter no. of divisions in time interval: ");
t = linspace(t1,t2,tt);

% sample equations of position w.r.t time
x = @(t) (0.3*t.^2);
y = @(t) (3*t.^1 + 4);
z = @(t) (0.05*t.^3+7);

% position matrices of UAV
X = x(t);
Y = y(t);
Z = z(t);

spherical_coordinates=ones(2,tt);
for i=1:tt
    spherical_coordinates(1,i)=direction(X(i),Y(i),Z(i));
    spherical_coordinates(2,i)=elevation(X(i),Y(i),Z(i));
end

% Calculating the set of coordinates obtained after first
% rotation (in the xy-plane).

ant_coordinates_after_direction_rotation=ones(3,tt);
for i=1:tt
    a=rotate(spherical_coordinates(1,i),[0;0;1],antenna_pos_initial);
    ant_coordinates_after_direction_rotation(1,i)=a(1,1);
    ant_coordinates_after_direction_rotation(2,i)=a(2,1);
    ant_coordinates_after_direction_rotation(3,i)=a(3,1);
end

% To calculate position after second rotation, we have to
% rotate our point obtained after first rotation by the
% elevation angle. The axis of rotation in this case
% will be the unit vector perpendicular to the vector of 
% our point obtained after first rotation.

ant_coordinates_after_elevation_rotation=ones(3,tt);
for i=1:tt
    x1= ant_coordinates_after_direction_rotation(1,i);
    y1= ant_coordinates_after_direction_rotation(2,i);
    z1= ant_coordinates_after_direction_rotation(3,i);
    coordinate_i=[x1;y1;z1];
    mag=sqrt((x1)^2 + (y1)^2 + (z1)^2);
    rot_vector=[x1/mag;y1/mag;z1/mag];
    rot_vector=[[0,1,0];[-1,0,0];[0,0,0]]*rot_vector;
    a=rotate(spherical_coordinates(2,i),rot_vector,coordinate_i);
    ant_coordinates_after_elevation_rotation(1,i)=a(1,1);
    ant_coordinates_after_elevation_rotation(2,i)=a(2,1);
    ant_coordinates_after_elevation_rotation(3,i)=a(3,1);
end

final_antenna_coordinates=ant_coordinates_after_elevation_rotation;

% Text output (position of UAV and antenna)
disp('Position of UAV is :')
pos=ones(3,tt);
for i=1:tt
    pos(1,i)=X(i);
    pos(2,i)=Y(i);
    pos(3,i)=Z(i);
end
pos

disp('Azimuth of Antenna')
spherical_coordinates(1,:)
disp('Elevation of Antenna')
spherical_coordinates(2,:)

final_antenna_coordinates
% For plotting the graph
ant_x=ones(1,tt);
ant_y=ones(1,tt);
ant_z=ones(1,tt);
for i=1:tt
    ant_x(1,i)=final_antenna_coordinates(1,i);
    ant_y(1,i)=final_antenna_coordinates(2,i);
    ant_z(1,i)=final_antenna_coordinates(3,i);
end

ant_x;
ant_y;
ant_z;

subplot(2,1,1);
scatter3(X,Y,Z)
title('Position of UAV')


subplot(2,1,2);
scatter3(ant_x,ant_y,ant_z,'LineWidth',0.5)
title('Position of tip of antenna')
for i=1:tt
    a=num2str(t(i));
    s=strcat(a,'s');
    text(ant_x(1,i)+0.001,ant_y(1,i)-0.002,ant_z(1,i)-0.01,s)
end

% This rotate functions rotates a 
% given point by an angle theta, about an axis
% described by a unit vector (Most complicated part of this
% problem).

function r = rotate(theta,axis_vector,point)
    q0=cos(theta/2);
    q1=(axis_vector(1,1))*sin(theta/2);
    q2=(axis_vector(2,1))*sin(theta/2);
    q3=(axis_vector(3,1))*sin(theta/2);
    a11=q0^2 + q1^2 - q2^2 - q3^2;
    a12=2*(q1*q2 - q0*q3);
    a13=2*(q1*q3 + q0*q2);
    a21=2*(q2*q1 + q0*q3);
    a22=(q0^2 - q1^2 + q2^2 - q3^2);
    a23=2*(q2*q3 - q0*q1);
    a31=2*(q3*q1 - q0*q2);
    a32=2*(q3*q2 + q0*q1);
    a33=(q0^2 - q1^2 - q2^2 + q3^2);
    rotation_matrix = [[a11,a12,a13];[a21,a22,a23];[a31,a32,a33]];
    r=rotation_matrix * point;
end

% Below two functions are converting the coordinates of
% UAV from rectangular to spherical polar coordinates

function a = elevation(i,j,k)
    if (k>0)
        a=pi/2 - atan((sqrt(i.^2 + j.^2))/k);
    elseif (k==0)
        a=0;
    end
end

function b = direction(i,j,k)
    if (i>0)
        b=atan(j/i);
    elseif (i<0 && j>=0)
        b=atan(j/i) + pi;
    elseif (i<0 && j<0)
        b=atan(j/i) - pi;
    elseif (i==0 && y>0)
        b=pi/2;
    elseif (i==0 && y<0)
        b=-pi/2;
    end
end



