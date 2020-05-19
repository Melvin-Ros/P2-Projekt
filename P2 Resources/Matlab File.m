%File to verify kinematics
%% Calculating the matrices
t1=10*pi/180;
t2=10*pi/180;
t3=10*pi/180;
t4=10*pi/180;
t5=10*pi/180;
t6=10*pi/180;

T01=TDH(0,0,0,t1);
T12=TDH(-90*pi/180,150,0,t2-(90*pi/180));
T23=TDH(180*pi/180,360,0,t3+t2);
T34=TDH(-90*pi/180,100,-430,t4);
T45=TDH(90*pi/180,0,0,t5);
T56=TDH(-90*pi/180,0,0,t6);

T06 = T01*T12*T23*T34*T45*T56

%% Rotation Matrix:
%OBS: Must be in radians
rad = pi/180;

%Roll
    g = 168.780*rad; %%Gamma
%Pitch
    B = -67.019*rad; %Beta
%Yaw
    a = 150.383*rad; %%Alpha
    
Rx = rotx(168.780*rad);
Py = roty(-67.019*rad);
Yz = rotz(150.383*rad);

RotationMatrixXYZ = Rx*Py*Yz %%Euler
RotationMatrixZYX = Yz*Py*Rx %%Fixed
    
%RotMatrix = [cos(a)*cos(B) cos(a)*sin(B)*sin(g)-sin(a)*cos(g) cos(a)*sin(B)*cos(g)+sin(a)*sin(g);
%             sin(a)*cos(B) sin(a)*sin(B)*sin(g)+cos(a)*cos(g) sin(a)*sin(B)*cos(g)-cos(a)*sin(g);
%             -sin(B) cos(B)*sin(g) cos(B)*cos(g)]
    
    
%Rz = [cos(a) -sin(a) 0;
%      sin(a) cos(a)  0;
%      0       0      1];

%Ry = [cos(B) 0 sin(B);
%      0      1      0;
%      -sin(B) 0 cos(B)];
%  
%Rx = [1   0    0;
%      0 cos(g) -sin(g);
%      0 sin(g) cos(g)];
%  
%RotationMatrixZYX = Rz*Ry*Rx %%Fixed
%RotationMatrixXYZ = Rx*Ry*Rz %%Euler



%% Forward Kinematics:
%Values from the DH-parameters.
a2 = 150; a3 = 360; a4 = 100; d4 = -430;

%Creating the links:
L(1) = Link('alpha', 0, 'a', 0, 'd', 0, 'modified');
L(2) = Link('alpha', -pi/2, 'a', a2, 'd', 0, 'offset', -pi/2, 'modified');
L(3) = Link('alpha', pi, 'a', a3, 'd', 0, 'modified');
L(4) = Link('alpha', -pi/2, 'a', a4, 'd', d4,'modified');
L(5) = Link('alpha', pi/2, 'a', 0, 'd', 0, 'modified');
L(6) = Link('alpha', -pi/2, 'a', 0, 'd', 0, 'modified');

%Theta values:
theta1 = 10*pi/180;
theta2 = 10*pi/180;
theta3 = 10*pi/180+theta2;
theta4 = 10*pi/180;
theta5 = 10*pi/180;
theta6 = 10*pi/180;
q = [theta1 theta2 theta3 theta4 theta5 theta6];


%Using the Robotics Toolbox to make the robot from the dh parameters:
r = SerialLink(L, 'name', 'Fanuc')

r.teach(q, 'rpy')


%% Inverse Kinematics
%%%%Keep in mind, this does not work perfectly for the FANUC, as theta3 =
%%%%theta3+theta2, hence the odd number.
%Creating the links:
L(1) = Link('alpha', 0, 'a', 0, 'd', 0, 'modified');
L(2) = Link('alpha', -pi/2, 'a', a2, 'd', 0, 'offset', -pi/2, 'modified');
L(3) = Link('alpha', pi, 'a', a3, 'd', 0, 'modified');
L(4) = Link('alpha', -pi/2, 'a', a4, 'd', d4,'modified');
L(5) = Link('alpha', pi/2, 'a', 0, 'd', 0, 'modified');
L(6) = Link('alpha', -pi/2, 'a', 0, 'd', 0, 'modified');

%Using the Robotics Toolbox to make the robot from the dh parameters:
r = SerialLink(L, 'name', 'Fanuc')

%Insert transformation matrix for T06 in here.
T1 = [-0.3249    0.9297   -0.1738  586.4709
    0.8261    0.1895   -0.5306  103.4106
   -0.4604   -0.3159   -0.8296  639.8919
         0         0         0    1.0000];

%Using Robotics Toolbox to validate the inverse kinematics.
r.ikine(T1)*180/pi


%% Inverse Kinematics for FANUC, author Søren Bonnerup
%Theta values are determined:
t1=5*pi/180;
t2=30*pi/180;
t3=60*pi/180;
t4=-90*pi/180;
t5=-15*pi/180;
t6=50*pi/180;

T01=TDH(0,0,0,t1);
T12=TDH(-90*pi/180,150,0,t2-(90*pi/180));
T23=TDH(180*pi/180,360,0,t3+t2);
T34=TDH(-90*pi/180,100,-430,t4);
T45=TDH(90*pi/180,0,0,t5);
T56=TDH(-90*pi/180,0,0,t6);

T06 = T01*T12*T23*T34*T45*T56

%The following entries will be used for calculating theta angles:
x=T06(1,4);
y=T06(2,4);
z=T06(3,4);

%Functions used to calculate theta1, theta2 and theta3 using trigonometri:
t1=atan(y/x);
d1=sqrt(x^2+y^2)-150;
d2=sqrt(d1^2+z^2);
phi1=acos((d2^2+360^2-441.47^2)/(2*d2*360));
phi2=atan(z/d1);
t2=pi/2-phi1-phi2;
d3=z-cos(t2)*360;
t3=asin(d3/441.47)-13.0919*pi/180;

%Theta1, theta2 and theta3 converted and output in degrees:
theta1=t1*180/pi
theta2=t2*180/pi
theta3=t3*180/pi

%Denavit Hartenberg parameters for the fanuc robot arm:
T01=TDH(0,0,0,t1);
T12=TDH(-90*pi/180,150,0,t2-(90*pi/180));
T23=TDH(180*pi/180,360,0,t3+t2);

%The first three angles is inserted to calculate T03
T03=T01*T12*T23;

%In order to calculate theta4, theta5 and theta6, the T36 matrix is needed.
%T36 is found by taking the inverse of T03 times T06:
T36=inv(T03)*T06;

%The following entries are needed to calculate the last three angles:
a=T36(2,3);
b=T36(2,2);
c=T36(3,3);

%T36 from our Denavit Hartenberg parameters are set equal to the one
%calculated above, and the angles are isolated:
t5=acos(a);
t4=asin(c/sin(t5));
t6=asin(b/(-sin(t5)));

%theta4, theta5 and theta6 and converted to degrees:
theta4=t4*180/pi
theta5=t5*180/pi
theta6=t6*180/pi