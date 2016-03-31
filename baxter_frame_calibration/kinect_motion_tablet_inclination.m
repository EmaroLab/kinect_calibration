
% compute the inclination of the baxter tablet w.r.t. to the z of the world 

% trasformation between world and head_camera (parent child)
tr = [0 0 0]';
qr = [0.549, 0.536, 0.448, 0.458];
Tr = quaternionToHomogenuesTrasnformation(tr,qr);

% compute y of head_camera in world
y = Tr * [0 1 0 0]'; 

% compute the angle of y w.r.t. z in the world
alpha = dot(y, [0 0 1 0]');
beta = norm(y)*norm([0 0 1]');
theta_rad = acos(alpha/beta);
theta_grad = (theta_rad * 180) / 3.14

% define kinect angle
tilt_rad = (-10*pi)/180;
tilt_real = tilt_rad - theta_rad;
T_tilt = [1 0 0 0; 0 cos(tilt_real) -sin(tilt_real) 0; 0 sin(tilt_real) cos(tilt_real) 0; 0 0 0 1];

% define the extimated rotation matrix from camerea link (Simetti method)
ts = [0.0539 0.0099 0.0436]';
qs = [-0.3516 -0.3173 -0.6416 0.6034];
Ts = quaternionToHomogenuesTrasnformation(ts,qs);

% define overall trasformation
Tt = T_tilt * Ts;
[t q] = homegenousTransformToQuaternion( Tt)


 
  
 