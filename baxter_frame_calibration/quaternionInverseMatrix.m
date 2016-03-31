
% devine the traslation
t = [-0.045 0 0]'; % [x y z]
% define the quaternions
q = [0.5 -0.5 0.5 0.5]; % [x y z w];

% get the rotation matrix from q
q = [q(4) q(1) q(2) q(3)]; % [w x y z]
R = [ 1 - 2*q(3).^2 - 2*q(4).^2,  ...
    2*q(2)*q(3) - 2*q(1)*q(4), ...
    2*q(4)*q(2) + 2*q(1)*q(3); ...
    ...
    2*q(2)*q(3) + 2*q(1)*q(4), ...
    1 - 2*q(2).^2 - 2*q(4).^2, ...
    2*q(3)*q(4) - 2*q(1)*q(2); ...
    ...
    2*q(4)*q(2) - 2*q(1)*q(3), ...
    2*q(3)*q(4) + 2*q(1)*q(2), ...
    1 - 2*q(2).^2 - 2*q(3).^2 ];
% set rotaion matrix around x axis (-60°)
%rad = (2*pi*-30)/360;
%R = [1 0 0; 0 cos(rad) -sin(rad); 0 sin(rad) cos(rad)];

% show input value
q
T = [R, t; 0 0 0 1]


% compute the inverse
R1 = R';
t1 = -(R1 * t);
% omegeneous matrix
T_inv = [R1, t1; 0 0 0 1];



% recompute quaternion   
R = T_inv(1:3, 1:3);   % Extract rotation part of T
% Find rotation axis as the eigenvector having unit eigenvalue
% Solve (R-I)v = 0;
[v,d] = eig(R-eye(3));   
% The following code assumes the eigenvalues returned are not necessarily
% sorted by size. This may be overcautious on my part.
d = diag(abs(d));   % Extract eigenvalues
[s, ind] = sort(d); % Find index of smallest one
if d(ind(1)) > 0.001   % Hopefully it is close to 0
    warning('Rotation matrix is dubious');
end
axis = v(:,ind(1)); % Extract appropriate eigenvector
if abs(norm(axis) - 1) > .0001     % Debug
    warning('non unit rotation axis');
end
% Now determine the rotation angle
twocostheta = trace(R)-1;
twosinthetav = [R(3,2)-R(2,3), R(1,3)-R(3,1), R(2,1)-R(1,2)]';
twosintheta = axis'*twosinthetav;
theta = atan2(twosintheta, twocostheta);
q_inv = [cos(theta/2); axis*sin(theta/2)]; % [w x y z]

% show results
q_inv = [q_inv(2) q_inv(3) q_inv(4) q_inv(1)] % [x y z w]
T_inv