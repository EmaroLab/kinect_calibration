function [t q] = homegenousTransformToQuaternion( T )
    % recompute quaternion   
    R = T(1:3, 1:3);   % Extract rotation part of T
    t = T(1:3, 4);
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
    q = [q_inv(2) q_inv(3) q_inv(4) q_inv(1)]; % [x y z w]
    t; %[x y z]
end

