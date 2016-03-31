function [ T_inv] = invertRotationMatrix(T)
    R = T(1:3, 1:3);
    t = T(1:3, 4);
    
    R_inv = R';
    t_inv = - (R_inv * t);
    % omegeneous matrix
    T_inv = [R_inv, t_inv; 0 0 0 1];
end

