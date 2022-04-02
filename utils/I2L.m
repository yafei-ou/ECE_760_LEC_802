function L = I2L(I,m,r)
%I2L Summary of this function goes here
%   Detailed explanation goes here
L = I + m*vec2so3(r).'*vec2so3(r);
end

