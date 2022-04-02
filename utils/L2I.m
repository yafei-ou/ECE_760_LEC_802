function I = L2I(L,m,r)
%LTOI Summary of this function goes here
%   Detailed explanation goes here
I = L - m*vec2so3(r).'*vec2so3(r);
end

