function mat = vec2so3(vec)
%VEC2SO3 Summary of this function goes here
%   Detailed explanation goes here
mat = [0,-vec(3),vec(2);
     vec(3),0,-vec(1);
     -vec(2),vec(1),0];
end

