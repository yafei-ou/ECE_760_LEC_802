function [angle, vectCross] = angleOfVectors(v1,v2)
vectCross = cross(v1,v2);
angle = atan2(norm(vectCross), dot(v1,v2));
% angle = atan(norm(vectCross)/dot(v1,v2));
end