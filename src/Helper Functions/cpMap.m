% cpMap: Returns the matrix packing of the cross product operator 
% (theta in radians)
%
% X = cpMap(w)
% Returns the matrix packing of the cross product operator,
% e.g., given vectors w, v; w X v = cross(w)*v = X*v.
%
% X = The skew symmetric matrix
%
% w = The vector to skew
%
% Noah Chapman
% 10883958
% MEGN544
% 10/11/2025
function X = cpMap(w)
X = [0, -w(3), w(2);
     w(3), 0, -w(1);
    -w(2), w(1), 0];
end