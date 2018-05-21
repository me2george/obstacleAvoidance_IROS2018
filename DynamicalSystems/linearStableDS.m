function xd = linearStableDS(x,x0)
% A = - I ; 
if nargin<2
    x0 = [0;0];
end

dim = size(x,1);

%xd = -(x-x0);
xd = x0-x;

end

