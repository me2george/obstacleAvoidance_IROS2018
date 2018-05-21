function [collision, X_noColl, Y_noColl] = obs_check_collision(obs_list, X, Y)
% At the moment only implemented for 2D
d = 2; % Only immplemented for 2d at the moment

%[x_obs, x_obs_sf] = obs_draw_ellipsoid(obs_list,50);
points = [X(:),Y(:)]';
N_points = size(points, 2);

collision = zeros(1,N_points);

for it_obs = 1: size(obs_list,2) % Check collisions
    % \Gamma = \sum_{i=1}^d (xt_i/a_i)^(2p_i) = 1
    R = compute_R(d,obs_list{it_obs}.th_r);
    Gamma = sum( (R'*(points-repmat(obs_list{it_obs}.x0,1,N_points) )...
                  ./repmat(obs_list{it_obs}.sf* obs_list{it_obs}.a, 1, N_points) ).^(2*obs_list{it_obs}.p),1);
    
    collision = collision | Gamma<1;
end


collision = reshape(collision, size(X));

X_noColl = X.*not(collision);
Y_noColl = Y.*not(collision);

end

% 
% function R = compute_R(d,th_r)
% % rotating the query point into the obstacle frame of reference
% 
% if d == 2 
%     R = [cos(th_r(1)) -sin(th_r(1));sin(th_r(1)) cos(th_r(1))];
% elseif d == 3
%     R_x = [ 1, 0, 0; 0, cos(th_r(1)), sin(th_r(1)); 0, -sin(th_r(1)), cos(th_r(1))];
%     R_y = [cos(th_r(2)), 0, -sin(th_r(2)); 0, 1, 0; sin(th_r(2)), 0, cos(th_r(2))];
%     R_z = [cos(th_r(3)), sin(th_r(3)), 0; -sin(th_r(3)), cos(th_r(3)), 0; 0, 0, 1];
%     R = R_x*R_y*R_z;
% else %rotation is not yet supported for d > 3
%     R = eye(d);
% end
% 
% 
% 
% end

% for it_x = 1:size(X,1)
%     for it_y = 1:size(X,2)
%         x0 = [X(1,it_x);Y(it_y,1)];
%         
%         % Check wheter objects intersect -- ONLY 2D
%         for it_obs = 1:size(x_obs_sf,3)
%             %x_obs_sf(:,:,obs) =x_obs_sf_sing;
% 
%             N_obs = size(x_obs_sf,2);
%             %for jj = 1:size(x_obs_sf(:,:,obs),2) % triplets of data
%             %x_obs_sf(:,end,obs) = []; % remove last point, cause same as first
% 
%             allDistances = sum((x_obs_sf(:,:,it_obs)-repmat(x0,1,N_obs)).^2,1);    
% 
%             [~, i1] = min(allDistances);
% 
%             % Indexes for the closest triangle
%             i0 = mod(N_obs+i1-2,N_obs)+1;
%             i2 = mod(N_obs+i1,N_obs)+1;
% 
%             % Find direction of convexity
%             vec1 = x_obs_sf(:,i1,it_obs)-x_obs_sf(:,i0,it_obs);
%             while(norm(vec1) == 0) % twice the point in loop (last&first)
%                 i0 = i0 -1;
%                 vec1 = x_obs_sf(:,i1,it_obs)-x_obs_sf(:,i0,it_obs);
%             end
% 
%             vec1 = vec1/norm(vec1);
% 
%             perpDir1 = x_obs_sf(:,i2,it_obs)-x_obs_sf(:,i0,it_obs);
%             perpDir1 = perpDir1-(perpDir1'*vec1)*vec1;
%             perpDir1 = perpDir1/norm(perpDir1);
% 
%             % Include other class
%             testVec = x0 -x_obs_sf(:,i0,it_obs);
% 
%             if(testVec'*perpDir1 > 0) % if on the convex side           
%                 vec2 = x_obs_sf(:,i2,it_obs)-x_obs_sf(:,i1,it_obs);
%                 while(norm(vec2) == 0) % twice the point in loop (last&first)
%                     i2 = i2 +1;
%                     vec2 = x_obs_sf(:,i2,it_obs)-x_obs_sf(:,i1,it_obs);
%                 end
%                 vec2 = vec2/norm(vec2);
%                 perpDir2 =  x_obs_sf(:,i0,it_obs)-x_obs_sf(:,i1,it_obs);
%                 perpDir2 = perpDir2-(perpDir2'*vec2)*vec2;
%                 perpDir2 = perpDir2/norm(perpDir2);
% 
%                 testVec = x0 -x_obs_sf(:,i1,it_obs);
%                 if(testVec'*perpDir2 > 0) % is in the convex region
%                     collision(it_y,it_x)  = true;
%                     X_noColl(it_y,it_x) = x_obs_sf(1,i1,it_obs);
%                     Y_noColl(it_y,it_x) = x_obs_sf(2,i1,it_obs);
%                     break;
%                 end
%             end
% 
%             if false
%                 figure(10);
%                 plot([x_obs_sf(1,i0,obs),x_obs_sf(1,i1,obs)], ...
%                      [x_obs_sf(2,i0,obs),x_obs_sf(2,i1,obs)], ...    
%                       'r--')
%                 plot([x_obs_sf(1,i1,obs),x_obs_sf(1,i2,obs)], ...
%                      [x_obs_sf(2,i1,obs),x_obs_sf(2,i2,obs)], ...    
%                       'r--')
%                 plot(x0(1), x0(2), 'kx', 'Linewidth', 10);
% 
%                 plot([x_obs_sf(1,i0,obs)],[x_obs_sf(2,i0,obs)],'gx'); hold on;
%                 plot([x_obs_sf(1,i0,obs)+vec1(1),x_obs_sf(1,i0,obs)],...
%                      [x_obs_sf(2,i0,obs)+vec1(2),x_obs_sf(2,i0,obs)],'k--')
%                 plot([x_obs_sf(1,i0,obs)+perpDir1(1),x_obs_sf(1,i0,obs)],...
%                      [x_obs_sf(2,i0,obs)+perpDir1(2),x_obs_sf(2,i0,obs)],'kx--')
%                  plot([x_obs_sf(1,i0,obs)+testVec(1),x_obs_sf(1,i0,obs)],...
%                      [x_obs_sf(2,i0,obs)+testVec(2),x_obs_sf(2,i0,obs)],'r')
%                 axis equal;
%         %             xlim([-10,-4]); ylim([-3,3])
%                 pause()
%                 close(10);
%             end
%            
%         end
%     end
% end
% 
% end