function [ obs ] = calculate_dynamic_center( obs, x_obs_sf, intersection_obs )
% Calculate the dynamic center for the applicaiton of the modulation
% matrix. A smooth transition is desired between objects far away (center
% in middle) and to convex obstacles intersecting. 
%
% This might not always be the completely optimal position of the
% dynamic_center of the obstacles, but it results in a continous movement
% of it and keeps computational cost low
%
% TODO --- 
% Increase sampling, when less thant 1/9 of original points are still 
% available; decrease inital number of sampling points to 27 initially
% do 3x -> sampling accuracy of 3^3*3^2*3^2 = 3^7 = 2187

% only implemented for 2d
dim = size(obs{1}.x0,1);

if(dim>2)
    warning('Not implemented for higher order than d=2! \n')
end

if nargin<10
    % Margin where it starts to influence the dynamic center
    marg_dynCenter = 1.3;
end

intersection_obs_temp=[];
for ii=1:length(intersection_obs)
    intersection_obs_temp=[intersection_obs_temp,intersection_obs{ii}];
end
intersection_obs = unique(intersection_obs_temp);

N_obs = size(obs,2);
if ~N_obs; return; end % No obstacle not intersectin

% Resolution of outside plot
% MAYBE - Change to user fixed size -- replot first oneh
[~, x_obs_sf] = obs_draw_ellipsoid(obs,16); % Resolution # 16
N_resol0 = size(x_obs_sf,2); 
N_resol = N_resol0;

% Number of incrementation steps
N_distStep = 3;

% Maximal surface resolution
resol_max = 3^10;

% Center normalized vector to surface points
rotMatrices = zeros(dim,dim,N_obs);

for ii = 1:N_obs
    x_obs_sf(:,:,ii) = x_obs_sf(:,:,ii)-obs{ii}.x0;
    rotMatrices(:,:,ii) = compute_R(dim,obs{ii}.th_r);
end

% Calculate distance between obstacles
weight_obs_temp = zeros(N_obs,N_obs); 

%x_middle = zeros(dim, N_obs, N_obs);
x_cenDyn_temp = zeros(dim, N_obs, N_obs); 

% close all;
% figure(12)%, 'Position', [300,600,400,400]);
% clf;
% x_obs = drawEllipse_bound(obs{1});
% plot(x_obs(1,:), x_obs(2,:), 'k'); 
% hold on; axis equal;
% x_obs = drawEllipse_bound(obs{2});
% plot(x_obs(1,:), x_obs(2,:), 'k'); 

for ii = 1:N_obs % Center around this obstacle
    rotMat = rotMatrices(:,:,ii); % Rotation Matrix
    
    % 2nd loop over all obstacles, but itself
    for jj = 1+ii:N_obs % only iterate over half the obstacles
        if and(ismember(jj, intersection_obs), ismember(ii,intersection_obs) )
            continue; % Don't reasign dynamic center if intersection exists
        end
        
        % Forr ellipse:
        %ref_dist = marg_dynCenter*0.5*sqrt(0.25*(sqrt(sum(obs{ii}.a)^2+max(obs{jj}.a)^2));
        dist_contact = 0.5*(sqrt(sum(sum(obs{ii}.a.^2))) + ...
                                      sqrt(sum(sum(obs{jj}.a.^2))));
        ref_dist = dist_contact*marg_dynCenter;
        
        %ref_dist(ii,jj) = ref_dist(jj,ii); % symmetric

        % Inside consideration region
        % TODO - second power. Does another one work to?! it should...
        ind = sum( (x_obs_sf(:,:,jj)+obs{jj}.x0-obs{ii}.x0).^2, 1) < ...
                                                            ref_dist^2;
        if ~sum(ind)
            continue; % Obstacle too far away
%        else
%             fprintf('entering looop --- TODO remove \n')
        end
        
        % Set increment step
        step_dist = (ref_dist)/(N_distStep);
        dist_start = 0;
        
        resol = size(x_obs_sf,2); % usually first point is double..
        
        thetaRange = []; % Start and endpoint of ellipse arc
        a_range_old = []; % Radius with which the ellipse arc was drawn
        
        itCount = 0; % Iteration counter
        itMax = 100; % Maximum number of iterations
        
        % Tries to find the distance to ellipse
        while(resol < resol_max)  % Continue while small resolution
            for it_gamma = 1:N_distStep

                delta_dist = step_dist*it_gamma+dist_start; % Renew distance
                
                [ind_intersec, n_intersec, x_obs_temp, a_range_old] ...
                        = check_for_intersection(obs{jj}, obs{ii}, delta_dist, N_resol, thetaRange, a_range_old, rotMat);
                
%                 plot(x_obs_temp(1,:), x_obs_temp(2,:), '--')
%                 plot(x_obs_temp(1,ind_intersec), x_obs_temp(2,ind_intersec), 'ko')
                % Increment iteratoin counter
                itCount = itCount + 1;

                

                if n_intersec  % Intersection found
                    dist_start = (delta_dist-step_dist); % Reset start position
                    step_dist = step_dist/(N_distStep); % Increase distance resolution
                    
                    % Increase resolution of outline - plot
                    if n_intersec < length(ind_intersec)-2
                        % all intersection in middle -  [ 0 0 1 1 1 0 ] 
                        % extrema 1 - [ 0 0 0 1 1 1 ]
                        % extrema 2 - [ 1 1 1 0 0 0 ]
                        indLow = find(ind_intersec,1, 'first');
                        indHigh = find(ind_intersec,1,'last');
                        
                        if resol > N_resol0 % only arc of convex obstacle is observed
                            indLow = max(indLow -1,1); 
                            indHigh= min(indHigh +1, N_resol);
                        else

                            if and(indHigh == length(ind_intersec), indLow == 1)
                                % split at border - [ 1 1 0 0 0 1 ] -- only
                                % relevant when analysing original convex
                                % obstacle
                                indHigh = find(~ind_intersec,1, 'first') - 1;
%                                 if indHigh == 0;  indHigh = N_resol; end

                                indLow = find(~ind_intersec,1,'last')  + 1;
%                                 if indLow > N_resol; indLow = 1;  end
                            end

                            % Increse resolution of obstacle
                            n_intersec = n_intersec+2; % Add one point to left, one to the right

                            %indLow = find(intersection_ind_temp,1)-1;
                            indLow = indLow -1;
                            if indLow == 0;  indLow = N_resol; end

                            %indHigh = find(intersection_ind_temp,1,'last')+1;
                            indHigh = indHigh + 1;
                            if indHigh > N_resol; indHigh = 1;  end
                        end
                        
                        xRange =  [x_obs_temp(:,indLow),x_obs_temp(:,indHigh)];
                        % TODO - remove after debugging
%                         plot( xRange(1,:), xRange(2,:),'g --');

                    else % too few intersections
                        xRange = [x_obs_temp(:,1), x_obs_temp(:,end)]; % keep same range
                    end
                    x_start = rotMatrices(:,:,jj)'*(xRange(:,1) - obs{jj}.x0);
                    x_end = rotMatrices(:,:,jj)'*(xRange(:,2) - obs{jj}.x0);

                    x_Arc = zeros(size(x_obs_temp));
                    for kk =1:size(x_obs_temp,2)
                        x_Arc(:,kk) = rotMatrices(:,:,jj)'*(x_obs_temp(:,kk) - obs{jj}.x0);
                    end
%                     plot(x_Arc(1,:), x_Arc(2,:),'k')
%                     plot(x_start(1),x_start(2), 'ro')
%                     plot(x_end(1),x_end(2), 'ro')

                    % TODO remove these
                    if max([abs(x_end(1)),abs(x_start(1))]) > abs(a_range_old(1,:))
                        warning('Numeric apprximation of intersection finder could have a complex angle values.')
%                         xStart = x_start(1)
%                         xEnd = x_end(1)
%                         aOld = a_range_old(1)

                        thetaRange = [sign(x_start(2))*acos(min(1,max(-1, (x_start(1)/a_range_old(1))))), ...
                                      sign(x_end(2))*acos(min(1,max(-1, (x_end(1)/a_range_old(1))))) ]
                        if sum(imag(thetaRange))
                            warning('Stayed complex!');
                        end
            
                    else
                        thetaRange = [sign(x_start(2))*acos(x_start(1)/a_range_old(1,:)), ...
                                        sign(x_end(2))*acos(x_end(1)/a_range_old(1,:))];
                    end

                    % Resolution of the surface mapping - 2D
                    resol = resol/(n_intersec-1)*N_resol;
                    
                    [~, n_intersec, x_obs_temp, a_range_old] ...
                          = check_for_intersection(obs{jj}, obs{ii}, dist_start, N_resol, thetaRange, a_range_old, rotMat);
                    
%                     plot(x_obs_temp(1,:), x_obs_temp(2,:),'--')
                    while n_intersec > 0
                        % The increasing resolution caused new points,
                        % lower value of dist_0 is not bounding anymore
                        dist_start = dist_start - step_dist;
                        
                        [~, n_intersec, x_obs_temp, a_range_old] ...
                          = check_for_intersection(obs{jj}, obs{ii}, dist_start, N_resol, thetaRange, a_range_old, rotMat);
                    end
                    break; % End current loop
                end
            end

            if itCount > itMax % Resolution max not reached in time
                warning('No close intersection found ...\n');
                break; % Emergency exiting -- in case of slow convergence
            end
            if delta_dist >= ref_dist
                break;
            end
        end
        
        % Negative step
        if(delta_dist == 0)
            % Limit intersection: weight is only assigned to one obstacle
            weight_obs_temp(ii,jj) = -1; 
        else
            weight_obs_temp(ii,jj) = max(1/delta_dist -1/(ref_dist-dist_contact),0); % if too far away/
        end
        weight_obs_temp(jj,ii) = weight_obs_temp(ii,jj);
        
        % Position the middle of shortest line connecting both obstacles
        x_middle = mean(x_obs_temp(:,ind_intersec),2);
        
%         plot(x_middle(1),x_middle(2), 'ko', 'LineWidth',4)
        %figure;
%         plot(x_obs_temp(1,:),x_obs_temp(2,:),'r')
%         plot(x_middle(1,:),x_middle(2,:),'ro')

%             x_close(:,it_obs2,it_obs1) = -x_close(:,it_obs1,it_obs2);
%             plot(x_close(1,ii,jj),x_close(2,ii,jj),'ok')

        % Desired Gamma in (0,1) to be on obstacle
        %Gamma_dynCenter = max(1-delta_dist/(ref_dist-dist_contact),realmin); % TODO REMOVE
        Gamma_dynCenter = max(1-delta_dist/(ref_dist-dist_contact),0);

        % Desired position of dynamic_center if only one obstacle existed
        Gamma_intersec = sum( ( (rotMat'*(x_middle-obs{ii}.x0) )./ ...
                                (obs{ii}.sf*obs{ii}.a)  ).^(2*obs{ii}.p),1);

        x_cenDyn_temp(:,ii,jj) = rotMat*(rotMat'*(x_middle - obs{ii}.x0).*( repmat(Gamma_dynCenter/Gamma_intersec, dim,1) ).^(1./(2*obs{ii}.p)));

        % Desired position if only one obstacle exists 
        Gamma_intersec = sum( ( (rotMatrices(:,:,jj)'*(x_middle-obs{jj}.x0) ) ./ ...
                                (obs{jj}.sf*obs{jj}.a)  ).^(2*obs{jj}.p), 1);
        %x_cenDyn_temp(:,ii,jj) = rotMat*(rotMat'*(x_middle - obs{ii}.x0).*(ones(dim,1)*Gamma_dynCenter/Gamma_intersec).^(1./(2*obs{ii}.p))) ;
        x_cenDyn_temp(:,jj,ii) = rotMatrices(:,:,jj)*( rotMatrices(:,:,jj)'*(x_middle-obs{jj}.x0) .* repmat(Gamma_dynCenter/Gamma_intersec,dim,1) .^(1./(2*obs{jj}.p))); % TODO rotation matrix?

    end
end

for ii=1:N_obs % Assign dynamic center 
    if ismember(ii,intersection_obs) 
        continue; % Don't reasign dynamic center if intersection exists
    end

    if sum(weight_obs_temp(ii,:))
    % There are points on the surface of the obstacle
        pointsOnSurface = weight_obs_temp == -1;
        if sum(pointsOnSurface)
            weight_obs_temp= 1.*pointOnSurface; % Bool to float
        else
            weight_obs = weight_obs_temp(:,ii)/ sum(weight_obs_temp(:,ii) );
        end

        % Linear interpolation if at least one close obstacle --- MAYBE
        % change to nonlinear
        x_centDyn = squeeze(x_cenDyn_temp(:,ii,:));

        obs{ii}.x_center_dyn = sum(x_centDyn.*repmat(weight_obs',dim,1), 2) ...
                                    +obs{ii}.x0;
    else % default center otherwise
        obs{ii}.x_center_dyn = obs{ii}.x0;
    end
end

end



function [intersection_ind_temp, n_intersection, x_obs_temp, a_range] ...
    = check_for_intersection(obs_samp, obs_test, delta_dist, N_resol, thetaRange, a_xRange_old, rotMat)

a_range = obs_samp.a+delta_dist; % New ellipse axis
[x_obs_temp] = drawEllipse_bound(obs_samp, N_resol, thetaRange, a_range);
                                                                
for kk = 1:size(x_obs_temp,2) % can loop be removed? one-liner -- remove rotation matrix
%                     Gamma(ii) = sum( ( 1/obs{ii}.sf*rotMat'*(x_obs_temp(:,ii) )./ ...
%                         repmat( (obs{ii}.a+delta_d) , 1, length(ind)) ).^(2*obs{ii}.p), 1);
    Gamma(kk) = sum( ( 1/obs_test.sf*rotMat'*(x_obs_temp(:,kk)-obs_test.x0 )./ ...
         (obs_test.a+delta_dist)).^(2*obs_test.p), 1);
end
intersection_ind_temp = Gamma<1;
n_intersection = sum(intersection_ind_temp); %, x_range, N_resol);
    
% % TODO - REMOVE after debugging
% obs_temp = obs_test;
% obs_temp.a = obs_temp.a + delta_dist;
% x_obs = drawEllipse_bound(obs_temp);
% plot(x_obs(1,:), x_obs(2,:), '.-'); hold on;
% 
% plot(x_obs_temp(1,:), x_obs_temp(2,:), 'b.-'); 

end



function [x_obs] = drawEllipse_bound(obs, N_resol, theta_range, axis_length, R)
th_r = obs.th_r;
x0 = obs.x0;
p = obs.p;

if nargin<2; N_resol = 16; end % 2^3
if nargin<4; axis_length = obs.a; end

dim = size(x0,1); % Dimension

if nargin < 5
    R = compute_R(dim, th_r);
end

if nargin<3 
    theta_min = 0;
    dTheta = 2*pi/N_resol;
elseif isempty(theta_range)
    theta_min = 0;
    dTheta = 2*pi/N_resol;
else
    
    % increase if to low
    theta_range(2)= (theta_range(2)<theta_range(1))*2*pi + theta_range(2);
    
    theta_min = theta_range(1);
    dTheta = (theta_range(2)- theta_range(1))/(N_resol-1);
    
    % TODO - REMOVE
%     plot([x_start(1,:), x_end(1,:)],[x_start(2,:), x_end(2,:)],'k--') % TODO remmove after debugging
    %theta_min = 0;
    %dTheta = 2*pi/N_resol;

end

% theta_list = []; % TODO -- delete this temporary list (only debugging)

x_obs = zeros(dim, N_resol);
for it_x = 1:N_resol
    
    theta = theta_min + (it_x-1)*dTheta ;
    theta = theta-(theta>pi)*2*pi; % Range of [-pi, pi]
%     theta_list = [theta_list, round(theta*180/pi)];
    
    x_obs(1,it_x) = axis_length(1,:).*cos(theta);
    x_obs(2,it_x) = axis_length(2,:).*sign(theta).*(1 - cos(theta).^(2.*p(1,:))).^(1./(2.*p(2,:)));
%     x_obs_notRot(:,it_x) = x_obs(:,it_x);
    x_obs(:,it_x) = R*x_obs(:,it_x);
end
x_obs = x_obs+x0;
%plot(x_obs_notRot(1,:), x_obs_notRot(2,:),'k--')
% theta_list


% Caclulate cylindric representation -- TODO - DELETE
% x_obs_sf = x_obs - x0;
% x_obs_rad= sqrt(sum(x_obs_sf.^2,1));
% x_obs_norm = x_obs_sf./repmat(x_obs_rad,dim,1);

end