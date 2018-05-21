function sp = subplot_results(mode,sp,x,xT,varargin)
if isempty(varargin) || isempty(varargin{1})
    b_obs = false;
else
    b_obs = true;
    obs = varargin{1};
end

[d, ~, nbSPoint] = size(x);
switch mode
    case 'i' %initializing the figure
        if d==2
            if isfield(sp,'fig')
                subplot(sp.fig)
            else
                sp.fig = figure('name','2D Simulation of the task','position',[650 550 560 420]);
            end
            sp.axis = gca;
            hold on
            sp.xT = plot(xT(1),xT(2),'k*','markersize',10,'linewidth',1.5);
            sp.xT_l = plot(xT(1),xT(2),'k--','linewidth',1.5);
            for j=1:nbSPoint
                plot(x(1,1,j),x(2,1,j),'ok','markersize',2,'linewidth',7.5)
                sp.x(j)= plot(x(1,1,j),x(2,1,j));
            end
            xlabel('$\xi_1$','interpreter','latex','fontsize',16);
            ylabel('$\xi_2$','interpreter','latex','fontsize',16);
            grid on; box on;

            if b_obs
                [x_obs, x_obs_sf] = obs_draw_ellipsoid(obs,40);
                for n=1:size(x_obs,3)
                    sp.obs(n) = patch(x_obs(1,:,n),x_obs(2,:,n),0.0*ones(1,size(x_obs,2)),[0.6 1 0.6], 'FaceAlpha', 0.7);
                    sp.obs_sf(n) = plot(x_obs_sf(1,:,n),x_obs_sf(2,:,n),'k--','linewidth',0.5);
                    
                    if isfield(obs{n}, 'x_center_dyn')
                        sp.x_center(n) = plot(obs{n}.x_center_dyn(1),obs{n}.x_center_dyn(2),'k+','LineWidth',2.3); hold on;
                    else
                        if ~isfield(obs{n}, 'x_center')
                            obs{n}.x_center = [0;0];
                        end
                        cosAng = cos(obs{n}.th_r);
                        sinAng = sin(obs{n}.th_r);
                        R = [cosAng, -sinAng; sinAng, cosAng];
                        pos = obs{n}.x0 + R*(obs{n}.a.*obs{n}.x_center);
                        sp.x_center(n) = plot(pos(1),pos(2),'ko','LineWidth',2);
                    end
                end
            end

        elseif d==3
            if isfield(sp,'fig')
                %figure(sp.fig)
                subplot(sp.fig)
            else
                sp.fig = figure('name','3D Simulation of the task','position',[650 550 560 420]);
            end
            sp.axis = gca;
            hold on
            sp.xT = plot3(xT(1),xT(2),xT(3),'k*','markersize',10,'linewidth',1.5);
            sp.xT_l = plot3(xT(1),xT(2),xT(3),'k--','linewidth',1.5);
            for j=1:nbSPoint
                plot3(x(1,1,j),x(2,1,j),x(3,1,j),'ok','markersize',2,'linewidth',7.5)
                sp.x(j)= plot3(x(1,1,j),x(2,1,j),x(3,1,j));
            end
            
            if b_obs
                n_theta = 15;
                n_phi = 10;
                x_obs = obs_draw_ellipsoid(obs,[n_theta n_phi]);
                for n=1:size(x_obs,3)
                    sp.obs(n) = surf(reshape(x_obs(1,:,n),n_phi,n_theta), reshape(x_obs(2,:,n),n_phi,n_theta), reshape(x_obs(3,:,n),n_phi,n_theta));
                    set(sp.obs(n),'FaceColor',[0.6 1 0.6],'linewidth',0.1)
                end
            end
            xlabel('$\xi_1$','interpreter','latex','fontsize',16);
            ylabel('$\xi_2$','interpreter','latex','fontsize',16);
            zlabel('$\xi_3$','interpreter','latex','fontsize',16);
            grid on
            view(-28,44)
        else
            if isfield(sp,'fig')
                %axis(sp.fig)
                subplot(sp.fig)
            else
                %sp.fig = figure('name','Simulation of the task','position',[540   150   510   810]);
                title('This a title')
            end
            for i=2:d
                sp.axis(i-1)=subplot(d-1,1,i-1);
                hold on
                sp.xT(i-1) = plot(xT(1),xT(i),'k*','markersize',10,'linewidth',1.5);
                sp.xT_l(i-1) = plot(xT(1),xT(i),'k--','linewidth',1.5);
                for j=1:nbSPoint
                    plot(x(1,1,j),x(i,1,j),'ok','markersize',2,'linewidth',7.5);
                    sp.x(i-1,j)= plot(x(1,1,j),x(i,1,j));
                end
                ylabel(['$\xi_' num2str(i) '$'],'interpreter','latex','fontsize',12);
                grid on
                if i==d
                    xlabel(['$\xi_' num2str(1) '$'],'interpreter','latex','fontsize',12);
                end
                grid on;box on
            end
        end

    case 'u' %updating the figure
        if gcf ~= sp.fig
            %figure(sp.fig)
            subplot(sp.fig)
        end
        
        if d==2
            ax=get(sp.axis);
            for j=1:nbSPoint
                set(sp.x(j),'XData',x(1,:,j),'YData',x(2,:,j));
            end

%             if max(x(1,end,:))>ax.XLim(2) || min(x(1,end,:))<ax.XLim(1) || max(x(2,end,:))>ax.YLim(2) || min(x(2,end,:))<ax.YLim(1)
%                 %axis(sp.axis,'tight');
%                 ax=get(sp.axis);
%                 axis(sp.axis,...
%                      [ax.XLim(1)-(ax.XLim(2)-ax.XLim(1))/10 ax.XLim(2)+(ax.XLim(2)-ax.XLim(1))/10 ...
%                       ax.YLim(1)-(ax.YLim(2)-ax.YLim(1))/10 ax.YLim(2)+(ax.YLim(2)-ax.YLim(1))/10]);
%             end
        elseif d==3
            ax=get(sp.axis);
            for j=1:nbSPoint
                set(sp.x(j),'XData',x(1,:,j),'YData',x(2,:,j),'ZData',x(3,:,j))
            end
            
            if max(x(1,end,:))>ax.XLim(2) || min(x(1,end,:))<ax.XLim(1) || max(x(2,end,:))>ax.YLim(2) || min(x(2,end,:))<ax.YLim(1) || max(x(3,end,:))>ax.ZLim(2) || min(x(3,end,:))<ax.ZLim(1)
                %axis(sp.axis,'tight');
                ax=get(sp.axis);
                axis(sp.axis,...
                     [ax.XLim(1)-(ax.XLim(2)-ax.XLim(1))/10 ax.XLim(2)+(ax.XLim(2)-ax.XLim(1))/10 ...
                      ax.YLim(1)-(ax.YLim(2)-ax.YLim(1))/10 ax.YLim(2)+(ax.YLim(2)-ax.YLim(1))/10 ...
                      ax.ZLim(1)-(ax.ZLim(2)-ax.ZLim(1))/10 ax.ZLim(2)+(ax.ZLim(2)-ax.ZLim(1))/10]);
            end
        else
            for i=1:d-1
                ax=get(sp.axis(i));
                for j=1:nbSPoint
                    set(sp.x(i,j),'XData',x(1,:,j),'YData',x(i+1,:,j))
                end
                
                if max(x(1,end,:))>ax.XLim(2) || min(x(1,end,:))<ax.XLim(1) || max(x(i+1,end,:))>ax.YLim(2) || min(x(i+1,end,:))<ax.YLim(1)
                    %axis(sp.axis(i),'tight');
                    ax=get(sp.axis(i));
                    axis(sp.axis(i),...
                        [ax.XLim(1)-(ax.XLim(2)-ax.XLim(1))/10 ax.XLim(2)+(ax.XLim(2)-ax.XLim(1))/10 ...
                        ax.YLim(1)-(ax.YLim(2)-ax.YLim(1))/10 ax.YLim(2)+(ax.YLim(2)-ax.YLim(1))/10]);
                end
            end
        end
        axis equal;
        
    case 't' %updating the figure
        if gcf ~= sp.fig
            %figure(sp.fig)
            subplot(sp.fig)
        end
        if d==2
            ax=get(sp.axis);
            set(sp.xT,'XData',xT(1,end),'YData',xT(2,end))
            set(sp.xT_l,'XData',xT(1,:),'YData',xT(2,:))
            
            if max(xT(1,end))>ax.XLim(2) || min(xT(1,end))<ax.XLim(1) || max(xT(2,end))>ax.YLim(2) || min(xT(2,end))<ax.YLim(1)
                %axis(sp.axis,'tight');
                ax=get(sp.axis);
                axis(sp.axis,...
                     [ax.XLim(1)-(ax.XLim(2)-ax.XLim(1))/10 ax.XLim(2)+(ax.XLim(2)-ax.XLim(1))/10 ...
                      ax.YLim(1)-(ax.YLim(2)-ax.YLim(1))/10 ax.YLim(2)+(ax.YLim(2)-ax.YLim(1))/10]);
            end
        elseif d==3
            ax=get(sp.axis);
            set(sp.xT,'XData',xT(1,end),'YData',xT(2,end),'ZData',xT(3,end))
            set(sp.xT_l,'XData',xT(1,:),'YData',xT(2,:),'ZData',xT(3,:))
            
            if max(xT(1,end))>ax.XLim(2) || min(xT(1,end))<ax.XLim(1) || max(xT(2,end))>ax.YLim(2) || min(xT(2,end))<ax.YLim(1) || max(xT(3,end))>ax.ZLim(2) || min(xT(3,end))<ax.ZLim(1)
                %axis(sp.axis,'tight');
                ax=get(sp.axis);
                axis(sp.axis,...
                     [ax.XLim(1)-(ax.XLim(2)-ax.XLim(1))/10 ax.XLim(2)+(ax.XLim(2)-ax.XLim(1))/10 ...
                      ax.YLim(1)-(ax.YLim(2)-ax.YLim(1))/10 ax.YLim(2)+(ax.YLim(2)-ax.YLim(1))/10 ...
                      ax.ZLim(1)-(ax.ZLim(2)-ax.ZLim(1))/10 ax.ZLim(2)+(ax.ZLim(2)-ax.ZLim(1))/10]);
            end
        else
            for i=1:d-1
                ax=get(sp.axis(i));
                set(sp.xT(i),'XData',xT(1,end),'YData',xT(i+1,end))
                set(sp.xT_l(i),'XData',xT(1,:),'YData',xT(i+1,:))
                
                if max(xT(1,end))>ax.XLim(2) || min(xT(1,end))<ax.XLim(1) || max(xT(i+1,end))>ax.YLim(2) || min(xT(i+1,end))<ax.YLim(1)
                    %axis(sp.axis(i),'tight');
                    ax=get(sp.axis(i));
                    axis(sp.axis(i),...
                        [ax.XLim(1)-(ax.XLim(2)-ax.XLim(1))/10 ax.XLim(2)+(ax.XLim(2)-ax.XLim(1))/10 ...
                        ax.YLim(1)-(ax.YLim(2)-ax.YLim(1))/10 ax.YLim(2)+(ax.YLim(2)-ax.YLim(1))/10]);
                end
            end
        end
        
    case 'o' % Updating the obstacle position
        if gcf ~= sp.fig
            % figure(sp.fig)
            subplot(sp.fig)
        end
        if b_obs 
            n = varargin{1};
            dx = varargin{2};
            if d==2 
                obs = varargin{3};
                if(isfield(obs{n}.perturbation,'w'))
                    
                    % Redraw whole circle
                    [x_obs, x_obs_sf] = obs_draw_ellipsoid(obs,40);
                    
                    set(sp.obs(n),'XData',x_obs(1,:,n));
                    set(sp.obs(n),'YData',x_obs(2,:,n));

                    set(sp.obs_sf(n),'XData',x_obs_sf(1,:,n))
                    set(sp.obs_sf(n),'YData',x_obs_sf(2,:,n))
                else
                    % Update only velocity
                    set(sp.obs(n),'XData',get(sp.obs(n),'XData')+ dx(1))
                    set(sp.obs(n),'YData',get(sp.obs(n),'YData')+ dx(2))

                    set(sp.obs_sf(n),'XData',get(sp.obs_sf(n),'XData')+ dx(1))
                    set(sp.obs_sf(n),'YData',get(sp.obs_sf(n),'YData')+ dx(2))
                end

            elseif d==3
                set(sp.obs(n),'XData',get(sp.obs(n),'XData')+ dx(1))
                set(sp.obs(n),'YData',get(sp.obs(n),'YData')+ dx(2))
                set(sp.obs(n),'ZData',get(sp.obs(n),'ZData')+ dx(2))
            end
        end
        
    case 'center_dyn' % adapt center case
        n_obs = varargin{1};
        obs = varargin{2};
        if d == 2
            for n = 1:n_obs
                if isfield(obs{n}, 'x_center_dyn')
                    set(sp.x_center(n), 'XData', obs{n}.x_center_dyn(1));
                    set(sp.x_center(n), 'YData', obs{n}.x_center_dyn(2));
                else
                    if ~isfield(obs{n}, 'x_center')
                        obs{n}.x_center = [0;0];
                    end
                    cosAng = cos(obs{n}.th_r);
                    sinAng = sin(obs{n}.th_r);
                    R = [cosAng, -sinAng; sinAng, cosAng];
                    pos = obs{n}.x0 + R*(obs{n}.a.*obs{n}.x_center);
                    set(sp.x_center(n), 'XData', pos(1))
                    set(sp.x_center(n), 'YData', pos(2))
                end
            end
        elseif d==3
            warning('3d case not yet implemented \n');
        end
        
    case 'center_stat' % adapt center case
        n_obs = varargin{1};
        obs = varargin{2};
        if d == 2
            for n = 1:n_obs
                set(sp.x_center(n), 'XData', obs{n}.x0(1))
                set(sp.x_center(n), 'YData', obs{n}.x0(2))
            end
        elseif d==3
            warning('3d case not yet implemented \n');
        end
        
    case 'f' % final alignment of axis
        if gcf ~= sp.fig
            %figure(sp.fig)
            %axis(sp.fig)   
            subplot(sp.fig)
        end
        if d==2
            %axis(sp.axis,'tight');
            ax=get(sp.axis);
            axis(sp.axis,...
                [ax.XLim(1)-(ax.XLim(2)-ax.XLim(1))/10 ax.XLim(2)+(ax.XLim(2)-ax.XLim(1))/10 ...
                 ax.YLim(1)-(ax.YLim(2)-ax.YLim(1))/10 ax.YLim(2)+(ax.YLim(2)-ax.YLim(1))/10]);
        elseif d==3
            %axis(sp.axis,'tight');
            ax=get(sp.axis);
            axis(sp.axis,...
                [ax.XLim(1)-(ax.XLim(2)-ax.XLim(1))/10 ax.XLim(2)+(ax.XLim(2)-ax.XLim(1))/10 ...
                 ax.YLim(1)-(ax.YLim(2)-ax.YLim(1))/10 ax.YLim(2)+(ax.YLim(2)-ax.YLim(1))/10 ...
                 ax.ZLim(1)-(ax.ZLim(2)-ax.ZLim(1))/10 ax.ZLim(2)+(ax.ZLim(2)-ax.ZLim(1))/10]);
        else
            for i=1:d-1
                %axis(sp.axis(i),'tight');
                ax=get(sp.axis(i));
                axis(sp.axis(i),...
                    [ax.XLim(1)-(ax.XLim(2)-ax.XLim(1))/10 ax.XLim(2)+(ax.XLim(2)-ax.XLim(1))/10 ...
                     ax.YLim(1)-(ax.YLim(2)-ax.YLim(1))/10 ax.YLi   m(2)+(ax.YLim(2)-ax.YLim(1))/10]);
            end
        end
end
drawnow