function animate_3dquad_load(torig, x, t_pd, xLd)
RATE = 25 * 1;
%==========================================
% initialize the animation figure and axes
%==========================================
figure_x_limits = [-4 8];
figure_y_limits = [-9 5];
figure_z_limits = [-2 3] ;
fig1 = figure;

set(0,'Units','pixels')
scnsize = get(0,'ScreenSize');

screen_width = scnsize(3);
screen_height = scnsize(4);

% find the minimum scaling factor
figure_x_size = figure_x_limits(2) - figure_x_limits(1);
figure_y_size = figure_y_limits(2) - figure_y_limits(1);

xfactor = screen_width/figure_x_size;
yfactor = screen_height/figure_y_size;

if (xfactor < yfactor)
    screen_factor = 0.5*xfactor;
else
    screen_factor = 0.5*yfactor;
end

% calculate screen offsets
screen_x_offset = (screen_width - screen_factor*figure_x_size)/2;
screen_y_offset = (screen_height - screen_factor*figure_y_size)/2;

% draw figure and axes
set(fig1,'Position', [screen_x_offset screen_y_offset screen_factor*figure_x_size screen_factor*figure_y_size]);
set(fig1,'MenuBar', 'none');
axes1 = axes;
set(axes1,'XLim',figure_x_limits,'YLim',figure_y_limits);
%     set(axes1,'Position',[0 0 1 1]);
%     set(axes1,'Color','w');
%set(axes1,'TickDir','out');
axis equal ;
box on;
xorig = x ;
xLdorig = xLd ;
[t, x] = even_sample(torig, x, RATE);
t = t+torig(1) ;
[~, xLd] = even_sample(t_pd, xLd, RATE) ;
L = 1 ; % Cable length
%% Define plotting length
hist = 2500;
MAKE_MOVIE = 1;
if(MAKE_MOVIE)
    M = moviein(length(t)) ;
end

% aviobj = avifile('sample2.avi','compression','None');
for i=1:length(t)
    %set(axes1,'XLim',figure_x_limits+pH(i,1)) ;
    drawone(axes1, x(i,:)');
    %% No shading plotting
    plot3(x(max(1,i-hist):i, 1), x(max(1,i-hist):i, 2), x(max(1,i-hist):i, 3), 'b');
    plot3(xLd(max(1,i-hist):i, 1), xLd(max(1,i-hist):i, 2), xLd(max(1,i-hist):i, 3), 'r');

    %         plot3(xorig(:, 1), xorig(:, 2), xorig(:, 3), 'b') ;
    %         plot3(xLdorig(:, 1), xLdorig(:, 2), xLdorig(:, 3), 'r') ;
    %         s = sprintf('Running\n t = %1.2fs \n 1/%d realtime speed',t(i), RATE/25);
    %         text(-1.3,2.4,s,'FontAngle','italic','FontWeight','bold');
    drawnow;
    set(axes1,'XLim',figure_x_limits,'YLim',figure_y_limits,'ZLim',figure_z_limits);
    if MAKE_MOVIE, M(:,i) = getframe; end
    %         aviobj = addframe(aviobj, fig1);
    
end
%     aviobj = close(aviobj);

% v = VideoWriter('newfile.avi','Uncompressed AVI');
% writeVideo(v,M);

% if(MAKE_MOVIE)
% %     for i = 1:length(M)
% %     if (i==1)
% %     [r1,c1,s] = size(M(i).cdata);
% %     rk = r1;
% %     ck = c1;
% %     else
% %     [rk,ck,s] = size(M(i).cdata);
% %     end
% %     r1 = min(r1,rk);
% %     c1 = min(c1,ck);
% %     end
% %
% %     for i = 1:length(M)
% %     [rk,ck] = size(M(i).cdata);
% %         if ((rk~= r1) || (ck ~=c1))
% %         M(i).cdata = M(i).cdata(1:r1, 1:c1,:);
% %         end
% %     end
% %     mpgwrite(M,jet,'movie.mpg', [1 0 0 1 10 8 10 25]);
%     mpgwrite(M,jet,'movie.mpg', [1 0 0 1 4 1 1 1]);
% end
end

function drawone(parent, x)
tem = get(parent,'Children');
delete(tem);

s.L = 0.175; %length of quadrotor boom
s.R = 0.1; %radius of propeller prop

L = 1 ; % Cable length

% Extract state x
xL = x(1:3) ;
%     vL = x(4:6) ;
p = x(7:9) ;
%     omega = x(10:12) ;
R = reshape(x(13:21), 3,3) ;
%     Omega = x(22:24) ;
xQ = xL - L*p ;

%     plot3(xQ(1), xQ(2), xQ(3), 'r.') ;

BRW = R' ;

point1 = BRW'*[s.L,0,0]';
point2 = BRW'*[0,s.L,0]';
point3 = BRW'*[-s.L,0,0]';
point4 = BRW'*[0,-s.L,0]';

nprop = 40;
propangs = linspace(0,2*pi,nprop);
proppts = s.R*BRW'*[cos(propangs);sin(propangs);zeros(1,nprop)];

wp = xQ ;
wp1 = wp + point1;
wp2 = wp + point2;
wp3 = wp + point3;
wp4 = wp + point4;
wp_cable_attach = wp + BRW'*[0;0;0] ; %[0;0;-params.cable_attach_d];

prop1 = proppts + wp1*ones(1,nprop);
prop2 = proppts + wp2*ones(1,nprop);
prop3 = proppts + wp3*ones(1,nprop);
prop4 = proppts + wp4*ones(1,nprop);

lwp = 2 ;
lw1 = 1 ;
lwc = 2 ;
lwl = 2 ;

s.qhandle1 = line([wp1(1),wp3(1)],[wp1(2),wp3(2)],[wp1(3),wp3(3)]); hold on ;
s.qhandle2 = line([wp2(1),wp4(1)],[wp2(2),wp4(2)],[wp2(3),wp4(3)]);
set(s.qhandle1,'Color','k', 'LineWidth',lw1);
set(s.qhandle2,'Color','k', 'LineWidth',lw1);

s.hprop1 = plot3(prop1(1,:),prop1(2,:),prop1(3,:),'r-', 'LineWidth',lwp);
s.hprop2 = plot3(prop2(1,:),prop2(2,:),prop2(3,:),'b-', 'LineWidth',lwp);
s.hprop3 = plot3(prop3(1,:),prop3(2,:),prop3(3,:),'b-', 'LineWidth',lwp);
s.hprop4 = plot3(prop4(1,:),prop4(2,:),prop4(3,:),'b-', 'LineWidth',lwp);

s.hload = plot3(xL(1,1), xL(2,1), xL(3,1), 'ko', 'LineWidth',lwl) ;
s.cable = plot3([wp_cable_attach(1) xL(1,1)], [wp_cable_attach(2) xL(2,1)], [wp_cable_attach(3) xL(3,1)], 'k') ;
s.cable_attach = plot3([wp(1) wp_cable_attach(1)], [wp(2) wp_cable_attach(2)], [wp(3) wp_cable_attach(3)], 'r') ;

% axis equal
% set(handles.axes10,'Xlim',[max(min(s.xs(s.goodv))-offset,-20), min(max(s.xs(s.goodv))+offset,20)]);
% set(handles.axes10,'Ylim',[max(min(s.ys(s.goodv))-offset,-20), min(max(s.ys(s.goodv))+offset,20)]);
% set(handles.axes10,'Zlim',[max(min(s.zs(s.goodv))-offset,-20), min(max(s.zs(s.goodv))+offset,20)]);
grid on
view([-1,-1,1])
xlabel('X')
ylabel('Y')
zlabel('Z')




%     L_quad = 0.15 ;
%     L_blade = 0.07 ;
%     L_axis = 0.05 ;
%
%     delta_p = L_quad*[cos(psi) ; sin(psi)] ;
%     p1 = [x ; y] - delta_p ;
%     p2 = [x ; y] + delta_p ;
%     plot([p1(1)  p2(1)], [p1(2) p2(2)], 'k', 'LineWidth', 2) ;
%     hold on ;
%
%     delta_a = L_axis*[cos(pi/2+psi) ; sin(pi/2+psi)] ;
%     a1 = p1 + delta_a ;
%     a2 = p2 + delta_a ;
%     plot([p1(1)  a1(1)], [p1(2) a1(2)], 'k', 'LineWidth', 2) ;
%     plot([p2(1)  a2(1)], [p2(2) a2(2)], 'k', 'LineWidth', 2) ;
%
%     delta_b = L_blade*[cos(psi) ; sin(psi)] ;
%     b11 = a1 - delta_b ;
%     b12 = a1 + delta_b ;
%     b21 = a2 - delta_b ;
%     b22 = a2 + delta_b ;
%     plot([b11(1)  b12(1)], [b11(2) b12(2)], 'k', 'LineWidth', 2) ;
%     plot([b21(1)  b22(1)], [b21(2) b22(2)], 'k', 'LineWidth', 2) ;
%
%
%     % Plot load
%     L = 1 ;
%     plot([x xL], [y yL], 'k', 'LineWidth', 1) ;
%     plot(xL, yL, 'ko') ;
%
%     % Plot Obstacles
%     for j=1:length(Obstacles_Lines)
%         L = Obstacles_Lines{j} ;
%         plot([L(:,1); L(1,1)], [L(:,2); L(1,2)], 'k', 'LineWidth', 2) ;
%     end
end

function drawtwo(parent, x)
% tem = get(parent,'Children');
% delete(tem);
s.L = 0.175; %length of quadrotor boom
s.R = 0.1; %radius of propeller prop

% Extract state x
xL = x(1:3) ;
%     vL = x(4:6) ;
p = x(7:9) ;
%     omega = x(10:12) ;
R = reshape(x(13:21), 3,3) ;
%     Omega = x(22:24) ;
L = 1;
xQ = xL - L*p ;

%     plot3(xQ(1), xQ(2), xQ(3), 'r.') ;

BRW = R' ;

point1 = BRW'*[s.L,0,0]';
point2 = BRW'*[0,s.L,0]';
point3 = BRW'*[-s.L,0,0]';
point4 = BRW'*[0,-s.L,0]';

nprop = 40;
propangs = linspace(0,2*pi,nprop);
proppts = s.R*BRW'*[cos(propangs);sin(propangs);zeros(1,nprop)];

wp = xQ ;
wp1 = wp + point1;
wp2 = wp + point2;
wp3 = wp + point3;
wp4 = wp + point4;
wp_cable_attach = wp + BRW'*[0;0;0] ; %[0;0;-params.cable_attach_d];

prop1 = proppts + wp1*ones(1,nprop);
prop2 = proppts + wp2*ones(1,nprop);
prop3 = proppts + wp3*ones(1,nprop);
prop4 = proppts + wp4*ones(1,nprop);

lwp = 2 ;
lw1 = 1 ;
lwc = 2 ;
lwl = 2 ;

s.qhandle1 = line([wp1(1),wp3(1)],[wp1(2),wp3(2)],[wp1(3),wp3(3)]); hold on ;
s.qhandle2 = line([wp2(1),wp4(1)],[wp2(2),wp4(2)],[wp2(3),wp4(3)]);
set(s.qhandle1,'Color','k', 'LineWidth',lw1);
set(s.qhandle2,'Color','k', 'LineWidth',lw1);

s.hprop1 = plot3(prop1(1,:),prop1(2,:),prop1(3,:),'r-', 'LineWidth',lwp);
s.hprop2 = plot3(prop2(1,:),prop2(2,:),prop2(3,:),'b-', 'LineWidth',lwp);
s.hprop3 = plot3(prop3(1,:),prop3(2,:),prop3(3,:),'b-', 'LineWidth',lwp);
s.hprop4 = plot3(prop4(1,:),prop4(2,:),prop4(3,:),'b-', 'LineWidth',lwp);

s.hload = plot3(xL(1,1), xL(2,1), xL(3,1), 'ko', 'LineWidth',lwl) ;
s.cable = plot3([wp_cable_attach(1) xL(1,1)], [wp_cable_attach(2) xL(2,1)], [wp_cable_attach(3) xL(3,1)], 'k') ;
s.cable_attach = plot3([wp(1) wp_cable_attach(1)], [wp(2) wp_cable_attach(2)], [wp(3) wp_cable_attach(3)], 'r') ;

offset = 0.5;

% axis equal
% set(handles.axes10,'Xlim',[max(min(s.xs(s.goodv))-offset,-20), min(max(s.xs(s.goodv))+offset,20)]);
% set(handles.axes10,'Ylim',[max(min(s.ys(s.goodv))-offset,-20), min(max(s.ys(s.goodv))+offset,20)]);
% set(handles.axes10,'Zlim',[max(min(s.zs(s.goodv))-offset,-20), min(max(s.zs(s.goodv))+offset,20)]);
grid on

view([-1,-1,1])
xlabel('X')
ylabel('Y')
zlabel('Z')




%     L_quad = 0.15 ;
%     L_blade = 0.07 ;
%     L_axis = 0.05 ;
%
%     delta_p = L_quad*[cos(psi) ; sin(psi)] ;
%     p1 = [x ; y] - delta_p ;
%     p2 = [x ; y] + delta_p ;
%     plot([p1(1)  p2(1)], [p1(2) p2(2)], 'k', 'LineWidth', 2) ;
%     hold on ;
%
%     delta_a = L_axis*[cos(pi/2+psi) ; sin(pi/2+psi)] ;
%     a1 = p1 + delta_a ;
%     a2 = p2 + delta_a ;
%     plot([p1(1)  a1(1)], [p1(2) a1(2)], 'k', 'LineWidth', 2) ;
%     plot([p2(1)  a2(1)], [p2(2) a2(2)], 'k', 'LineWidth', 2) ;
%
%     delta_b = L_blade*[cos(psi) ; sin(psi)] ;
%     b11 = a1 - delta_b ;
%     b12 = a1 + delta_b ;
%     b21 = a2 - delta_b ;
%     b22 = a2 + delta_b ;
%     plot([b11(1)  b12(1)], [b11(2) b12(2)], 'k', 'LineWidth', 2) ;
%     plot([b21(1)  b22(1)], [b21(2) b22(2)], 'k', 'LineWidth', 2) ;
%
%
%     % Plot load
%     L = 1 ;
%     plot([x xL], [y yL], 'k', 'LineWidth', 1) ;
%     plot(xL, yL, 'ko') ;
%
%     % Plot Obstacles
%     for j=1:length(Obstacles_Lines)
%         L = Obstacles_Lines{j} ;
%         plot([L(:,1); L(1,1)], [L(:,2); L(1,2)], 'k', 'LineWidth', 2) ;
%     end
end