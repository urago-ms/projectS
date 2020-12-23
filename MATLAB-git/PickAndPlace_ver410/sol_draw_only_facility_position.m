function sol_draw(file_name, cl_s, cl_l, ml, pl, m_max_range, m_min_range, p_pos_x, p_pos_y, m_pos_x, m_pos_y, c_pos_x, c_pos_y)
% p_pos_x
% p_pos_y
% m_pos_x
% m_pos_y
% c_pos_x
% c_pos_y

% % Delete all figure
% close
% clc
disp('Solution plot %%%%%%%%%%%%%%%%%%%%%%%%%%');

%%%%%%%%%%%%%%% Solution plot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Draw palette range %%%%%%%%%%%%%%%%%%%%%%%%%%
axis([-4000 4000 -4000 4000])
pbaspect([1 1 1])

% center
scatter(p_pos_x, p_pos_y, 'red', 'filled');
hold on;

% horizontal line
x = [p_pos_x - 1/2*pl p_pos_x + 1/2*pl];
y1 = [p_pos_y - 1/2*pl p_pos_y - 1/2*pl];
y2 = [p_pos_y + 1/2*pl p_pos_y + 1/2*pl];
plot(x,y1,'red','LineWidth',1.5)
plot(x,y2,'red','LineWidth',1.5)

% vertical line
x1 = [p_pos_x - 1/2*pl p_pos_x - 1/2*pl];
x2 = [p_pos_x + 1/2*pl p_pos_x + 1/2*pl];
y = [p_pos_y - 1/2*pl p_pos_y + 1/2*pl];
plot(x1,y,'red','LineWidth',1.5)
plot(x2,y,'red','LineWidth',1.5)


% Draw conveyor range %%%%%%%%%%%%%%%%%%%%%%%%%%
axis([-4000 4000 -4000 4000])
pbaspect([1 1 1])

% center
%d_green = [0.4660 0.6740 0.1880];
scatter(c_pos_x, c_pos_y,'green','filled');
hold on;

% horizontal line
x = [c_pos_x - 1/2*cl_s c_pos_x + 1/2*cl_s];
y1 = [c_pos_y - 1/2*cl_l c_pos_y - 1/2*cl_l];
y2 = [c_pos_y + 1/2*cl_l c_pos_y + 1/2*cl_l];
plot(x,y1,'Color','green','LineWidth',1.5)
plot(x,y2,'Color','green','LineWidth',1.5)
%         plot(x,y1,'Color',d_green,'LineWidth',2.0)
%         plot(x,y2,'Color',d_green,'LineWidth',2.0)

% vertical line
x1 = [c_pos_x - 1/2*cl_s c_pos_x - 1/2*cl_s];
x2 = [c_pos_x + 1/2*cl_s c_pos_x + 1/2*cl_s];
y = [c_pos_y - 1/2*cl_l c_pos_y + 1/2*cl_l];
plot(x1,y,'Color','green','LineWidth',1.5)
plot(x2,y,'Color','green','LineWidth',1.5)


% Draw manipulator range %%%%%%%%%%%%%%%%%%%%%%%%%%
axis([-4000 4000 -4000 4000])
pbaspect([1 1 1])

% center
scatter(m_pos_x, m_pos_y, 'blue', 'filled');
hold on;

% horizontal line
x = [m_pos_x - 1/2*ml m_pos_x + 1/2*ml];
y1 = [m_pos_y - 1/2*ml m_pos_y - 1/2*ml];
y2 = [m_pos_y + 1/2*ml m_pos_y + 1/2*ml];
plot(x,y1,'blue','LineWidth',1.5)
plot(x,y2,'blue','LineWidth',1.5)

% vertical line
x1 = [m_pos_x - 1/2*ml m_pos_x - 1/2*ml];
x2 = [m_pos_x + 1/2*ml m_pos_x + 1/2*ml];
y = [m_pos_y - 1/2*ml m_pos_y + 1/2*ml];
plot(x1,y,'blue','LineWidth',1.5)
plot(x2,y,'blue','LineWidth',1.5)


% Manipulator movable range plot %%%%%%%%%%%%%%%%%%%%%%%%%%
t = linspace(0,2*pi,100);
cx = m_pos_x;   %center
cy = m_pos_y;
r = m_max_range;    % radius
plot(r*sin(t)+cx,r*cos(t)+cy,'blue','LineWidth',1.5)
hold on;

plot(m_min_range*sin(t)+cx,m_min_range*cos(t)+cy,'blue','LineWidth',1.5)
%hold on;


% save Initial solution graph
set(gca,'FontSize',15);
ylabel('y','FontSize',24)
xlabel('x','FontSize',24)

%     time = datestr(date,'yyyymmdd_HHMMSS');
%     file_name = strcat(time,'_');
%     file_name = strcat(file_name,id_num_st);
%     file_name = strcat(file_name,'_InitialSol.pdf');

saveas(gcf,file_name)

end
