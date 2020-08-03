%clear

% numnum = 10;
% numnum_max = numnum;
% sum = 0;
%
% while numnum > 0


%rng(0,'twister');   % randam number initialization
rng('shuffle','twister');

orange = '[0.9300 0.4431 0.1804]';
d_green = '[0.4660 0.6740 0.1880]';
% d_green = [0.4660 0.6740 0.1880];

lim_x = 1000;  % space limit
lim_y = 2000;  % space limit

ml = 300; % manipulator base one side length
pl = 400; % palette one side length

link_1 = 262.5; % manipulator link1 length
link_2 = 262.5; % manipulator link2 length

cl_l = 1000;   % conveyor long side length
cl_l_max = 1000;   % conveyor long side max length
cl_l_min = 500;   % conveyor long side min length
cl_s = 300;   % conveyor short side length

% m_pos = zeros(1,2); % manipulator position
% p_pos = zeros(1,2); % palette position
% c_pos = zeros(1,2); % conveyor position
%c_pos = [0 0.5];

theta_1 = 0;    % Initial angle of J1 axis
theta_2 = 0;    % Initial angle of J2 axis

phi_1 = 0;    % Target angle of J1 axis
phi_2 = 0;    % Target angle of J2 axis

j1_max = pi;    % Maximum angle of J1 axis
j2_max = 2/3*pi;    % Maximum angle of J2 axis

omega_1 = 375;  % Max velocity of J1 axis[deg/s]
omega_2 = 375;  % Max velocity of J2 axis[deg/s]

% only for 120deg
range_max = link_1 + link_2;  % Maximum manipulator range
range_min = link_1;  % Minimum manipulator range


m_pos_x = 0;
p_pos_x = 0;
c_pos_x = 0;

m_pos_y = 0;
p_pos_y = 0;
c_pos_y = 1/2*cl_l;

pos_x = [m_pos_x p_pos_x c_pos_x];  % pos_x = [m_pos_x p_pos_x c_pos_x]
pos_y = [m_pos_y p_pos_y c_pos_y];  % pos_y = [m_pos_x p_pos_x c_pos_x]


in_pos = [0 0];     %inlet position
out_pos = [0 1000];    %outlet position

m_hand_pos = zeros(1,2); % manipulator hand position

m_min_range = sqrt(link_1^2 + link_2^2 - 2*link_1*link_2*cos(pi - j2_max)); % manipulator min range

g_initial_theta_1 = 0;

% Figure ID
date = datetime;
id_num = rem(second(datetime)*1000,1000);
id_num_st = num2str(id_num);

% Delete all figure
close
clc

% Number of trials
num_t = 100;
max_num_t = num_t;
interval = 5000;

num_best_angle = max_num_t/interval;
best_min_angle_array = zeros(100000,1);


% Timer start
tic




while num_t > 0
    
    %%%%%% palette initial position %%%%%%%%%%%%%%%%%%%%%%%%%%%
    while 1
        p_pos_x = -lim_x + (lim_x-(-lim_x))*rand;
        p_pos_y = -lim_y + (lim_y-(-lim_y))*rand;
        
        % The distance from the conveyor is within the movable range of the manipulator
        if (p_pos_x - c_pos_x)^2 + (p_pos_y - c_pos_y)^2 < (2*(link_1 + link_2))^2
            % Whether to overlap with the conveyor
            if (p_pos_x > (c_pos_x + 1/2*cl_s + 1/2*pl)) || (p_pos_x < (c_pos_x - 1/2*cl_s - 1/2*pl))   % If it does not overlap in the X-axis direction
                p_pos_x;
                p_pos_y;
                break;
            elseif (p_pos_y > (c_pos_y + 1/2*cl_l + 1/2*pl)) || (p_pos_y < (c_pos_y - 1/2*cl_l - 1/2*pl))   % If it does not overlap in the Y-axis direction
                p_pos_x;
                p_pos_y;
                break;
            else
            end
        end
    end
    
    
    
    %%%%%% manipulator initial position %%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % h = waitbar(0,'running');
    % t=0;
    % while 1
    %     m_pos_x = -lim_x + (lim_x-(-lim_x))*rand;
    %     m_pos_y = -lim_y + (lim_y-(-lim_y))*rand;
    %     if (m_pos_x < (p_pos_x + 1/2*pl + 1/2*ml)) && (m_pos_y < (p_pos_y + 1/2*pl + 1/2*ml))   % If it overlapping the upper right of the pallet
    %         if (m_pos_x < (p_pos_x + 1/2*pl + 1/2*ml)) && (m_pos_y > (p_pos_y - 1/2*pl - 1/2*ml))   % If it overlaps the bottom right of the pallet
    %             %if (m_pos_x > (p_pos_x - 1/2*pl - 1/2*ml)) && (m_pos_y < (p_pos_y + 1/2*pl + 1/2*ml))   % If it overlapping the upper left of the pallet
    %                 %if (m_pos_x > (p_pos_x - 1/2*pl - 1/2*ml)) && (m_pos_y > (p_pos_y - 1/2*pl - 1/2*ml))   % If it overlapping the bottom left of the pallet
    %                 else
    %                     m_pos_x
    %                     m_pos_y
    %                     break
    %                  end
    % %             end
    % %         end
    %     end
    %     t=t+1;
    %     waitbar(t/100000000)
    %
    % end
    % close(h)
    % h = waitbar(0,'running');
    % t=0;
    
    %%%%%% manipulator initial position %%%%%%%%%%%%%%%%%%%%%%%%%%%
    while 1
        m_pos_x = -lim_x + (lim_x-(-lim_x))*rand;
        m_pos_y = -lim_y + (lim_y-(-lim_y))*rand;
        % The distance from the conveyor is within the movable range of the manipulator
        if ((m_pos_x - c_pos_x)^2 + (m_pos_y - c_pos_y)^2 < (link_1 + link_2)^2) && ((m_pos_x - c_pos_x)^2 + (m_pos_y - c_pos_y)^2 > m_min_range^2)
            % The distance from the palette is within the movable range of the manipulator
            if (m_pos_x - p_pos_x)^2 + (m_pos_y - p_pos_y)^2 < (link_1 + link_2)^2 &&  ((m_pos_x - p_pos_x)^2 + (m_pos_y - p_pos_y)^2 > m_min_range^2)
                
                % Whether to overlap with the palette
                if (m_pos_x > (p_pos_x + 1/2*pl + 1/2*ml)) || (m_pos_x < (p_pos_x - 1/2*pl - 1/2*ml))   % If it does not overlap in the X-axis direction
                    % Whether to overlap with the conveyor
                    if (m_pos_x > (c_pos_x + 1/2*cl_s + 1/2*ml)) || (m_pos_x < (c_pos_x - 1/2*cl_s - 1/2*ml))   % If it does not overlap in the X-axis direction
                        m_pos_x;
                        m_pos_y;
                        break;
                    elseif (m_pos_y > (c_pos_y + 1/2*cl_l + 1/2*ml)) || (m_pos_y < (c_pos_y - 1/2*cl_l - 1/2*ml))   % If it does not overlap in the Y-axis direction
                        m_pos_x;
                        m_pos_y;
                        break;
                    else
                    end
                    
                elseif (m_pos_y > (p_pos_y + 1/2*pl + 1/2*ml)) || (m_pos_y < (p_pos_y - 1/2*pl - 1/2*ml))   % If it does not overlap in the Y-axis direction
                    % Whether to overlap with the conveyor
                    if (m_pos_x > (c_pos_x + 1/2*cl_s + 1/2*ml)) || (m_pos_x < (c_pos_x - 1/2*cl_s - 1/2*ml))   % If it does not overlap in the X-axis direction
                        m_pos_x;
                        m_pos_y;
                        break;
                    elseif (m_pos_y > (c_pos_y + 1/2*cl_l + 1/2*ml)) || (m_pos_y < (c_pos_y - 1/2*cl_l - 1/2*ml))   % If it does not overlap in the Y-axis direction
                        m_pos_x;
                        m_pos_y;
                        break;
                    else
                    end
                else
                end
            else
            end
        else
        end
    end
    
    
    if num_t == max_num_t    % Run only the first time
        %%%%%%%%%%%%%%% Solution plot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Draw palette range %%%%%%%%%%%%%%%%%%%%%%%%%%
        axis([-2000 2000 -2000 2000])
        pbaspect([1 1 1])
        
        % center
        scatter(p_pos_x, p_pos_y, 'red', 'filled');
        hold on;
        
        % horizontal line
        x = [p_pos_x - 1/2*pl p_pos_x + 1/2*pl];
        y1 = [p_pos_y - 1/2*pl p_pos_y - 1/2*pl];
        y2 = [p_pos_y + 1/2*pl p_pos_y + 1/2*pl];
        plot(x,y1,'red','LineWidth',2.0)
        plot(x,y2,'red','LineWidth',2.0)
        
        % vertical line
        x1 = [p_pos_x - 1/2*pl p_pos_x - 1/2*pl];
        x2 = [p_pos_x + 1/2*pl p_pos_x + 1/2*pl];
        y = [p_pos_y - 1/2*pl p_pos_y + 1/2*pl];
        plot(x1,y,'red','LineWidth',2.0)
        plot(x2,y,'red','LineWidth',2.0)
        
        
        % Draw conveyor range %%%%%%%%%%%%%%%%%%%%%%%%%%
        axis([-2000 2000 -2000 2000])
        pbaspect([1 1 1])
        
        % center
        %d_green = [0.4660 0.6740 0.1880];
        scatter(c_pos_x, c_pos_y,'green','filled');
        hold on;
        
        % horizontal line
        x = [c_pos_x - 1/2*cl_s c_pos_x + 1/2*cl_s];
        y1 = [c_pos_y - 1/2*cl_l c_pos_y - 1/2*cl_l];
        y2 = [c_pos_y + 1/2*cl_l c_pos_y + 1/2*cl_l];
        plot(x,y1,'Color','green','LineWidth',2.0)
        plot(x,y2,'Color','green','LineWidth',2.0)
        %         plot(x,y1,'Color',d_green,'LineWidth',2.0)
        %         plot(x,y2,'Color',d_green,'LineWidth',2.0)
        
        % vertical line
        x1 = [c_pos_x - 1/2*cl_s c_pos_x - 1/2*cl_s];
        x2 = [c_pos_x + 1/2*cl_s c_pos_x + 1/2*cl_s];
        y = [c_pos_y - 1/2*cl_l c_pos_y + 1/2*cl_l];
        plot(x1,y,'Color','green','LineWidth',2.0)
        plot(x2,y,'Color','green','LineWidth',2.0)
        
        
        % Draw manipulator range %%%%%%%%%%%%%%%%%%%%%%%%%%
        axis([-2000 2000 -2000 2000])
        pbaspect([1 1 1])
        
        % center
        scatter(m_pos_x, m_pos_y, 'blue', 'filled');
        hold on;
        
        % horizontal line
        x = [m_pos_x - 1/2*ml m_pos_x + 1/2*ml];
        y1 = [m_pos_y - 1/2*ml m_pos_y - 1/2*ml];
        y2 = [m_pos_y + 1/2*ml m_pos_y + 1/2*ml];
        plot(x,y1,'blue','LineWidth',2.0)
        plot(x,y2,'blue','LineWidth',2.0)
        
        % vertical line
        x1 = [m_pos_x - 1/2*ml m_pos_x - 1/2*ml];
        x2 = [m_pos_x + 1/2*ml m_pos_x + 1/2*ml];
        y = [m_pos_y - 1/2*ml m_pos_y + 1/2*ml];
        plot(x1,y,'blue','LineWidth',2.0)
        plot(x2,y,'blue','LineWidth',2.0)
        
        
        % Manipulator movable range plot %%%%%%%%%%%%%%%%%%%%%%%%%%
        t = linspace(0,2*pi,100);
        cx = m_pos_x;   %center
        cy = m_pos_y;
        r = link_1 + link_2;    % radius
        plot(r*sin(t)+cx,r*cos(t)+cy,'blue','LineWidth',2.0)
        hold on;
        
        plot(m_min_range*sin(t)+cx,m_min_range*cos(t)+cy,'blue','LineWidth',2.0)
        %hold on;
        
        
        % save Initial solution graph
        set(gca,'FontSize',15);
        ylabel('y','FontSize',24)
        xlabel('x','FontSize',24)
        
        time = datestr(date,'yyyymmdd_HHMMSS');
        file_name = strcat(time,'_');
        file_name = strcat(file_name,id_num_st);
        file_name = strcat(file_name,'_InitialSol.pdf');
        saveas(gcf,file_name)
        
    end
    
    
    
    
    %%%%%%%%%%%%%%% Current Position IK %%%%%%%%%%%%%%%%%%%%%%
    %%%%% Inverse kinematics Solution 1 %%%%%%%%%%%%
    cos_alpha = (-((p_pos_x - m_pos_x)^2 + (p_pos_y - m_pos_y)^2) + link_1^2 + link_2^2) / (2*link_1*link_2);    % () of (2*link_1*link_2) is necessary
    alpha = atan2(sqrt(1 - cos_alpha*cos_alpha), cos_alpha);
    %alpha = atan2(sqrt(1-((-(p_pos_x - m_pos_x)^2 - (p_pos_y - m_pos_y)^2 + link_1^2 + link_2^2) / 2*link_1*link_2)^2), (-(p_pos_x - m_pos_x)^2 - (p_pos_y - m_pos_y)^2 + link_1^2 + link_2^2) / 2*link_1*link_2);
    c_ik_sol_1_theta_2 = pi - alpha;
    c_ik_sol_1_theta_1 = atan2((p_pos_y - m_pos_y), (p_pos_x - m_pos_x)) - atan2((link_2*sin(c_ik_sol_1_theta_2)), (link_1 + link_2*cos(c_ik_sol_1_theta_2)));
    %c_ik_sol_1_theta_1 = atan2((p_pos_y - m_pos_y), (p_pos_x - m_pos_x)) - atan2(link_2*sin(c_ik_sol_1_theta_2), link_1 + link_2*cos(c_ik_sol_1_theta_2))
    
    % % link_1 plot
    % joint_x1 = [m_pos_x m_pos_x + link_1*cos(c_ik_sol_1_theta_1)]
    % joint_y1 = [m_pos_y m_pos_y + link_1*sin(c_ik_sol_1_theta_1)]
    % plot(joint_x1,joint_y1,'blue')
    %
    % % link_2 plot
    % joint_x2 = [m_pos_x + link_1*cos(c_ik_sol_1_theta_1) m_pos_x + link_1*cos(c_ik_sol_1_theta_1) + link_2*cos(c_ik_sol_1_theta_1 + c_ik_sol_1_theta_2)]
    % joint_y2 = [m_pos_y + link_1*sin(c_ik_sol_1_theta_1) m_pos_y + link_1*sin(c_ik_sol_1_theta_1) + link_2*sin(c_ik_sol_1_theta_1 + c_ik_sol_1_theta_2)]
    % plot(joint_x2,joint_y2,'blue')
    
    
    %%%%% Inverse kinematics Solution 2 %%%%%%%%%%%%
    cos_alpha = (-((p_pos_x - m_pos_x)^2 + (p_pos_y - m_pos_y)^2) + link_1^2 + link_2^2) / (2*link_1*link_2);    % () of (2*link_1*link_2) is necessary
    alpha = - atan2(sqrt(1 - cos_alpha*cos_alpha), cos_alpha);
    %alpha = atan2(sqrt(1-((-(p_pos_x - m_pos_x)^2 - (p_pos_y - m_pos_y)^2 + link_1^2 + link_2^2) / 2*link_1*link_2)^2), (-(p_pos_x - m_pos_x)^2 - (p_pos_y - m_pos_y)^2 + link_1^2 + link_2^2) / 2*link_1*link_2);
    c_ik_sol_2_theta_2 = pi - alpha;
    c_ik_sol_2_theta_1 = atan2((p_pos_y - m_pos_y), (p_pos_x - m_pos_x)) - atan2((link_2*sin(c_ik_sol_2_theta_2)), (link_1 + link_2*cos(c_ik_sol_2_theta_2)));
    %c_ik_sol_2_theta_1 = atan2((p_pos_y - m_pos_y), (p_pos_x - m_pos_x)) - atan2(link_2*sin(c_ik_sol_2_theta_2), link_1 + link_2*cos(c_ik_sol_2_theta_2))
    
    % % link_1 plot
    % joint_x1 = [m_pos_x m_pos_x + link_1*cos(c_ik_sol_2_theta_1)]
    % joint_y1 = [m_pos_y m_pos_y + link_1*sin(c_ik_sol_2_theta_1)]
    % plot(joint_x1,joint_y1,'blue')
    %
    % % link_2 plot
    % joint_x2 = [m_pos_x + link_1*cos(c_ik_sol_2_theta_1) m_pos_x + link_1*cos(c_ik_sol_2_theta_1) + link_2*cos(c_ik_sol_2_theta_1 + c_ik_sol_2_theta_2)]
    % joint_y2 = [m_pos_y + link_1*sin(c_ik_sol_2_theta_1) m_pos_y + link_1*sin(c_ik_sol_2_theta_1) + link_2*sin(c_ik_sol_2_theta_1 + c_ik_sol_2_theta_2)]
    % plot(joint_x2,joint_y2,'blue')
    
    % % Forward kinematics (For confirmation)
    % X = link_1*cos(c_ik_sol_2_theta_1) + link_2*cos(c_ik_sol_2_theta_1 + c_ik_sol_2_theta_2)+m_pos_x
    % Y = link_1*sin(c_ik_sol_2_theta_1) + link_2*sin(c_ik_sol_2_theta_1 + c_ik_sol_2_theta_2)+m_pos_y
    % scatter(m_pos_x + link_1*cos(c_ik_sol_2_theta_1) + link_2*cos(c_ik_sol_2_theta_1 + c_ik_sol_2_theta_2), m_pos_y + link_1*sin(c_ik_sol_2_theta_1) + link_2*sin(c_ik_sol_2_theta_1 + c_ik_sol_2_theta_2), 'cyan', 'filled');
    
    
    %%%%%%%%%%%%%%% Goal Position IK %%%%%%%%%%%%%%%%%%%%%%
    %%%%% Inverse kinematics Solution 1 %%%%%%%%%%%%
    cos_alpha = (-((c_pos_x - m_pos_x)^2 + (c_pos_y - m_pos_y)^2) + link_1^2 + link_2^2) / (2*link_1*link_2);    % () of (2*link_1*link_2) is necessary
    alpha = atan2(sqrt(1 - cos_alpha*cos_alpha), cos_alpha);
    %alpha = atan2(sqrt(1-((-(c_pos_x - m_pos_x)^2 - (c_pos_y - m_pos_y)^2 + link_1^2 + link_2^2) / 2*link_1*link_2)^2), (-(c_pos_x - m_pos_x)^2 - (c_pos_y - m_pos_y)^2 + link_1^2 + link_2^2) / 2*link_1*link_2);
    g_ik_sol_1_theta_2 = pi - alpha;
    g_ik_sol_1_theta_1 = atan2((c_pos_y - m_pos_y), (c_pos_x - m_pos_x)) - atan2((link_2*sin(g_ik_sol_1_theta_2)), (link_1 + link_2*cos(g_ik_sol_1_theta_2)));
    %g_ik_sol_1_theta_1 = atan2((c_pos_y - m_pos_y), (c_pos_x - m_pos_x)) - atan2(link_2*sin(g_ik_sol_1_theta_2), link_1 + link_2*cos(g_ik_sol_1_theta_2))
    
    % % link_1 plot
    % joint_x1 = [m_pos_x m_pos_x + link_1*cos(g_ik_sol_1_theta_1)]
    % joint_y1 = [m_pos_y m_pos_y + link_1*sin(g_ik_sol_1_theta_1)]
    % plot(joint_x1,joint_y1,'blue')
    %
    % % link_2 plot
    % joint_x2 = [m_pos_x + link_1*cos(g_ik_sol_1_theta_1) m_pos_x + link_1*cos(g_ik_sol_1_theta_1) + link_2*cos(g_ik_sol_1_theta_1 + g_ik_sol_1_theta_2)]
    % joint_y2 = [m_pos_y + link_1*sin(g_ik_sol_1_theta_1) m_pos_y + link_1*sin(g_ik_sol_1_theta_1) + link_2*sin(g_ik_sol_1_theta_1 + g_ik_sol_1_theta_2)]
    % plot(joint_x2,joint_y2,'blue')
    
    
    %%%%% Inverse kinematics Solution 2 %%%%%%%%%%%%
    cos_alpha = (-((c_pos_x - m_pos_x)^2 + (c_pos_y - m_pos_y)^2) + link_1^2 + link_2^2) / (2*link_1*link_2);    % () of (2*link_1*link_2) is necessary
    alpha = - atan2(sqrt(1 - cos_alpha*cos_alpha), cos_alpha);
    %alpha = atan2(sqrt(1-((-(c_pos_x - m_pos_x)^2 - (c_pos_y - m_pos_y)^2 + link_1^2 + link_2^2) / 2*link_1*link_2)^2), (-(c_pos_x - m_pos_x)^2 - (c_pos_y - m_pos_y)^2 + link_1^2 + link_2^2) / 2*link_1*link_2);
    g_ik_sol_2_theta_2 = pi - alpha;
    g_ik_sol_2_theta_1 = atan2((c_pos_y - m_pos_y), (c_pos_x - m_pos_x)) - atan2((link_2*sin(g_ik_sol_2_theta_2)), (link_1 + link_2*cos(g_ik_sol_2_theta_2)));
    %g_ik_sol_2_theta_1 = atan2((c_pos_y - m_pos_y), (c_pos_x - m_pos_x)) - atan2(link_2*sin(g_ik_sol_2_theta_2), link_1 + link_2*cos(g_ik_sol_2_theta_2))
    
    % % link_1 plot
    % joint_x1 = [m_pos_x m_pos_x + link_1*cos(g_ik_sol_2_theta_1)]
    % joint_y1 = [m_pos_y m_pos_y + link_1*sin(g_ik_sol_2_theta_1)]
    % plot(joint_x1,joint_y1,'blue')
    %
    % % link_2 plot
    % joint_x2 = [m_pos_x + link_1*cos(g_ik_sol_2_theta_1) m_pos_x + link_1*cos(g_ik_sol_2_theta_1) + link_2*cos(g_ik_sol_2_theta_1 + g_ik_sol_2_theta_2)]
    % joint_y2 = [m_pos_y + link_1*sin(g_ik_sol_2_theta_1) m_pos_y + link_1*sin(g_ik_sol_2_theta_1) + link_2*sin(g_ik_sol_2_theta_1 + g_ik_sol_2_theta_2)]
    % plot(joint_x2,joint_y2,'blue')
    
    % % Forward kinematics (For confirmation)
    % X = link_1*cos(g_ik_sol_2_theta_1) + link_2*cos(g_ik_sol_2_theta_1 + g_ik_sol_2_theta_2)+m_pos_x
    % Y = link_1*sin(g_ik_sol_2_theta_1) + link_2*sin(g_ik_sol_2_theta_1 + g_ik_sol_2_theta_2)+m_pos_y
    % scatter(m_pos_x + link_1*cos(g_ik_sol_2_theta_1) + link_2*cos(g_ik_sol_2_theta_1 + g_ik_sol_2_theta_2), m_pos_y + link_1*sin(g_ik_sol_2_theta_1) + link_2*sin(g_ik_sol_2_theta_1 + g_ik_sol_2_theta_2), 'cyan', 'filled');
    
    % convert to 0 ~ 360 deg
    if c_ik_sol_1_theta_1 < 0
        c_ik_sol_1_theta_1 = 2*pi -abs(c_ik_sol_1_theta_1);
    end
    if c_ik_sol_1_theta_2 < 0
        c_ik_sol_1_theta_2 = 2*pi -abs(c_ik_sol_1_theta_2);
    end
    if c_ik_sol_2_theta_1 < 0
        c_ik_sol_2_theta_1 = 2*pi -abs(c_ik_sol_2_theta_1);
    end
    if c_ik_sol_2_theta_2 < 0
        c_ik_sol_2_theta_2 = 2*pi -abs(c_ik_sol_2_theta_2);
    end
    
    if g_ik_sol_1_theta_1 < 0
        g_ik_sol_1_theta_1 = 2*pi -abs(g_ik_sol_1_theta_1);
    end
    if g_ik_sol_1_theta_2 < 0
        g_ik_sol_1_theta_2 = 2*pi -abs(g_ik_sol_1_theta_2);
    end
    if g_ik_sol_2_theta_1 < 0
        g_ik_sol_2_theta_1 = 2*pi -abs(g_ik_sol_2_theta_1);
    end
    if g_ik_sol_2_theta_2 < 0
        g_ik_sol_2_theta_2 = 2*pi -abs(g_ik_sol_2_theta_2);
    end
    
    
    
    
    %     %%%%%%%%%%%%%%% The initial path of the arm (Min-Max)%%%%%%%%%%%%%%%%%%%%%%
    %     %%%%% From 1 to 1 %%%%%%%%%%%%
    %     a_11_delta_theta_1 = g_ik_sol_1_theta_1 - c_ik_sol_1_theta_1;
    %     while a_11_delta_theta_1 > 2*pi    % Smaller than 2*pi
    %         a_11_delta_theta_1 = a_11_delta_theta_1 - 2*pi
    %     end
    %
    %     b_11_delta_theta_1 = 2*pi - abs(a_11_delta_theta_1);
    %
    %     a_11_delta_theta_2 = abs(g_ik_sol_1_theta_2 - c_ik_sol_1_theta_2);
    %     % If a_delta_theta_2 is outside the range of axis J2
    % %     if a_11_delta_theta_2 > 2*j2_max
    % %         a_11_delta_theta_2 = abs(2*pi - a_11_delta_theta_2);
    % %     end
    %
    %
    %     from_1_to_1 = [a_11_delta_theta_1 b_11_delta_theta_1 a_11_delta_theta_2];
    %     max_1 = max(from_1_to_1);
    %
    %     %%%%% From 1 to 2 %%%%%%%%%%%%
    %     a_12_delta_theta_1 = g_ik_sol_2_theta_1 - c_ik_sol_1_theta_1;
    %     while a_12_delta_theta_1 > 2*pi    % Smaller than 2*pi
    %         a_12_delta_theta_1 = a_12_delta_theta_1 - 2*pi
    %     end
    %
    %     b_12_delta_theta_1 = 2*pi - abs(a_12_delta_theta_1);
    %
    %     a_12_delta_theta_2 = abs(g_ik_sol_2_theta_2 - c_ik_sol_1_theta_2);
    %     % If a_delta_theta_2 is outside the range of axis J2
    % %     if a_12_delta_theta_2 > 2*j2_max
    % %         a_12_delta_theta_2 = abs(2*pi - a_12_delta_theta_2);
    % %     end
    %
    %     from_1_to_2 = [a_12_delta_theta_1 b_12_delta_theta_1 a_12_delta_theta_2];
    %     max_2 = max(from_1_to_2);
    %
    %     %%%%% From 2 to 1 %%%%%%%%%%%%
    %     a_21_delta_theta_1 = g_ik_sol_1_theta_1 - c_ik_sol_2_theta_1;
    %     while a_21_delta_theta_1 > 2*pi    % Smaller than 2*pi
    %         a_21_delta_theta_1 = a_21_delta_theta_1 - 2*pi
    %     end
    %
    %     b_21_delta_theta_1 = 2*pi - abs(a_21_delta_theta_1);
    %
    %     a_21_delta_theta_2 = abs(g_ik_sol_1_theta_2 - c_ik_sol_2_theta_2);
    %     % If a_delta_theta_2 is outside the range of axis J2
    % %     if a_21_delta_theta_2 > 2*j2_max
    % %         a_21_delta_theta_2 = abs(2*pi - a_21_delta_theta_2);
    % %     end
    %
    %     from_2_to_1 = [a_21_delta_theta_1 b_21_delta_theta_1 a_21_delta_theta_2];
    %     max_3 = max(from_2_to_1);
    %
    %     %%%%% From 2 to 2 %%%%%%%%%%%%
    %     a_22_delta_theta_1 = g_ik_sol_2_theta_1 - c_ik_sol_2_theta_1;
    %     while a_22_delta_theta_1 > 2*pi    % Smaller than 2*pi
    %         a_22_delta_theta_1 = a_22_delta_theta_1 - 2*pi
    %     end
    %
    %     b_22_delta_theta_1 = 2*pi - abs(a_22_delta_theta_1);
    %
    %     a_22_delta_theta_2 = abs(g_ik_sol_2_theta_2 - c_ik_sol_2_theta_2);
    %     % If a_delta_theta_2 is outside the range of axis J2
    % %     if a_22_delta_theta_2 > 2*j2_max
    % %         a_22_delta_theta_2 = abs(2*pi - a_22_delta_theta_2);
    % %     end
    %
    %     from_2_to_2 = [a_22_delta_theta_1 b_22_delta_theta_1 a_22_delta_theta_2];
    %     max_4 = max(from_2_to_2);
    %
    %
    %     min_angle = min([max_1 max_2 max_3 max_4]);
    %
    %
    
    %%%%%%%%%%%%%%% The initial path of the arm %%%%%%%%%%%%%%%%%%%%%%
    %%%%% From 1 to 1 %%%%%%%%%%%%
    a_11_delta_theta_1 = g_ik_sol_1_theta_1 - c_ik_sol_1_theta_1;
    while a_11_delta_theta_1 > 2*pi    % Smaller than 2*pi
        a_11_delta_theta_1 = a_11_delta_theta_1 - 2*pi
    end
    
    b_11_delta_theta_1 = 2*pi - abs(a_11_delta_theta_1);
    
    a_11_delta_theta_2 = abs(g_ik_sol_1_theta_2 - c_ik_sol_1_theta_2);
    % If a_delta_theta_2 is outside the range of axis J2
    %     if a_11_delta_theta_2 > 2*j2_max
    %         a_11_delta_theta_2 = abs(2*pi - a_11_delta_theta_2);
    %     end
    
    from_1_to_1_a = [abs(a_11_delta_theta_1) abs(a_11_delta_theta_2)];
    from_1_to_1_b = [abs(b_11_delta_theta_1) abs(a_11_delta_theta_2)];
    max_1 = max(from_1_to_1_a);
    max_2 = max(from_1_to_1_b);
    
    %%%%% From 1 to 2 %%%%%%%%%%%%
    a_12_delta_theta_1 = g_ik_sol_2_theta_1 - c_ik_sol_1_theta_1;
    while a_12_delta_theta_1 > 2*pi    % Smaller than 2*pi
        a_12_delta_theta_1 = a_12_delta_theta_1 - 2*pi
    end
    
    b_12_delta_theta_1 = 2*pi - abs(a_12_delta_theta_1);
    
    a_12_delta_theta_2 = abs(g_ik_sol_2_theta_2 - c_ik_sol_1_theta_2);
    % If a_delta_theta_2 is outside the range of axis J2
    %     if a_12_delta_theta_2 > 2*j2_max
    %         a_12_delta_theta_2 = abs(2*pi - a_12_delta_theta_2);
    %     end
    
    from_1_to_2_a = [abs(a_12_delta_theta_1) abs(a_12_delta_theta_2)];
    from_1_to_2_b = [abs(b_12_delta_theta_1) abs(a_12_delta_theta_2)];
    max_3 = max(from_1_to_2_a);
    max_4 = max(from_1_to_2_b);
    
    
    %%%%% From 2 to 1 %%%%%%%%%%%%
    a_21_delta_theta_1 = g_ik_sol_1_theta_1 - c_ik_sol_2_theta_1;
    while a_21_delta_theta_1 > 2*pi    % Smaller than 2*pi
        a_21_delta_theta_1 = a_21_delta_theta_1 - 2*pi
    end
    
    b_21_delta_theta_1 = 2*pi - abs(a_21_delta_theta_1);
    
    a_21_delta_theta_2 = abs(g_ik_sol_1_theta_2 - c_ik_sol_2_theta_2);
    % If a_delta_theta_2 is outside the range of axis J2
    %     if a_21_delta_theta_2 > 2*j2_max
    %         a_21_delta_theta_2 = abs(2*pi - a_21_delta_theta_2);
    %     end
    
    from_2_to_1_a = [abs(a_21_delta_theta_1) abs(a_21_delta_theta_2)];
    from_2_to_1_b = [abs(b_21_delta_theta_1) abs(a_21_delta_theta_2)];
    max_5 = max(from_2_to_1_a);
    max_6 = max(from_2_to_1_b);
    
    %%%%% From 2 to 2 %%%%%%%%%%%%
    a_22_delta_theta_1 = g_ik_sol_2_theta_1 - c_ik_sol_2_theta_1;
    while a_22_delta_theta_1 > 2*pi    % Smaller than 2*pi
        a_22_delta_theta_1 = a_22_delta_theta_1 - 2*pi
    end
    
    b_22_delta_theta_1 = 2*pi - abs(a_22_delta_theta_1);
    
    a_22_delta_theta_2 = abs(g_ik_sol_2_theta_2 - c_ik_sol_2_theta_2);
    % If a_delta_theta_2 is outside the range of axis J2
    %     if a_22_delta_theta_2 > 2*j2_max
    %         a_22_delta_theta_2 = abs(2*pi - a_22_delta_theta_2);
    %     end
    
    from_2_to_2_a = [abs(a_22_delta_theta_1) abs(a_22_delta_theta_2)];
    from_2_to_2_b = [abs(b_22_delta_theta_1) abs(a_22_delta_theta_2)];
    max_7 = max(from_2_to_2_a);
    max_8 = max(from_2_to_2_b);
    
    min_angle = min([max_1 max_2 max_3 max_4 max_5 max_6 max_7 max_8]);
    
    
    
    
    
    %%%%% smallest angle (Min-Max)%%%%%%%%%%%%
    if num_t == max_num_t    % Run only the first time
        %         while min_angle > 2*pi    % Smaller than 2*pi
        %             min_angle = min_angle - 2*pi
        %         end
        
        best_min_angle = min_angle
       
        
        best_min_angle_array(1,1) = rad2deg(best_min_angle);
        %     best_min_angle_array(num_t,1) = best_min_angle;
%         csvwrite(strcat(max_num_t_str,file_name_csv),best_min_angle)
        
    elseif best_min_angle > min_angle    % update smallest angle
        best_min_angle = min_angle;
       
        best_p_pos_x = p_pos_x;
        best_p_pos_y = p_pos_y;
        best_m_pos_x = m_pos_x;
        best_m_pos_y = m_pos_y;
    end
    
    best_min_angle_array(max_num_t + 1 - num_t,1) = rad2deg(best_min_angle);
    
%     gomi='gosssssssssssssssssssssssssssssssssssssssssssssssssssmi'
%     rem(max_num_t + 1 - num_t,interval + 1)
%     if rem(10000,5000) == 0
%         best_min_angle_array(max_num_t + 1 - num_t,1) = rad2deg(best_min_angle);
%     end
    
    
    if num_t == max_num_t    % Run only the first time
        %%%%%%%%%%%%%%% Arm posture %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%% From 1 to 1 %%%%%%%%%%%%
        if best_min_angle == abs(a_11_delta_theta_1)
            c_initial_theta_1 = c_ik_sol_1_theta_1
            c_initial_theta_2 = c_ik_sol_1_theta_2
            g_initial_theta_1 = g_ik_sol_1_theta_1
            g_initial_theta_2 = g_ik_sol_1_theta_2
            %             direction = 'cw'
            %             path = 'a From 1 to 1'
            
        elseif best_min_angle == abs(b_11_delta_theta_1)
            c_initial_theta_1 = c_ik_sol_1_theta_1
            c_initial_theta_2 = c_ik_sol_1_theta_2
            g_initial_theta_1 = g_ik_sol_1_theta_1
            g_initial_theta_2 = g_ik_sol_1_theta_2
            %             direction = 'ccw'
            %             path = 'b From 1 to 1'
            
            
        elseif best_min_angle == abs(a_11_delta_theta_2)
            c_initial_theta_1 = c_ik_sol_1_theta_1
            c_initial_theta_2 = c_ik_sol_1_theta_2
            g_initial_theta_1 = g_ik_sol_1_theta_1
            g_initial_theta_2 = g_ik_sol_1_theta_2
            
            %             if a_11_delta_theta_1 > b_11_delta_theta_1    % Select small theta_1, maybe regardless of result
            %                 direction = 'ccw'
            %                 path = 'b From 1 to 1'
            %             else
            %                 initial_theta_1 = a_11_delta_theta_1
            %                 direction = 'cw'
            %                 path = 'a From 1 to 1'
            %             end
            
            %%%%% From 1 to 2 %%%%%%%%%%%%
        elseif best_min_angle == abs(a_12_delta_theta_1)
            c_initial_theta_1 = c_ik_sol_1_theta_1
            c_initial_theta_2 = c_ik_sol_1_theta_2
            g_initial_theta_1 = g_ik_sol_2_theta_1
            g_initial_theta_2 = g_ik_sol_2_theta_2
            %             direction = 'cw'
            %             path = 'a From 1 to 2'
            
        elseif best_min_angle == abs(b_12_delta_theta_1)
            c_initial_theta_1 = c_ik_sol_1_theta_1
            c_initial_theta_2 = c_ik_sol_1_theta_2
            g_initial_theta_1 = g_ik_sol_2_theta_1
            g_initial_theta_2 = g_ik_sol_2_theta_2
            %             direction = 'ccw'
            %             path = 'b From 1 to 2'
            
        elseif best_min_angle == abs(a_12_delta_theta_2)
            c_initial_theta_1 = c_ik_sol_1_theta_1
            c_initial_theta_2 = c_ik_sol_1_theta_2
            g_initial_theta_1 = g_ik_sol_2_theta_1
            g_initial_theta_2 = g_ik_sol_2_theta_2
            
            %             if a_12_delta_theta_1 > b_12_delta_theta_1    % Select small theta_1, maybe regardless of result
            %                 direction = 'ccw'
            %                 path = 'b From 1 to 2'
            %             else
            %                 direction = 'cw'
            %                 path = 'a From 1 to 2'
            %             end
            
            %%%%% From 2 to 1 %%%%%%%%%%%%
        elseif best_min_angle == abs(a_21_delta_theta_1)
            c_initial_theta_1 = c_ik_sol_2_theta_1
            c_initial_theta_2 = c_ik_sol_2_theta_2
            g_initial_theta_1 = g_ik_sol_1_theta_1
            g_initial_theta_2 = g_ik_sol_1_theta_2
            %             direction = 'cw'
            %             path = 'a From 2 to 1'
            
        elseif best_min_angle == abs(b_21_delta_theta_1)
            c_initial_theta_1 = c_ik_sol_2_theta_1
            c_initial_theta_2 = c_ik_sol_2_theta_2
            g_initial_theta_1 = g_ik_sol_1_theta_1
            g_initial_theta_2 = g_ik_sol_1_theta_2
            %             direction = 'ccw'
            %             path = 'b From 2 to 1'
            
        elseif best_min_angle == abs(a_21_delta_theta_2)
            c_initial_theta_1 = c_ik_sol_2_theta_1
            c_initial_theta_2 = c_ik_sol_2_theta_2
            g_initial_theta_1 = g_ik_sol_1_theta_1
            g_initial_theta_2 = g_ik_sol_1_theta_2
            
            %             if a_21_delta_theta_1 > b_21_delta_theta_1    % Select small theta_1, maybe regardless of result
            %                 direction = 'ccw'
            %                 path = 'b From 2 to 1'
            %             else
            %                 direction = 'cw'
            %                 path = 'a From 2 to 1'
            %             end
            
            %%%%% From 2 to 2 %%%%%%%%%%%%
        elseif best_min_angle == abs(a_22_delta_theta_1)
            c_initial_theta_1 = c_ik_sol_2_theta_1
            c_initial_theta_2 = c_ik_sol_2_theta_2
            g_initial_theta_1 = g_ik_sol_2_theta_1
            g_initial_theta_2 = g_ik_sol_2_theta_2
            %             direction = 'cw'
            %             path = 'a From 2 to 2'
            
        elseif best_min_angle == abs(b_22_delta_theta_1)
            c_initial_theta_1 = c_ik_sol_2_theta_1
            c_initial_theta_2 = c_ik_sol_2_theta_2
            g_initial_theta_1 = g_ik_sol_2_theta_1
            g_initial_theta_2 = g_ik_sol_2_theta_2
            %             direction = 'ccw'
            %             path = 'b From 2 to 2'
            
        elseif best_min_angle == abs(a_22_delta_theta_2)
            c_initial_theta_1 = c_ik_sol_2_theta_1
            c_initial_theta_2 = c_ik_sol_2_theta_2
            g_initial_theta_1 = g_ik_sol_2_theta_1
            g_initial_theta_2 = g_ik_sol_2_theta_2
            
            %             if a_22_delta_theta_1 > b_22_delta_theta_1    % Select small theta_1, maybe regardless of result
            %                 direction = 'ccw'
            %                 path = 'b From 2 to 2'
            %             else
            %                 direction = 'cw'
            %                 path = 'a From 2 to 2'
            %             end
        else
            gomigomi = 'dddddddddddddddddddddddddddddddddddddddddddddddddd'
        end
        
        
        c_initial_theta_1
        c_initial_theta_2
        g_initial_theta_1
        g_initial_theta_2
%         gomi
        
        % J1 axis rotation direction
        if ((g_initial_theta_1 - c_initial_theta_1) >= 0)
            if abs(g_initial_theta_1 - c_initial_theta_1) <= pi
                j1_direction = 'ccw'
            else
                j1_direction = 'cw'
            end
        elseif ((g_initial_theta_1 - c_initial_theta_1) <= 0)
            if abs(g_initial_theta_1 - c_initial_theta_1) <= pi
                j1_direction = 'cw'
            else
                j1_direction = 'ccw'
            end
        end
        
        % J2 axis rotation direction
        if ((g_initial_theta_2 - c_initial_theta_2) >= 0)
            if abs(g_initial_theta_2 - c_initial_theta_2) <= pi
                j2_direction = 'ccw'
            else
                j2_direction = 'cw'
            end
        elseif ((g_initial_theta_2 - c_initial_theta_2) <= 0)
            if abs(g_initial_theta_2 - c_initial_theta_2) <= pi
                j2_direction = 'cw'
            else
                j2_direction = 'ccw'
            end
        end
        
        %         % J2 axis rotation direction
        %         if ((g_initial_theta_2 - c_initial_theta_2) > 0) && ((g_initial_theta_2 - c_initial_theta_2) < 2*pi - abs(g_initial_theta_2 - c_initial_theta_2))
        %             j2_direction = 'ccw'
        %         else
        %             j2_direction = 'cw'
        %         end
        %
        %         if ((g_initial_theta_2 - c_initial_theta_2) < 0) && ((g_initial_theta_2 - c_initial_theta_2) < 2*pi - abs(g_initial_theta_2 - c_initial_theta_2))
        %             j2_direction = 'cw'
        %         end
        
        
        %         c1 = rad2deg(c_initial_theta_1)
        %         c2 = rad2deg(c_initial_theta_2)
        %         g1 = rad2deg(g_initial_theta_1)
        %         g2 = rad2deg(g_initial_theta_2)
        
        %%%%%%%%%%%%%%% Arm plot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%% Current initial arm %%%%%%%%%%%%
        % link_1 plot
        joint_x1 = [m_pos_x m_pos_x + link_1*cos(c_initial_theta_1)];
        joint_y1 = [m_pos_y m_pos_y + link_1*sin(c_initial_theta_1)];
        plot(joint_x1,joint_y1,'Color',orange,'LineWidth',1.5)
        
        % link_2 plot
        joint_x2 = [m_pos_x + link_1*cos(c_initial_theta_1) m_pos_x + link_1*cos(c_initial_theta_1) + link_2*cos(c_initial_theta_1 + c_initial_theta_2)];
        joint_y2 = [m_pos_y + link_1*sin(c_initial_theta_1) m_pos_y + link_1*sin(c_initial_theta_1) + link_2*sin(c_initial_theta_1 + c_initial_theta_2)];
        plot(joint_x2,joint_y2,'Color',orange,'LineWidth',1.5)
        
        %%%%% Goal initial arm %%%%%%%%%%%%
        % link_1 plot
        joint_x1 = [m_pos_x m_pos_x + link_1*cos(g_initial_theta_1)];
        joint_y1 = [m_pos_y m_pos_y + link_1*sin(g_initial_theta_1)];
        plot(joint_x1,joint_y1,'Color',orange,'LineWidth',1.5)
        
        % link_2 plot
        joint_x2 = [m_pos_x + link_1*cos(g_initial_theta_1) m_pos_x + link_1*cos(g_initial_theta_1) + link_2*cos(g_initial_theta_1 + g_initial_theta_2)];
        joint_y2 = [m_pos_y + link_1*sin(g_initial_theta_1) m_pos_y + link_1*sin(g_initial_theta_1) + link_2*sin(g_initial_theta_1 + g_initial_theta_2)];
        plot(joint_x2,joint_y2,'Color',orange,'LineWidth',1.5)
        
        % Axis of best_min_angle
        if (best_min_angle == a_11_delta_theta_2) || (best_min_angle == a_12_delta_theta_2) || (best_min_angle == a_21_delta_theta_2) || (best_min_angle == a_22_delta_theta_2)
            best_angle_axis = 'J2';
        else
            best_angle_axis = 'J1';
        end
        
        % Annotation of graph
        dim = [0.76 0.2 0.3 0.3];
        best_min_angle_deg = rad2deg(best_min_angle)
        round_best_min_angle = num2str(round(best_min_angle_deg,6))
        str = {strcat('J1 axis:',j1_direction),strcat('J2 axis:',j2_direction),strcat(best_angle_axis),strcat(round_best_min_angle,'[deg]')};
        annotation('textbox',dim,'FontSize',15,'String',str,'FitBoxToText','on')
        
        % save graph
        %date = datetime;
        set(gca,'FontSize',15);
        ylabel('y','FontSize',24)
        xlabel('x','FontSize',24)
        
        %         annotation('textbox','String',direction,'FitBoxToText','on')
        time = datestr(date,'yyyymmdd_HHMMSS');
        file_name = strcat(time,'_');
        file_name = strcat(file_name,id_num_st);
        file_name = strcat(file_name,'_InitialPosture.pdf');
        saveas(gcf,file_name)
    end
    
    
    num_t = num_t - 1;
    
end


close

%%%%%%%%%%%%%%% Solution plot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Draw palette range %%%%%%%%%%%%%%%%%%%%%%%%%%
axis([-2000 2000 -2000 2000])
pbaspect([1 1 1])

% center
scatter(best_p_pos_x, best_p_pos_y, 'red', 'filled');
hold on;

% horizontal line
x = [best_p_pos_x - 1/2*pl best_p_pos_x + 1/2*pl];
y1 = [best_p_pos_y - 1/2*pl best_p_pos_y - 1/2*pl];
y2 = [best_p_pos_y + 1/2*pl best_p_pos_y + 1/2*pl];
plot(x,y1,'red','LineWidth',2.0)
plot(x,y2,'red','LineWidth',2.0)

% vertical line
x1 = [best_p_pos_x - 1/2*pl best_p_pos_x - 1/2*pl];
x2 = [best_p_pos_x + 1/2*pl best_p_pos_x + 1/2*pl];
y = [best_p_pos_y - 1/2*pl best_p_pos_y + 1/2*pl];
plot(x1,y,'red','LineWidth',2.0)
plot(x2,y,'red','LineWidth',2.0)


% Draw conveyor range %%%%%%%%%%%%%%%%%%%%%%%%%%
axis([-2000 2000 -2000 2000])
pbaspect([1 1 1])

% center
scatter(c_pos_x, c_pos_y, 'green', 'filled');
hold on;

% horizontal line
x = [c_pos_x - 1/2*cl_s c_pos_x + 1/2*cl_s];
y1 = [c_pos_y - 1/2*cl_l c_pos_y - 1/2*cl_l];
y2 = [c_pos_y + 1/2*cl_l c_pos_y + 1/2*cl_l];
plot(x,y1,'green','LineWidth',2.0)
plot(x,y2,'green','LineWidth',2.0)

% vertical line
x1 = [c_pos_x - 1/2*cl_s c_pos_x - 1/2*cl_s];
x2 = [c_pos_x + 1/2*cl_s c_pos_x + 1/2*cl_s];
y = [c_pos_y - 1/2*cl_l c_pos_y + 1/2*cl_l];
plot(x1,y,'green','LineWidth',2.0)
plot(x2,y,'green','LineWidth',2.0)


% Draw manipulator range %%%%%%%%%%%%%%%%%%%%%%%%%%
axis([-2000 2000 -2000 2000])
pbaspect([1 1 1])

% center
scatter(best_m_pos_x, best_m_pos_y, 'blue', 'filled');
hold on;

% horizontal line
x = [best_m_pos_x - 1/2*ml best_m_pos_x + 1/2*ml];
y1 = [best_m_pos_y - 1/2*ml best_m_pos_y - 1/2*ml];
y2 = [best_m_pos_y + 1/2*ml best_m_pos_y + 1/2*ml];
plot(x,y1,'blue','LineWidth',2.0)
plot(x,y2,'blue','LineWidth',2.0)

% vertical line
x1 = [best_m_pos_x - 1/2*ml best_m_pos_x - 1/2*ml];
x2 = [best_m_pos_x + 1/2*ml best_m_pos_x + 1/2*ml];
y = [best_m_pos_y - 1/2*ml best_m_pos_y + 1/2*ml];
plot(x1,y,'blue','LineWidth',2.0)
plot(x2,y,'blue','LineWidth',2.0)


% Manipulator movable range plot %%%%%%%%%%%%%%%%%%%%%%%%%%
t = linspace(0,2*pi,100);
cx = best_m_pos_x;   %center
cy = best_m_pos_y;
r = link_1 + link_2;    % radius
plot(r*sin(t)+cx,r*cos(t)+cy,'blue','LineWidth',2.0)
hold on;

plot(m_min_range*sin(t)+cx,m_min_range*cos(t)+cy,'blue','LineWidth',2.0)
%hold on;



%%%%%%%%%%%%%%% Current Position IK %%%%%%%%%%%%%%%%%%%%%%
%%%%% Inverse kinematics Solution 1 %%%%%%%%%%%%
cos_alpha = (-((best_p_pos_x - best_m_pos_x)^2 + (best_p_pos_y - best_m_pos_y)^2) + link_1^2 + link_2^2) / (2*link_1*link_2);    % () of (2*link_1*link_2) is necessary
alpha = atan2(sqrt(1 - cos_alpha*cos_alpha), cos_alpha);
%alpha = atan2(sqrt(1-((-(best_p_pos_x - best_m_pos_x)^2 - (best_p_pos_y - best_m_pos_y)^2 + link_1^2 + link_2^2) / 2*link_1*link_2)^2), (-(best_p_pos_x - best_m_pos_x)^2 - (best_p_pos_y - best_m_pos_y)^2 + link_1^2 + link_2^2) / 2*link_1*link_2);
c_ik_sol_1_theta_2 = pi - alpha;
c_ik_sol_1_theta_1 = atan2((best_p_pos_y - best_m_pos_y), (best_p_pos_x - best_m_pos_x)) - atan2((link_2*sin(c_ik_sol_1_theta_2)), (link_1 + link_2*cos(c_ik_sol_1_theta_2)));
%c_ik_sol_1_theta_1 = atan2((best_p_pos_y - best_m_pos_y), (best_p_pos_x - best_m_pos_x)) - atan2(link_2*sin(c_ik_sol_1_theta_2), link_1 + link_2*cos(c_ik_sol_1_theta_2))

%     % link_1 plot
%     joint_x1 = [best_m_pos_x best_m_pos_x + link_1*cos(c_ik_sol_1_theta_1)];
%     joint_y1 = [best_m_pos_y best_m_pos_y + link_1*sin(c_ik_sol_1_theta_1)];
%     plot(joint_x1,joint_y1,'blue')
%
%     % link_2 plot
%     joint_x2 = [best_m_pos_x + link_1*cos(c_ik_sol_1_theta_1) best_m_pos_x + link_1*cos(c_ik_sol_1_theta_1) + link_2*cos(c_ik_sol_1_theta_1 + c_ik_sol_1_theta_2)];
%     joint_y2 = [best_m_pos_y + link_1*sin(c_ik_sol_1_theta_1) best_m_pos_y + link_1*sin(c_ik_sol_1_theta_1) + link_2*sin(c_ik_sol_1_theta_1 + c_ik_sol_1_theta_2)];
%     plot(joint_x2,joint_y2,'blue')


%%%%% Inverse kinematics Solution 2 %%%%%%%%%%%%
cos_alpha = (-((best_p_pos_x - best_m_pos_x)^2 + (best_p_pos_y - best_m_pos_y)^2) + link_1^2 + link_2^2) / (2*link_1*link_2);    % () of (2*link_1*link_2) is necessary
alpha = - atan2(sqrt(1 - cos_alpha*cos_alpha), cos_alpha);
%alpha = atan2(sqrt(1-((-(best_p_pos_x - best_m_pos_x)^2 - (best_p_pos_y - best_m_pos_y)^2 + link_1^2 + link_2^2) / 2*link_1*link_2)^2), (-(best_p_pos_x - best_m_pos_x)^2 - (best_p_pos_y - best_m_pos_y)^2 + link_1^2 + link_2^2) / 2*link_1*link_2);
c_ik_sol_2_theta_2 = pi - alpha;
c_ik_sol_2_theta_1 = atan2((best_p_pos_y - best_m_pos_y), (best_p_pos_x - best_m_pos_x)) - atan2((link_2*sin(c_ik_sol_2_theta_2)), (link_1 + link_2*cos(c_ik_sol_2_theta_2)));
%c_ik_sol_2_theta_1 = atan2((best_p_pos_y - best_m_pos_y), (best_p_pos_x - best_m_pos_x)) - atan2(link_2*sin(c_ik_sol_2_theta_2), link_1 + link_2*cos(c_ik_sol_2_theta_2))

%     % link_1 plot
%     joint_x1 = [best_m_pos_x best_m_pos_x + link_1*cos(c_ik_sol_2_theta_1)];
%     joint_y1 = [best_m_pos_y best_m_pos_y + link_1*sin(c_ik_sol_2_theta_1)];
%     plot(joint_x1,joint_y1,'blue')
%
%     % link_2 plot
%     joint_x2 = [best_m_pos_x + link_1*cos(c_ik_sol_2_theta_1) best_m_pos_x + link_1*cos(c_ik_sol_2_theta_1) + link_2*cos(c_ik_sol_2_theta_1 + c_ik_sol_2_theta_2)];
%     joint_y2 = [best_m_pos_y + link_1*sin(c_ik_sol_2_theta_1) best_m_pos_y + link_1*sin(c_ik_sol_2_theta_1) + link_2*sin(c_ik_sol_2_theta_1 + c_ik_sol_2_theta_2)];
%     plot(joint_x2,joint_y2,'blue')

% % Forward kinematics (For confirmation)
% X = link_1*cos(c_ik_sol_2_theta_1) + link_2*cos(c_ik_sol_2_theta_1 + c_ik_sol_2_theta_2)+best_m_pos_x;
% Y = link_1*sin(c_ik_sol_2_theta_1) + link_2*sin(c_ik_sol_2_theta_1 + c_ik_sol_2_theta_2)+best_m_pos_y;
% scatter(best_m_pos_x + link_1*cos(c_ik_sol_2_theta_1) + link_2*cos(c_ik_sol_2_theta_1 + c_ik_sol_2_theta_2), best_m_pos_y + link_1*sin(c_ik_sol_2_theta_1) + link_2*sin(c_ik_sol_2_theta_1 + c_ik_sol_2_theta_2), 'cyan', 'filled');


%%%%%%%%%%%%%%% Goal Position IK %%%%%%%%%%%%%%%%%%%%%%
%%%%% Inverse kinematics Solution 1 %%%%%%%%%%%%
cos_alpha = (-((c_pos_x - best_m_pos_x)^2 + (c_pos_y - best_m_pos_y)^2) + link_1^2 + link_2^2) / (2*link_1*link_2);    % () of (2*link_1*link_2) is necessary
alpha = atan2(sqrt(1 - cos_alpha*cos_alpha), cos_alpha);
%alpha = atan2(sqrt(1-((-(c_pos_x - best_m_pos_x)^2 - (c_pos_y - best_m_pos_y)^2 + link_1^2 + link_2^2) / 2*link_1*link_2)^2), (-(c_pos_x - best_m_pos_x)^2 - (c_pos_y - best_m_pos_y)^2 + link_1^2 + link_2^2) / 2*link_1*link_2);
g_ik_sol_1_theta_2 = pi - alpha;
g_ik_sol_1_theta_1 = atan2((c_pos_y - best_m_pos_y), (c_pos_x - best_m_pos_x)) - atan2((link_2*sin(g_ik_sol_1_theta_2)), (link_1 + link_2*cos(g_ik_sol_1_theta_2)));
%g_ik_sol_1_theta_1 = atan2((c_pos_y - best_m_pos_y), (c_pos_x - best_m_pos_x)) - atan2(link_2*sin(g_ik_sol_1_theta_2), link_1 + link_2*cos(g_ik_sol_1_theta_2))

% % link_1 plot
% joint_x1 = [best_m_pos_x best_m_pos_x + link_1*cos(g_ik_sol_1_theta_1)]
% joint_y1 = [best_m_pos_y best_m_pos_y + link_1*sin(g_ik_sol_1_theta_1)]
% plot(joint_x1,joint_y1,'blue')
%
% % link_2 plot
% joint_x2 = [best_m_pos_x + link_1*cos(g_ik_sol_1_theta_1) best_m_pos_x + link_1*cos(g_ik_sol_1_theta_1) + link_2*cos(g_ik_sol_1_theta_1 + g_ik_sol_1_theta_2)]
% joint_y2 = [best_m_pos_y + link_1*sin(g_ik_sol_1_theta_1) best_m_pos_y + link_1*sin(g_ik_sol_1_theta_1) + link_2*sin(g_ik_sol_1_theta_1 + g_ik_sol_1_theta_2)]
% plot(joint_x2,joint_y2,'blue')


%%%%% Inverse kinematics Solution 2 %%%%%%%%%%%%
cos_alpha = (-((c_pos_x - best_m_pos_x)^2 + (c_pos_y - best_m_pos_y)^2) + link_1^2 + link_2^2) / (2*link_1*link_2);    % () of (2*link_1*link_2) is necessary
alpha = - atan2(sqrt(1 - cos_alpha*cos_alpha), cos_alpha);
%alpha = atan2(sqrt(1-((-(c_pos_x - best_m_pos_x)^2 - (c_pos_y - best_m_pos_y)^2 + link_1^2 + link_2^2) / 2*link_1*link_2)^2), (-(c_pos_x - best_m_pos_x)^2 - (c_pos_y - best_m_pos_y)^2 + link_1^2 + link_2^2) / 2*link_1*link_2);
g_ik_sol_2_theta_2 = pi - alpha;
g_ik_sol_2_theta_1 = atan2((c_pos_y - best_m_pos_y), (c_pos_x - best_m_pos_x)) - atan2((link_2*sin(g_ik_sol_2_theta_2)), (link_1 + link_2*cos(g_ik_sol_2_theta_2)));
%g_ik_sol_2_theta_1 = atan2((c_pos_y - best_m_pos_y), (c_pos_x - best_m_pos_x)) - atan2(link_2*sin(g_ik_sol_2_theta_2), link_1 + link_2*cos(g_ik_sol_2_theta_2))

% % link_1 plot
% joint_x1 = [best_m_pos_x best_m_pos_x + link_1*cos(g_ik_sol_2_theta_1)]
% joint_y1 = [best_m_pos_y best_m_pos_y + link_1*sin(g_ik_sol_2_theta_1)]
% plot(joint_x1,joint_y1,'blue')
%
% % link_2 plot
% joint_x2 = [best_m_pos_x + link_1*cos(g_ik_sol_2_theta_1) best_m_pos_x + link_1*cos(g_ik_sol_2_theta_1) + link_2*cos(g_ik_sol_2_theta_1 + g_ik_sol_2_theta_2)]
% joint_y2 = [best_m_pos_y + link_1*sin(g_ik_sol_2_theta_1) best_m_pos_y + link_1*sin(g_ik_sol_2_theta_1) + link_2*sin(g_ik_sol_2_theta_1 + g_ik_sol_2_theta_2)]
% plot(joint_x2,joint_y2,'blue')

% % Forward kinematics (For confirmation)
% X = link_1*cos(g_ik_sol_2_theta_1) + link_2*cos(g_ik_sol_2_theta_1 + g_ik_sol_2_theta_2)+best_m_pos_x
% Y = link_1*sin(g_ik_sol_2_theta_1) + link_2*sin(g_ik_sol_2_theta_1 + g_ik_sol_2_theta_2)+best_m_pos_y
% scatter(best_m_pos_x + link_1*cos(g_ik_sol_2_theta_1) + link_2*cos(g_ik_sol_2_theta_1 + g_ik_sol_2_theta_2), best_m_pos_y + link_1*sin(g_ik_sol_2_theta_1) + link_2*sin(g_ik_sol_2_theta_1 + g_ik_sol_2_theta_2), 'cyan', 'filled');

% convert to 0 ~ 360 deg
if c_ik_sol_1_theta_1 < 0
    c_ik_sol_1_theta_1 = 2*pi -abs(c_ik_sol_1_theta_1);
end
if c_ik_sol_1_theta_2 < 0
    c_ik_sol_1_theta_2 = 2*pi -abs(c_ik_sol_1_theta_2);
end
if c_ik_sol_2_theta_1 < 0
    c_ik_sol_2_theta_1 = 2*pi -abs(c_ik_sol_2_theta_1);
end
if c_ik_sol_2_theta_2 < 0
    c_ik_sol_2_theta_2 = 2*pi -abs(c_ik_sol_2_theta_2);
end

if g_ik_sol_1_theta_1 < 0
    g_ik_sol_1_theta_1 = 2*pi -abs(g_ik_sol_1_theta_1);
end
if g_ik_sol_1_theta_2 < 0
    g_ik_sol_1_theta_2 = 2*pi -abs(g_ik_sol_1_theta_2);
end
if g_ik_sol_2_theta_1 < 0
    g_ik_sol_2_theta_1 = 2*pi -abs(g_ik_sol_2_theta_1);
end
if g_ik_sol_2_theta_2 < 0
    g_ik_sol_2_theta_2 = 2*pi -abs(g_ik_sol_2_theta_2);
end



%   %%%%%%%%%%%%%%% The initial path of the arm %%%%%%%%%%%%%%%%%%%%%%
%     %%%%% From 1 to 1 %%%%%%%%%%%%
%     a_11_delta_theta_1 = g_ik_sol_1_theta_1 - c_ik_sol_1_theta_1;
%     while a_11_delta_theta_1 > 2*pi    % Smaller than 2*pi
%         a_11_delta_theta_1 = a_11_delta_theta_1 - 2*pi
%     end
%
%     b_11_delta_theta_1 = 2*pi - abs(a_11_delta_theta_1);
%
%     a_11_delta_theta_2 = abs(g_ik_sol_1_theta_2 - c_ik_sol_1_theta_2);
%     % If a_delta_theta_2 is outside the range of axis J2
% %     if a_11_delta_theta_2 > 2*j2_max
% %         a_11_delta_theta_2 = abs(2*pi - a_11_delta_theta_2);
% %     end
%
%     from_1_to_1 = [a_11_delta_theta_1 b_11_delta_theta_1 a_11_delta_theta_2];
%     max_1 = max(from_1_to_1);
%
%     %%%%% From 1 to 2 %%%%%%%%%%%%
%     a_12_delta_theta_1 = g_ik_sol_2_theta_1 - c_ik_sol_1_theta_1;
%     while a_12_delta_theta_1 > 2*pi    % Smaller than 2*pi
%         a_12_delta_theta_1 = a_12_delta_theta_1 - 2*pi
%     end
%
%     b_12_delta_theta_1 = 2*pi - abs(a_12_delta_theta_1);
%
%     a_12_delta_theta_2 = abs(g_ik_sol_2_theta_2 - c_ik_sol_1_theta_2);
%     % If a_delta_theta_2 is outside the range of axis J2
% %     if a_12_delta_theta_2 > 2*j2_max
% %         a_12_delta_theta_2 = abs(2*pi - a_12_delta_theta_2);
% %     end
%
%     from_1_to_2 = [a_12_delta_theta_1 b_12_delta_theta_1 a_12_delta_theta_2];
%     max_2 = max(from_1_to_2);
%
%     %%%%% From 2 to 1 %%%%%%%%%%%%
%     a_21_delta_theta_1 = g_ik_sol_1_theta_1 - c_ik_sol_2_theta_1;
%     while a_21_delta_theta_1 > 2*pi    % Smaller than 2*pi
%         a_21_delta_theta_1 = a_21_delta_theta_1 - 2*pi
%     end
%
%     b_21_delta_theta_1 = 2*pi - abs(a_21_delta_theta_1);
%
%     a_21_delta_theta_2 = abs(g_ik_sol_1_theta_2 - c_ik_sol_2_theta_2);
%     % If a_delta_theta_2 is outside the range of axis J2
% %     if a_21_delta_theta_2 > 2*j2_max
% %         a_21_delta_theta_2 = abs(2*pi - a_21_delta_theta_2);
% %     end
%
%     from_2_to_1 = [a_21_delta_theta_1 b_21_delta_theta_1 a_21_delta_theta_2];
%     max_3 = max(from_2_to_1);
%
%     %%%%% From 2 to 2 %%%%%%%%%%%%
%     a_22_delta_theta_1 = g_ik_sol_2_theta_1 - c_ik_sol_2_theta_1;
%     while a_22_delta_theta_1 > 2*pi    % Smaller than 2*pi
%         a_22_delta_theta_1 = a_22_delta_theta_1 - 2*pi
%     end
%
%     b_22_delta_theta_1 = 2*pi - abs(a_22_delta_theta_1);
%
%     a_22_delta_theta_2 = abs(g_ik_sol_2_theta_2 - c_ik_sol_2_theta_2);
%     % If a_delta_theta_2 is outside the range of axis J2
% %     if a_22_delta_theta_2 > 2*j2_max
% %         a_22_delta_theta_2 = abs(2*pi - a_22_delta_theta_2);
% %     end
%
%     from_2_to_2 = [a_22_delta_theta_1 b_22_delta_theta_1 a_22_delta_theta_2];
%     max_4 = max(from_2_to_2);
%
%     % min_angle = min([max_1 max_2 max_3 max_4]);




%%%%%%%%%%%%%%% The initial path of the arm %%%%%%%%%%%%%%%%%%%%%%
%%%%% From 1 to 1 %%%%%%%%%%%%
a_11_delta_theta_1 = g_ik_sol_1_theta_1 - c_ik_sol_1_theta_1;
while a_11_delta_theta_1 > 2*pi    % Smaller than 2*pi
    a_11_delta_theta_1 = a_11_delta_theta_1 - 2*pi
end

b_11_delta_theta_1 = 2*pi - abs(a_11_delta_theta_1);

a_11_delta_theta_2 = abs(g_ik_sol_1_theta_2 - c_ik_sol_1_theta_2);
% If a_delta_theta_2 is outside the range of axis J2
%     if a_11_delta_theta_2 > 2*j2_max
%         a_11_delta_theta_2 = abs(2*pi - a_11_delta_theta_2);
%     end

from_1_to_1_a = [abs(a_11_delta_theta_1) abs(a_11_delta_theta_2)];
from_1_to_1_b = [abs(b_11_delta_theta_1) abs(a_11_delta_theta_2)];
max_1 = max(from_1_to_1_a);
max_2 = max(from_1_to_1_b);

%%%%% From 1 to 2 %%%%%%%%%%%%
a_12_delta_theta_1 = g_ik_sol_2_theta_1 - c_ik_sol_1_theta_1;
while a_12_delta_theta_1 > 2*pi    % Smaller than 2*pi
    a_12_delta_theta_1 = a_12_delta_theta_1 - 2*pi
end

b_12_delta_theta_1 = 2*pi - abs(a_12_delta_theta_1);

a_12_delta_theta_2 = abs(g_ik_sol_2_theta_2 - c_ik_sol_1_theta_2);
% If a_delta_theta_2 is outside the range of axis J2
%     if a_12_delta_theta_2 > 2*j2_max
%         a_12_delta_theta_2 = abs(2*pi - a_12_delta_theta_2);
%     end

from_1_to_2_a = [abs(a_12_delta_theta_1) abs(a_12_delta_theta_2)];
from_1_to_2_b = [abs(b_12_delta_theta_1) abs(a_12_delta_theta_2)];
max_3 = max(from_1_to_2_a);
max_4 = max(from_1_to_2_b);


%%%%% From 2 to 1 %%%%%%%%%%%%
a_21_delta_theta_1 = g_ik_sol_1_theta_1 - c_ik_sol_2_theta_1;
while a_21_delta_theta_1 > 2*pi    % Smaller than 2*pi
    a_21_delta_theta_1 = a_21_delta_theta_1 - 2*pi
end

b_21_delta_theta_1 = 2*pi - abs(a_21_delta_theta_1);

a_21_delta_theta_2 = abs(g_ik_sol_1_theta_2 - c_ik_sol_2_theta_2);
% If a_delta_theta_2 is outside the range of axis J2
%     if a_21_delta_theta_2 > 2*j2_max
%         a_21_delta_theta_2 = abs(2*pi - a_21_delta_theta_2);
%     end

from_2_to_1_a = [abs(a_21_delta_theta_1) abs(a_21_delta_theta_2)];
from_2_to_1_b = [abs(b_21_delta_theta_1) abs(a_21_delta_theta_2)];
max_5 = max(from_2_to_1_a);
max_6 = max(from_2_to_1_b);

%%%%% From 2 to 2 %%%%%%%%%%%%
a_22_delta_theta_1 = g_ik_sol_2_theta_1 - c_ik_sol_2_theta_1;
while a_22_delta_theta_1 > 2*pi    % Smaller than 2*pi
    a_22_delta_theta_1 = a_22_delta_theta_1 - 2*pi
end

b_22_delta_theta_1 = 2*pi - abs(a_22_delta_theta_1);

a_22_delta_theta_2 = abs(g_ik_sol_2_theta_2 - c_ik_sol_2_theta_2);
% If a_delta_theta_2 is outside the range of axis J2
%     if a_22_delta_theta_2 > 2*j2_max
%         a_22_delta_theta_2 = abs(2*pi - a_22_delta_theta_2);
%     end

from_2_to_2_a = [abs(a_22_delta_theta_1) abs(a_22_delta_theta_2)];
from_2_to_2_b = [abs(b_22_delta_theta_1) abs(a_22_delta_theta_2)];
max_7 = max(from_2_to_2_a);
max_8 = max(from_2_to_2_b);

best_min_angle = min([max_1 max_2 max_3 max_4 max_5 max_6 max_7 max_8]);

%     sum = sum + best_min_angle;



%%%%%%%%%%%%%%% Arm posture %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% From 1 to 1 %%%%%%%%%%%%
if best_min_angle == abs(a_11_delta_theta_1)
    c_initial_theta_1 = c_ik_sol_1_theta_1
    c_initial_theta_2 = c_ik_sol_1_theta_2
    g_initial_theta_1 = g_ik_sol_1_theta_1
    g_initial_theta_2 = g_ik_sol_1_theta_2
    %             direction = 'cw'
    %             path = 'a From 1 to 1'
    
elseif best_min_angle == abs(b_11_delta_theta_1)
    c_initial_theta_1 = c_ik_sol_1_theta_1
    c_initial_theta_2 = c_ik_sol_1_theta_2
    g_initial_theta_1 = g_ik_sol_1_theta_1
    g_initial_theta_2 = g_ik_sol_1_theta_2
    %             direction = 'ccw'
    %             path = 'b From 1 to 1'
    
    
elseif best_min_angle == abs(a_11_delta_theta_2)
    c_initial_theta_1 = c_ik_sol_1_theta_1
    c_initial_theta_2 = c_ik_sol_1_theta_2
    g_initial_theta_1 = g_ik_sol_1_theta_1
    g_initial_theta_2 = g_ik_sol_1_theta_2
    
    %             if a_11_delta_theta_1 > b_11_delta_theta_1    % Select small theta_1, maybe regardless of result
    %                 direction = 'ccw'
    %                 path = 'b From 1 to 1'
    %             else
    %                 initial_theta_1 = a_11_delta_theta_1
    %                 direction = 'cw'
    %                 path = 'a From 1 to 1'
    %             end
    
    %%%%% From 1 to 2 %%%%%%%%%%%%
elseif best_min_angle == abs(a_12_delta_theta_1)
    c_initial_theta_1 = c_ik_sol_1_theta_1
    c_initial_theta_2 = c_ik_sol_1_theta_2
    g_initial_theta_1 = g_ik_sol_2_theta_1
    g_initial_theta_2 = g_ik_sol_2_theta_2
    %             direction = 'cw'
    %             path = 'a From 1 to 2'
    
elseif best_min_angle == abs(b_12_delta_theta_1)
    c_initial_theta_1 = c_ik_sol_1_theta_1
    c_initial_theta_2 = c_ik_sol_1_theta_2
    g_initial_theta_1 = g_ik_sol_2_theta_1
    g_initial_theta_2 = g_ik_sol_2_theta_2
    %             direction = 'ccw'
    %             path = 'b From 1 to 2'
    
elseif best_min_angle == abs(a_12_delta_theta_2)
    c_initial_theta_1 = c_ik_sol_1_theta_1
    c_initial_theta_2 = c_ik_sol_1_theta_2
    g_initial_theta_1 = g_ik_sol_2_theta_1
    g_initial_theta_2 = g_ik_sol_2_theta_2
    
    %             if a_12_delta_theta_1 > b_12_delta_theta_1    % Select small theta_1, maybe regardless of result
    %                 direction = 'ccw'
    %                 path = 'b From 1 to 2'
    %             else
    %                 direction = 'cw'
    %                 path = 'a From 1 to 2'
    %             end
    
    %%%%% From 2 to 1 %%%%%%%%%%%%
elseif best_min_angle == abs(a_21_delta_theta_1)
    c_initial_theta_1 = c_ik_sol_2_theta_1
    c_initial_theta_2 = c_ik_sol_2_theta_2
    g_initial_theta_1 = g_ik_sol_1_theta_1
    g_initial_theta_2 = g_ik_sol_1_theta_2
    %             direction = 'cw'
    %             path = 'a From 2 to 1'
    
elseif best_min_angle == abs(b_21_delta_theta_1)
    c_initial_theta_1 = c_ik_sol_2_theta_1
    c_initial_theta_2 = c_ik_sol_2_theta_2
    g_initial_theta_1 = g_ik_sol_1_theta_1
    g_initial_theta_2 = g_ik_sol_1_theta_2
    %             direction = 'ccw'
    %             path = 'b From 2 to 1'
    
elseif best_min_angle == abs(a_21_delta_theta_2)
    c_initial_theta_1 = c_ik_sol_2_theta_1
    c_initial_theta_2 = c_ik_sol_2_theta_2
    g_initial_theta_1 = g_ik_sol_1_theta_1
    g_initial_theta_2 = g_ik_sol_1_theta_2
    
    %             if a_21_delta_theta_1 > b_21_delta_theta_1    % Select small theta_1, maybe regardless of result
    %                 direction = 'ccw'
    %                 path = 'b From 2 to 1'
    %             else
    %                 direction = 'cw'
    %                 path = 'a From 2 to 1'
    %             end
    
    %%%%% From 2 to 2 %%%%%%%%%%%%
elseif best_min_angle == abs(a_22_delta_theta_1)
    c_initial_theta_1 = c_ik_sol_2_theta_1
    c_initial_theta_2 = c_ik_sol_2_theta_2
    g_initial_theta_1 = g_ik_sol_2_theta_1
    g_initial_theta_2 = g_ik_sol_2_theta_2
    %             direction = 'cw'
    %             path = 'a From 2 to 2'
    
elseif best_min_angle == abs(b_22_delta_theta_1)
    c_initial_theta_1 = c_ik_sol_2_theta_1
    c_initial_theta_2 = c_ik_sol_2_theta_2
    g_initial_theta_1 = g_ik_sol_2_theta_1
    g_initial_theta_2 = g_ik_sol_2_theta_2
    %             direction = 'ccw'
    %             path = 'b From 2 to 2'
    
elseif best_min_angle == abs(a_22_delta_theta_2)
    c_initial_theta_1 = c_ik_sol_2_theta_1
    c_initial_theta_2 = c_ik_sol_2_theta_2
    g_initial_theta_1 = g_ik_sol_2_theta_1
    g_initial_theta_2 = g_ik_sol_2_theta_2
    
    %             if a_22_delta_theta_1 > b_22_delta_theta_1    % Select small theta_1, maybe regardless of result
    %                 direction = 'ccw'
    %                 path = 'b From 2 to 2'
    %             else
    %                 direction = 'cw'
    %                 path = 'a From 2 to 2'
    %             end
else
    gomigomi = 'dddddddddddddddddddddddddddddddddddddddddddddddddd'
end




%  % J1 axis rotation direction
% if ((g_initial_theta_1 - c_initial_theta_1) > 0) && ((g_initial_theta_1 - c_initial_theta_1) < pi)
%     j1_direction = 'ccw'
% % else
% %     j1_direction = 'cw'
% end
%
% if ((g_initial_theta_1 - c_initial_theta_1) < 0) && ((g_initial_theta_1 - c_initial_theta_1) < pi)
%     j1_direction = 'cw'
% end


% J1 axis rotation direction
if ((g_initial_theta_1 - c_initial_theta_1) >= 0)
    if abs(g_initial_theta_1 - c_initial_theta_1) <= pi
        j1_direction = 'ccw'
    else
        j1_direction = 'cw'
    end
elseif ((g_initial_theta_1 - c_initial_theta_1) <= 0)
    if abs(g_initial_theta_1 - c_initial_theta_1) <= pi
        j1_direction = 'cw'
    else
        j1_direction = 'ccw'
    end
end


% J2 axis rotation direction
if ((g_initial_theta_2 - c_initial_theta_2) >= 0)
    if abs(g_initial_theta_2 - c_initial_theta_2) <= pi
        j2_direction = 'ccw'
    else
        j2_direction = 'cw'
    end
elseif ((g_initial_theta_2 - c_initial_theta_2) <= 0)
    if abs(g_initial_theta_2 - c_initial_theta_2) <= pi
        j2_direction = 'cw'
    else
        j2_direction = 'ccw'
    end
end

% % J2 axis rotation direction
% if ((g_initial_theta_2 - c_initial_theta_2) > 0) && ((g_initial_theta_2 - c_initial_theta_2) < pi)
%     j2_direction = 'ccw'
% else
%     j2_direction = 'cw'
% end
%
% if ((g_initial_theta_2 - c_initial_theta_2) < 0) && ((g_initial_theta_2 - c_initial_theta_2) < pi)
%     j2_direction = 'cw'
% end





c1 = rad2deg(c_initial_theta_1)
c2 = rad2deg(c_initial_theta_2)
g1 = rad2deg(g_initial_theta_1)
g2 = rad2deg(g_initial_theta_2)



%%%%%%%%%%%%%%% Arm plot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Current initial arm %%%%%%%%%%%%
% link_1 plot
joint_x1 = [best_m_pos_x best_m_pos_x + link_1*cos(c_initial_theta_1)];
joint_y1 = [best_m_pos_y best_m_pos_y + link_1*sin(c_initial_theta_1)];
plot(joint_x1,joint_y1,'Color',orange,'LineWidth',1.5)

% link_2 plot
joint_x2 = [best_m_pos_x + link_1*cos(c_initial_theta_1) best_m_pos_x + link_1*cos(c_initial_theta_1) + link_2*cos(c_initial_theta_1 + c_initial_theta_2)];
joint_y2 = [best_m_pos_y + link_1*sin(c_initial_theta_1) best_m_pos_y + link_1*sin(c_initial_theta_1) + link_2*sin(c_initial_theta_1 + c_initial_theta_2)];
plot(joint_x2,joint_y2,'Color',orange,'LineWidth',1.5)

%%%%% Goal initi0al arm %%%%%%%%%%%%
% link_1 plot
joint_x1 = [best_m_pos_x best_m_pos_x + link_1*cos(g_initial_theta_1)];
joint_y1 = [best_m_pos_y best_m_pos_y + link_1*sin(g_initial_theta_1)];
plot(joint_x1,joint_y1,'Color',orange,'LineWidth',1.5)

% link_2 plot
joint_x2 = [best_m_pos_x + link_1*cos(g_initial_theta_1) best_m_pos_x + link_1*cos(g_initial_theta_1) + link_2*cos(g_initial_theta_1 + g_initial_theta_2)];
joint_y2 = [best_m_pos_y + link_1*sin(g_initial_theta_1) best_m_pos_y + link_1*sin(g_initial_theta_1) + link_2*sin(g_initial_theta_1 + g_initial_theta_2)];
plot(joint_x2,joint_y2,'Color',orange,'LineWidth',1.5)

% Axis of best_min_angle
if (best_min_angle == a_11_delta_theta_2) || (best_min_angle == a_12_delta_theta_2) || (best_min_angle == a_21_delta_theta_2) || (best_min_angle == a_22_delta_theta_2)
    best_angle_axis = 'J2';
else
    best_angle_axis = 'J1';
end

% Timer stop
elapsedTime = toc


%%%%% Save csv %%%%%%%%%%%%
time = datestr(date,'yyyymmdd_HHMMSS');
file_name_csv = strcat(time,'_');
file_name_csv = strcat(file_name_csv,id_num_st);
file_name_csv = strcat(file_name_csv,'_');
max_num_t_str = num2str(max_num_t);

file_name_csv = strcat(file_name_csv,max_num_t_str);
file_name_csv = strcat(file_name_csv,'best_min_angle.csv');
csvwrite(file_name_csv,best_min_angle_array)


% Annotation of graph
dim = [0.76 0.2 0.3 0.3];
best_min_angle_deg = rad2deg(best_min_angle)
trials_number_str = num2str(max_num_t)
round_best_min_angle = num2str(round(best_min_angle_deg,6))
elapsedTime_str = num2str(elapsedTime)
str = {strcat('Trials:',trials_number_str),strcat('J1 axis:',j1_direction),strcat('J2 axis:',j2_direction),strcat(best_angle_axis),strcat(round_best_min_angle,'[deg]'),strcat(elapsedTime_str,'[sec]')};
annotation('textbox',dim,'FontSize',15,'String',str,'FitBoxToText','on')

% save Initial solution graph
set(gca,'FontSize',15);
ylabel('y','FontSize',24)
xlabel('x','FontSize',24)

time = datestr(date,'yyyymmdd_HHMMSS');
file_name = strcat(time,'_');
file_name = strcat(file_name,id_num_st);
file_name = strcat(file_name,'_Sol.pdf');
saveas(gcf,file_name)



% numnum = numnum - 1;
% end
%
% ave = rad2deg(sum / numnum_max)
