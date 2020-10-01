function [m_pos_random, p_pos_random, c_pos_random] = random_research()

% old file name "optim_test_1_random_minmax_with_spawn_and_grasp_2.m"
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

link_1 = 260; % manipulator link1 length
link_2 = 260; % manipulator link2 length

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

m_pos_z = 0;
p_pos_z = 0;
c_pos_z = 0;

m_pos_random = [m_pos_x, m_pos_y, m_pos_z];
p_pos_random = [p_pos_x, p_pos_y, p_pos_z];
c_pos_random = [c_pos_x, c_pos_y, c_pos_z];



in_pos = [0 0];     %inlet position
out_pos = [0 1000];    %outlet position

m_hand_pos = zeros(1,2); % manipulator hand position

% m_min_range = sqrt(link_1^2 + link_2^2 - 2*link_1*link_2*cos(pi - j2_max)); % manipulator min range
m_min_range = 188; % manipulator min range (radius)
m_max_range = 495;  % Maximum manipulator range531 (radius)


g_initial_theta_1 = 0;

% Figure ID
date = datetime;
id_num = rem(second(datetime)*1000,1000);
id_num_st = num2str(id_num);

% Delete all figure
close
clc

% Number of trials
num_t = 10;
max_num_t = num_t;
interval = 5;

num_best_angle = max_num_t/interval;
best_min_angle_array = zeros(100000,1);


% Timer start
tic

% % % % while num_t > 0

%%%%%% Manipulator position %%%%%%%%%%%%%%%%%%%%%%%%%%%
while 1
    %                 disp('test');
    m_pos_x_s = -lim_x + (lim_x-(-lim_x))*rand;
    m_pos_y_s = -lim_y + (lim_y-(-lim_y))*rand;
    m_pos_x = round(m_pos_x_s);
    m_pos_y = round(m_pos_y_s);
    
    
    % The distance from the conveyor is within the movable range of the manipulator
    if ((m_pos_x - c_pos_x)^2 + (m_pos_y - c_pos_y)^2 < m_max_range^2) && ((m_pos_x - c_pos_x)^2 + (m_pos_y - c_pos_y)^2 > m_min_range^2)
        
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
end


%%%%%% Palette position %%%%%%%%%%%%%%%%%%%%%%%%%%%
while 1
    p_pos_x_s = -lim_x + (lim_x-(-lim_x))*rand;
    p_pos_y_s = -lim_y + (lim_y-(-lim_y))*rand;
    p_pos_x = round(p_pos_x_s);
    p_pos_y = round(p_pos_y_s);
    
    % The distance from the conveyor is within the movable range of the manipulator
    if (p_pos_x - c_pos_x)^2 + (p_pos_y - c_pos_y)^2 < (2*m_max_range)^2
        
        % The distance from the manipulator is within the movable range of the manipulator
        if ((m_pos_x - p_pos_x)^2 + (m_pos_y - p_pos_y)^2 < m_max_range^2) && ((m_pos_x - p_pos_x)^2 + (m_pos_y - p_pos_y)^2 > m_min_range^2)
            
            % Whether to overlap with the palette
            if (m_pos_x > (p_pos_x + 1/2*pl + 1/2*ml)) || (m_pos_x < (p_pos_x - 1/2*pl - 1/2*ml))   % If it does not overlap in the X-axis direction
                
                % Whether to overlap with the conveyor
                if (p_pos_x > (c_pos_x + 1/2*cl_s + 1/2*pl)) || (p_pos_x < (c_pos_x - 1/2*cl_s - 1/2*pl))   % If it does not overlap in the X-axis direction
                    p_pos_x;
                    p_pos_y;
                    %                         disp('Decide the position of the palette1111');
                    break;
                    
                elseif (p_pos_y > (c_pos_y + 1/2*cl_l + 1/2*pl)) || (p_pos_y < (c_pos_y - 1/2*cl_l - 1/2*pl))   % If it does not overlap in the Y-axis direction
                    p_pos_x;
                    p_pos_y;
                    %                         disp('Decide the position of the palette2222');
                    break;
                else
                end
                
            elseif (m_pos_y > (p_pos_y + 1/2*pl + 1/2*ml)) || (m_pos_y < (p_pos_y - 1/2*pl - 1/2*ml))   % If it does not overlap in the Y-axis direction
                
                % Whether to overlap with the conveyor
                if (p_pos_x > (c_pos_x + 1/2*cl_s + 1/2*pl)) || (p_pos_x < (c_pos_x - 1/2*cl_s - 1/2*pl))   % If it does not overlap in the X-axis direction
                    p_pos_x;
                    p_pos_y;
                    %                         disp('Decide the position of the palette1111');
                    break;
                    
                elseif (p_pos_y > (c_pos_y + 1/2*cl_l + 1/2*pl)) || (p_pos_y < (c_pos_y - 1/2*cl_l - 1/2*pl))   % If it does not overlap in the Y-axis direction
                    p_pos_x;
                    p_pos_y;
                    %                         disp('Decide the position of the palette2222');
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

%     [mm] -> [m]
m_pos_random = [1/1000*m_pos_x, 1/1000*m_pos_y, 1/1000*m_pos_z];
p_pos_random = [1/1000*p_pos_x, 1/1000*p_pos_y, 1/1000*p_pos_z];
c_pos_random = [1/1000*c_pos_x, 1/1000*c_pos_y, 1/1000*c_pos_z];



end

