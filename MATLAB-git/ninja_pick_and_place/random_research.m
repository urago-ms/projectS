function [m_pos_optim, p_pos_optim, c_pos_optim] = random_research()

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

m_pos_optim = [m_pos_x, m_pos_y, m_pos_z];
p_pos_optim = [p_pos_x, p_pos_y, p_pos_z];
c_pos_optim = [c_pos_x, c_pos_y, c_pos_z];



in_pos = [0 0];     %inlet position
out_pos = [0 1000];    %outlet position

m_hand_pos = zeros(1,2); % manipulator hand position

% m_min_range = sqrt(link_1^2 + link_2^2 - 2*link_1*link_2*cos(pi - j2_max)); % manipulator min range
m_min_range = 188; % manipulator min range
m_max_range = 525;  % Maximum manipulator range531


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
m_pos_optim = [1/1000*m_pos_x, 1/1000*m_pos_y, 1/1000*m_pos_z]
p_pos_optim = [1/1000*p_pos_x, 1/1000*p_pos_y, 1/1000*p_pos_z]
c_pos_optim = [1/1000*c_pos_x, 1/1000*c_pos_y, 1/1000*c_pos_z]

% % % %     %     close
% % % %     time = datestr(date,'yyyymmdd_HHMMSS');
% % % %     file_name = strcat(time,'_');
% % % %     file_name = strcat(file_name,id_num_st);
% % % %     file_name = strcat(file_name,'_Sol.png');
% % % %
% % % %     sol_draw_only_facility_position(file_name, cl_s, cl_l, ml, pl, m_max_range, m_min_range, p_pos_x, p_pos_y, m_pos_x, m_pos_y, c_pos_x, c_pos_y);



%{
    if num_t == max_num_t    % Run only the first time
        % save Initial solution graph
        %         set(gca,'FontSize',15);
        %         ylabel('y','FontSize',24)
        %         xlabel('x','FontSize',24)
        
        
        
        time = datestr(date,'yyyymmdd_HHMMSS');
        file_name = strcat(time,'_');
        file_name = strcat(file_name,id_num_st);
        file_name = strcat(file_name,'_InitialSol.png');
        
        sol_draw_only_facility_position(file_name, cl_s, cl_l, ml, pl, m_max_range, m_min_range, p_pos_x, p_pos_y, m_pos_x, m_pos_y, c_pos_x, c_pos_y)
        
        %         saveas(gcf,file_name)
        
    end
%}

% % % %     num_t = num_t - 1;
% % % %
% % % % end


% close


% Timer stop
elapsedTime = toc


%%%%% Save png %%%%%%%%%%%%
time = datestr(date,'yyyymmdd_HHMMSS');
file_name = strcat(time,'_');
file_name = strcat(file_name,id_num_st);
file_name = strcat(file_name,'_Sol.png');

sol_fig_path = strcat('C:\Users\mslab\github\projectS\MATLAB-git\ninja_pick_and_place\sol_fig\', file_name);

sol_draw_only_facility_position(sol_fig_path, cl_s, cl_l, ml, pl, m_max_range, m_min_range, p_pos_x, p_pos_y, m_pos_x, m_pos_y, c_pos_x, c_pos_y);

% saveas(gcf,file_name);


%%%%% Save csv %%%%%%%%%%%%
% time = datestr(date,'yyyymmdd_HHMMSS');
% file_name_csv = strcat(time,'_');
% file_name_csv = strcat(file_name_csv,id_num_st);
% file_name_csv = strcat(file_name_csv,'_');
% max_num_t_str = num2str(max_num_t);

% file_name_csv = strcat(file_name_csv,max_num_t_str);
% file_name_csv = strcat(file_name_csv,'best_min_angle.csv');
% csvwrite(file_name_csv,best_min_angle_array)


% % Annotation of graph
% dim = [0.76 0.2 0.3 0.3];
% best_min_angle_deg = rad2deg(best_min_angle)
% trials_number_str = num2str(max_num_t)
% round_best_min_angle = num2str(round(best_min_angle_deg,6))
% elapsedTime_str = num2str(elapsedTime)
% str = {strcat('Trials:',trials_number_str),strcat('J1 axis:',j1_direction),strcat('J2 axis:',j2_direction),strcat(best_angle_axis),strcat(round_best_min_angle,'[deg]'),strcat(elapsedTime_str,'[sec]')};
% annotation('textbox',dim,'FontSize',15,'String',str,'FitBoxToText','on')


% save Initial solution graph
%         set(gca,'FontSize',15);
%         ylabel('y','FontSize',24)
%         xlabel('x','FontSize',24)


% numnum = numnum - 1;
% end
%
% ave = rad2deg(sum / numnum_max)

end

