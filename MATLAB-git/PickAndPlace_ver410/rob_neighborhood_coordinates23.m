function [neighbour_m_pos] = rob_neighborhood_coordinates23(m_pos_current, p_pos_current, c_pos_current, neighbour_step)

% % Delete all figure
% close
% clc

%rng(0,'twister');   % randam number initialization
rng('shuffle','twister');

orange = '[0.9216 0.4745 0.000]';
d_yellow = '[0.8157 0.6902 0.000]';
black = '[0 0 0]';
d_green = '[0.4660 0.6740 0.1880]';

% d_green = [0.4660 0.6740 0.1880];


ml = 800; % manipulator base one side length
pl = 500; % palette one side length

link_1 = 260; % manipulator link1 length
link_2 = 260; % manipulator link2 length

cl_l = 2000;   % conveyor long side length
cl_l_max = 1000;   % conveyor long side max length
cl_l_min = 500;   % conveyor long side min length
cl_s = 600;   % conveyor short side length



m_min_range = 593; % manipulator min range (radius)
m_max_range = 2051;  % Maximum manipulator range531 (radius)

% % % [m] -> [mm]
c_pos_x = 1000*c_pos_current(1);
c_pos_y = 1000*c_pos_current(2);

p_pos_x = 1000*p_pos_current(1);
p_pos_y = 1000*p_pos_current(2);



search_range = 9;

% objective function of 9 neighbourhood
o_function = zeros(9,1);

% palette position and manipulator position of 9 neighbourhood
neighbour_p_pos = zeros(9,2);
neighbour_m_pos = zeros(9,2);


% Figure ID
date = datetime;
id_num = rem(second(datetime)*1000,1000);
id_num_st = num2str(id_num);




%%%%%%%%%%%%%%%%%%%%% Manipulator Local Search %%%%%%%%%%%%%%%%%%%%%%%%%%%

current_m_pos_x = 1000*m_pos_current(1);
current_m_pos_y = 1000*m_pos_current(2);



origin_m_pos_x = current_m_pos_x;
origin_m_pos_y = current_m_pos_y;



% Position No.1
m_pos_x = current_m_pos_x - neighbour_step;
m_pos_y = current_m_pos_y - neighbour_step;


side_length_y = 3;
side_length_x = 3;

max_side_length_y = side_length_y;
max_side_length_x = side_length_x;

% constraint_flag = 0;


while side_length_x > 0
    %     m_pos_x = m_pos_x + 10
    
    side_length_y = 3;  % reset
    m_pos_y = origin_m_pos_y - neighbour_step; % reset
    
    
    while side_length_y > 0
        %         m_pos_y = m_pos_y + 10;
        
        constraint_flag = 0; % reset
        %%%%%% manipulator new position %%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % The distance from the conveyor is within the movable range of the manipulator
        if ((m_pos_x - c_pos_x)^2 + (m_pos_y - c_pos_y)^2 < m_max_range^2) && ((m_pos_x - c_pos_x)^2 + (m_pos_y - c_pos_y)^2 > m_min_range^2)
            % The distance from the palette is within the movable range of the manipulator
            if (m_pos_x - p_pos_x)^2 + (m_pos_y - p_pos_y)^2 < m_max_range^2 &&  ((m_pos_x - p_pos_x)^2 + (m_pos_y - p_pos_y)^2 > m_min_range^2)
                
                % Whether to overlap with the palette
                if (m_pos_x > (p_pos_x + 1/2*pl + 1/2*ml)) || (m_pos_x < (p_pos_x - 1/2*pl - 1/2*ml))   % If it does not overlap in the X-axis direction
                    % Whether to overlap with the conveyor
                    if (m_pos_x > (c_pos_x + 1/2*cl_s + 1/2*ml)) || (m_pos_x < (c_pos_x - 1/2*cl_s - 1/2*ml))   % If it does not overlap in the X-axis direction
                        m_pos_x;
                        m_pos_y;
                        %                             break;
                    elseif (m_pos_y > (c_pos_y + 1/2*cl_l + 1/2*ml)) || (m_pos_y < (c_pos_y - 1/2*cl_l - 1/2*ml))   % If it does not overlap in the Y-axis direction
                        m_pos_x;
                        m_pos_y;
                        %                             break;
                    else
                        constraint_flag = 1;
                    end
                    
                elseif (m_pos_y > (p_pos_y + 1/2*pl + 1/2*ml)) || (m_pos_y < (p_pos_y - 1/2*pl - 1/2*ml))   % If it does not overlap in the Y-axis direction
                    % Whether to overlap with the conveyor
                    if (m_pos_x > (c_pos_x + 1/2*cl_s + 1/2*ml)) || (m_pos_x < (c_pos_x - 1/2*cl_s - 1/2*ml))   % If it does not overlap in the X-axis direction
                        m_pos_x;
                        m_pos_y;
                        %                             break;
                    elseif (m_pos_y > (c_pos_y + 1/2*cl_l + 1/2*ml)) || (m_pos_y < (c_pos_y - 1/2*cl_l - 1/2*ml))   % If it does not overlap in the Y-axis direction
                        m_pos_x;
                        m_pos_y;
                        %                             break;
                    else
                        constraint_flag = 1;
                        disp('falaggggggggg');
                    end
                else
                    constraint_flag = 1;
                    disp('falaggggggggg22');
                end
            else
                constraint_flag = 1;
                disp('falaggggggggg33');
            end
        else
            constraint_flag = 1;
            disp('falaggggggggg44');
        end
        %         end
        
        
        if constraint_flag == 0
            % % %                 Store the coordinates of the vicinity of the device
            neighbour_num = max_side_length_y + 1 - side_length_y + (3*(max_side_length_x - side_length_x));
            
            % % % [mm] -> [m]
            neighbour_m_pos(neighbour_num,1) = 1/1000*m_pos_x;
            neighbour_m_pos(neighbour_num,2) = 1/1000*m_pos_y;
            
        else
            neighbour_num = max_side_length_y + 1 - side_length_y + (3*(max_side_length_x - side_length_x));
            
            neighbour_m_pos(neighbour_num,1) = 9999;
            neighbour_m_pos(neighbour_num,2) = 9999;
            
        end
        
        %         %%%%% smallest angle (Min-Max)%%%%%%%%%%%%
        %         if best_min_angle > min_angle    % update smallest angle
        %             best_min_angle = min_angle;
        %             gomi = 'update smallest angle'
        %
        %             %best_min_angle = min([max_1 max_2 max_3 max_4]);
        %
        %             optimized_p_pos_x = p_pos_x;
        %             optimized_p_pos_y = p_pos_y;
        %             optimized_m_pos_x = m_pos_x;
        %             optimized_m_pos_y = m_pos_y;
        %         end
        
        m_pos_y = m_pos_y + neighbour_step;
        side_length_y = side_length_y - 1;
    end
    
    m_pos_x = m_pos_x + neighbour_step;
    side_length_x = side_length_x - 1;
end

time = datestr(date,'yyyymmdd_HHMMSS');
file_name = strcat(time,'_');
file_name = strcat(file_name,id_num_st);
file_name = strcat(file_name,'_m_local.png');

% p_pos_x = 1000*p_pos_x;
% p_pos_y = 1000*p_pos_y;
% m_pos_x = 1000*m_pos_x;
% m_pos_y = 1000*m_pos_y;
% c_pos_x = 1000*c_pos_x;
% c_pos_y = 1000*c_pos_y;

% % % [m] -> [mm]
p_pos_x = 1000*p_pos_x;
p_pos_y = 1000*p_pos_y;
m_pos_x = 1000*m_pos_x;
m_pos_y = 1000*m_pos_y;
c_pos_x = 1000*c_pos_x;
c_pos_y = 1000*c_pos_y;


sol_draw_only_facility_position(file_name, cl_s, cl_l, ml, pl, m_max_range, m_min_range, p_pos_x, p_pos_y, m_pos_x, m_pos_y, c_pos_x, c_pos_y);

end


