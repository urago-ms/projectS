function [neighbour_p_pos] = tab_neighborhood_coordinates23(m_pos_current, p_pos_current, c_pos_current, neighbour_step)

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


c_pos_x = 1000*c_pos_current(1);
c_pos_y = 1000*c_pos_current(2);

p_pos_x = 1000*p_pos_current(1);
p_pos_y = 1000*p_pos_current(2);



search_range = 9;

% objective function of 9 neighbourhood
o_function = zeros(9,1);

% palette position and manipulator position of 9 neighbourhood
neighbour_p_pos = zeros(9,2);
neighbour_p_pos = zeros(9,2);


% Figure ID
date = datetime;
id_num = rem(second(datetime)*1000,1000);
id_num_st = num2str(id_num);




%%%%%%%%%%%%%%%%%%%%% tab Local Search %%%%%%%%%%%%%%%%%%%%%%%%%%%

optimized_m_pos_x = 1000*m_pos_current(1)
optimized_m_pos_y = 1000*m_pos_current(2)

current_p_pos_x = 1000*p_pos_current(1)
current_p_pos_y = 1000*p_pos_current(2)



origin_p_pos_x = current_p_pos_x;
origin_p_pos_y = current_p_pos_y;



% Position No.1
p_pos_x = current_p_pos_x - neighbour_step;
p_pos_y = current_p_pos_y - neighbour_step;


side_length_y = 3;
side_length_x = 3;

max_side_length_y = side_length_y;
max_side_length_x = side_length_x;

% constraint_flag = 0;


while side_length_x > 0
    %     p_pos_x = p_pos_x + 10
    
    side_length_y = 3;  % reset
    p_pos_y = origin_p_pos_y - neighbour_step; % reset
    
    
    while side_length_y > 0
        %         p_pos_y = p_pos_y + 10;
        
        constraint_flag = 0; % reset
        
        %%%%%% Palette new position %%%%%%%%%%%%%%%%%%%%%%%%%%%
        if (optimized_m_pos_x - p_pos_x)^2 + (optimized_m_pos_y - p_pos_y)^2 < m_max_range^2 &&  ((optimized_m_pos_x - p_pos_x)^2 + (optimized_m_pos_y - p_pos_y)^2 > m_min_range^2)
            
            % Whether to overlap with the palette
            if (optimized_m_pos_x > (p_pos_x + 1/2*pl + 1/2*ml)) || (optimized_m_pos_x < (p_pos_x - 1/2*pl - 1/2*ml))   % If it does not overlap in the X-axis direction
                % Whether the pallet overlaps the conveyor
                if (p_pos_x > (c_pos_x + 1/2*cl_s + 1/2*pl)) || (p_pos_x < (c_pos_x - 1/2*cl_s - 1/2*pl))   % If it does not overlap in the X-axis direction
                    p_pos_x;
                    p_pos_y;
                    %                         break;
                elseif (p_pos_y > (c_pos_y + 1/2*cl_l + 1/2*pl)) || (p_pos_y < (c_pos_y - 1/2*cl_l - 1/2*pl))   % If it does not overlap in the Y-axis direction
                    p_pos_x;
                    p_pos_y;
                    %                         break;
                else
                    constraint_flag = 1;
                    disp('111111111sssssssssssssfalaggggggggg');
                    
                end
                
            elseif (optimized_m_pos_y > (p_pos_y + 1/2*pl + 1/2*ml)) || (optimized_m_pos_y < (p_pos_y - 1/2*pl - 1/2*ml))   % If it does not overlap in the Y-axis direction
                % Whether the pallet overlaps the conveyor
                if (p_pos_x > (c_pos_x + 1/2*cl_s + 1/2*pl)) || (p_pos_x < (c_pos_x - 1/2*cl_s - 1/2*pl))   % If it does not overlap in the X-axis direction
                    p_pos_x;
                    p_pos_y;
                    %                         break;
                elseif (p_pos_y > (c_pos_y + 1/2*cl_l + 1/2*pl)) || (p_pos_y < (c_pos_y - 1/2*cl_l - 1/2*pl))   % If it does not overlap in the Y-axis direction
                    p_pos_x;
                    p_pos_y;
                    %                         break;
                else
                    constraint_flag = 1;
                    disp('222222222sssssssssssssfalaggggggggg');
                    
                end
            else
                constraint_flag = 1;
                disp('333333sssssssssssssfalaggggggggg');
                
            end
        else
            constraint_flag = 1;
            disp('44444sssssssssssssfalaggggggggg');
            
        end
        %             else
        %                 constraint_flag = 1;
        %             end
        %         end
        
        
        if constraint_flag == 0
            % % %                 Store the coordinates of the vicinity of the device
            neighbour_num = max_side_length_y + 1 - side_length_y + (3*(max_side_length_x - side_length_x));
            
            neighbour_p_pos(neighbour_num,1) = 1/1000*p_pos_x;
            neighbour_p_pos(neighbour_num,2) = 1/1000*p_pos_y;
            
        else
            neighbour_num = max_side_length_y + 1 - side_length_y + (3*(max_side_length_x - side_length_x));
            
            neighbour_p_pos(neighbour_num,1) = 9999;
            neighbour_p_pos(neighbour_num,2) = 9999;
            
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
        
        p_pos_y = p_pos_y + neighbour_step;
        side_length_y = side_length_y - 1;
    end
    
    p_pos_x = p_pos_x + neighbour_step;
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

p_pos_x = 1000*p_pos_x;
p_pos_y = 1000*p_pos_y;
m_pos_x = 1000*optimized_m_pos_x;
m_pos_y = 1000*optimized_m_pos_y;
c_pos_x = 1000*c_pos_x;
c_pos_y = 1000*c_pos_y;


sol_draw_only_facility_position(file_name, cl_s, cl_l, ml, pl, m_max_range, m_min_range, p_pos_x, p_pos_y, m_pos_x, m_pos_y, c_pos_x, c_pos_y);

end


