function [pos_local] = neighborhood_coordinates23(m_pos_current, p_pos_current, c_pos_current, neighbour_step)


%rng(0,'twister');   % randam number initialization
rng('shuffle','twister');

orange = '[0.9216 0.4745 0.000]';
d_yellow = '[0.8157 0.6902 0.000]';
black = '[0 0 0]';
d_green = '[0.4660 0.6740 0.1880]';

% d_green = [0.4660 0.6740 0.1880];

lim_x = 2000;  % space limit
lim_y = 4000;  % space limit

ml = 800; % manipulator base one side length
pl = 500; % palette one side length

link_1 = 260; % manipulator link1 length
link_2 = 260; % manipulator link2 length

cl_l = 2000;   % conveyor long side length
cl_l_max = 1000;   % conveyor long side max length
cl_l_min = 500;   % conveyor long side min length
cl_s = 600;   % conveyor short side length


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


m_min_range = 593; % manipulator min range (radius)
m_max_range = 2051;  % Maximum manipulator range531 (radius)


pos_local = zeros(9, 2);


% neighbour_step = 50;
% neighbour_step_small = 5;

search_range = 9;

% objective function of 9 neighbourhood
o_function = zeros(9,1);

% palette position and manipulator position of 9 neighbourhood
neighbour_p_pos = zeros(9,2);
neighbour_m_pos = zeros(9,2);

% % Best palette position and manipulator position in 9 neighbourhood
% neighbour_optimized_p_pos_x = 0;
% neighbour_optimized_p_pos_y = 0;
% neighbour_optimized_m_pos_x = 0;
% neighbour_optimized_m_pos_y = 0;

% Optimized palette position and manipulator position
optimized_p_pos_x = 0;
optimized_p_pos_y = 0;
optimized_m_pos_x = 0;
optimized_m_pos_y = 0;

% Figure ID
date = datetime;
id_num = rem(second(datetime)*1000,1000);
id_num_st = num2str(id_num);

halfway_count = 0;

% Delete all figure
close
clc

% Number of trials
num_t = 1000;
max_num_t = num_t;


% Timer start
tic


%%%%%%%%%%%%%%%%%%%%% Manipulator Local Search %%%%%%%%%%%%%%%%%%%%%%%%%%%
current_o_function = best_min_angle;

current_m_pos_y = m_pos_current(1);
current_m_pos_x = m_pos_current(2);

while 1
    origin_m_pos_y = current_m_pos_y
    origin_m_pos_x = current_m_pos_x
    
    % start_m_pos_y = m_pos_y - 10;
    % start_m_pos_x = m_pos_x - 10;
    
    % Position No.1
    m_pos_y = current_m_pos_y - neighbour_step;
    m_pos_x = current_m_pos_x - neighbour_step;
    
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
            %         while 1
            %     m_pos_x = -lim_x + (lim_x-(-lim_x))*rand;
            %     m_pos_y = -lim_y + (lim_y-(-lim_y))*rand;
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
                        end
                    else
                        constraint_flag = 1;
                    end
                else
                    constraint_flag = 1;
                end
            elses
                constraint_flag = 1;
            end
            %         end
            
            
            if constraint_flag == 0
                
              
                
                
                %%%%% Objective Function of 9 Neighbourhood (Min-Max) %%%%%%%%%%%%
                neighbour_num = max_side_length_y + 1 - side_length_y + (3*(max_side_length_x - side_length_x));
%                 o_function(neighbour_num,1) = min_angle;
                
                %%%%% Manipulator Position of 9 Neighbourhood %%%%%%%%%%%%
                %             neighbour_p_pos(neighbour_num,1) = p_pos_x;
                %             neighbour_p_pos(neighbour_num,2) = p_pos_y;
                
                neighbour_m_pos(neighbour_num,1) = m_pos_x;
                neighbour_m_pos(neighbour_num,2) = m_pos_y;
                
                
%                 time = datestr(date,'yyyymmdd_HHMMSS');
%                 file_name = strcat(time,'_');
%                 file_name = strcat(file_name,id_num_st);
%                 file_name = strcat(file_name,'_InitialSol_mani_neighbour.png');
                
                %                 sol_draw_3(file_name, cl_s, cl_l, ml, pl, link_1, link_2, m_min_range, p_pos_x, p_pos_y, m_pos_x, m_pos_y, c_pos_x, c_pos_y)
                
                
            else
                
                %%%%% Objective Function of 9 Neighbourhood (Min-Max) %%%%%%%%%%%%
                neighbour_num = max_side_length_y + 1 - side_length_y + (3*(max_side_length_x - side_length_x));
%                 o_function(neighbour_num,1) = 9999;
                
                %%%%% Manipulator Position of 9 Neighbourhood %%%%%%%%%%%%
                %             neighbour_p_pos(neighbour_num,1) = 9999;
                %             neighbour_p_pos(neighbour_num,2) = 9999;
                neighbour_m_pos(neighbour_num,1) = 9999;
                neighbour_m_pos(neighbour_num,2) = 9999;
                
%                 time = datestr(date,'yyyymmdd_HHMMSS');
%                 file_name = strcat(time,'_');
%                 file_name = strcat(file_name,id_num_st);
%                 file_name = strcat(file_name,'_InitialSol_mani_neighbour.png');
                
                %                 sol_draw_3(file_name, cl_s, cl_l, ml, pl, link_1, link_2, m_min_range, p_pos_x, p_pos_y, m_pos_x, m_pos_y, c_pos_x, c_pos_y)
                
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
            side_length_y = side_length_y - 1
        end
        
        m_pos_x = m_pos_x + neighbour_step;
        side_length_x = side_length_x - 1
    end
    
    
    %%%%% Manipulator Best Position in 9 Neighbourhood %%%%%%%%%%%%
    neighbour_best_o_function = min(o_function);
    
    
    %     neighbour_num = 0;
    %   Update manipulator position %%%%
    for i = 1:search_range
        o_function(i,1)
        if neighbour_best_o_function == o_function(i,1)
            halfway_count = halfway_count + 1;
            current_o_function = neighbour_best_o_function
            current_m_pos_x = neighbour_m_pos(i,1)
            current_m_pos_y = neighbour_m_pos(i,2)
            
            
            
            
            
            close
            
            % Timer stop
            % elapsedTime = toc
            
            
            
            break;
        end
    end
    
    
    
    
    
    
    
    
    
    
    
    
    
    %%%%% When not moving %%%%%%%%%%%%
    %     if neighbour_best_o_function == current_o_function
    %     if neighbour_best_o_function == o_function(5,1)
    if  (current_m_pos_x == origin_m_pos_x) && (current_m_pos_y == origin_m_pos_y)
        break;
    end
    
end

% Optimized palette position and manipulator position
optimized_m_pos_x = current_m_pos_x;
optimized_m_pos_y = current_m_pos_y;


% clos  e
time = datestr(date,'yyyymmdd_HHMMSS');
file_name = strcat(time,'_');
file_name = strcat(file_name,id_num_st);
file_name = strcat(file_name,'_OptimizedSol.png');

% sol_draw_3(file_name, cl_s, cl_l, ml, pl, link_1, link_2, m_min_range, p_pos_x, p_pos_y, optimized_m_pos_x, optimized_m_pos_y, c_pos_x, c_pos_y)







%%%%%%%%%%%%%%%%%%%%% Palette Local Search %%%%%%%%%%%%%%%%%%%%%%%%%%%
% current_o_function = best_min_angle;
current_o_function = neighbour_best_o_function;

current_p_pos_y = p_pos_y;
current_p_pos_x = p_pos_x;

while 1
    origin_p_pos_y = current_p_pos_y
    origin_p_pos_x = current_p_pos_x
    
    % start_p_pos_y = p_pos_y - 10;
    % start_p_pos_x = p_pos_x - 10;
    
    % Position No.1
    p_pos_y = current_p_pos_y - neighbour_step;
    p_pos_x = current_p_pos_x - neighbour_step;
    
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
            %         while 1
            %     p_pos_x = -lim_x + (lim_x-(-lim_x))*rand;
            %     p_pos_y = -lim_y + (lim_y-(-lim_y))*rand;
            % The distance from the conveyor is within the movable range of the manipulator
            %             if ((optimized_m_pos_x - c_pos_x)^2 + (optimized_m_pos_y - c_pos_y)^2 < (link_1 + link_2)^2) && ((optimized_m_pos_x - c_pos_x)^2 + (optimized_m_pos_y - c_pos_y)^2 > m_min_range^2)
            % The distance from the palette is within the movable range of the manipulator
            if (optimized_m_pos_x - p_pos_x)^2 + (optimized_m_pos_y - p_pos_y)^2 < (link_1 + link_2)^2 &&  ((optimized_m_pos_x - p_pos_x)^2 + (optimized_m_pos_y - p_pos_y)^2 > m_min_range^2)
                
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
                    end
                else
                    constraint_flag = 1;
                end
            else
                constraint_flag = 1;
            end
            %             else
            %                 constraint_flag = 1;
            %             end
            %         end
            
            
            if constraint_flag == 0

                %%%%% Manipulator Position of 9 Neighbourhood %%%%%%%%%%%%
                %             neighbour_p_pos(neighbour_num,1) = p_pos_x;
                %             neighbour_p_pos(neighbour_num,2) = p_pos_y;
                
                neighbour_p_pos(neighbour_num,1) = p_pos_x;
                neighbour_p_pos(neighbour_num,2) = p_pos_y;
                
                
                time = datestr(date,'yyyymmdd_HHMMSS');
                file_name = strcat(time,'_');
                file_name = strcat(file_name,id_num_st);
                file_name = strcat(file_name,'_InitialSol_mani_neighbour.png');
                
                %                 sol_draw_3(file_name, cl_s, cl_l, ml, pl, link_1, link_2, m_min_range, p_pos_x, p_pos_y, optimized_m_pos_x, optimized_m_pos_y, c_pos_x, c_pos_y)
                
                
            else
                
                %%%%% Objective Function of 9 Neighbourhood (Min-Max) %%%%%%%%%%%%
                neighbour_num = max_side_length_y + 1 - side_length_y + (3*(max_side_length_x - side_length_x));
                o_function(neighbour_num,1) = 9999;
                
                %%%%% Manipulator Position of 9 Neighbourhood %%%%%%%%%%%%
                %             neighbour_p_pos(neighbour_num,1) = 9999;
                %             neighbour_p_pos(neighbour_num,2) = 9999;
                neighbour_p_pos(neighbour_num,1) = 9999;
                neighbour_p_pos(neighbour_num,2) = 9999;
                
                time = datestr(date,'yyyymmdd_HHMMSS');
                file_name = strcat(time,'_');
                file_name = strcat(file_name,id_num_st);
                file_name = strcat(file_name,'_InitialSol_mani_neighbour.png');
                
                %                 sol_draw_3(file_name, cl_s, cl_l, ml, pl, link_1, link_2, m_min_range, p_pos_x, p_pos_y, optimized_m_pos_x, optimized_m_pos_y, c_pos_x, c_pos_y)
                
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
            side_length_y = side_length_y - 1
        end
        
        p_pos_x = p_pos_x + neighbour_step;
        side_length_x = side_length_x - 1
    end
    
    
    %%%%% Manipulator Best Position in 9 Neighbourhood %%%%%%%%%%%%
    neighbour_best_o_function = min(o_function);
    
    
    %     neighbour_num = 0;
    for i = 1:search_range
        o_function(i,1)
        if neighbour_best_o_function == o_function(i,1)
            halfway_count = halfway_count + 1;
            current_o_function = neighbour_best_o_function
            current_p_pos_x = neighbour_p_pos(i,1)
            current_p_pos_y = neighbour_p_pos(i,2)
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            close
            
            
            
            
            
            
            
            break;
        end
    end
    
    %%%%% When not moving %%%%%%%%%%%%
    if  (current_p_pos_x == origin_p_pos_x) && (current_p_pos_y == origin_p_pos_y)
        break;
    end
    
end

% Optimized palette position and manipulator position
optimized_p_pos_x = current_p_pos_x;
optimized_p_pos_y = current_p_pos_y;

% % % % Delete all figure
% % % close
% % %
% % % time = datestr(date,'yyyymmdd_HHMMSS');
% % % file_name = strcat(time,'_');
% % % file_name = strcat(file_name,id_num_st);
% % % file_name = strcat(file_name,'_OptimizedSol.png');

% sol_draw_3(file_name, cl_s, cl_l, ml, pl, link_1, link_2, m_min_range, optimized_p_pos_x, optimized_p_pos_y, optimized_m_pos_x, optimized_m_pos_y, c_pos_x, c_pos_y)




% Timer stop
elapsedTime = toc;



% optimized_p_pos_x = p_pos_x;
% optimized_p_pos_y = p_pos_y;



close