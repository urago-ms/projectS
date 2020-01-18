%%%%%%%%%%%%%%%%%%%%% Palette Local Search %%%%%%%%%%%%%%%%%%%%%%%%%%%
current_o_function = best_min_angle;

current_p_pos_y = p_pos_y;
current_p_pos_x = p_pos_x;

while 1
    origin_p_pos_y = p_pos_y
    origin_p_pos_x = p_pos_x
    
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
            
            %%%%%% manipulator new position %%%%%%%%%%%%%%%%%%%%%%%%%%%
            %         while 1
            %     p_pos_x = -lim_x + (lim_x-(-lim_x))*rand;
            %     p_pos_y = -lim_y + (lim_y-(-lim_y))*rand;
            % The distance from the conveyor is within the movable range of the manipulator
            if ((optimized_m_pos_x - c_pos_x)^2 + (optimized_m_pos_y - c_pos_y)^2 < (link_1 + link_2)^2) && ((optimized_m_pos_x - c_pos_x)^2 + (optimized_m_pos_y - c_pos_y)^2 > m_min_range^2)
                % The distance from the palette is within the movable range of the manipulator
                if (optimized_m_pos_x - p_pos_x)^2 + (optimized_m_pos_y - p_pos_y)^2 < (link_1 + link_2)^2 &&  ((optimized_m_pos_x - p_pos_x)^2 + (optimized_m_pos_y - p_pos_y)^2 > m_min_range^2)
                    
                    % Whether to overlap with the palette
                    if (optimized_m_pos_x > (p_pos_x + 1/2*pl + 1/2*ml)) || (optimized_m_pos_x < (p_pos_x - 1/2*pl - 1/2*ml))   % If it does not overlap in the X-axis direction
                        % Whether to overlap with the conveyor
                        if (optimized_m_pos_x > (c_pos_x + 1/2*cl_s + 1/2*ml)) || (optimized_m_pos_x < (c_pos_x - 1/2*cl_s - 1/2*ml))   % If it does not overlap in the X-axis direction
                            p_pos_x;
                            p_pos_y;
                            %                             break;
                        elseif (optimized_m_pos_y > (c_pos_y + 1/2*cl_l + 1/2*ml)) || (optimized_m_pos_y < (c_pos_y - 1/2*cl_l - 1/2*ml))   % If it does not overlap in the Y-axis direction
                            p_pos_x;
                            p_pos_y;
                            %                             break;
                        else
                            constraint_flag = 1;
                        end
                        
                    elseif (optimized_m_pos_y > (p_pos_y + 1/2*pl + 1/2*ml)) || (optimized_m_pos_y < (p_pos_y - 1/2*pl - 1/2*ml))   % If it does not overlap in the Y-axis direction
                        % Whether to overlap with the conveyor
                        if (optimized_m_pos_x > (c_pos_x + 1/2*cl_s + 1/2*ml)) || (optimized_m_pos_x < (c_pos_x - 1/2*cl_s - 1/2*ml))   % If it does not overlap in the X-axis direction
                            p_pos_x;
                            p_pos_y;
                            %                             break;
                        elseif (optimized_m_pos_y > (c_pos_y + 1/2*cl_l + 1/2*ml)) || (optimized_m_pos_y < (c_pos_y - 1/2*cl_l - 1/2*ml))   % If it does not overlap in the Y-axis direction
                            p_pos_x;
                            p_pos_y;
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
            else
                constraint_flag = 1;
            end
            %         end
            
            
            if constraint_flag == 0
                %%%%%%%%%%%%%%% Current Position IK %%%%%%%%%%%%%%%%%%%%%%
                %%%%% Inverse kinematics Solution 1 %%%%%%%%%%%%
                cos_alpha = (-((p_pos_x - optimized_m_pos_x)^2 + (p_pos_y - optimized_m_pos_y)^2) + link_1^2 + link_2^2) / (2*link_1*link_2);    % () of (2*link_1*link_2) is necessary
                alpha = atan2(sqrt(1 - cos_alpha*cos_alpha), cos_alpha);
                %alpha = atan2(sqrt(1-((-(p_pos_x - optimized_m_pos_x)^2 - (p_pos_y - optimized_m_pos_y)^2 + link_1^2 + link_2^2) / 2*link_1*link_2)^2), (-(p_pos_x - optimized_m_pos_x)^2 - (p_pos_y - optimized_m_pos_y)^2 + link_1^2 + link_2^2) / 2*link_1*link_2);
                c_ik_sol_1_theta_2 = pi - alpha;
                c_ik_sol_1_theta_1 = atan2((p_pos_y - optimized_m_pos_y), (p_pos_x - optimized_m_pos_x)) - atan2((link_2*sin(c_ik_sol_1_theta_2)), (link_1 + link_2*cos(c_ik_sol_1_theta_2)));
                %c_ik_sol_1_theta_1 = atan2((p_pos_y - optimized_m_pos_y), (p_pos_x - optimized_m_pos_x)) - atan2(link_2*sin(c_ik_sol_1_theta_2), link_1 + link_2*cos(c_ik_sol_1_theta_2))
                
                
                %%%%% Inverse kinematics Solution 2 %%%%%%%%%%%%
                cos_alpha = (-((p_pos_x - optimized_m_pos_x)^2 + (p_pos_y - optimized_m_pos_y)^2) + link_1^2 + link_2^2) / (2*link_1*link_2);    % () of (2*link_1*link_2) is necessary
                alpha = - atan2(sqrt(1 - cos_alpha*cos_alpha), cos_alpha);
                %alpha = atan2(sqrt(1-((-(p_pos_x - optimized_m_pos_x)^2 - (p_pos_y - optimized_m_pos_y)^2 + link_1^2 + link_2^2) / 2*link_1*link_2)^2), (-(p_pos_x - optimized_m_pos_x)^2 - (p_pos_y - optimized_m_pos_y)^2 + link_1^2 + link_2^2) / 2*link_1*link_2);
                c_ik_sol_2_theta_2 = pi - alpha;
                c_ik_sol_2_theta_1 = atan2((p_pos_y - optimized_m_pos_y), (p_pos_x - optimized_m_pos_x)) - atan2((link_2*sin(c_ik_sol_2_theta_2)), (link_1 + link_2*cos(c_ik_sol_2_theta_2)));
                %c_ik_sol_2_theta_1 = atan2((p_pos_y - optimized_m_pos_y), (p_pos_x - optimized_m_pos_x)) - atan2(link_2*sin(c_ik_sol_2_theta_2), link_1 + link_2*cos(c_ik_sol_2_theta_2))
                
                
                
                %%%%%%%%%%%%%%% Goal Position IK %%%%%%%%%%%%%%%%%%%%%%
                %%%%% Inverse kinematics Solution 1 %%%%%%%%%%%%
                cos_alpha = (-((c_pos_x - optimized_m_pos_x)^2 + (c_pos_y - optimized_m_pos_y)^2) + link_1^2 + link_2^2) / (2*link_1*link_2);    % () of (2*link_1*link_2) is necessary
                alpha = atan2(sqrt(1 - cos_alpha*cos_alpha), cos_alpha);
                %alpha = atan2(sqrt(1-((-(c_pos_x - optimized_m_pos_x)^2 - (c_pos_y - optimized_m_pos_y)^2 + link_1^2 + link_2^2) / 2*link_1*link_2)^2), (-(c_pos_x - optimized_m_pos_x)^2 - (c_pos_y - optimized_m_pos_y)^2 + link_1^2 + link_2^2) / 2*link_1*link_2);
                g_ik_sol_1_theta_2 = pi - alpha;
                g_ik_sol_1_theta_1 = atan2((c_pos_y - optimized_m_pos_y), (c_pos_x - optimized_m_pos_x)) - atan2((link_2*sin(g_ik_sol_1_theta_2)), (link_1 + link_2*cos(g_ik_sol_1_theta_2)));
                %g_ik_sol_1_theta_1 = atan2((c_pos_y - optimized_m_pos_y), (c_pos_x - optimized_m_pos_x)) - atan2(link_2*sin(g_ik_sol_1_theta_2), link_1 + link_2*cos(g_ik_sol_1_theta_2))
                
                
                
                %%%%% Inverse kinematics Solution 2 %%%%%%%%%%%%
                cos_alpha = (-((c_pos_x - optimized_m_pos_x)^2 + (c_pos_y - optimized_m_pos_y)^2) + link_1^2 + link_2^2) / (2*link_1*link_2);    % () of (2*link_1*link_2) is necessary
                alpha = - atan2(sqrt(1 - cos_alpha*cos_alpha), cos_alpha);
                %alpha = atan2(sqrt(1-((-(c_pos_x - optimized_m_pos_x)^2 - (c_pos_y - optimized_m_pos_y)^2 + link_1^2 + link_2^2) / 2*link_1*link_2)^2), (-(c_pos_x - optimized_m_pos_x)^2 - (c_pos_y - optimized_m_pos_y)^2 + link_1^2 + link_2^2) / 2*link_1*link_2);
                g_ik_sol_2_theta_2 = pi - alpha;
                g_ik_sol_2_theta_1 = atan2((c_pos_y - optimized_m_pos_y), (c_pos_x - optimized_m_pos_x)) - atan2((link_2*sin(g_ik_sol_2_theta_2)), (link_1 + link_2*cos(g_ik_sol_2_theta_2)));
                %g_ik_sol_2_theta_1 = atan2((c_pos_y - optimized_m_pos_y), (c_pos_x - optimized_m_pos_x)) - atan2(link_2*sin(g_ik_sol_2_theta_2), link_1 + link_2*cos(g_ik_sol_2_theta_2))
                
                
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
                
                
                %%%%%%%%%%%%%%% The initial path of the arm %%%%%%%%%%%%%%%%%%%%%%
                %%%%% From 1 to 1 %%%%%%%%%%%%
                a_11_delta_theta_1 = g_ik_sol_1_theta_1 - c_ik_sol_1_theta_1;
                while a_11_delta_theta_1 > 2*pi    % Smaller than 2*pi
                    a_11_delta_theta_1 = a_11_delta_theta_1 - 2*pi;
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
                    a_12_delta_theta_1 = a_12_delta_theta_1 - 2*pi;
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
                    a_21_delta_theta_1 = a_21_delta_theta_1 - 2*pi;
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
                    a_22_delta_theta_1 = a_22_delta_theta_1 - 2*pi;
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
                
                
                %%%%% Objective Function of 9 Neighbourhood (Min-Max) %%%%%%%%%%%%
                neighbour_num = max_side_length_y + 1 - side_length_y + (3*(max_side_length_x - side_length_x));
                o_function(neighbour_num,1) = min_angle;
                
                %%%%% Manipulator Position of 9 Neighbourhood %%%%%%%%%%%%
                %             neighbour_p_pos(neighbour_num,1) = p_pos_x;
                %             neighbour_p_pos(neighbour_num,2) = p_pos_y;
                
                neighbour_p_pos(neighbour_num,1) = p_pos_x;
                neighbour_p_pos(neighbour_num,2) = p_pos_y;
                
                
                time = datestr(date,'yyyymmdd_HHMMSS');
                file_name = strcat(time,'_');
                file_name = strcat(file_name,id_num_st);
                file_name = strcat(file_name,'_InitialSol_mani_neighbour.pdf');
                
                %                 posture_draw(file_name, cl_s, cl_l, ml, pl, link_1, link_2, m_min_range, p_pos_x, p_pos_y, optimized_m_pos_x, optimized_m_pos_y, c_pos_x, c_pos_y)
                
                
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
                file_name = strcat(file_name,'_InitialSol_mani_neighbour.pdf');
                
                %                 posture_draw(file_name, cl_s, cl_l, ml, pl, link_1, link_2, m_min_range, p_pos_x, p_pos_y, optimized_m_pos_x, optimized_m_pos_y, c_pos_x, c_pos_y)
                
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
            current_o_function = neighbour_best_o_function
            current_p_pos_x = neighbour_p_pos(i,1)
            current_p_pos_y = neighbour_p_pos(i,2)
            break;
        end
    end
    
    %%%%% When not moving %%%%%%%%%%%%
    if neighbour_best_o_function == current_o_function
        break;
    end
    
end

% Optimized palette position and manipulator position
optimized_p_pos_x = current_p_pos_x;
optimized_p_pos_y = current_p_pos_y;