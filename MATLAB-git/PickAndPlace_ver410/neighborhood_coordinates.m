function [m_pos_local] = neighborhood_coordinates(m_pos_current, in_displacement)

% function [m_pos_local, p_pos_local, c_pos_local] = neighborhood_coordinates(m_pos_current, p_pos_current, c_pos_current, in_displacement)

displacement_large = 50;
displacement_small = 5;


x_axis_local_num = 9;
y_axis_local_num = 9;

m_pos_local = zeros(x_axis_local_num, 2);
p_pos_local = zeros(x_axis_local_num, 2);
c_pos_local = zeros(x_axis_local_num, 2);

init_y_coordinate_num = m_pos_current(1,2) + in_displacement;
init_x_coordinate_num = m_pos_current(1,1) - in_displacement;

% y_num = 1;
% x_num = 1;


for y_axis = 1:3
    
    for x_axis = 1:3
        if (y_axis == 1)
            Num = x_axis;
        else
            Num = (y_axis - 1)*3 + x_axis;
        end
        
        if x_axis == 1
            m_pos_local(Num,1) = init_x_coordinate_num;
            m_pos_local(Num,2) = init_y_coordinate_num;
        else
            m_pos_local(Num,1) = x_coordinate_num;
            m_pos_local(Num,2) = y_coordinate_num;
        end

        x_coordinate_num = x_coordinate_num + in_displacement;
    end
    
    y_coordinate_num = y_coordinate_num - in_displacement;
    
end



end
