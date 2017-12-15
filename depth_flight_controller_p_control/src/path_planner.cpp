 #include "path_planner.h"

namespace depth_flight_controller

  class PathPlanner 
  {
      float target_angle_deg;  % in DEG
      float target_depth;       % in m
      
      float target_angle_deg = 20;  % in DEG
      float target_depth = 5;  

      float target_x = target_depth;
      float target_angle_rad = 2*pi/360*target_angle_deg;
      float target_y = tan(target_angle_rad)*target_depth;

      // calculate first circle information
      float beta = atan(target_y/target_x);
      float alpha = pi - 2*(pi/2 - beta);

      float sample_step_dist = abs_vel / controller_freq;
      float sample_step_rad_diff = sample_step_dist / target_radius;
      float sample_step_rad = sample_step_rad_diff;
      float number_samples_curve = round(alpha/sample_step_rad_diff);

      float abs_acc = abs_vel * abs_vel / target_radius;

      float circle_x = 0;
      float circle_y = target_radius;

      Eigen::Matrix3f C;
      C << 1, 0, -circle_x, 0, 1, -circle_y, -circle_x, -circle_y, circle_x^2+circle_y^2-target_radius^2;
      Eigen::Vector3f P;
      P << target_x, target_y, 1;
      Eigen::Vector3f L = C*P;

      float tangent_x = -L(3)/L(1);
      float p = -2*circle_y;
      float q = tangent_x^2 - 2*tangent_x * circle_x + circle_x^2 + circle_y^2 - target_radius^2;
      float tangent_y = -p/2 - sqrt((p/2)^2-q);

      float tangent_beta = atan(tangent_y/tangent_x);
      float tangent_alpha = pi - 2*(pi/2 - tangent_beta);
      float yaw_switch = round(tangent_alpha/sample_step_rad_diff);

      float direction_diff_curve_end = alpha - target_angle_rad;
      float change_curve_angle = alpha - direction_diff_curve_end / 4;

      float m1 = tan(target_angle_rad)
      float m2 = tan(change_curve_angle)

      float p1_x = 0;
      float p1_y = 0;
      float a1 = 1;
      float b1 = -m1;
      float c1 = -p1_y+m1*p1_x;
      float d1 = sqrt(1 + b1^2);

      float p2_x = target_radius * sin(change_curve_angle);
      float p2_y = target_radius * (1 - cos(change_curve_angle));
      float a2 = 1;
      float b2 = -m2;
      float c2 = -p2_y+m2*p2_x;

      float d2 = sqrt(1 + b2^2);

      float A = a1*d2 - a2*d1; %%
      float B = b1*d2 - b2*d1; %%
      float C = c1*d2 - c2*d1; %%

      float E = -B/A;
      float F = -C/A;

      float G = F-p2_y;
      float H = b1+a1*E;
      float I = a1*F+c1;

      float J = (1+E^2)*d1^2 - H^2;

      float P = ((-2*p2_x + 2*E*G)*d1^2 - 2*H*I)/J;
      float Q = ((p2_x^2 + G^2)*d1^2 - I^2)/J;

      !!!!!float x_c = real(-P/2 + sqrt((P/2)^2 - Q));
      float y_c = E*x_c + F;

      !!!!!float samples_curve = round(change_curve_angle/sample_step_rad_diff);
      path_curve = zeros(number_samples_curve,8);

      for i = 1:number_samples_curve
          x_pos = target_radius * sin(sample_step_rad);
          y_pos = target_radius * (1 - cos(sample_step_rad));
          x_vel = abs_vel * cos(sample_step_rad);
          y_vel = abs_vel * sin(sample_step_rad);
          x_acc = -abs_acc * sin(sample_step_rad);
          y_acc = abs_acc * cos(sample_step_rad);

          path_curve(i,1) = x_pos;
          path_curve(i,2) = y_pos;
          path_curve(i,3) = x_vel;
          path_curve(i,4) = y_vel;
          path_curve(i,5) = x_acc;
          path_curve(i,6) = y_acc;
          
          if (i <= yaw_switch)
               yaw = sample_step_rad;
               path_curve(i,7) = yaw;
               yaw_rate = sample_step_rad_diff*controller_freq;
               path_curve(i,8) = yaw_rate;  
          end
          
          sample_step_rad = sample_step_rad + sample_step_rad_diff; 
      end 


      %%
      alpha_2 = change_curve_angle - target_angle_rad;
      c2_radius = sqrt((x_c-p2_x)^2 + (y_c - p2_y)^2);

      sample_step_rad_diff_2 = sample_step_dist / c2_radius;
      sample_step_rad_2 = sample_step_rad_diff_2;
      number_samples_curve_2 = round(alpha_2/sample_step_rad_diff_2);
      path_curve_2 = zeros(number_samples_curve_2,8);

      for i = 1:number_samples_curve_2
          x_pos_b = c2_radius * sin(sample_step_rad_2);
          y_pos_b = c2_radius * cos(sample_step_rad_2);
          x_vel_b = abs_vel * cos(sample_step_rad_2);
          y_vel_b = -abs_vel * sin(sample_step_rad_2);
          x_acc_b = -abs_acc * sin(sample_step_rad_2);
          y_acc_b = -abs_acc * cos(sample_step_rad_2);
          
          x_pos = x_pos_b * cos(change_curve_angle) - y_pos_b * sin(change_curve_angle)+x_c;
          y_pos = x_pos_b * sin(change_curve_angle) + y_pos_b * cos(change_curve_angle)+y_c;
          x_vel = x_vel_b * cos(change_curve_angle) - y_vel_b * sin(change_curve_angle);
          y_vel = x_vel_b * sin(change_curve_angle) + y_vel_b * cos(change_curve_angle);
          x_acc = x_acc_b * cos(change_curve_angle) - y_acc_b * sin(change_curve_angle);
          y_acc = x_acc_b * sin(change_curve_angle) + y_acc_b * cos(change_curve_angle);
          
          path_curve_2(i,1) = x_pos;
          path_curve_2(i,2) = y_pos;
          path_curve_2(i,3) = x_vel;
          path_curve_2(i,4) = y_vel;
          path_curve_2(i,5) = x_acc;
          path_curve_2(i,6) = y_acc;
          
          sample_step_rad_2 = sample_step_rad_2 + sample_step_rad_diff_2;
      end

      % Path straight
      straight_pos_x = path_curve_2(number_samples_curve_2,1);
      straight_pos_y = path_curve_2(number_samples_curve_2,2);

      dist_straight_target = sqrt((target_x-straight_pos_x)^2 + (target_y-straight_pos_y)^2);
      number_samples_straight = round(dist_straight_target / sample_step_dist);
      sample_step_dist_total = sample_step_dist;
      path_straight = zeros(number_samples_straight,8);

      for i = 1:number_samples_straight
          x_pos = straight_pos_x + sample_step_dist_total * cos(target_angle_rad);
          y_pos = straight_pos_y + sample_step_dist_total * sin(target_angle_rad);
          x_vel = abs_vel * cos(target_angle_rad);
          y_vel = abs_vel * sin(target_angle_rad);
          x_acc = 0;
          y_acc = 0;
          yaw   = target_angle_rad;
          yaw_rate = 0;
          
          path_straight(i,1) = x_pos;
          path_straight(i,2) = y_pos;
          path_straight(i,3) = x_vel;
          path_straight(i,4) = y_vel;
          path_straight(i,5) = x_acc;
          path_straight(i,6) = y_acc;
          path_straight(i,7) = yaw;
          path_straight(i,8) = yaw_rate;
          
          sample_step_dist_total = sample_step_dist_total + sample_step_dist; 
      end

      %% yaw rate

      path = [path_curve; path_curve_2; path_straight];

      for i = yaw_switch:(number_samples_curve+number_samples_curve_2+1)
                   yaw = atan((target_y-path(i,2))/(target_x-path(i,1)));
                   path(i,7) = yaw;
                   yaw_rate = (yaw - path(i-1,7))*controller_freq;
                   path(i,8) = yaw_rate;        
      end  

      number_samples = number_samples_curve + number_samples_curve_2 + number_samples_straight

      path_low_pass(:,1) = smooth(path(:,1),41);
      path_low_pass(:,2) = smooth(path(:,2),41);
      path_low_pass(:,3) = smooth(path(:,3),41);
      path_low_pass(:,4) = smooth(path(:,4),41);
      path_low_pass(:,5) = smooth(path(:,5),41);
      path_low_pass(:,6) = smooth(path(:,6),41);
      path_low_pass(:,7) = smooth(path(:,7),41);
      path_low_pass(:,8) = smooth(path(:,8),21);
  }
}
