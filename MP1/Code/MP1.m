% =========================================================================
% Mini-Project 1: Robot Model, Workspace and Drive Feasibility
% File Name: MP1.m
% Mohammed Qasim Ghareeb
% ID/481017316
% S/11
% =========================================================================
clc; clear; close all;

%% 1. Student-Specific Data & Common Parameters
S = 11;

% Robot Parameters Structure
robot.L1 = 0.45;        % m
robot.L2 = 0.35;        % m
robot.m1 = 2.0;         % kg
robot.m2 = 1.5;         % kg
robot.lc1 = 0.225;      % m
robot.lc2 = 0.175;      % m
robot.mp = 0.50;        % kg
robot.g = 9.81;         % m/s^2

% Joint Limits (Degrees)
robot.q1_min = -160;    robot.q1_max = 160;
robot.q2_min = -150;    robot.q2_max = 150;

% Drive & Motor Specifications
robot.ns = 1.5;         % Safety factor
robot.G = 50;           % Gearbox ratio
robot.eta_g = 0.85;     % Gearbox efficiency
robot.motor_torque_rated = 0.8; % N.m
robot.motor_speed_rpm = 3000;   % rpm
robot.motor_speed_rads = robot.motor_speed_rpm * (2*pi/60); % Convert to rad/s

% Joint Velocity Requirements
req_wj1 = 1.5; % rad/s
req_wj2 = 2.0; % rad/s

% Task Points for S = 11
P1 = [0.52 + 0.004*S; 0.10 + 0.002*S];
P2 = [0.34 + 0.002*S; 0.43 - 0.001*S];
P3 = [0.82 + 0.003*S; 0];
Points = [P1, P2, P3];
Point_Names = {'P1', 'P2', 'P3'};

%% 2. Workspace Generation (Random Sampling)
N_samples = 21100;

% Uniform random sampling within joint limits
q1_rand_deg = robot.q1_min + (robot.q1_max - robot.q1_min) * rand(1, N_samples);
q2_rand_deg = robot.q2_min + (robot.q2_max - robot.q2_min) * rand(1, N_samples);

% Convert to radians for FK
q1_rand = deg2rad(q1_rand_deg);
q2_rand = deg2rad(q2_rand_deg);

% Forward Kinematics (FK)
X_ws = robot.L1 * cos(q1_rand) + robot.L2 * cos(q1_rand + q2_rand);
Y_ws = robot.L1 * sin(q1_rand) + robot.L2 * sin(q1_rand + q2_rand);

%% 3. Plotting Workspace and Task Points
figure('Name', 'Robot Workspace and Task Points', 'Color', 'w');
scatter(X_ws, Y_ws, 1, [0.7 0.8 1], 'filled'); hold on;
plot(Points(1,:), Points(2,:), 'ro', 'MarkerSize', 8, 'LineWidth', 2, 'MarkerFaceColor', 'r');

% Add Labels for Points
for i = 1:3
    text(Points(1,i)+0.02, Points(2,i)+0.02, Point_Names{i}, 'FontSize', 12, 'FontWeight', 'bold');
end

title('Joint-Limited Workspace and Task Points');
xlabel('X (m)'); ylabel('Y (m)');
axis equal; grid on;
% Draw maximum and minimum radial bounds for visual reference
theta_circle = linspace(0, 2*pi, 100);
r_max = robot.L1 + robot.L2;
r_min = abs(robot.L1 - robot.L2);
plot(r_max*cos(theta_circle), r_max*sin(theta_circle), 'k--', 'LineWidth', 1.5);
plot(r_min*cos(theta_circle), r_min*sin(theta_circle), 'k--', 'LineWidth', 1.5);
legend('Workspace Samples', 'Task Points', 'Max/Min Radial Bounds');

%% 4. Reachability Analysis (Radial Bound & Analytical IK)
disp('===============================================================');
disp('                    REACHABILITY TABLE                         ');
disp('===============================================================');
fprintf('%-5s | %-12s | %-15s | %-15s\n', 'Point', 'Radial Bound', 'IK Limit Check', 'Status');
disp('---------------------------------------------------------------');

for i = 1:3
    x = Points(1,i);
    y = Points(2,i);
    r = sqrt(x^2 + y^2);
    
    % 1. Radial Bound Check
    radial_pass = (r >= r_min) && (r <= r_max);
    radial_str = evalc('if radial_pass, fprintf(''Pass''); else, fprintf(''Fail''); end');
    
    % 2. Analytical IK Check
    ik_pass = false;
    if radial_pass
        c2 = (r^2 - robot.L1^2 - robot.L2^2) / (2 * robot.L1 * robot.L2);
        
        if abs(c2) <= 1
            % Two possible solutions for q2 (Elbow up / Elbow down)
            s2_1 = sqrt(1 - c2^2);
            s2_2 = -sqrt(1 - c2^2);
            
            q2_rad_1 = atan2(s2_1, c2);
            q2_rad_2 = atan2(s2_2, c2);
            
            q1_rad_1 = atan2(y, x) - atan2(robot.L2 * s2_1, robot.L1 + robot.L2 * c2);
            q1_rad_2 = atan2(y, x) - atan2(robot.L2 * s2_2, robot.L1 + robot.L2 * c2);
            
            % Convert to degrees
            q1_deg_1 = rad2deg(q1_rad_1); q2_deg_1 = rad2deg(q2_rad_1);
            q1_deg_2 = rad2deg(q1_rad_2); q2_deg_2 = rad2deg(q2_rad_2);
            
            % Check against joint limits
            valid_1 = (q1_deg_1 >= robot.q1_min && q1_deg_1 <= robot.q1_max) && ...
                      (q2_deg_1 >= robot.q2_min && q2_deg_1 <= robot.q2_max);
                  
            valid_2 = (q1_deg_2 >= robot.q1_min && q1_deg_2 <= robot.q1_max) && ...
                      (q2_deg_2 >= robot.q2_min && q2_deg_2 <= robot.q2_max);
                  
            if valid_1 || valid_2
                ik_pass = true;
            end
        end
    end
    ik_str = evalc('if ik_pass, fprintf(''Pass''); else, fprintf(''Fail''); end');
    
    % Final Status
    if radial_pass && ik_pass
        status = 'Reachable';
    else
        status = 'Unreachable';
    end
    
    fprintf('%-5s | %-12s | %-15s | %-15s\n', Point_Names{i}, strtrim(radial_str), strtrim(ik_str), status);
end

%% 5. Drive-Feasibility Analysis
% Static Gravity Torques (Worst-case horizontal posture)
tau2_g_max = robot.g * (robot.m2 * robot.lc2 + robot.mp * robot.L2);
tau1_g_max = robot.g * (robot.m1 * robot.lc1 + robot.m2 * (robot.L1 + robot.lc2) + robot.mp * (robot.L1 + robot.L2));

% Required Joint Torques
tau_j1_req = robot.ns * tau1_g_max;
tau_j2_req = robot.ns * tau2_g_max;

% Required Motor Torques
tau_m1_req = tau_j1_req / (robot.G * robot.eta_g);
tau_m2_req = tau_j2_req / (robot.G * robot.eta_g);

% Required Motor Speeds (rad/s)
wm1_req = robot.G * req_wj1;
wm2_req = robot.G * req_wj2;

% Pass/Fail Evaluation
% Pass/Fail Evaluation for Joint 1
if (tau_m1_req <= robot.motor_torque_rated) && (wm1_req <= robot.motor_speed_rads)
    status_m1 = 'Pass';
else
    status_m1 = 'Fail';
end

% Pass/Fail Evaluation for Joint 2
if (tau_m2_req <= robot.motor_torque_rated) && (wm2_req <= robot.motor_speed_rads)
    status_m2 = 'Pass';
else
    status_m2 = 'Fail';
end
disp(' ');
disp('=============================================================================');
disp('                           DRIVE FEASIBILITY TABLE                           ');
disp('=============================================================================');
fprintf('%-7s | %-15s | %-15s | %-15s | %-10s\n', 'Joint', 'Req. Joint Trq', 'Req. Motor Trq', 'Req. Motor Spd', 'Status');
disp('-----------------------------------------------------------------------------');
fprintf('Joint 1 | %-11.2f N.m | %-11.2f N.m | %-10.2f rad/s | %-10s\n', tau_j1_req, tau_m1_req, wm1_req, status_m1);
fprintf('Joint 2 | %-11.2f N.m | %-11.2f N.m | %-10.2f rad/s | %-10s\n', tau_j2_req, tau_m2_req, wm2_req, status_m2);
disp('=============================================================================');
fprintf('Rated Motor Torque: %.2f N.m \n', robot.motor_torque_rated);
fprintf('Rated Motor Speed: %.2f rad/s (3000 rpm) \n', robot.motor_speed_rads);