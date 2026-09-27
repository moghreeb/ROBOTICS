% =========================================================================
% Mini-Project 2: Robot Model, Workspace and Drive Feasibility
% File Name: MP2.m
% Mohammed Qasim Ghareeb
% ID/481017316
% S/11
% =========================================================================
clc; clear; close all;

%% 1. Student-Specific Data & Common Parameters
S = 11; % Student parameter

% Common Robot Data
L1 = 0.45; % m
L2 = 0.35; % m
G = 50;    % Gearbox ratio

% Encoder parameters
Nc = 4096; % decoded counts/rev

% Specific data based on S
rng_seed = 2200 + 31*S;
theta_s_deg = 20 + S;
tx = 0.30 + 0.002*S;
ty = 0.12 - 0.001*S;
q1_verify_deg = 25 + S;
q2_verify_deg = 35 + 0.5*S;
sigma_q_deg = 0.10 + 0.005*S;

% Convert to radians for computation
theta_s = deg2rad(theta_s_deg);
q1_verify = deg2rad(q1_verify_deg);
q2_verify = deg2rad(q2_verify_deg);
sigma_q = deg2rad(sigma_q_deg);

%% 2. Encoder Resolution and Quantization Bounds
% Motor-side and joint-side resolution
dTheta_m_rad = (2*pi) / Nc;
dTheta_m_deg = 360 / Nc;

dq_rad = dTheta_m_rad / G;
dq_deg = dTheta_m_deg / G;

disp('===============================================================');
disp('                    ENCODER RESOLUTION                         ');
disp('===============================================================');
fprintf('Motor-side resolution: %.6f rad/count (%.4f deg/count)\n', dTheta_m_rad, dTheta_m_deg);
fprintf('Joint-side resolution: %.6f rad/count (%.4f deg/count)\n', dq_rad, dq_deg);
fprintf('Max quantization error bound: +/- %.6f rad\n\n', dq_rad/2);

%% 3. Smooth Signal, Noise, and Quantization (Sine vs Polynomial)
rng(rng_seed); % Set random seed based on S for reproducibility
t = linspace(0, 5, 500); % Time from 0 to 5 s

% ==========================================
% Example 1: Sine Wave Trajectory (Continuous Sweep)
% ==========================================
q_sine_true = deg2rad(45) + deg2rad(30) * sin(2*pi*0.2*t);
q_sine_noisy = q_sine_true + sigma_q * randn(size(q_sine_true)); % Add noise[cite: 1]
q_sine_quantized = round(q_sine_noisy / dq_rad) * dq_rad; % Quantize[cite: 1]

% ==========================================
% Example 2: Cubic Polynomial Trajectory (Point-to-Point)
% ==========================================
q0 = deg2rad(15); % Start angle
qf = deg2rad(75); % End angle
u = t / 5;        % Normalized time (T = 5s)
s_cubic = 3*u.^2 - 2*u.^3; % Cubic zero-end-velocity law
q_poly_true = q0 + (qf - q0) * s_cubic;
q_poly_noisy = q_poly_true + sigma_q * randn(size(q_poly_true)); % Add noise[cite: 1]
q_poly_quantized = round(q_poly_noisy / dq_rad) * dq_rad; % Quantize[cite: 1]

% ==========================================
% Plotting Both Examples
% ==========================================
figure('Name', 'Encoder Signal Analysis (Sine vs Polynomial)', 'Color', 'w', 'Position', [100, 100, 800, 600]);

% Plot 1: Sine Wave
subplot(2, 1, 1);
plot(t, rad2deg(q_sine_true), 'y', 'LineWidth', 2); hold on;
plot(t, rad2deg(q_sine_noisy), 'r-.', 'MarkerSize', 5);
stairs(t, rad2deg(q_sine_quantized), 'b--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Joint Angle (Degrees)');
title('Example 1: Sine Wave Trajectory (Continuous Sweeping)');
legend('True Signal', 'Noisy Signal', 'Quantized Signal', 'Location', 'best');
grid on;

% Plot 2: Cubic Polynomial
subplot(2, 1, 2);
plot(t, rad2deg(q_poly_true), 'y', 'LineWidth', 2); hold on;
plot(t, rad2deg(q_poly_noisy), 'r-.', 'MarkerSize', 5);
stairs(t, rad2deg(q_poly_quantized), 'b--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Joint Angle (Degrees)');
title('Example 2: Cubic Polynomial Trajectory (Point-to-Point)');
legend('True Signal', 'Noisy Signal', 'Quantized Signal', 'Location', 'best');
grid on;
%% 4. Coordinate Frame Transformation
% Sensor frame points (homogeneous coordinates)
p1_s = [0.10; 0.02; 1];
p2_s = [0.16; -0.03; 1];
p3_s = [0.22; 0.04; 1];
P_s = [p1_s, p2_s, p3_s];

% Planar Transformation Matrix T_s^0
T_s0 = [cos(theta_s), -sin(theta_s), tx;
        sin(theta_s),  cos(theta_s), ty;
        0,             0,            1];

% Transform points to base frame
P_0 = T_s0 * P_s;

disp('===============================================================');
disp('                SENSOR TO BASE FRAME TRANSFORMATION            ');
disp('===============================================================');
fprintf('T_s^0 Matrix:\n');
disp(T_s0);
for i = 1:3
    fprintf('Point %d in Sensor Frame: [%.3f, %.3f, 1]^T \n', i, P_s(1,i), P_s(2,i));
    fprintf('Point %d in Base Frame:   [%.3f, %.3f, 1]^T \n\n', i, P_0(1,i), P_0(2,i));
end

%% 5. DH Representation and Verification
% Standard DH Table for Planar 2R Robot: [theta, d, a, alpha]
% Link 1: theta = q1, d = 0, a = L1, alpha = 0
% Link 2: theta = q2, d = 0, a = L2, alpha = 0

% Transformation Matrices at the verification pose using standard DH formulation
A1 = [cos(q1_verify), -sin(q1_verify), 0, L1*cos(q1_verify);
      sin(q1_verify),  cos(q1_verify), 0, L1*sin(q1_verify);
      0,               0,              1, 0;
      0,               0,              0, 1];

A2 = [cos(q2_verify), -sin(q2_verify), 0, L2*cos(q2_verify);
      sin(q2_verify),  cos(q2_verify), 0, L2*sin(q2_verify);
      0,               0,              1, 0;
      0,               0,              0, 1];

T0_2 = A1 * A2;

disp('===============================================================');
disp('                    DH MATRICES AT VERIFICATION POSE           ');
disp('===============================================================');
fprintf('A1 Matrix:\n'); disp(A1);
fprintf('A2 Matrix:\n'); disp(A2);
fprintf('T_0^2 Matrix:\n'); disp(T0_2);

%% 6. Mathematical Verification
tol = 1e-10; % Tolerance for numerical verification

R_T0_2 = T0_2(1:3, 1:3); % Extract rotation matrix from T_0^2
I3 = eye(3);
I4 = eye(4);

% Verify matrix properties
check1 = max(abs( (R_T0_2' * R_T0_2) - I3 ), [], 'all') < tol;
check2 = abs( det(R_T0_2) - 1 ) < tol;
check3 = max(abs( (inv(T0_2) * T0_2) - I4 ), [], 'all') < tol;

disp('===============================================================');
disp('                    NUMERICAL VERIFICATION                     ');
disp('===============================================================');
fprintf('Tolerance used: %e\n', tol);
if check1, fprintf('1. R^T * R = I verified successfully.\n'); else, fprintf('1. R^T * R = I FAILED.\n'); end
if check2, fprintf('2. det(R) = 1 verified successfully.\n'); else, fprintf('2. det(R) = 1 FAILED.\n'); end
if check3, fprintf('3. T^-1 * T = I verified successfully.\n'); else, fprintf('3. T^-1 * T = I FAILED.\n'); end