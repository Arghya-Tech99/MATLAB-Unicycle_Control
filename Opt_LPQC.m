clear all;
clc;

%  DEFINING THE UNICYCLE SYSTEM PARAMETERS 
Ff = 0; % Wheel friction factor with the ground
Fa = 0; % Wheel hub friction factor
Fpr = 0; % Arm friction factor with wheel
Frw = 0; % Wheel axle backlash friction factor

%  PITCH & ROLL CONTROL (ROBOT ARM) 
% Define the matrix A for the robot arm system 
% State vector is x_p = [alpha1; d_alpha1; alpha2; d_alpha2]
AP = [0, 1, 0, 0;
      0, -192.3077*(Ff+Fa), 0, 9.0577 + 192.3077*Fa;
      0, 0, 0, 1;
      0, 1.4824, 12.9155, (-12.7944 + 13.4268*Ff + 11.9444*Fa)];

% Define the matrix B for the robot arm system
BP = [0; 192.3077; 0; -14.9091];

%  WHEEL ROTATION CONTROL (ROBOT WHEEL) 
% Define the matrix A for the robot wheel system 
% State vector is x_w = [beta1; d_beta1; beta2; d_beta2]
AW = [0, 1, 0, 0;
      -94.7754, -5.6433*(Fpr+Frw), 0, 5.6433;
      0, 0, 0, 1;
      0, 1587.3*Frw, 0, -1587.3*Frw];

% Define the matrix B for the robot wheel system
BW = [0; -5.6433; 0; 1587.3];

%  LQC FOR ROBOT ARM 
% Define the weight matrices for the robot arm
% QP weights the states (x_p), RP weights the input (u)
% Adjust the QP and RP values to tune the controller response
QP = 10*eye(4); 
RP = 10; 

[KP, S_p, E_p] = lqr(AP, BP, QP, RP); % KP - optimal gain matrix

% SIMULATE THE CLOSED LOOP SYSTEM FOR THE ROBOT ARM
% Closed-loop A matrix: A_cl_p = AP - BP*KP
A_cl_p = AP - BP*KP;
B_cl_p = BP;
C_cl_p = eye(4);
D_cl_p = 0;

sys_cl_p = ss(A_cl_p, B_cl_p, C_cl_p, D_cl_p); % Closed-loop state-space model

% Step response of the closed-loop system
initial_state_arm = [0.1; 0; 0; 0]; % Non-zero initial state
t = 0:0.01:10;
[y_p, t_p, x_p] = initial(sys_cl_p, initial_state_arm, t);

% Plot the response
figure;
hold on; % Hold the plot to overlay multiple lines

% Plot each state with a unique line style and color
plot(t_p, rad2deg(x_p(:,1)), 'LineWidth', 1.5, 'Color', 'b'); % Pitch angle
plot(t_p, rad2deg(x_p(:,2)), 'LineWidth', 1.5, 'Color', 'r'); % Pitch rate
plot(t_p, rad2deg(x_p(:,3)), 'LineWidth', 1.5, 'Color', 'g'); % Roll angle
plot(t_p, rad2deg(x_p(:,4)), 'LineWidth', 1.5, 'Color', 'm'); % Roll rate

hold off;

title('Robot Arm Pitch and Roll Response');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('Pitch Angle $\alpha_1$', 'Pitch Rate $\dot{\alpha}_1$', 'Roll Angle $\alpha_2$', 'Roll Rate $\dot{\alpha}_2$', 'Interpreter', 'latex');
grid on;

% --- LQC FOR ROBOT WHEEL ---
% Define the weighting matrices for the robot wheel
QW = 10*eye(4);
RW = 10;
% Find the optimal gain matrix KW
[KW, S_w, E_w] = lqr(AW, BW, QW, RW);

% Simulate the closed-loop system for the robot wheel
A_cl_w = AW - BW*KW;
B_cl_w = BW;
C_cl_w = eye(4);
D_cl_w = 0;

% Create the closed-loop state-space model
sys_cl_w = ss(A_cl_w, B_cl_w, C_cl_w, D_cl_w);

% Set an initial state for the wheel system
% State vector is x_w = [beta1; d_beta1; beta2; d_beta2]
initial_state_wheel = [0.1; 0; 0; 0]; % Example non-zero initial state
t = 0:0.01:10;
[y_w, t_w, x_w] = initial(sys_cl_w, initial_state_wheel, t);

% Plot the response with customized line properties
figure;
hold on; % Hold the plot to overlay multiple lines

% Plot each state with a unique line style and color
plot(t_w, rad2deg(x_w(:,1)), 'LineWidth', 1.5, 'Color', 'b'); % Wheel angle 1
plot(t_w, rad2deg(x_w(:,2)), 'LineWidth', 1.5, 'Color', 'r'); % Wheel angle rate 1
plot(t_w, rad2deg(x_w(:,3)), 'LineWidth', 1.5, 'Color', 'g'); % Wheel angle 2
plot(t_w, rad2deg(x_w(:,4)), 'LineWidth', 1.5, 'Color', 'c'); % Wheel angle rate 2

hold off;

title('Robot Wheel Rotation Response');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('Wheel Angle $\beta_1$', 'Wheel Rate $\dot{\beta}_1$', 'Wheel Angle $\beta_2$', 'Wheel Rate $\dot{\beta}_2$', 'Interpreter', 'latex');
grid on;