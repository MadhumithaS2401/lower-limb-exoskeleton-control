function model = exoskeleton_model(PARAM)
%% LOWER-LIMB EXOSKELETON FOR REHABILITATION
% Part 2 - Mathematical Modelling & Validation of Exoskeleton Joint Plant
%
% Purpose:
% Formulates, verifies, and analyzes the continuous-time linear rotational
% dynamic model of the single-joint exoskeleton in the sagittal plane.
%
% Governing Differential Equation:
%   J * d2(theta)/dt2 + B * d(theta)/dt + K * theta = tau(t)
%
% Laplace Transform (Zero Initial Conditions):
%   (J*s^2 + B*s + K) * Theta(s) = Tau(s)
%
% Transfer Function:
%                 1
%   G(s) = -----------------
%           J*s^2 + B*s + K
%
% State-Space Form:
%   x = [theta; theta_dot]
%   x_dot = A*x + B_u*tau
%   y     = C*x + D*tau

%% 1. Load Parameters if Not Provided
if nargin < 1 || isempty(PARAM)
    script_dir = fileparts(mfilename('fullpath'));
    project_root = fileparts(script_dir);
    SUPPRESS_PARAM_DISPLAY = true;
    run(fullfile(project_root, 'scripts', 'project_parameters.m'));
end

%% 2. Plant Physical Parameters
J = PARAM.joint.inertia;    % kg*m^2
B = PARAM.joint.damping;    % N*m*s/rad
K = PARAM.joint.stiffness;  % N*m/rad

%% 3. Continuous-Time Transfer Function
s = tf('s');
G = 1 / (J*s^2 + B*s + K);

%% 4. State-Space Representation
A = [0, 1;
    -K/J, -B/J];
B_u = [0;
       1/J];
C = [1, 0];
D = 0;
G_ss = ss(A, B_u, C, D);

%% 5. Modal and Dynamic Stability Analysis
poles = pole(G);
zeros_sys = zero(G);

% Natural frequency, damping ratio
wn = sqrt(K / J);               % Undamped natural frequency (rad/s)
zeta = B / (2 * sqrt(J * K));   % Damping ratio (dimensionless)
dc_gain = dcgain(G);            % Steady-state gain (rad/(N*m)) = 1/K

% Controllability & Observability
Ctrb = ctrb(A, B_u);
Obsv = obsv(A, C);
rank_ctrb = rank(Ctrb);
rank_obsv = rank(Obsv);

%% 6. Step Response Analysis (1 N*m Input Torque)
t_span = 0:0.005:PARAM.simulation.time;
[y_step, t_step] = step(G, t_span);
step_info = stepinfo(G);

%% 7. Visualization - Open-Loop Step Response
hFig1 = figure('Name', 'Exoskeleton Open-Loop Step Response', 'NumberTitle', 'off', ...
    'Units', 'pixels', 'Position', [150, 150, 850, 550], 'Color', 'w');

plot(t_step, y_step, 'b-', 'LineWidth', 2.0); hold on;
yline(dc_gain, 'r--', 'LineWidth', 1.5, 'DisplayName', sprintf('Steady-State (%.2f rad)', dc_gain));
grid on; grid minor;
xlim([0, PARAM.simulation.time]);
ylim([0, max(y_step) * 1.15]);
xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Joint Angular Deflection \theta (rad)', 'FontSize', 11, 'FontWeight', 'bold');
title(sprintf('Open-Loop Step Response (1 N\\cdot m Input Torque) | \\zeta = %.4f, \\omega_n = %.3f rad/s', ...
    zeta, wn), 'FontSize', 12, 'FontWeight', 'bold');
legend('Angular Response \theta(t)', sprintf('DC Value (1/K = %.2f rad)', dc_gain), 'Location', 'northeast');

% Save Step Response Plot
fig1_path = fullfile(PARAM.dirs.results, 'fig02_open_loop_step_response.png');
try
    saveas(hFig1, fig1_path);
    fprintf('Saved open-loop step response plot to: %s\n', fig1_path);
catch ME
    warning('Could not save figure 2: %s', ME.message);
end

%% 8. Visualization - Frequency Response (Bode Plot)
hFig2 = figure('Name', 'Exoskeleton Plant Frequency Response', 'NumberTitle', 'off', ...
    'Units', 'pixels', 'Position', [200, 200, 850, 600], 'Color', 'w');

bode(G);
grid on; grid minor;
title('Open-Loop Bode Diagram of Exoskeleton Joint Plant', 'FontSize', 12, 'FontWeight', 'bold');

% Save Bode Plot
fig2_path = fullfile(PARAM.dirs.results, 'fig03_plant_bode_plot.png');
try
    saveas(hFig2, fig2_path);
    fprintf('Saved plant Bode plot to: %s\n', fig2_path);
catch ME
    warning('Could not save figure 3: %s', ME.message);
end

%% 9. Model Output Struct
model.J = J;
model.B = B;
model.K = K;
model.G = G;
model.G_ss = G_ss;
model.A = A;
model.B_u = B_u;
model.C = C;
model.D = D;
model.poles = poles;
model.zeros = zeros_sys;
model.wn = wn;
model.zeta = zeta;
model.dc_gain = dc_gain;
model.step_info = step_info;
model.rank_ctrb = rank_ctrb;
model.rank_obsv = rank_obsv;
model.is_stable = all(real(poles) < 0);

%% 10. Console Summary Output
fprintf('\n============================================================\n');
fprintf(' EXOSKELETON JOINT DYNAMIC MODEL VERIFICATION\n');
fprintf('============================================================\n');
fprintf(' Parameters: J = %.2f kg*m^2, B = %.2f N*m*s/rad, K = %.2f N*m/rad\n', J, B, K);
fprintf(' Transfer Function G(s):\n');
disp(G);
fprintf(' State-Space A Matrix:\n');
disp(A);
fprintf(' State-Space B Matrix:\n');
disp(B_u);
fprintf(' Poles:\n');
for p = 1:length(poles)
    fprintf('   p%d = %.4f + %.4fi rad/s\n', p, real(poles(p)), imag(poles(p)));
end
fprintf(' Natural Frequency (wn) : %.4f rad/s\n', wn);
if zeta < 1
    damping_label = 'Underdamped';
elseif zeta == 1
    damping_label = 'Critically damped';
else
    damping_label = 'Overdamped';
end
fprintf(' Damping Ratio (zeta)   : %.4f (%s)\n', zeta, damping_label);
fprintf(' DC Steady-State Gain   : %.4f rad/(N*m) (%.2f deg/(N*m))\n', dc_gain, rad2deg(dc_gain));
fprintf(' Controllability Rank   : %d / 2 (Full Controllability)\n', rank_ctrb);
fprintf(' Observability Rank     : %d / 2 (Full Observability)\n', rank_obsv);
fprintf(' BIBO Stability Status  : %s\n', mat2str(model.is_stable));
fprintf(' Open-Loop Rise Time    : %.4f s\n', step_info.RiseTime);
fprintf(' Open-Loop Settling Time: %.4f s (to within 2%% band)\n', step_info.SettlingTime);
fprintf(' Open-Loop Peak Overshoot: %.2f %%\n', step_info.Overshoot);
fprintf('============================================================\n');

if nargout == 0
    clear model;
end
end