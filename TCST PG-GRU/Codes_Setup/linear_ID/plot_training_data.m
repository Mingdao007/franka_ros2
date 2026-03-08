clear;clc;close all;

run("Par_ZPETC.m"); % initialize ZPETC
subFolderPath = fullfile('..', 'data', 'data_real_5');

% Load reference data for training
subFolderPath_ref = fullfile('..', 'data', 'reference/reference_training_1.csv');
ref = readtable(subFolderPath_ref);
r_training_Array = table2array(ref(:,2)); % Position reference data
vel_training_Array = table2array(ref(:,3)); % Velocity reference data

% Define time vector for plotting
t_end = Ts * (length(r_training_Array) - 1);
t = 0:Ts:t_end;

% Create tiled layout for multiple plots
ax1 = tiledlayout(4, 1, 'TileSpacing', 'Compact', 'Padding', 'Compact');

% Plot position and velocity reference data
nexttile;
plot(t, r_training_Array, 'DisplayName', 'Position', 'LineWidth', 1);
hold on; grid on; xlim([0, t_end]);
plot(t, vel_training_Array, 'DisplayName', 'Velocity', 'LineWidth', 1);
legend('Location', 'best');
ylabel('Reference [rad, rad/s]', 'Interpreter', 'latex');
hold off;
ax = gca; ylim([-130, 130]); % Set y-axis limit
 ax.FontSize = 15; set(gca, 'TickLabelInterpreter', 'latex');
title('Data Sets Example', 'Interpreter', 'latex');

% Load data set 1
filename_training1 = fullfile(subFolderPath, sprintf('data_%d.csv', 1));
data_training1 = readtable(filename_training1);
r_training_Array1 = table2array(data_training1(:,1)); % Reference position
uff_training_Array1 = table2array(data_training1(:,2)); % Feedforward control input
y_training_Array1 = table2array(data_training1(:,3)); % Output data
u_training_Array1 = table2array(data_training1(:,4)); % Control input

% Load data set 2
filename_training2 = fullfile(subFolderPath, sprintf('data_%d.csv', 2));
data_training2 = readtable(filename_training2);
uff_training_Array2 = table2array(data_training2(:,2)); % Feedforward control input
y_training_Array2 = table2array(data_training2(:,3)); % Output data
u_training_Array2 = table2array(data_training2(:,4)); % Control input

% Plot feedforward control inputs for both data sets
nexttile;
h1 = plot(t, uff_training_Array1', 'LineWidth', 1, 'DisplayName', 'Set 1');
hold on;
h2 = plot(t, uff_training_Array2', 'LineWidth', 1, 'DisplayName', 'Set 2');
legend([h1, h2], {'Data Set 1', 'Data Set 2'}, 'Location', 'best');
uistack(h1, 'top'); % Bring Set 1 to the front
grid on; xlim([0, t_end]);
ylabel('Feedforward [Nm]', 'Interpreter', 'latex');
hold off;
ax = gca; set(gca, 'TickLabelInterpreter', 'latex');  ax.FontSize = 15;

% Plot control inputs for both data sets
nexttile;
h1 = plot(t, u_training_Array1', 'LineWidth', 1, 'DisplayName', 'Set 1');
hold on;
plot(t, u_training_Array2', 'LineWidth', 1, 'DisplayName', 'Set 2');
grid on; xlim([0, t_end]);
ylabel('Input [Nm]', 'Interpreter', 'latex');
hold off;
uistack(h1, 'top');
ax = gca; set(gca, 'TickLabelInterpreter', 'latex');  ax.FontSize = 15;

% Plot output data for both data sets
nexttile;
h1 = plot(t, y_training_Array1', 'LineWidth', 1, 'DisplayName', 'Set 1');
hold on;
plot(t, y_training_Array2', 'LineWidth', 1, 'DisplayName', 'Set 2');
grid on; xlim([0, t_end]);
ylabel('Output [rad]', 'Interpreter', 'latex');
hold off;
xlabel('Time [s]', 'Interpreter', 'latex');
uistack(h1, 'top');
ax = gca; set(gca, 'TickLabelInterpreter', 'latex');  ax.FontSize = 15;

% Set the figure size and position
set(gcf, 'Units', 'normalized');
set(gcf, 'OuterPosition', [0 0 0.5 1]);

% Optional: export plot as pdf
exportgraphics(ax1, 'figures/long_reference_for_training.pdf', 'ContentType', 'vector')