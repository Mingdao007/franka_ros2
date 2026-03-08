clear;close all;clc;

%% load data
data = readmatrix('data_real_Linear_ID\data_1.csv'); % this data is lost, you can collect a new set or just use data in folder "data/data_5"
r_data = data(:,1)';
uff_data = data(:,2)';
y_data = data(:,3)';

%% parameter initialization
Ts = 5e-4;
t_end = (length(r_data)-1)*Ts;
t = 0:Ts:t_end;
t = t';

% feedback controller initialization
load("ctrl10.mat"); 
Cfbz = c2d(shapeit_data.C_tf, Ts, "tustin");
[Ak_fb,Bk_fb,Ck_fb,Dk_fb] = tf2ss(Cfbz.num{1},Cfbz.den{1});
if isempty(Dk_fb)
    Dk_fb = [0];
end

%% System identification
options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt',... 
    'StepTolerance',1e-10,'FunctionTolerance',1e-10,'MaxFunctionEvaluations',1e3); 

% Define loss function for system identification
f = @(par) loss_func(par, Ts, r_data, uff_data, y_data, Ak_fb, Bk_fb, Ck_fb, Dk_fb);

% Lower bounds for parameter estimation
lb = 1e-16 * ones(1,6); 
res_threshold = 100; 
max_iterations = 500; 
res = inf * ones(max_iterations, 1);
para = zeros(6, max_iterations); 

for i = 1:max_iterations
    disp(['iteration = ', num2str(i)]); % Display current iteration
    
    if i == 1 || res(i-1) >= res_threshold % Condition to run optimization
        par_prior = [1e-6, 1e-6, 1e-1, 1e-5, 1e-6, 1e-6]; % Initial guess for parameters
        
        % Randomly modify the initial guess by a factor
        for k = 1:length(par_prior)
            factor = 10;
            random_factor = 1/factor + (factor - 1/factor) * rand;
            par_prior(k) = par_prior(k) * random_factor;
        end
        par_init = par_prior; % Set the initialized parameters

        try
            % Run the least squares non-linear optimization
            [para(:,i), res(i)] = lsqnonlin(f, par_init, lb, [], options);
        catch
            res(i) = inf; % In case of failure, set residual to infinity
        end
        disp(res(i)); % Display the current residual
    else
        break; % Stop iterations if residual is below threshold
    end
end

%% Display results
[min_res, idx] = min(res); 
disp(para(:,idx)); 
par = para(:,idx); 

%% Prediction
n_sim = size(r_data, 2); 
[J1, J2, k1, b1, kv1, kv2] = deal(par(1), par(2), par(3), par(4), par(5), par(6));
A = [zeros(2), eye(2);...
    -k1/J1, k1/J1, -(b1+kv1)/J1, b1/J1;...
    k1/J2, -k1/J2, b1/J2, -(b1+kv2)/J2];
B = [0 0 1/J1 0]';
C = [0 1 0 0];
D = 0;

% Convert continuous-time system to discrete-time system
sys_d = c2d(ss(A,B,C,D), Ts, 'zoh');
Ak = sys_d.A; 
Bk = sys_d.B; 
Ck = sys_d.C; 
Dk = sys_d.D;

[nx, nu] = size(Bk); % Number of states and inputs
ny = nu; % Output dimension

% Initialize state vectors
xk = zeros(nx, n_sim+1); % System state
uk = zeros(nu, n_sim); % Control input
yk = zeros(ny, n_sim); % System output

% Feedback controller state initialization
nx_fb = size(Ak_fb, 1);
nu_fb = size(Dk_fb, 1);
xk_fb = zeros(nx_fb, n_sim+1); % Feedback controller state
uk_fb = zeros(nu_fb, n_sim); % Feedback controller input

for k = 1:n_sim
    % Calculate the current system output
    yk(:, k) = Ck * xk(:, k) + Dk * uk(:, k);

    % Update feedback controller states
    xk_fb(:, k+1) = Ak_fb * xk_fb(:, k) + Bk_fb * (r_data(:, k) - yk(:, k));
    uk_fb(:, k) = Ck_fb * xk_fb(:, k) + Dk_fb * (r_data(:, k) - yk(:, k));
    
    % Calculate the total control input
    uk(:, k) = uk_fb(:, k) + uff_data(:, k);

    % Update system states
    xk(:, k+1) = Ak * xk(:, k) + Bk * uk(:, k);
end

Y_pred = yk; % Predicted output

%% Calculate and display RMSE
disp('rmse =')
disp(sqrt(mse(y_data - Y_pred(1:length(t))))) % Display root mean square error (RMSE)

%% Plot and display results
close all;
figure;
ax1 = tiledlayout(4, 1, 'TileSpacing', 'Compact', 'Padding', 'Compact'); 

% Plot reference data
nexttile;
plot(t, r_data, 'LineWidth', 1, 'DisplayName', '$r^d$');
xlim([0, t_end]);
ylabel('$\mathrm{r^d}$ [rad]', 'Interpreter', 'latex');
title('Forward Identification Results', 'Interpreter', 'latex');
grid on;
ax = gca; set(gca, 'TickLabelInterpreter', 'latex');
ax.FontSize = 13;

% Plot feedforward control input
nexttile;
plot(t, uff_data, 'LineWidth', 1, 'DisplayName', '$u^d_{ff}$');
xlim([0, t_end]);
ylabel('$\mathrm{u^d_{ff}}$ [Nm]', 'Interpreter', 'latex');
grid on;
box on;
ax = gca; set(ax, 'YTickLabelMode', 'auto'); 
set(gca, 'TickLabelInterpreter', 'latex');
ax.FontSize = 13; ax.YAxis.Exponent = 0;

% Plot total control input
nexttile;
plot(t, uk, 'LineWidth', 1, 'DisplayName', '$\hat{u}$');


%% save parameters
save('Linear_system_LM.mat', 'Ak', 'Bk');

%% loss function
function loss = loss_func(par,Ts,r_data,uff_data,y_data,Ak_fb,Bk_fb,Ck_fb,Dk_fb)
n_sim = size(r_data,2);

[J1, J2, k1, b1, kv1, kv2] = deal(par(1), par(2), par(3), par(4), par(5), par(6));

A = [zeros(2),eye(2);...
    -k1/J1,k1/J1,-(b1+kv1)/J1,b1/J1;...
    k1/J2,-k1/J2,b1/J2,-(b1+kv2)/J2];
B = [0 0 1/J1 0]';
C = [0 1 0 0];
D = 0;

% Convert to discrete-time state-space system
sys_d = c2d(ss(A,B,C,D), Ts, 'zoh');
Ak = sys_d.A;
Bk = sys_d.B;
Ck = sys_d.C;
Dk = sys_d.D;

[nx,nu] = size(Bk);
ny = nu;

xk = zeros(nx,n_sim+1);
uk = zeros(nu,n_sim);
yk = zeros(ny,n_sim);

nx_fb = size(Ak_fb, 1);
nu_fb = size(Dk_fb, 1);

xk_fb = zeros(nx_fb,n_sim+1);
uk_fb = zeros(nu_fb,n_sim);

for k = 1:n_sim
    % current output
    yk(:,k) = Ck*xk(:,k) + Dk*uk(:,k);

    % feedback controller
    xk_fb(:,k+1) = Ak_fb*xk_fb(:,k) + Bk_fb*(r_data(:,k)-yk(:,k));
    uk_fb(:,k) = Ck_fb*xk_fb(:,k) + Dk_fb*(r_data(:,k)-yk(:,k));
    uk(:,k) = uk_fb(:,k) + uff_data(:,k);

    % update states
    xk(:,k+1) = Ak*xk(:,k) + Bk*uk(:,k);
end

loss = yk - y_data;
end