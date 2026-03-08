clear;clc;close all;
run("Par_ZPETC.m");
subFolderPath = fullfile('..', 'data', 'data_real_5');

%% loop for all test data (i = 9, 10, 11)
for i = 9:11
    filename_training = fullfile(subFolderPath, sprintf('data_%d.csv', i));
    data_training = readtable(filename_training);
    r_training_Array = table2array(data_training(:,1));
    u_training_Array = table2array(data_training(:,4));
    u_training_Array = zeros(size(u_training_Array));

    % Set filtering parameters
    Fs = 1 / Ts; 
    order = 3; % Filter order
    window_size = round(Fs * 0.07); % Define the window size for smoothing
    if mod(window_size, 2) == 0 % Ensure the window size is odd for the filter
        window_size = window_size + 1;
    end
    pad_size = (window_size-1) / 2;

    % Apply padding and Savitzky-Golay filtering to smooth the reference data
    r_padded = padarray(r_training_Array, pad_size, 'replicate', 'both');
    r_smoothed = sgolayfilt(r_padded, order, window_size); % Apply filter
    r_smoothed = sgolayfilt(r_smoothed, order, window_size); % Apply filter twice for stronger smoothing
    r_smoothed = r_smoothed(pad_size+1:end-pad_size); % Remove padding
    
    % Apply ZPETC
    UFF = F_ZPETC(ZPETC, u_training_Array, r_smoothed, num_p, num_d);
    
    
    % subFolderPath_new = fullfile('..', 'data', 'data_real_5_uphy_sgolayfilt_padding');
    % if ~exist(subFolderPath_new, 'dir')
    %     mkdir(subFolderPath_new);
    % end
    % filename = fullfile(subFolderPath_new, sprintf('data_%d_application.csv', i));
    % title_table = {'uff','rd_filtered'};
    % result_table = table(UFF, r_smoothed, 'VariableNames', title_table);
    % writetable(result_table, filename);
end

t_end = Ts*(length(r_training_Array)-1);
t = 0:Ts:t_end;
fig = figure();
plot(t,UFF, 'LineWidth', 1);xlim([0, t_end]);grid on;hold on;
xlim([0, t_end]);
grid on;
set(fig, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);
ylabel('$\hat{u}^d_{phy}$ [rad]', 'Interpreter', 'latex');
xlabel('Time [s]', 'Interpreter', 'latex');