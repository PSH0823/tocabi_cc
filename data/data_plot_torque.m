%%
clc
clear all
close all

alpha = 30;
epsilon = 0;

% output_dir = '/home/sanghyuk/tocabi_ws/src/tocabi_cc/data/torque/wo_self_col';

output_dir = sprintf( ...
    '/home/sanghyuk/tocabi_ws/src/tocabi_cc/data/torque/alpha_%d', ...
    alpha);

% output_dir = sprintf( ...
%     '/home/sanghyuk/tocabi_ws/src/tocabi_cc/data/torque/alpha_%d_eps_%d', ...
%     alpha, epsilon);

if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

data = readmatrix('torque_sum_log.txt');
torque_sol = data(:, 1:33);

data = readmatrix('torque_idn_log.txt');
torque_id = data(:, 1:33);

% 
data = readmatrix('torque_pd_log.txt');
torque_pd = data(:, 1:33);

for torque_cnt = 13:1:15
    fig = figure();
    sgtitle(sprintf('WAIST JOINT TORQUE: %d', torque_cnt - 12))
    % torque_cnt = 2
    plot(torque_sol(:,torque_cnt))    
    hold on 
    plot(torque_id(:,torque_cnt))
    plot(torque_pd(:,torque_cnt))
    legend('sum','idn','pd')

    filename = fullfile(output_dir, ...
        sprintf('waist_joint_%d_torque.png', torque_cnt - 12));

    exportgraphics(fig, filename, 'Resolution',300);

    close(fig)
end

for torque_cnt = 16:1:23
    fig = figure();
    sgtitle(sprintf('ARM JOINT TORQUE: %d', torque_cnt - 15))
    plot(torque_sol(:,torque_cnt))    
    hold on 
    plot(torque_id(:,torque_cnt))
    plot(torque_pd(:,torque_cnt))
    legend('sum','idn','pd')

    filename = fullfile(output_dir, ...
        sprintf('arm_joint_%d_torque.png', torque_cnt - 15));

    exportgraphics(fig, filename, 'Resolution', 300);

    close(fig)
end
%%

% data = readmatrix('computation_time_log.txt');
% dt = data(:, 1);
% figure()
% plot(dt(:))