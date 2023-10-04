%% This program serves as the main program for adjusting control parameters.
%% Since our actual entry points for the experiment are discrete values, the entry points are not considered as control parameters in the ILC part.
%% This version does not consider whether the corrected trajectory will encounter obstacles.
%% This version does not optimize the program rate, i.e., unnecessary parts are not removed.
%% Li Murong, July 2021

% Load control parameters:
load('L');
load('spinning_z');
load('arc_success');
load('initialPoint');
load('world');

for kk = 2
    fprintf('Entering iteration for trajectory %d...\n', kk);
    depth_control = L{kk};  % Keep the first useless point as 0.
    spinning_zi  =  spinning_z{kk};  % Already converted to relative rotation angle.
    initialPoint0 = initialPoint{kk}; % Entry point
    % initialPoint0 = [25 25 0];
    % Target point:
    Goal =  world.goal;
    % Initialize iteration count:
    Iteration = 20; 
    % Initialize control variables
    % u = [depth_control*1e-3,spinning_zi]; % Control variables are rotation angle and rotation depth, without changing entry point
    u = [initialPoint0(1:2), depth_control(end)]; % Three variables correspond to offsets in three directions
    u = u';
    [r, l] = size(u); % Record the size of control variables u
    n = r * l; % Number of control variables

    % Initialize PD matrices, P and D are both n*3 matrices, in this case, n = 8, the first four variables are rotation depth settings, the last four are rotation angle settings
    % The three values in the ith row represent the influence of the ith rotation depth/angle control variable on the offset in three directions
    % The first row and the fifth row are all zeros, and the weight of PD values in this row has no effect and is set to 1
    % Consult literature for the right values to choose...
    P = [0.2 0 0; 0 0.2 0; 0 0 -0.1];
    D = [-0.03 0 0; 0 -0.02 0; 0 0 0.015];

    % Initialize error and error gradient
    error = [];
    error_dot = []; 
    E = [];

    % Enter the main iteration loop:
    for i = 1: Iteration 
        fprintf('Step %d control variables are %f\t %f\t %f\t\n', i, u(1), u(2), u(3));
        % [NeedlePos, target] = pathPlanningFEM(initialPoint0, u, spinning_zi, world);
        [NeedlePos, target] = pathPlanningFEM([u(1), u(2), 0], [depth_control(1:end-1); u(3)], spinning_zi, world);
        Needle_tip = NeedlePos(end,:);
        Goal_conter = target(end,:);
        delta = (Goal_conter - Needle_tip) * 1e3; % Since control variables are in mm, convert here as well
        fprintf('Step %d error values are %f\t %f\t %f\t\n', i, delta(1), delta(2), delta(3));
        delta = delta'; % Transpose
        error = [error, delta];
        if (size(error, 2) > 1)     % D learning will be added if the iteration is greater than 1
            error_dot = error(:, end) - error(:, end-1); 
        else
            u = u + P * delta;
            E(i) = sqrt(error(1, i)^2 + error(2, i)^2 + error(3, i)^2);
            fprintf('At this iteration step %d, absolute error is %f\n', i, E(i));
            plot_NeedleWithGoal(NeedlePos, arc_success, kk, world, target);  
            if E(i) < 1
                break;
            end
            continue; 
        end
        u = u + P * delta + D * error_dot;
        E(i) = sqrt(error(1, i)^2 + error(2, i)^2 + error(3, i)^2);
        fprintf('At this iteration step %d, absolute error is %f\n', i, E(i));
        plot_NeedleWithGoal(NeedlePos, arc_success, kk, world, target);  
        if E(i) < 1
            break;
        end
    end
    t = 1:length(E);
    figure(kk)
    plot(t, E, 'ro-'); hold on; % Observe if it converges
    plot(t, error(1, :), 'bo-'); hold on;
    plot(t, error(2, :), 'mo-'); hold on;
    plot(t, error(3, :), 'ko-'); hold on;
    legend('Total Error', 'X', 'Y', 'Z');
end
