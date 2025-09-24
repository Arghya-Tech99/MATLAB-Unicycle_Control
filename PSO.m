% Defining the Parameters
num_particles = 6;
max_iterations = 21;
c1 = 2; % Cognitive parameter
c2 = 2; % Social parameter
w_max = 0.9; % Maximum Inertia weight
w_min = 0.4; % Minimum Inertia weight

% PID Gains (Kp, Ki, Kd)
lb = [1, 1, 1]; % Lower bounds
ub = [200, 200, 200]; % Upper bounds

num_variables = 3; % Kp, Ki, Kd

% Initialize Particles
particles.position = zeros(num_particles, num_variables); % zeros(num_rows, num_columns) creates array of zeroes
particles.velocity = zeros(num_particles, num_variables);
particles.pbest_position = zeros(num_particles, num_variables);
particles.pbest_cost = inf(num_particles, 1); % inf(num_rows, num_columns) creates array where all elements are infinite

gbest_position = zeros(1, num_variables);
gbest_cost_history = zeros(1, max_iterations);
gbest_cost = inf;

% Initialize positions randomly within bounds
for i = 1:num_particles
    particles.position(i, :) = lb + (ub - lb) .* rand(1, num_variables);
end

% PSO Main Loop

for iter = 1:max_iterations
    % Update inertia weight linearly
    w = w_max - (w_max - w_min) * (iter / max_iterations);
    for i = 1:num_particles
        % Simulate and calculate cost for current particle's position
        cost = calculate_fitness(particles.position(i, :));

        % Update personal best (pbest)
        if cost < particles.pbest_cost(i)
            particles.pbest_cost(i) = cost;
            particles.pbest_position(i, :) = particles.position(i, :);
        end

        % Update global best (gbest)
        if cost < gbest_cost
            gbest_cost = cost;
            gbest_position = particles.position(i, :);
        end
        % Store the current global best cost
        gbest_cost_history(iter) = gbest_cost;

    end

    % Update particle velocities and positions
    for i = 1:num_particles
        r1 = rand(1, num_variables);
        r2 = rand(1, num_variables);
        
        particles.velocity(i, :) = w * particles.velocity(i, :) + ...
            c1 * r1 .* (particles.pbest_position(i, :) - particles.position(i, :)) + ...
            c2 * r2 .* (gbest_position - particles.position(i, :));

        particles.position(i, :) = particles.position(i, :) + particles.velocity(i, :);
        
        % Clamp positions to stay within bounds
        particles.position(i, :) = max(particles.position(i, :), lb);
        particles.position(i, :) = min(particles.position(i, :), ub);
    end

    fprintf('Iteration %d, Best Cost: %f\n', iter, gbest_cost);
end

% Final Result
fprintf('\nOptimization complete!\n');
fprintf('Optimal PID Parameters:\n');
fprintf('Kp = %f\n', gbest_position(1));
fprintf('Ki = %f\n', gbest_position(2));
fprintf('Kd = %f\n', gbest_position(3));

% fprintf('Kpw = %f\n', gbest_position(4));
% fprintf('Kiw = %f\n', gbest_position(5));
% fprintf('Kdw = %f\n', gbest_position(6));
fprintf('Best Performance (Cost): %f\n', gbest_cost);

% Plot the response of the system with the optimal PID parameters
% figure;
% plot(1:max_iterations, gbest_cost_history, 'b-', 'LineWidth', 2);
% title('PSO Convergence Curve');
% xlabel('Iteration');
% ylabel('Best IAE (Fitness Value)');
% grid on;