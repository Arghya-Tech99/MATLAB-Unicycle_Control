function cost = calculate_fitness(pid_gains)
    % pid_gains- vector [Kp, Ki, Kd]
    
    % Simulate the unicycle system
    [error] = unicycle_model(pid_gains);
    
    % Calculate the cost (e.g., IAE)
    Err = abs(error);
    cost = sum(Err);
end