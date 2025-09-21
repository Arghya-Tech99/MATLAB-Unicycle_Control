% Time vector
t = linspace(0, 2*pi, 500);  % 0 to 2*pi for full cycle

%Get the real parameters from simulink model
simOut = sim('VelocityControl');
xreal = simOut.get('x');
yreal = simOut.get('y');
xdes = simOut.get('xd');
ydes = simOut.get("yd");
% Reference trajectory (figure-8 shape)
%x_ref = sin(t);
%y_ref = sin(t).*cos(t) ;

figure;
plot(xdes, ydes, 'g-', 'LineWidth', 2);
%hold on;
%plot(xreal, yreal, 'b-');
grid on;
axes equal;

