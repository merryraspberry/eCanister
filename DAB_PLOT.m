% Sampling time
Ts = 0.001;

% Load the data
y = readtable('data_DAB_solo.csv'); % Output data
x = readtable('data_DAB_solo_input.csv'); % Input data

% Extract values
y1 = table2array(y(:,2)); % Output values (Napięcie wyjściowe DAB)
x1 = table2array(x(:,2)); % Input values (Napięcie zadane)
time_output = table2array(y(:,1)); % Time for output
time_input = table2array(x(:,1)); % Time for input

% Plot the output with red line
figure;
plot(time_output, y1, 'r', 'LineWidth', 1.5); % Napięcie wyjściowe DAB
hold on;

% Plot the input with blue dashed line
plot(time_input, x1, 'b--', 'LineWidth', 1.5); % Napięcie zadane

% Add labels, title, legend, and grid
xlabel('Czas (s)');
ylabel('Napięcie (V)');
title('Napięcie wyjściowe i napięcie zadane DAB');
legend('Napięcie wyjściowe DAB', 'Napięcie zadane', 'Location', 'Best');
grid on;

% Finalize plot
hold off;