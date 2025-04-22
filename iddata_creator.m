Ts = 0.001;
y = readtable('data_DAB_solo.csv');
x = readtable('data_DAB_solo_input.csv');
y1 = table2array(y(:,2));
x1 = table2array(x(:,2));
DAB_solo = iddata(y1,x1,Ts);
values = table2array(T(:,2));
time = table2array(T(:,1));

% Plot with red line
plot(time, values, 'r', 'LineWidth', 1.5);

% Add labels and legend
xlabel('Czas (s)');
ylabel('Napięcie (V)');
title('Napięcie wyjściowe DAB');
legend('Napięcie wyjściowe DAB');

% Display the grid for better visualization
grid on;