addpath("")

%WiFi settings
IP = '192.168.4.1';
PORT_NUMBER = 80;

%USB settings
port_name = '/dev/cu.usbmodemF412FA70B6442';
baudrate = 230400;

%Recording time
Tmax = 40;

%Communication method
method = 'WiFi';

%Gains
uBar1 = 0.55;
sigma = 9;
coeffs = K(uBar1, sigma);
K1_send = [coeffs(2),  coeffs(1), coeffs(3)];

uBar2 = 0.5;
sigma = 9;
coeffs = K(uBar2, sigma);
K2_send = [coeffs(2),  coeffs(1), coeffs(3)];
gains = [39, uBar1, uBar2, 3.5];

K_send = [K1_send, K2_send, gains]

if strcmp(method,'WiFi')
    [log_time, data_values, line_idx] = get_data_WiFi(IP, PORT_NUMBER, Tmax, K_send);
elseif strcmp(method,'USB')
    [log_time, data_values, line_idx] = get_data_USB(port_name,Tmax,baudrate, K_send);
else
    print('Use a valid connection method')
end

%Display logged variable names
disp(data_values.keys)

% Plot logged variables
figure; 
hold on;
legend_labels = data_values.keys;
for i = 1:numel(legend_labels)
    d = legend_labels{i};
    v = data_values(d);
    plot(log_time, v, 'DisplayName', d);
end
hold off;
legend('Location', 'best');
grid on;


