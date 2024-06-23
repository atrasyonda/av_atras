% Baca data dari file Excel
filename = 'DATA PLOT MATLAB.xlsx'; % Nama file Excel
sheet = 5;              % Nomor sheet (1 berarti sheet pertama)
xlRange = 'A:O';        % Rentang data yang akan dibaca, misalnya kolom A dan B

% Baca data dari Excel
data = xlsread(filename, sheet, xlRange);

% Ekstrak kolom data
time = data(:, 1); % Kolom pertama sebagai sumbu x
delta = data(:, 2); % Kolom kedua sebagai sumbu y
accel = data(:, 3);
odom_yaw = data(:,4);
odom_x = data(:,5);
odom_y = data(:,6);
odom_vel = data(:,7);
ref_yaw = data(:,8);
ref_x = data(:,9);
ref_y = data(:,10);
ref_vel = data(:,11);
error_X = data(:,12);
error_Y = data(:,13);
error_Psi = data(:,14);
error_Vel = data(:,15);

% Plot data
% 
% figure; % Membuat figure baru
% plot(odom_x, odom_y, '-',Color= 'blue'); % Plot
% hold on;
% plot(ref_x, ref_y, '-', Color= 'red');
% xlabel('X (m)'); % Label sumbu x
% ylabel('Y (m) '); % Label sumbu y
% title('Lintasan Mobil Pada Simulasi ROS'); % Judul plot
% legend('Odometri Mobil', 'Referensi trayektori')
% hold off;
% 
% figure; % Membuat figure baru
% plot(time, odom_x, '-',Color= 'blue'); % Plot
% hold on;
% plot(time, ref_x, '-', Color= 'red');
% xlabel('Waktu (s)'); % Label sumbu x
% ylabel('X (m) '); % Label sumbu y
% title('Grafik Posisi X Terhadap Waktu'); % Judul plot
% legend('Odometri Mobil', 'Referensi trayektori')
% hold off;
% 
% figure;
% plot(time, odom_y, '-', Color='blue');
% hold on;
% plot(time, ref_y, '-', Color='red');
% xlabel('Waktu (s)');
% ylabel('Y (m)');
% title('Grafik Posisi Y Terhadap Waktu');
% legend('Odometri Mobil', 'Referensi Trayektori')
% hold off;
% 
% figure;
% plot(time, odom_yaw, '-', Color='blue');
% hold on;
% plot(time, ref_yaw, '-', Color='red');
% xlabel('Waktu (s)');
% ylabel('Orientasi (Rad)');
% title('Grafik Orientasi Mobil Terhadap Waktu');
% legend('Odometri Mobil', 'Referensi Trayektori')
% hold off;
% 
% figure;
% plot(time, odom_vel, '-', Color='blue');
% hold on;
% plot(time, ref_vel, '-', Color='red');
% xlabel('Waktu (s)');
% ylabel('Kecepatan (m/s)');
% title('Grafik Kecepatan Mobil Terhadap Waktu');
% legend('Odometri Mobil', 'Referensi Trayektori')
% hold off;

figure;
plot (time, delta, '-', Color='blue');
xlabel('Waktu (s)')
ylabel('Sudut Steering (rad)')
title('Grafik Steering Terhadap Waktu');

figure;
plot (time, accel,'-', Color='blue');
xlabel('Waktu (s)');
ylabel('Akselerasi (m/s^2)');
title('Grafik Akselerasi Terhadap Waktu');

figure;
plot(time,error_X,'-', Color='red');
hold on;
plot(time,error_Y,'-', Color='green');
hold on;
plot(time,error_Psi,'-', Color='blue');
hold on;
plot(time,error_Vel,'-', Color='magenta');
legend('Error X','Error Y', 'Error Psi', 'Error Kecepatan');
xlabel('Waktu (s)');
ylabel('Error');
title('Grafik Error Terhadap Waktu');
hold off;





