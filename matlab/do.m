log_mpu9250 = readLog_mpu9250('capture.txt');
log_mpu9250.dt_s = log_mpu9250.time_s(2:end) - log_mpu9250.time_s(1:end-1);

% Plot:
figure(1);
subplot(4,1,1);
plot(log_mpu9250.dt_s);
ylabel('dt [s]')
subplot(4,1,2);
plot(log_mpu9250.acc_mps2);
ylabel('acc. [m/s^2]')
subplot(4,1,3);
plot(log_mpu9250.gyr_degps);
ylabel('omega [deg/s]')
subplot(4,1,4);
plot(log_mpu9250.temp_degC);
ylabel('Temp. [deg. C]')
