function data = readLog_mpu9250(filename)
disp([mfilename '>> Reading file ' filename ' ...'])
% Open file
fid = fopen(filename);
% Read file as a whole:
d = textscan(fid,'%f %f %f %f %f %f %f %f',...
'Delimiter',',');
% Ignore first and last lines, as they may be corrupted:
data.time_s = 1e-6 * d{1}(2:end-1);
data.acc_mps2 = [d{2}(2:end-1) ...
                 d{3}(2:end-1) ...
                 d{4}(2:end-1)];
data.gyr_degps = [d{5}(2:end-1) ...
                  d{6}(2:end-1) ...
                  d{7}(2:end-1)];
data.temp_degC = d{8}(2:end-1);
             
% Close file after data extraction:
fclose(fid);
end