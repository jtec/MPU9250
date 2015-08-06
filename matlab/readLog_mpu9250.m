function data = readLog_mpu9250(filename)
disp([mfilename '>> Reading file ' filename ' ...'])
% Open file
fid = fopen(filename);
% Read file line by line:
d = textscan(fid,'%f %f %f %f %f %f %f %f',...
'Delimiter',',', 'EmptyValue',0);
% Ignore first and last lines, as they may be corrupted.
maxl = length(d{1});
for k=1:length(d)
    if maxl > length(d{k})
       maxl = length(d{k});
    end
end
data.time_s = 1e-6 * d{1}(2:maxl-1);
data.acc_mps2 = [d{2}(2:maxl-1) ...
                 d{3}(2:maxl-1) ...
                 d{4}(2:maxl-1)];
data.gyr_degps = [d{5}(2:maxl-1) ...
                  d{6}(2:maxl-1) ...
                  d{7}(2:maxl-1)];
data.temp_degC = d{8}(2:maxl-1);
             
% Close file after data extraction:
fclose(fid);
end