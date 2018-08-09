fileID = fopen(logfile, 'r');
fileDir = dir(logfile);
if(fileID)
   [LogHeader.start_time,count] = fread(fileID, [1 1], 'uint32=>uint32'); 
   [LogHeader.log_perid,count] = fread(fileID, [1 1], 'uint32=>uint32'); 
   [LogHeader.element_num,count] = fread(fileID, [1 1], 'uint32=>uint32'); 
   [LogHeader.header_size,count] = fread(fileID, [1 1], 'uint32=>uint32');
   [LogHeader.filed_size,count] = fread(fileID, [1 1], 'uint32=>uint32');
   for n = 1:LogHeader.element_num
       [LogHeader.element_info(n).name,count] = fread(fileID, [1 20], 'uint8=>char'); 
       cell{1,n} = LogHeader.element_info(n).name;
       [LogHeader.element_info(n).type,count] = fread(fileID, [1 1], 'uint32=>uint32');
       linebuff.add(n) = false;
   end

   field_num = (fileDir.bytes - LogHeader.header_size)/LogHeader.filed_size;
   % TODO: load for other type
   for n = 1:field_num
        [LogField(n,:),count] = fread(fileID, [1 LogHeader.element_num], 'float=>float');
   end
end
fclose(fileID);

index = strfind(cell, 'GPS_X');
isOne = cellfun(@(x)isequal(x,1),index);
[row,gps_x_col] = find(isOne);
index = strfind(cell, 'GPS_Y');
isOne = cellfun(@(x)isequal(x,1),index);
[row,gps_y_col] = find(isOne);
index = strfind(cell, 'BARO_ALT');
isOne = cellfun(@(x)isequal(x,1),index);
[row,baro_z_col] = find(isOne);
index = strfind(cell, 'GPS_VN');
isOne = cellfun(@(x)isequal(x,1),index);
[row,gps_vx_col] = find(isOne);
index = strfind(cell, 'GPS_VE');
isOne = cellfun(@(x)isequal(x,1),index);
[row,gps_vy_col] = find(isOne);
index = strfind(cell, 'BARO_VEL');
isOne = cellfun(@(x)isequal(x,1),index);
[row,baro_vz_col] = find(isOne);
index = strfind(cell, 'ACC_FILTER_X');
isOne = cellfun(@(x)isequal(x,1),index);
[row,acc_x_col] = find(isOne);
index = strfind(cell, 'ACC_FILTER_Y');
isOne = cellfun(@(x)isequal(x,1),index);
[row,acc_y_col] = find(isOne);
index = strfind(cell, 'ACC_FILTER_Z');
isOne = cellfun(@(x)isequal(x,1),index);
[row,acc_z_col] = find(isOne);
index = strfind(cell, 'GYR_FILTER_X');
isOne = cellfun(@(x)isequal(x,1),index);
[row,gyr_x_col] = find(isOne);
index = strfind(cell, 'GYR_FILTER_Y');
isOne = cellfun(@(x)isequal(x,1),index);
[row,gyr_y_col] = find(isOne);
index = strfind(cell, 'GYR_FILTER_Z');
isOne = cellfun(@(x)isequal(x,1),index);
[row,gyr_z_col] = find(isOne);
index = strfind(cell, 'MAG_FILTER_X');
isOne = cellfun(@(x)isequal(x,1),index);
[row,mag_x_col] = find(isOne);
index = strfind(cell, 'MAG_FILTER_Y');
isOne = cellfun(@(x)isequal(x,1),index);
[row,mag_y_col] = find(isOne);
index = strfind(cell, 'MAG_FILTER_Z');
isOne = cellfun(@(x)isequal(x,1),index);
[row,mag_z_col] = find(isOne);
index = strfind(cell, 'QUATERNION_W');
isOne = cellfun(@(x)isequal(x,1),index);
[row,qw_col] = find(isOne);
index = strfind(cell, 'QUATERNION_X');
isOne = cellfun(@(x)isequal(x,1),index);
[row,qx_col] = find(isOne);
index = strfind(cell, 'QUATERNION_Y');
isOne = cellfun(@(x)isequal(x,1),index);
[row,qy_col] = find(isOne);
index = strfind(cell, 'QUATERNION_Z');
isOne = cellfun(@(x)isequal(x,1),index);
[row,qz_col] = find(isOne);
index = strfind(cell, 'ROLL');
isOne = cellfun(@(x)isequal(x,1),index);
[row,roll_col] = find(isOne);
index = strfind(cell, 'PITCH');
isOne = cellfun(@(x)isequal(x,1),index);
[row,pitch_col] = find(isOne);
index = strfind(cell, 'YAW');
isOne = cellfun(@(x)isequal(x,1),index);
[row,yaw_col] = find(isOne);