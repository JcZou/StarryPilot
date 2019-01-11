function log_parse(logfile)

% defiine data type
data_type = ["int8=>int8", "uint8=>uint8", "int16=>int16", "uint16=>uint16",...
    "int32=>int32", "uint32=>uint32", "float=>float", "double=>double"];
skip_bytes = [7, 7, 6, 6, 4, 4, 4, 0];

% open log file
fileID = fopen(logfile, 'r');
fileDir = dir(logfile);
if ~fileID
    disp('log file open fail');
    return;
end

%% read log header

% read log filed
LogHeader.num_filed = fread(fileID, 1, 'uint8=>uint8');

for n = 1:LogHeader.num_filed
    LogHeader.field_list(n).name = fread(fileID, [1 20], 'uint8=>char');
    LogHeader.field_list(n).msg_id = fread(fileID, 1, 'uint8=>uint8');
    LogHeader.field_list(n).num_elem = fread(fileID, 1, 'uint8=>uint8');

    for k = 1:LogHeader.field_list(n).num_elem
        LogHeader.field_list(n).elem_list(k).name = fread(fileID, [1 20], 'uint8=>char');
        LogHeader.field_list(n).elem_list(k).type = fread(fileID, 1, 'uint16=>uint16');
        LogHeader.field_list(n).elem_list(k).number = fread(fileID, 1, 'uint16=>uint16');
    end
    
    msg_cnt{LogHeader.field_list(n).msg_id} = 0;
    LogMsg{LogHeader.field_list(n).msg_id} = {};
end

% read parameter
LogHeader.num_param_group = fread(fileID, 1, 'uint8=>uint8');

for n = 1:LogHeader.num_param_group
    LogHeader.param_group_list(n).name = fread(fileID, [1 20], 'uint8=>char');
    LogHeader.param_group_list(n).num_param = fread(fileID, 1, 'uint32=>uint32');
    
    for k = 1:LogHeader.param_group_list(n).num_param
        LogHeader.param_group_list(n).param(k).name = fread(fileID, [1 20], 'uint8=>char');
        LogHeader.param_group_list(n).param(k).type = fread(fileID, 1, 'uint8=>uint8');
        index = LogHeader.param_group_list(n).param(k).type+1;
        LogHeader.param_group_list(n).param(k).val = fread(fileID, 1, data_type(index));
    end
end
%% read log msg
while ~feof(fileID) && ftell(fileID)<fileDir.bytes
    msg_id = fread(fileID, 1, 'uint8=>uint8');
    index = -1;
    for n = 1 : LogHeader.num_filed
        if msg_id == LogHeader.field_list(n).msg_id
            index = n;
            break;
        end
    end

    if index < 0
        fprintf('unknow msg id:%d\r\n', msg_id);
        fclose(fileID);
        return;
    end

    msg_cnt{msg_id} = msg_cnt{msg_id} + 1;
    for k = 1:LogHeader.field_list(index).num_elem
        type = LogHeader.field_list(index).elem_list(k).type+1;
        number = LogHeader.field_list(index).elem_list(k).number;
        
        [msg_content, rb] = fread(fileID, [number, 1], data_type(type));
        
        if rb < number
            fprintf('msg:%d read err, stop parsing\n', msg_id);
            
            % clear msg data
            for p = 1:k-1
                LogMsg{msg_id}{p}(:, msg_cnt{msg_id}) = [];
            end
            msg_cnt{msg_id} = msg_cnt{msg_id} - 1;
            break;
        else
            LogMsg{msg_id}{k}(1:number,msg_cnt{msg_id}) = msg_content;
        end
%         LogMsg{msg_id}{k}(1:number,msg_cnt{msg_id}) = fread(fileID, [number, 1], data_type(type));
    end
end
fclose(fileID);

%% create Bus variable and save into .mat file
for n = 1:LogHeader.num_filed
    msg_id = LogHeader.field_list(n).msg_id;
    if isempty(LogMsg{msg_id})
       continue; 
    end
    
    FieldName = strrep(LogHeader.field_list(n).name, '"', '');
    FieldName = FieldName(find(~isspace(FieldName)));
    
    % find timestamp
    timestamp_id = 0;
    for k = 1:LogHeader.field_list(n).num_elem
        ElemName = strrep(LogHeader.field_list(n).elem_list(k).name, '"', '');
        ElemName = ElemName(find(~isspace(ElemName)));
        if strcmp(ElemName, "timestamp_ms")
            timestamp_id = k;
        end
    end
    
    if timestamp_id <= 0
       fprintf("create timestamp for %s\n", LogHeader.field_list(n).name);
       % construct a fake time_stamp
       [row col] = size(LogMsg{msg_id}{1});
       period = 0.001;	% TODO: add period list for each msg
       time_stamp = 0:period:((double(col)-1)*period);
    else
       time_stamp = double(LogMsg{msg_id}{timestamp_id}-LogMsg{msg_id}{timestamp_id}(1)) * 0.001;   % milli second to second
    end
    
    % construct Bsu variable
    for k = 1:LogHeader.field_list(n).num_elem
        ElemName = strrep(LogHeader.field_list(n).elem_list(k).name, '"', '');
        ElemName = ElemName(find(~isspace(ElemName)));
        
        exp = sprintf('timeseries(LogMsg{msg_id}{k}'', time_stamp);');
        if length(ElemName)
            eval([FieldName, '.', ElemName, '=', exp]);
        else
            eval([FieldName, '=', exp]);
        end
    end
    
    % save as .mat file
    save(strcat(fileDir.folder, ['\', FieldName, '.mat']), FieldName);
end

end

