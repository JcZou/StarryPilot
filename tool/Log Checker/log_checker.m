clear all;

global popup_menu;
global LogHeader;
global LogField;
global linebuff;
global log_period_t;

fig = figure('units','normalized','position',[.2 .2 .6 .6],'name','Log Checker','numbertitle','off','color','w');
axes('units','normalized','position',[.05 .07 .9 .7]);
grid on;
xlabel('time (s)')

load_btn = uicontrol('units','normalized','position',[.05 .9 .1 .05],'style','pushbutton','String','Load');
popup_menu = uicontrol('units','normalized','position',[.05 .8 .15 .05],'style','popupmenu', 'String', {'Null'});
add_btn = uicontrol('units','normalized','position',[.25 .8 .1 .05],'style','pushbutton','String','Add');
delete_btn = uicontrol('units','normalized','position',[.37 .8 .1 .05],'style','pushbutton','String','Delete');
delete_all_btn = uicontrol('units','normalized','position',[.49 .8 .1 .05],'style','pushbutton','String','Delete All');

uicontrol('units','normalized','position',[.2 .9 .1 .03],'style','text', 'string', 'log period:');
log_period_t = uicontrol('units','normalized','position',[.305 .9 .05 .03],'style','text');

load_btn.Callback = @load_click_cb;
add_btn.Callback = @add_click_cb;
delete_btn.Callback = @delete_click_cb;
delete_all_btn.Callback = @delete_all_click_cb;

function load_click_cb(src, event)
    global popup_menu;
    global LogHeader;
    global LogField;
    global linebuff;
    global log_period_t;
    
%     clear LogField;
%     clear LogHeader;
    
    [file, folder] = uigetfile('.log');
    logfile = strcat(folder, file);
    
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
       popup_menu.String = cell;

       field_num = (fileDir.bytes - LogHeader.header_size)/LogHeader.filed_size;
       % TODO: load for other type
       for n = 1:field_num
            [LogField(n,:),count] = fread(fileID, [1 LogHeader.element_num], 'float=>float');
       end
    end
    fclose(fileID); 
    set(log_period_t ,'string',sprintf('%d ms', LogHeader.log_perid));
end

function add_click_cb(src, event)
    global popup_menu;
    global LogHeader;
    global LogField;
    global linebuff;
    
    val = popup_menu.Value;
    if linebuff.add(val) == true
       return ; 
    end
    [row, col] = size(LogField);
    hold on;
    linebuff.line(val) = plot(0.001*((1:row)*double(LogHeader.log_perid)), ...
            LogField(:,val), 'DisplayName', LogHeader.element_info(val).name);
    linebuff.add(val) = true;
    legend('show');
end

function delete_click_cb(src, event)
    global popup_menu;
    global linebuff;

    val = popup_menu.Value;
    if linebuff.add(val) == true
        delete(linebuff.line(val));
        linebuff.add(val) = false;
    end
end

function delete_all_click_cb(src, event)
    global popup_menu;
    global linebuff;
    global LogHeader;

    for n = 1:LogHeader.element_num
        if linebuff.add(n) == true
            delete(linebuff.line(n));
            linebuff.add(n) = false;
        end
    end
end