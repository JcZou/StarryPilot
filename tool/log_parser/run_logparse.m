
%% open log file
[file, folder] = uigetfile({'*.bin'; '*.log'});
logfile = strcat(folder, file);

log_parse(logfile);

clear file;
clear folder;
clear logfile;
