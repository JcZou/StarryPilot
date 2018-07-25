/*
 * File      : msh_usr_cmd.c
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2018-06-13     zoujiachi   	the first version
 */
 
#include <rtthread.h>
#include <finsh.h>
#include <shell.h>
#include <string.h>
#include "calibration.h"
#include "starryio_uploader.h"
#include "pos_estimator.h"
#include "msh_usr_cmd.h"
#include "msh.h"
#include "console.h"

int handle_help_shell_cmd(int argc, char** argv)
{	
	if(argc > 1){
		if( strcmp(argv[1], "help") == 0 ){
			Console.print("StarryPilot shell help.\n");
			Console.print("Usage: help [command]\n");
		}
		if( strcmp(argv[1], "reboot") == 0 ){
			Console.print("Reboot system.\n");
		}
		if( strcmp(argv[1], "sys") == 0 ){
			Console.print("Show system status.\n");
		}
		if( strcmp(argv[1], "cali") == 0 ){
			Console.print("Calibrate sensors.\n");
			Console.print("Usage: cali <sensor>\n");
			Console.print("\n");
			Console.print("sensor:\n");
			Console.print("%9s,\t%s\n", "gyr", "Calibrate the gyroscope.");
			Console.print("%8s,\t%s\n", "acc", "Calibrate the accelerometer.");
			Console.print("%8s,\t%s\n", "mag", "Calibrate the magnetometer.");
		}
		if( strcmp(argv[1], "uploader") == 0 ){
			Console.print("Upload bin file to starry io.\n");
		}
		if( strcmp(argv[1], "sensor") == 0 ){
			Console.print("Get sensor information.\n");
			Console.print("Usage: sensor <sensor> [-n <cnt> | -t <period> | -r | -nc]\n");
			Console.print("\n");
			
			Console.print("sensor:\n");
			Console.print("\t%-5s - %s\n", "gyr", "Get gyroscope data.");
			Console.print("\t%-5s - %s\n", "acc", "Get accelerometer data.");
			Console.print("\t%-5s - %s\n", "mag", "Get magnetometer data.");
			
			Console.print("-n <cnt>:\n");
			Console.print("\t%s\n", "Set repeat count.");
			Console.print("-t <period>:\n");
			Console.print("\t%s\n", "Set repeat period (ms).");
			Console.print("-r:\n");
			Console.print("\t%s\n", "Show raw data.");
			Console.print("-nc:\n");
			Console.print("\t%s\n", "Show not calibrated data.");
		}
		if( strcmp(argv[1], "motor") == 0 ){
			Console.print("Motor operations.\n");
			Console.print("Usage: motor <action> [...]\n");
			Console.print("\n");
			
			Console.print("action:\n");
			Console.print("\t%-33s - %s\n", "setall <throttle>", "Set throttle for all motors.");
			Console.print("\t%-33s - %s\n", "set <throttle1, throttle2, ...>", "Set throttle for each motor.");
			Console.print("\t%-33s - %s\n", "get", "Get current motor throttle.");
			Console.print("\t%-33s - %s\n", "switch <on | off>", "Switch motor status.");
		}
		if( strcmp(argv[1], "rc") == 0 ){
			Console.print("Remote Controller information.\n");
			Console.print("Usage: rc <action> [...]\n");
			Console.print("\n");
			
			Console.print("action:\n");
			Console.print("%8s,\t%s\n", "status", "Show RC status.");
		}
		if( strcmp(argv[1], "param") == 0 ){
			Console.print("Configure parameters.\n");
			Console.print("Usage: param <action> [...]\n");
			Console.print("\n");
			
			Console.print("action:\n");
			Console.print("\t%-29s - %s\n", "load", "Load parameter from file.");
			Console.print("\t%-29s - %s\n", "store", "Store parameter to file.");
			Console.print("\t%-29s - %s\n", "get [group [param] | -g]", "Get parameter value.");
			Console.print("\t%-29s - %s\n", "set <group> <param> <value>", "Set parameter value.");
		}
		if( strcmp(argv[1], "test") == 0 ){
			Console.print("Customn test function.\n");
		}
		if( strcmp(argv[1], "ls") == 0 ){
			Console.print("List files/directories.\n");
			Console.print("Usage: ls [-l]\n");
			Console.print("\n");
			
			Console.print("-l:\n");
			Console.print("\t%s\n", "Show detail information.");
		}
		if( strcmp(argv[1], "cd") == 0 ){
			Console.print("Change current directory.\n");
			Console.print("Usage: cd <dir>\n");
		}
		if( strcmp(argv[1], "mv") == 0 ){
			Console.print("Move file/directory.\n");
			Console.print("Usage: mv <src> <des>\n");
		}
		if( strcmp(argv[1], "rm") == 0 ){
			Console.print("Remove file/directory.\n");
			Console.print("Usage: rm <file | dir> <des>\n");
		}
		if( strcmp(argv[1], "cat") == 0 ){
			Console.print("Show file content.\n");
			Console.print("Usage: cat <file> <des>\n");
		}
		if( strcmp(argv[1], "mkfs") == 0 ){
			Console.print("Formate file system.\n");
		}
		if( strcmp(argv[1], "getfree") == 0 ){
			Console.print("Show storage usage.\n");
		}
		if( strcmp(argv[1], "rtt") == 0 ){
			Console.print("RT-Thread system commands.\n");
			Console.print("Usage: rtt <action>\n");
			Console.print("\n");
			Console.print("action:\n");
			Console.print("\t%-15s - %s\n", "list_thread", "List thread.");
			Console.print("\t%-15s - %s\n", "list_sem", "List semaphone in system.");
			Console.print("\t%-15s - %s\n", "list_event", "List event in system.");
			Console.print("\t%-15s - %s\n", "list_mutex", "List mutex in system.");
			Console.print("\t%-15s - %s\n", "list_mailbox", "List mail box in system.");
			Console.print("\t%-15s - %s\n", "list_msgqueue", "List message queue in system.");
			Console.print("\t%-15s - %s\n", "list_mempool", "List memory pool in system.");
			Console.print("\t%-15s - %s\n", "list_timer", "List timer in system.");
			Console.print("\t%-15s - %s\n", "list_device", "List device in system.");
		}
		if( strcmp(argv[1], "att_est") == 0 ){
			Console.print("Attitude estimator commands.\n");
			Console.print("Usage: att_est <action>\n");
			Console.print("\n");
			Console.print("action:\n");
			Console.print("\t%-7s - %s\n", "show", "Show attitude information.");
			Console.print("\t%-7s - %s\n", "reset", "Reset attitude estimator.");
		}
		if( strcmp(argv[1], "att_est") == 0 ){
			Console.print("Attitude estimator commands.\n");
			Console.print("Usage: att_est <action>\n");
			Console.print("\n");
			Console.print("action:\n");
			Console.print("\t%-7s - %s\n", "show", "Show attitude information.");
			Console.print("\t%-7s - %s\n", "reset", "Reset attitude estimator.");
		}
		if( strcmp(argv[1], "logger") == 0 ){
			Console.print("Log operations.\n");
			Console.print("Usage: logger <action> [...]\n");
			Console.print("\n");
			Console.print("action:\n");
			Console.print("\t%-23s - %s\n", "start <file> [period]", "Start logger.");
			Console.print("\t%-23s - %s\n", "stop", "Stop logger.");
			Console.print("\t%-23s - %s\n", "info <file>", "Show log file information.");
		}
		if( strcmp(argv[1], "control") == 0 ){
			Console.print("Control commands.\n");
			Console.print("Usage: control <action> [...]\n");
			Console.print("\n");
			Console.print("action:\n");
			Console.print("\t%-13s - %s\n", "request", "Request control permission.");
			Console.print("\t%-13s - %s\n", "release", "Release control permission.");
			Console.print("\t%-13s - %s\n", "set <param>", "Set the value of control parameter.");
			Console.print("\t%-13s - %s\n", "get", "Get the value of control parameter.");
		}
	}else{
		// list all commands
        struct finsh_syscall *index;

		Console.print("StarryPilot shell commands:\n");
        for (index = _syscall_table_begin;
                index < _syscall_table_end;
                FINSH_NEXT_SYSCALL(index))
        {
            if (strncmp(index->name, "__cmd_", 6) != 0) continue;
#if defined(FINSH_USING_DESCRIPTION) && defined(FINSH_USING_SYMTAB)
            Console.print("%-16s - %s\n", &index->name[6], index->desc);
#else
            Console.print("%s ", &index->name[6]);
#endif
        }
    }
    Console.print("\n");

    return 0;
}

int cmd_reboot(int argc, char** argv)
{
	rt_kprintf("rebooting...\n\n");
	NVIC_SystemReset();
	return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_reboot, __cmd_reboot, reboot the system);

int handle_sys_shell_cmd(int argc, char** argv);
int cmd_sys(int argc, char** argv)
{
	return handle_sys_shell_cmd(argc, argv);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_sys, __cmd_sys, system status);

int cmd_cali(int argc, char** argv)
{
	struct finsh_shell* shell = finsh_get_shell();

	if(argc != 2)
	{
		rt_kprintf("invalid param\r\n");
		return 1;
	}
	
	if(strcmp(argv[1] , "acc") == 0)
	{

	}
	else if(strcmp(argv[1] , "mag") == 0)
	{

	}
	else if(strcmp(argv[1] , "gyr") == 0)
	{
		calibrate_gyr_run(shell);
	}
	else
	{
		rt_kprintf("invalid param\r\n");
		return 1;
	}
	
	return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_cali, __cmd_cali, calibrate the acc and mag sensor.);

int cmd_uploader(int argc, char** argv)
{
	starryio_upload();
	
	return 1;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_uploader, __cmd_uploader, upload bin file to starryio.);

int handle_sensor_shell_cmd(int argc, char** argv);
int cmd_sensor(int argc, char** argv)
{
	return handle_sensor_shell_cmd(argc, argv);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_sensor, __cmd_sensor, get sensor information.);

int handle_gps_shell_cmd(int argc, char** argv);
int cmd_gps(int argc, char** argv)
{
	return handle_gps_shell_cmd(argc, argv);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_gps, __cmd_gps, get gps information.);

int handle_motor_shell_cmd(int argc, char** argv);
int cmd_motor(int argc, char** argv)
{
	return handle_motor_shell_cmd(argc, argv);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_motor, __cmd_motor, motor operation);

int handle_rc_shell_cmd(int argc, char** argv);
int cmd_rc(int argc, char** argv)
{
	return handle_rc_shell_cmd(argc, argv);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_rc, __cmd_rc, rc operation);

int handle_param_shell_cmd(int argc, char** argv);
int cmd_param(int argc, char** argv)
{
	return handle_param_shell_cmd(argc, argv);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_param, __cmd_param, configure parameter);

int handle_test_shell_cmd(int argc, char** argv);
int cmd_test(int argc, char** argv)
{
	return handle_test_shell_cmd(argc, argv);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_test, __cmd_test, test function);

int handle_fm_shell_cmd(int argc, char** argv);
int cmd_ls(int argc, char** argv)
{
	return handle_fm_shell_cmd(argc, argv);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_ls, __cmd_ls, list files/directories);

int cmd_cd(int argc, char** argv)
{
	return handle_fm_shell_cmd(argc, argv);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_cd, __cmd_cd, change directory);

int cmd_mkdir(int argc, char** argv)
{
	return handle_fm_shell_cmd(argc, argv);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_mkdir, __cmd_mkdir, make directory);

int cmd_rm(int argc, char** argv)
{
	return handle_fm_shell_cmd(argc, argv);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_rm, __cmd_rm, remove file/directory);

int cmd_mkfs(int argc, char** argv)
{
	return handle_fm_shell_cmd(argc, argv);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_mkfs, __cmd_mkfs, make file system);

int cmd_getfree(int argc, char** argv)
{
	return handle_fm_shell_cmd(argc, argv);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_getfree, __cmd_getfree, get total space and free space of the file system);

int cmd_cat(int argc, char** argv)
{
	return handle_fm_shell_cmd(argc, argv);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_cat, __cmd_cat, read the contents of files);

int cmd_mv(int argc, char** argv)
{
	return handle_fm_shell_cmd(argc, argv);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_mv, __cmd_mv, move or rename files and directories);

int handle_rtt_shell_cmd(int argc, char** argv);
int cmd_rtt(int argc, char** argv)
{
	return handle_rtt_shell_cmd(argc, argv);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_rtt, __cmd_rtt, rt-thread rtos commands);

int handle_att_est_shell_cmd(int argc, char** argv);
int cmd_att_est(int argc, char** argv)
{
	return handle_att_est_shell_cmd(argc, argv);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_att_est, __cmd_att_est, attitude estimator commands);

int handle_pos_est_shell_cmd(int argc, char** argv);
int cmd_pos_est(int argc, char** argv)
{
	return handle_pos_est_shell_cmd(argc, argv);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_pos_est, __cmd_pos_est, position estimator commands);

int handle_mavproxy_shell_cmd(int argc, char** argv);
int cmd_mavproxy(int argc, char** argv)
{
	return handle_mavproxy_shell_cmd(argc, argv);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_mavproxy, __cmd_mavproxy, mavlink proxy commands);

int handle_logger_shell_cmd(int argc, char** argv);
int cmd_logger(int argc, char** argv)
{
	return handle_logger_shell_cmd(argc, argv);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_logger, __cmd_logger, log operations);

int handle_control_shell_cmd(int argc, char** argv);
int cmd_control(int argc, char** argv)
{
	return handle_control_shell_cmd(argc, argv);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_control, __cmd_control, control operations);

