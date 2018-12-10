/*
 * File      : msh_usr_cmd.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2018-06-13     zoujiachi   	the first version
 */
 
#ifndef __MSH_USR_CMD_H__
#define __MSH_USR_CMD_H__

typedef struct
{
	char *opt;
	char *val;
}sh_optv;

int handle_help_shell_cmd(int argc, char** argv);

#endif