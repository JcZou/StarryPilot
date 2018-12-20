/*
 * File      : param.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2016-07-01     zoujiachi    first version.
 */

//#include <rthw.h>
//#include <rtdevice.h>
//#include <rtthread.h>
#include <string.h>
#include "global.h"
#include "param.h"
#include "console.h"
#include "ff.h"
#include "file_manager.h"
#include "yxml.h"

#define PARAM_FILE_NAME				"/sys/param.xml"
#define YXML_STACK_SIZE				1024

//PARAM_Def global_param;
//PARAM_Def* global_param_t = &global_param;
//static uint32_t _param_user_cnt;

typedef enum {
	PARAM_PARSE_START = 0,
	PARAM_PARSE_LIST,
	PARAM_PARSE_GROUP_INFO,
	PARAM_PARSE_GROUP_NAME,
	PARAM_PARSE_GROUP,
	PARAM_PARSE_PARAM,
	PARAM_PARSE_PARAM_NAME,
	PARAM_PARSE_PARAM_VAL,
	PARAM_PARSE_PARAM_VAL_CONTENT,
} PARAM_PARSE_STATE;

/* step 3: Define Parameter */
PARAM_GROUP(CALIBRATION) PARAM_DECLARE_GROUP(CALIBRATION) = \
{
	\
	PARAM_DEFINE_FLOAT(GYR_BIAS_X, 0.0),
	PARAM_DEFINE_FLOAT(GYR_BIAS_Y, 0.0),
	PARAM_DEFINE_FLOAT(GYR_BIAS_Z, 0.0),
	PARAM_DEFINE_FLOAT(GYR_X_GAIN, 1.0),
	PARAM_DEFINE_FLOAT(GYR_Y_GAIN, 1.0),
	PARAM_DEFINE_FLOAT(GYR_Z_GAIN, 1.0),
	PARAM_DEFINE_UINT32(GYR_CALIB, 0),
	PARAM_DEFINE_FLOAT(ACC_BIAS_X, 0.0),
	PARAM_DEFINE_FLOAT(ACC_BIAS_Y, 0.0),
	PARAM_DEFINE_FLOAT(ACC_BIAS_Z, 0.0),
	PARAM_DEFINE_FLOAT(ACC_ROT_MAT_1, 1.0),
	PARAM_DEFINE_FLOAT(ACC_ROT_MAT_2, 0.0),
	PARAM_DEFINE_FLOAT(ACC_ROT_MAT_3, 0.0),
	PARAM_DEFINE_FLOAT(ACC_ROT_MAT_4, 0.0),
	PARAM_DEFINE_FLOAT(ACC_ROT_MAT_5, 1.0),
	PARAM_DEFINE_FLOAT(ACC_ROT_MAT_6, 0.0),
	PARAM_DEFINE_FLOAT(ACC_ROT_MAT_7, 0.0),
	PARAM_DEFINE_FLOAT(ACC_ROT_MAT_8, 0.0),
	PARAM_DEFINE_FLOAT(ACC_ROT_MAT_9, 1.0),
	PARAM_DEFINE_UINT32(ACC_CALIB, 0),
	PARAM_DEFINE_FLOAT(MAG_BIAS_X, 0.0),
	PARAM_DEFINE_FLOAT(MAG_BIAS_Y, 0.0),
	PARAM_DEFINE_FLOAT(MAG_BIAS_Z, 0.0),
	PARAM_DEFINE_FLOAT(MAG_ROT_MAT_1, 1.0),
	PARAM_DEFINE_FLOAT(MAG_ROT_MAT_2, 0.0),
	PARAM_DEFINE_FLOAT(MAG_ROT_MAT_3, 0.0),
	PARAM_DEFINE_FLOAT(MAG_ROT_MAT_4, 0.0),
	PARAM_DEFINE_FLOAT(MAG_ROT_MAT_5, 1.0),
	PARAM_DEFINE_FLOAT(MAG_ROT_MAT_6, 0.0),
	PARAM_DEFINE_FLOAT(MAG_ROT_MAT_7, 0.0),
	PARAM_DEFINE_FLOAT(MAG_ROT_MAT_8, 0.0),
	PARAM_DEFINE_FLOAT(MAG_ROT_MAT_9, 1.0),
	PARAM_DEFINE_UINT32(MAG_CALIB, 0),
};

PARAM_GROUP(SYSTEM) PARAM_DECLARE_GROUP(SYSTEM) = \
{
	\
	PARAM_DEFINE_UINT8(LOG_AUTO_START, 0),
};

/* step 4: Define param list */
param_list_t param_list = { \
                            PARAM_DEFINE_GROUP(CALIBRATION),
                            PARAM_DEFINE_GROUP(SYSTEM),
                          };
/* Define Parameter End */

static char* TAG = "PARAM";

void param_store(void);

void param_traverse(void (*param_ops)(param_info_t* param))
{
	param_info_t* p;
	param_group_info* gp = (param_group_info*)&param_list;

	if(!param_ops)
		return;

	for(int j = 0 ; j < sizeof(param_list) / sizeof(param_group_info) ; j++) {
		p = gp->content;

		for(int i = 0 ; i < gp->param_num ; i++) {
			param_ops(p);
			p++;
		}

		gp++;
	}
}

param_info_t* param_get_by_name(char* param_name)
{
	param_info_t* p;
	param_group_info* gp = (param_group_info*)&param_list;

	for(int j = 0 ; j < sizeof(param_list) / sizeof(param_group_info) ; j++) {
		p = gp->content;

		for(int i = 0 ; i < gp->param_num ; i++) {
			if(strcmp(param_name, p->name) == 0)
				return p;

			p++;
		}

		gp++;
	}

	return NULL;
}

uint32_t param_get_info_count(void)
{
	uint32_t count = 0;
	param_info_t* p;
	param_group_info* gp = (param_group_info*)&param_list;

	for(int j = 0 ; j < sizeof(param_list) / sizeof(param_group_info) ; j++) {
		count += gp->param_num;
		gp++;
	}

	return count;
}

uint32_t param_get_info_index(char* param_name)
{
	uint32_t index = 0;
	param_info_t* p;
	param_group_info* gp = (param_group_info*)&param_list;

	for(int j = 0 ; j < sizeof(param_list) / sizeof(param_group_info) ; j++) {
		p = gp->content;

		for(int i = 0 ; i < gp->param_num ; i++) {
			if(strcmp(param_name, p->name) == 0)
				return index;

			p++;
			index++;
		}

		gp++;
	}

	return index;
}

int param_set_by_info(param_info_t* param, float val)
{
	switch(param->type) {

		case PARAM_TYPE_INT8:
			memcpy(&(param->val.i8), &val, sizeof(param->val.i8));
			break;

		case PARAM_TYPE_UINT8:
			memcpy(&(param->val.u8), &val, sizeof(param->val.u8));
			break;

		case PARAM_TYPE_INT16:
			memcpy(&(param->val.i16), &val, sizeof(param->val.i16));
			break;

		case PARAM_TYPE_UINT16:
			memcpy(&(param->val.u16), &val, sizeof(param->val.u16));
			break;

		case PARAM_TYPE_INT32:
			memcpy(&(param->val.i32), &val, sizeof(param->val.i32));
			break;

		case PARAM_TYPE_UINT32:
			memcpy(&(param->val.u32), &val, sizeof(param->val.u32));
			break;

		case PARAM_TYPE_FLOAT:
			param->val.f = val;
			break;

		case PARAM_TYPE_DOUBLE:
			param->val.lf = val;
			break;

		default:
			param->val.f = val;
			break;
	}

	param_store();

	return 0;
}

int param_get_by_info(param_info_t* param, float* val)
{
	switch(param->type) {

		case PARAM_TYPE_INT8:
			memcpy(&val, &(param->val.i8), sizeof(param->val.i8));
			break;

		case PARAM_TYPE_UINT8:
			memcpy(&val, &(param->val.u8), sizeof(param->val.u8));
			break;

		case PARAM_TYPE_INT16:
			memcpy(&val, &(param->val.i16), sizeof(param->val.i16));
			break;

		case PARAM_TYPE_UINT16:
			memcpy(&val, &(param->val.u16), sizeof(param->val.u16));
			break;

		case PARAM_TYPE_INT32:
			memcpy(&val, &(param->val.i32), sizeof(param->val.i32));
			break;

		case PARAM_TYPE_UINT32:
			memcpy(&val, &(param->val.u32), sizeof(param->val.u32));
			break;

		case PARAM_TYPE_FLOAT:
			*val = param->val.f;
			break;

		case PARAM_TYPE_DOUBLE:
			*val = param->val.lf;
			break;

		default:
			*val = param->val.f;
			break;
	}

	return 0;
}

void param_show_group_list(void)
{
	param_group_info* gp = (param_group_info*)&param_list;

	for(int j = 0 ; j < sizeof(param_list) / sizeof(param_group_info) ; j++) {
		Console.print("%s:\n", gp->name);
		gp++;
	}
}

void param_disp(param_info_t* p)
{
	if(p == NULL)
		return;

	Console.print("%25s: ", p->name);

	if(p->type == PARAM_TYPE_INT8) {
		Console.print("%d\n", p->val.i8);
	}

	if(p->type == PARAM_TYPE_UINT8) {
		Console.print("%d\n", p->val.u8);
	}

	if(p->type == PARAM_TYPE_INT16) {
		Console.print("%d\n", p->val.i16);
	}

	if(p->type == PARAM_TYPE_UINT16) {
		Console.print("%d\n", p->val.u16);
	}

	if(p->type == PARAM_TYPE_INT32) {
		Console.print("%d\n", p->val.i32);
	}

	if(p->type == PARAM_TYPE_UINT32) {
		Console.print("%d\n", p->val.u32);
	}

	if(p->type == PARAM_TYPE_FLOAT) {
		Console.print("%f\n", p->val.f);
	}

	if(p->type == PARAM_TYPE_DOUBLE) {
		Console.print("%lf\n", p->val.lf);
	}
}

void param_dump(void)
{
	param_info_t* p;
	param_group_info* gp = (param_group_info*)&param_list;

	for(int j = 0 ; j < sizeof(param_list) / sizeof(param_group_info) ; j++) {
		Console.print("%s:\n", gp->name);
		p = gp->content;

		for(int i = 0 ; i < gp->param_num ; i++) {
			param_disp(p);
			p++;
		}

		gp++;
	}
}

int param_dump_group(char* group_name)
{
	param_info_t* p;
	param_group_info* gp = (param_group_info*)&param_list;

	for(int j = 0 ; j < sizeof(param_list) / sizeof(param_group_info) ; j++) {
		if(strcmp(group_name, gp->name) == 0) {
			Console.print("%s:\n", gp->name);
			p = gp->content;

			for(int i = 0 ; i < gp->param_num ; i++) {
				param_disp(p);
				p++;
			}

			return 1;
		}

		gp++;
	}

	return 0;
}

int param_dump_param(char* group_name, char* param_name)
{
	param_info_t* p = param_get(group_name, param_name);

	if(p != NULL) {
		param_disp(p);

		return 1;
	}

	return 0;
}

param_info_t* param_get(char* group_name, char* param_name)
{
	param_info_t* p;
	param_group_info* gp = (param_group_info*)&param_list;

	for(int j = 0 ; j < sizeof(param_list) / sizeof(param_group_info) ; j++) {
		if(strcmp(group_name, gp->name) == 0) {
			p = gp->content;

			for(int i = 0 ; i < gp->param_num ; i++) {
				if(strcmp(param_name, p->name) == 0)
					return p;

				p++;
			}
		}

		gp++;
	}

	return NULL;
}

int param_set(char* group_name, char* param_name, char* val)
{
	param_info_t* p = param_get(group_name, param_name);

	if(p != NULL) {
		if(p->type == PARAM_TYPE_INT8) {
			p->val.i8 = atoi(val);
		}

		if(p->type == PARAM_TYPE_UINT8) {
			p->val.u8 = atoi(val);
		}

		if(p->type == PARAM_TYPE_INT16) {
			p->val.i16 = atoi(val);
		}

		if(p->type == PARAM_TYPE_UINT16) {
			p->val.u16 = atoi(val);
		}

		if(p->type == PARAM_TYPE_INT32) {
			p->val.i32 = atoi(val);
		}

		if(p->type == PARAM_TYPE_UINT32) {
			p->val.u32 = atoi(val);
		}

		if(p->type == PARAM_TYPE_FLOAT) {
			p->val.f = atof(val);
		}

		if(p->type == PARAM_TYPE_DOUBLE) {
			p->val.lf = atof(val);
		}

		return 0;
	} else {
		return 1;
	}
}

int param_parse_state_machine(yxml_t* x, yxml_ret_t r, PARAM_PARSE_STATE* status)
{
	static char attr_cnt = 0;
	static char group_name[30];
	static char param_name[30];
	static char content[20];

	switch(*status) {
		case PARAM_PARSE_START: {
			if(r == YXML_ELEMSTART) {
				if(strcmp("param_list", x->elem) == 0) {
					*status = PARAM_PARSE_LIST;
				}
			}
		}
		break;

		case PARAM_PARSE_LIST: {
			if(r == YXML_ELEMSTART) {
				if(strcmp("group", x->elem) == 0) {
					*status = PARAM_PARSE_GROUP_INFO;
				}
			}

			if(r == YXML_ELEMEND) {
				*status = PARAM_PARSE_START;
				//Console.print("total parse end\n");
				return 0;
			}
		}
		break;

		case PARAM_PARSE_GROUP_INFO: {
			if(r == YXML_ATTRSTART) {
				if(strcmp("name", x->attr) == 0) {
					*status = PARAM_PARSE_GROUP_NAME;
					attr_cnt = 0;
				} else {
					//TODO
					Console.print("parse group name err:%s\n", x->attr);
					return 2;
				}
			}
		}
		break;

		case PARAM_PARSE_GROUP_NAME: {
			if(r == YXML_ATTRVAL) {
				group_name[attr_cnt++] = x->data[0];
			}

			if(r == YXML_ATTREND) {
				group_name[attr_cnt] = '\0';
				//Console.print("group %s\n", group_name);
				attr_cnt = 0;
				*status = PARAM_PARSE_GROUP;
			}
		}
		break;

		case PARAM_PARSE_GROUP: {
			if(r == YXML_ELEMSTART) {
				if(strcmp("param", x->elem) == 0) {
					*status = PARAM_PARSE_PARAM;
				} else {
					//TODO
					Console.print("parse param ele err:%s\n", x->elem);
					return 2;
				}
			}

			if(r == YXML_ELEMEND) {
				//Console.print("group %s end\n", group_name);
				*status = PARAM_PARSE_LIST;
			}
		}
		break;

		case PARAM_PARSE_PARAM: {
			if(r == YXML_ATTRSTART) {
				if(strcmp("name", x->attr) == 0) {
					*status = PARAM_PARSE_PARAM_NAME;
					attr_cnt = 0;
				} else {
					//TODO
					Console.print("parse param name err:%s\n", x->attr);
					return 2;
				}
			}

			if(r == YXML_ELEMEND) {
				*status = PARAM_PARSE_GROUP;
				//Console.print("param parse end\n");
			}
		}
		break;

		case PARAM_PARSE_PARAM_NAME: {
			if(r == YXML_ATTRVAL) {
				param_name[attr_cnt++] = x->data[0];
			}

			if(r == YXML_ATTREND) {
				param_name[attr_cnt] = '\0';
				//Console.print("param %s\n", param_name);
				attr_cnt = 0;
				*status = PARAM_PARSE_PARAM_VAL;
			}
		}
		break;

		case PARAM_PARSE_PARAM_VAL: {
			if(r == YXML_ELEMSTART) {
				if(strcmp("value", x->elem) == 0) {
					*status = PARAM_PARSE_PARAM_VAL_CONTENT;
					attr_cnt = 0;
				} else {
					//TODO
					Console.print("parse param val err:%s\n", x->elem);
					return 2;
				}
			}
		}
		break;

		case PARAM_PARSE_PARAM_VAL_CONTENT: {
			if(r == YXML_CONTENT) {
				content[attr_cnt++] = x->data[0];
			}

			if(r == YXML_ELEMEND) {
				content[attr_cnt] = '\0';

				attr_cnt = 0;
				*status = PARAM_PARSE_PARAM;

				param_set(group_name, param_name, content);
			}
		}
		break;
	}

	return 1;
}

void param_store(void)
{
	if(filemanager_status() == fm_init_ok) {
		FIL fp;
		FRESULT res = f_open(&fp, PARAM_FILE_NAME, FA_OPEN_ALWAYS | FA_WRITE);

		if(res == FR_OK) {
			/* add title */
			f_printf(&fp, "<?xml version=\"1.0\"?>\n");
			/* add param_list element */
			f_printf(&fp, "<param_list>\n");

			param_info_t* p;
			param_group_info* gp = (param_group_info*)&param_list;

			for(int j = 0 ; j < sizeof(param_list) / sizeof(param_group_info) ; j++) {
				/* add group element */
				f_printf(&fp, "\x20\x20<group name=\"%s\">\n", gp->name);
				p = gp->content;

				for(int i = 0 ; i < gp->param_num ; i++) {
					/* add param element */
					f_printf(&fp, "\x20\x20\x20\x20<param name=\"%s\">\n", p->name);

					/* add value element */
					if(p->type == PARAM_TYPE_INT8) {
						f_printf(&fp, "\x20\x20\x20\x20\x20\x20<value>%d</value>\n", p->val.i8);
					}

					if(p->type == PARAM_TYPE_UINT8) {
						f_printf(&fp, "\x20\x20\x20\x20\x20\x20<value>%d</value>\n", p->val.u8);
					}

					if(p->type == PARAM_TYPE_INT16) {
						f_printf(&fp, "\x20\x20\x20\x20\x20\x20<value>%d</value>\n", p->val.i16);
					}

					if(p->type == PARAM_TYPE_UINT16) {
						f_printf(&fp, "\x20\x20\x20\x20\x20\x20<value>%d</value>\n", p->val.u16);
					}

					if(p->type == PARAM_TYPE_INT32) {
						f_printf(&fp, "\x20\x20\x20\x20\x20\x20<value>%d</value>\n", p->val.i32);
					}

					if(p->type == PARAM_TYPE_UINT32) {
						f_printf(&fp, "\x20\x20\x20\x20\x20\x20<value>%d</value>\n", p->val.u32);
					}

					if(p->type == PARAM_TYPE_FLOAT) {
						char val[32];
						sprintf(val, "%f", p->val.f);
						/* f_printf do not support %f */
						f_printf(&fp, "\x20\x20\x20\x20\x20\x20<value>%s</value>\n", val);
					}

					if(p->type == PARAM_TYPE_DOUBLE) {
						char val[50];
						sprintf(val, "%lf", p->val.lf);
						/* f_printf do not support %f */
						f_printf(&fp, "\x20\x20\x20\x20\x20\x20<value>%s</value>\n", val);
					}

					p++;
					f_printf(&fp, "\x20\x20\x20\x20</param>\n");
				}

				gp++;
				f_printf(&fp, "\x20\x20</group>\n");
			}

			f_printf(&fp, "</param_list>\n");
		}

		f_close(&fp);
		Console.print("parameter store success\n");
	}
}

void param_load(void)
{
	FIL fp;
	UINT br;
	yxml_ret_t yxml_r;
	char c;
	FRESULT res = f_open(&fp, PARAM_FILE_NAME, FA_OPEN_EXISTING | FA_READ);

	PARAM_PARSE_STATE status = PARAM_PARSE_START;

	if(res == FR_OK) {
		char* yxml_stack = (char*)rt_malloc(YXML_STACK_SIZE);

		if(yxml_stack != NULL) {
			yxml_t yxml_handle;
			yxml_init(&yxml_handle, yxml_stack, YXML_STACK_SIZE);

			while(!f_eof(&fp)) {
				OS_ENTER_CRITICAL;
				res = f_read(&fp, &c, 1, &br);
				OS_EXIT_CRITICAL;

				if(res == FR_OK && br == 1) {
					yxml_r = yxml_parse(&yxml_handle, c);
					param_parse_state_machine(&yxml_handle, yxml_r, &status);
				} else {
					Console.e(TAG, "xml file read err\n");
					break;
				}
			}

			if(yxml_eof(&yxml_handle) < 0)
				Console.print("xml parse err\n");
			else {
				//Console.print("parameter load success!\n");
			}
		} else {
			Console.e(TAG, "param malloc fail\n");
		}

		rt_free(yxml_stack);
	} else {
		//Console.print("can not find %s, use default parameters.\n", PARAM_FILE_NAME);
	}

	f_close(&fp);
}

int handle_param_shell_cmd(int argc, char** argv)
{
	uint8_t group_flag = 0;
	int flag_cnt = 0;
	int param_num;

	if(argc > 1) {
		for(int i = 0 ; i < argc - 1 ; i++) {
			if(strcmp(argv[i + 1], "-g") == 0 || strcmp(argv[i + 1], "--group") == 0) {
				group_flag = 1;
			}

			if(argv[i + 1][0] == '-')
				flag_cnt++;
		}

		param_num = argc - flag_cnt;

		if(strcmp(argv[1], "load") == 0) {
			param_load();
		}

		if(strcmp(argv[1], "store") == 0) {
			param_store();
		}

		if(strcmp(argv[1], "get") == 0 && param_num == 2) {
			if(group_flag)
				param_show_group_list();
			else
				param_dump();
		}

		if(strcmp(argv[1], "get") == 0 && param_num == 3) {
			param_dump_group(argv[2]);
		}

		if(strcmp(argv[1], "get") == 0 && param_num == 4) {
			param_dump_param(argv[2], argv[3]);
		}

		if(strcmp(argv[1], "set") == 0 && argc == 5) {
			if(param_set(argv[2], argv[3], argv[4]))
				Console.print("fail, can not find %s in group %s\n", argv[3], argv[2]);
			else
				Console.print("success, %s in group %s is set to %s\n", argv[3], argv[2], argv[4]);
		}
	}

	return 0;
}

uint8_t param_init(void)
{
	//_param_user_cnt = 0;
	//load_param(global_param_t);
	param_load();

	return 0;
}
