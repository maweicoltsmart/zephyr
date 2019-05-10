#ifndef __DTU_PARAMETER_NVS_H__

#define __DTU_PARAMETER_NVS_H__

#define NVS_STRING_MAX_LEN	32
#define DTU_PARAM_FLASH_OFFSET (0x60000)
#define FLASH_PAGE_SIZE   (128 * 1024)

struct dtu_comport {
	char protocol[NVS_STRING_MAX_LEN];
	int baud;
	char databit[NVS_STRING_MAX_LEN];
	char stopbit[NVS_STRING_MAX_LEN];
	char paritybit[NVS_STRING_MAX_LEN];
	char flowctrl[NVS_STRING_MAX_LEN];
	int timeout;
	int maxlen;
	bool autobaud;
};

struct dtu_param_nvs_struct{
	char dev_ip[NVS_STRING_MAX_LEN];
	char dev_name[NVS_STRING_MAX_LEN];
	char dev_mac[NVS_STRING_MAX_LEN];
	char dev_version[NVS_STRING_MAX_LEN];
	char dev_id[NVS_STRING_MAX_LEN];
	char dev_type[NVS_STRING_MAX_LEN];
	bool conn2mj;
	char iptype[NVS_STRING_MAX_LEN];
	char user_name[NVS_STRING_MAX_LEN];
	char user_pwd[NVS_STRING_MAX_LEN];
	char net_mask[NVS_STRING_MAX_LEN];
	char gateway[NVS_STRING_MAX_LEN];
	char work_type[NVS_STRING_MAX_LEN];
	char dest_ip[NVS_STRING_MAX_LEN];
	int dest_port;
	int local_port;
	struct dtu_comport com[2];
};

extern struct dtu_param_nvs_struct dtu_param_nvs;
extern struct k_sem dtu_param_nvs_restore;

void dtu_nvs_init(void);
void dtu_nvs_restore(void);

#endif