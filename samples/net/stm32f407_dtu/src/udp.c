#include <logging/log.h>
LOG_MODULE_REGISTER(net_echo_server_sample, LOG_LEVEL_ERR);

#include <stdio.h>
#include <string.h>
#include <net/socket.h>
#include <unistd.h>
#include <errno.h>
#include <zephyr.h>
#include <kernel.h>
#include <json.h>
#include "dtunvs.h"

#define MY_PORT	5000
#define SERVER_PORT 5001
#define BUFF_LEN 1024
/* size of stack area used by each thread */
#define STACKSIZE (1024 * 4)

/* scheduling priority used by each thread */
#define PRIORITY 7

#define RECV_BUFFER_SIZE 1280

struct data {
	const char *proto;

	struct {
		int sock;
		char recv_buffer[RECV_BUFFER_SIZE];
		u32_t counter;
	} udp;

	struct {
		int sock;
		char recv_buffer[RECV_BUFFER_SIZE];
		u32_t counter;
	} tcp;
};

struct scanrsp_struct {
	char *dev_ip;
	char *dev_name;
	char * dev_mac;
	char *dev_version;
	char * dev_id;
	char *dev_type;
};

struct getnetparam_struct {
	bool conn2mj;
	char *iptype;
	char *dev_name;
	char * dev_mac;
	char * dev_id;
	char *user_name;
	char *user_pwd;
	char *dev_ip;
	char *net_mask;
	char *gateway;
	char *work_type;
	char *dest_ip;
	int dest_port;
	int local_port;
};

/*struct getcomparam_struct {
	char *protocol;
	int baud;
	char *databit;
	char *stopbit;
	char *paritybit;
	char *flowctrl;
	int timeout;
	int maxlen;
	bool autobaud;
};*/

static const struct json_obj_descr scanrspdes[] = {
	JSON_OBJ_DESCR_PRIM(struct scanrsp_struct, dev_ip, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct scanrsp_struct, dev_name, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct scanrsp_struct, dev_mac, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct scanrsp_struct, dev_version, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct scanrsp_struct, dev_id, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct scanrsp_struct, dev_type, JSON_TOK_STRING),
};

static const struct json_obj_descr respnetdes[] = {
	JSON_OBJ_DESCR_PRIM(struct getnetparam_struct, conn2mj, JSON_TOK_TRUE),
	JSON_OBJ_DESCR_PRIM(struct getnetparam_struct, iptype, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct getnetparam_struct, dev_name, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct getnetparam_struct, dev_mac, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct getnetparam_struct, dev_id, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct getnetparam_struct, user_name, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct getnetparam_struct, user_pwd, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct getnetparam_struct, dev_ip, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct getnetparam_struct, net_mask, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct getnetparam_struct, gateway, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct getnetparam_struct, work_type, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct getnetparam_struct, dest_ip, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct getnetparam_struct, dest_port, JSON_TOK_NUMBER),
	JSON_OBJ_DESCR_PRIM(struct getnetparam_struct, local_port, JSON_TOK_NUMBER),
};

struct comport {
	char *protocol;
	int baud;
	char *databit;
	char *stopbit;
	char *paritybit;
	char *flowctrl;
	int timeout;
	int maxlen;
	bool autobaud;
};

struct getcomparam_struct {
	struct comport serial[2];
	size_t num_serial;
};

static const struct json_obj_descr comport_descr[] = {
	JSON_OBJ_DESCR_PRIM(struct comport, protocol, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct comport, baud, JSON_TOK_NUMBER),
	JSON_OBJ_DESCR_PRIM(struct comport, databit, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct comport, stopbit, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct comport, paritybit, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct comport, flowctrl, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct comport, timeout, JSON_TOK_NUMBER),
	JSON_OBJ_DESCR_PRIM(struct comport, maxlen, JSON_TOK_NUMBER),
	JSON_OBJ_DESCR_PRIM(struct comport, autobaud, JSON_TOK_TRUE),
};

static const struct json_obj_descr rspserialdes[] = {
	JSON_OBJ_DESCR_OBJ_ARRAY(struct getcomparam_struct, serial, 2, num_serial,
				 comport_descr, ARRAY_SIZE(comport_descr)),
};

/*static const struct json_obj_descr rspserialdes[] = {
	JSON_OBJ_DESCR_PRIM(struct getcomparam_struct, protocol, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct getcomparam_struct, baud, JSON_TOK_NUMBER),
	JSON_OBJ_DESCR_PRIM(struct getcomparam_struct, databit, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct getcomparam_struct, stopbit, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct getcomparam_struct, paritybit, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct getcomparam_struct, flowctrl, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct getcomparam_struct, timeout, JSON_TOK_NUMBER),
	JSON_OBJ_DESCR_PRIM(struct getcomparam_struct, maxlen, JSON_TOK_NUMBER),
	JSON_OBJ_DESCR_PRIM(struct getcomparam_struct, autobaud, JSON_TOK_TRUE),
};*/

struct configs {
	struct data ipv4;
	struct data ipv6;
};

struct configs conf = {
	.ipv4 = {
		.proto = "IPv4",
	},
	.ipv6 = {
		.proto = "IPv6",
	},
};

extern char localip[];
extern struct k_sem got_address;

static int start_udp_proto(struct data *data, struct sockaddr *bind_addr,
			   socklen_t bind_addrlen)
{
	int ret;

	data->udp.sock = socket(bind_addr->sa_family, SOCK_DGRAM, IPPROTO_UDP);

	if (data->udp.sock < 0) {
		NET_ERR("Failed to create UDP socket (%s): %d", data->proto,
			errno);
		return -errno;
	}

	ret = bind(data->udp.sock, bind_addr, bind_addrlen);
	if (ret < 0) {
		NET_ERR("Failed to bind UDP socket (%s): %d", data->proto,
			errno);
		ret = -errno;
	}

	return ret;
}

static int process_udp(struct data *data)
{
	int ret = 0;
	int received;
	struct sockaddr client_addr;
	socklen_t client_addr_len;
	struct sockaddr_in *addr4;

	NET_INFO("Waiting for UDP packets on port %d (%s)...",
		 MY_PORT, data->proto);

	do {
		client_addr_len = sizeof(client_addr);
		received = recvfrom(data->udp.sock, data->udp.recv_buffer,
				    sizeof(data->udp.recv_buffer), 0,
				    &client_addr, &client_addr_len);

		if (received < 0) {
			/* Socket error */
			NET_ERR("UDP (%s): Connection error %d", data->proto,
				errno);
			ret = -errno;
			break;
		}
		if(memcmp(data->udp.recv_buffer,"www.coltsmart.com",17) == 0)
		{
			struct scanrsp_struct rsp = {
				.dev_ip = dtu_param_nvs.dev_ip,
				.dev_name = dtu_param_nvs.dev_name,
				.dev_mac = dtu_param_nvs.dev_mac,
				.dev_version = dtu_param_nvs.dev_version,
				.dev_id = dtu_param_nvs.dev_id,
				.dev_type = dtu_param_nvs.dev_type,
			};
			memset(data->udp.recv_buffer,0,RECV_BUFFER_SIZE);
			ret = json_obj_encode_buf(scanrspdes, ARRAY_SIZE(scanrspdes),
					  &rsp, data->udp.recv_buffer, RECV_BUFFER_SIZE);
			if(ret != 0)
			{
				printk("encode error\r\n");
				continue;
			}
			printk("%s",data->udp.recv_buffer);

			addr4 = (struct sockaddr_in *)&client_addr;
			addr4->sin_port = htons(SERVER_PORT);
			ret = sendto(data->udp.sock, data->udp.recv_buffer, strlen(data->udp.recv_buffer), 0,
				     &client_addr, client_addr_len);
			if (ret < 0) {
				NET_ERR("UDP (%s): Failed to send %d", data->proto,
					errno);
				ret = -errno;
				break;
			}
		}
		else if(memcmp(data->udp.recv_buffer,"get net param",13) == 0)
		{
			struct getnetparam_struct rspnet = {
				.conn2mj = dtu_param_nvs.conn2mj,
				.iptype = dtu_param_nvs.iptype,
				.dev_name = dtu_param_nvs.dev_name,
				.dev_mac = dtu_param_nvs.dev_mac,
				.dev_id = dtu_param_nvs.dev_id,
				.user_name = dtu_param_nvs.user_name,
				.user_pwd = dtu_param_nvs.user_pwd,
				.dev_ip = dtu_param_nvs.dev_ip,
				.net_mask = dtu_param_nvs.net_mask,
				.gateway = dtu_param_nvs.gateway,
				.work_type = dtu_param_nvs.work_type,
				.dest_ip = dtu_param_nvs.dest_ip,
				.dest_port = dtu_param_nvs.dest_port,
				.local_port = dtu_param_nvs.local_port,
			};
			memset(data->udp.recv_buffer,0,RECV_BUFFER_SIZE);
			ret = json_obj_encode_buf(respnetdes, ARRAY_SIZE(respnetdes),
					  &rspnet, data->udp.recv_buffer, RECV_BUFFER_SIZE);
			if(ret != 0)
			{
				printk("encode error\r\n");
				continue;
			}
			printk("%s",data->udp.recv_buffer);

			addr4 = (struct sockaddr_in *)&client_addr;
			addr4->sin_port = htons(SERVER_PORT);
			ret = sendto(data->udp.sock, data->udp.recv_buffer, strlen(data->udp.recv_buffer), 0,
				     &client_addr, client_addr_len);
			if (ret < 0) {
				NET_ERR("UDP (%s): Failed to send %d", data->proto,
					errno);
				ret = -errno;
				break;
			}
		}
		else if(memcmp(data->udp.recv_buffer,"get serial param",16) == 0)
		{
			struct getcomparam_struct rspserial = {
				.serial = {
					[0] = { 
						.protocol = dtu_param_nvs.com[0].protocol,
						.baud = dtu_param_nvs.com[0].baud,
						.databit = dtu_param_nvs.com[0].databit,
						.stopbit = dtu_param_nvs.com[0].stopbit,
						.paritybit = dtu_param_nvs.com[0].paritybit,
						.flowctrl = dtu_param_nvs.com[0].flowctrl,
						.timeout = dtu_param_nvs.com[0].timeout,
						.maxlen = dtu_param_nvs.com[0].maxlen,
						.autobaud = dtu_param_nvs.com[0].autobaud,
					},
					[1] = {
						.protocol = dtu_param_nvs.com[1].protocol,
						.baud = dtu_param_nvs.com[1].baud,
						.databit = dtu_param_nvs.com[1].databit,
						.stopbit = dtu_param_nvs.com[1].stopbit,
						.paritybit = dtu_param_nvs.com[1].paritybit,
						.flowctrl = dtu_param_nvs.com[1].flowctrl,
						.timeout = dtu_param_nvs.com[1].timeout,
						.maxlen = dtu_param_nvs.com[1].maxlen,
						.autobaud = dtu_param_nvs.com[1].autobaud,
					},
				},
				.num_serial = 2,
			};

			memset(data->udp.recv_buffer,0,RECV_BUFFER_SIZE);
			ret = json_obj_encode_buf(rspserialdes, ARRAY_SIZE(rspserialdes),
					  &rspserial, data->udp.recv_buffer, RECV_BUFFER_SIZE);
			if(ret != 0)
			{
				printk("encode error\r\n");
				continue;
			}
			printk("%s",data->udp.recv_buffer);

			addr4 = (struct sockaddr_in *)&client_addr;
			addr4->sin_port = htons(SERVER_PORT);
			ret = sendto(data->udp.sock, data->udp.recv_buffer, strlen(data->udp.recv_buffer), 0,
				     &client_addr, client_addr_len);
			if (ret < 0) {
				NET_ERR("UDP (%s): Failed to send %d", data->proto,
					errno);
				ret = -errno;
				break;
			}
		}
		else
		{
			printk("%s\r\n",data->udp.recv_buffer);
			struct getnetparam_struct netparam;
			data->udp.recv_buffer[RECV_BUFFER_SIZE - 1] = 0x00;
			ret = json_obj_parse(data->udp.recv_buffer, sizeof(data->udp.recv_buffer) - 1, respnetdes,
			     ARRAY_SIZE(respnetdes), &netparam);
			if(ret < 0)
			{
				struct getcomparam_struct comparam;
				data->udp.recv_buffer[RECV_BUFFER_SIZE - 1] = 0x00;
				ret = json_obj_parse(data->udp.recv_buffer, sizeof(data->udp.recv_buffer) - 1, rspserialdes,
			     	ARRAY_SIZE(rspserialdes), &comparam);
				if(ret < 0)
				{
				}
				else
				{
					printk("serial conf\r\n");
					for(int i = 0;i < 2;i ++)
					{
						memset(dtu_param_nvs.com[i].protocol,0,NVS_STRING_MAX_LEN);
						memset(dtu_param_nvs.com[i].databit,0,NVS_STRING_MAX_LEN);
						memset(dtu_param_nvs.com[i].stopbit,0,NVS_STRING_MAX_LEN);
						memset(dtu_param_nvs.com[i].paritybit,0,NVS_STRING_MAX_LEN);
						memset(dtu_param_nvs.com[i].flowctrl,0,NVS_STRING_MAX_LEN);


						strcpy(dtu_param_nvs.com[0].protocol,comparam.serial[i].protocol);
						dtu_param_nvs.com[0].baud = comparam.serial[i].baud;
						strcpy(dtu_param_nvs.com[0].databit,comparam.serial[i].databit);
						strcpy(dtu_param_nvs.com[0].stopbit,comparam.serial[i].stopbit);
						strcpy(dtu_param_nvs.com[0].paritybit,comparam.serial[i].paritybit);
						strcpy(dtu_param_nvs.com[0].flowctrl,comparam.serial[i].flowctrl);
						dtu_param_nvs.com[0].timeout = comparam.serial[i].timeout;
						dtu_param_nvs.com[0].maxlen = comparam.serial[i].maxlen;
						dtu_param_nvs.com[0].autobaud = comparam.serial[i].autobaud;
						k_sem_give(&dtu_param_nvs_restore);
						//dtu_nvs_restore();
						//dtu_nvs_init();
					}

				}
			}
			else
			{
				printk("net conf\r\n");
				memset(dtu_param_nvs.dev_ip,0,NVS_STRING_MAX_LEN);
				memset(dtu_param_nvs.dev_name,0,NVS_STRING_MAX_LEN);
				memset(dtu_param_nvs.dev_id,0,NVS_STRING_MAX_LEN);
				memset(dtu_param_nvs.dev_mac,0,NVS_STRING_MAX_LEN);
				memset(dtu_param_nvs.dev_version,0,NVS_STRING_MAX_LEN);
				memset(dtu_param_nvs.dev_type,0,NVS_STRING_MAX_LEN);
				memset(dtu_param_nvs.iptype,0,NVS_STRING_MAX_LEN);
				memset(dtu_param_nvs.user_name,0,NVS_STRING_MAX_LEN);
				memset(dtu_param_nvs.user_pwd,0,NVS_STRING_MAX_LEN);
				memset(dtu_param_nvs.net_mask,0,NVS_STRING_MAX_LEN);
				memset(dtu_param_nvs.gateway,0,NVS_STRING_MAX_LEN);
				memset(dtu_param_nvs.work_type,0,NVS_STRING_MAX_LEN);
				memset(dtu_param_nvs.dest_ip,0,NVS_STRING_MAX_LEN);

				strcpy(dtu_param_nvs.dev_ip,netparam.dev_ip);
				strcpy(dtu_param_nvs.dev_name,netparam.dev_name);
				strcpy(dtu_param_nvs.dev_id,netparam.dev_id);
				strcpy(dtu_param_nvs.dev_mac,netparam.dev_mac);
				//strcpy(dtu_param_nvs.dev_version,netparam.dev_version);
				//strcpy(dtu_param_nvs.dev_type,netparam.dev_type);
				dtu_param_nvs.conn2mj = netparam.conn2mj;
				strcpy(dtu_param_nvs.iptype,netparam.iptype);
				strcpy(dtu_param_nvs.user_name,netparam.user_name);
				strcpy(dtu_param_nvs.user_pwd,netparam.user_pwd);
				strcpy(dtu_param_nvs.net_mask,netparam.net_mask);
				strcpy(dtu_param_nvs.gateway,netparam.gateway);
				strcpy(dtu_param_nvs.work_type,netparam.work_type);
				strcpy(dtu_param_nvs.dest_ip,netparam.dest_ip);
				dtu_param_nvs.dest_port = netparam.dest_port;
				dtu_param_nvs.local_port = netparam.local_port;
				k_sem_give(&dtu_param_nvs_restore);
				//dtu_nvs_restore();
				//dtu_nvs_init();
			}
		}
		if (++data->udp.counter % 1000 == 0U) {
			NET_INFO("%s UDP: Sent %u packets", data->proto,
				 data->udp.counter);
		}

		NET_DBG("UDP (%s): Received and replied with %d bytes",
			data->proto, received);
	} while (true);

	return ret;
}

void udp_conf_process(void)
{
	int ret;
	struct sockaddr_in addr4;

	//k_sem_take(&got_address,K_FOREVER);
	//sleep(1);
	//dtu_nvs_init();

	(void)memset(&addr4, 0, sizeof(addr4));
	addr4.sin_family = AF_INET;
	addr4.sin_port = htons(MY_PORT);

	ret = start_udp_proto(&conf.ipv4, (struct sockaddr *)&addr4,
			      sizeof(addr4));
	if (ret < 0) {
		//quit();
		return;
	}

	while (ret == 0) {
		ret = process_udp(&conf.ipv4);
		if (ret < 0) {
			//quit();
		}
	}
}
/*K_THREAD_DEFINE(udp_conf_process_id, STACKSIZE, udp_conf_process, NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);*/