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

#define MY_PORT	5000
#define SERVER_PORT 4999
#define BUFF_LEN 1024
/* size of stack area used by each thread */
#define STACKSIZE 1024

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
				.dev_ip = localip,
				.dev_name = "device001",
				.dev_mac = "201905040001",
				.dev_version = "V1.0",
				.dev_id = "2019050400000001",
				.dev_type = "ETH_DTU",
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
				.conn2mj = true,
				.iptype = "DHCP",
				.dev_name = "device001",
				.dev_mac = "201905040001",
				.dev_id = "2019050400000001",
				.user_name = "coltsmart",
				.user_pwd = "www.coltsmart.com",
				.dev_ip = localip,
				.net_mask = "255.255.255.255",
				.gateway = "192.168.1.1",
				.work_type = "TCP Sever",
				.dest_ip = "192.168.1.2",
				.dest_port = 4999,
				.local_port = 5000,
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
						.protocol = "RS232",
						.baud = 115200,
						.databit = "8",
						.stopbit = "1",
						.paritybit = "None",
						.flowctrl = "None",
						.timeout = 5,
						.maxlen = 1024,
						.autobaud = false,
					},
					[1] = {
						.protocol = "RS485",
						.baud = 115200,
						.databit = "8",
						.stopbit = "1",
						.paritybit = "None",
						.flowctrl = "None",
						.timeout = 5,
						.maxlen = 1024,
						.autobaud = false,
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
				}
			}
			else
			{
				printk("net conf\r\n");
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
K_THREAD_DEFINE(udp_conf_process_id, STACKSIZE, udp_conf_process, NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);