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

struct getcomparam_struct {
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

static const struct json_obj_descr scanrspdes[] = {
	JSON_OBJ_DESCR_PRIM(struct scanrsp_struct, dev_ip, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct scanrsp_struct, dev_name, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct scanrsp_struct, dev_mac, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct scanrsp_struct, dev_version, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct scanrsp_struct, dev_id, JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct scanrsp_struct, dev_type, JSON_TOK_STRING),
};

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
		struct scanrsp_struct rsp = {
			.dev_ip = "192.168.1.1",
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