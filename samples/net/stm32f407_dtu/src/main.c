/* Networking DHCPv4 client */

/*
 * Copyright (c) 2017 ARM Ltd.
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log.h>
LOG_MODULE_REGISTER(net_dhcpv4_client_sample, LOG_LEVEL_DBG);

#include <zephyr.h>
#include <linker/sections.h>
#include <errno.h>
#include <stdio.h>

#include <net/net_if.h>
#include <net/net_core.h>
#include <net/net_context.h>
#include <net/net_mgmt.h>
#include <gpio.h>

#define GPIO_OUT_DRV_NAME LED2_GPIO_CONTROLLER
#define GPIO_OUT_PIN  LED2_GPIO_PIN

static struct net_mgmt_event_callback mgmt_cb;

static void handler(struct net_mgmt_event_callback *cb,
		    u32_t mgmt_event,
		    struct net_if *iface)
{
	int i = 0;

	if (mgmt_event != NET_EVENT_IPV4_ADDR_ADD) {
		return;
	}

	for (i = 0; i < NET_IF_MAX_IPV4_ADDR; i++) {
		char buf[NET_IPV4_ADDR_LEN];

		if (iface->config.ip.ipv4->unicast[i].addr_type !=
							NET_ADDR_DHCP) {
			continue;
		}

		LOG_INF("Your address: %s",
			log_strdup(net_addr_ntop(AF_INET,
			    &iface->config.ip.ipv4->unicast[i].address.in_addr,
						  buf, sizeof(buf))));
		LOG_INF("Lease time: %u seconds",
			 iface->config.dhcpv4.lease_time);
		LOG_INF("Subnet: %s",
			log_strdup(net_addr_ntop(AF_INET,
				       &iface->config.ip.ipv4->netmask,
				       buf, sizeof(buf))));
		LOG_INF("Router: %s",
			log_strdup(net_addr_ntop(AF_INET,
						 &iface->config.ip.ipv4->gw,
						 buf, sizeof(buf))));
	}
}

void main(void)
{
	struct net_if *iface;

	struct device *gpio_out_dev;
	int ret;

	gpio_out_dev = device_get_binding(GPIO_OUT_DRV_NAME);
	if (!gpio_out_dev) {
		printk("Cannot find %s!\n", GPIO_OUT_DRV_NAME);
		return;
	}

	/* Setup GPIO output */
	ret = gpio_pin_configure(gpio_out_dev, GPIO_OUT_PIN, (GPIO_DIR_OUT));
	if (ret) {
		printk("Error configuring pin %d!\n", GPIO_OUT_PIN);
	}

	printk("Toggling pin %d\n", GPIO_OUT_PIN);

	ret = gpio_pin_write(gpio_out_dev, GPIO_OUT_PIN, 0);
	if (ret) {
		printk("Error set pin %d!\n", GPIO_OUT_PIN);
	}
	k_busy_wait(5000);
	ret = gpio_pin_write(gpio_out_dev, GPIO_OUT_PIN, 1);
	if (ret) {
		printk("Error set pin %d!\n", GPIO_OUT_PIN);
	}
	k_busy_wait(50000);
	
	LOG_INF("Run dhcpv4 client");

	net_mgmt_init_event_callback(&mgmt_cb, handler,
				     NET_EVENT_IPV4_ADDR_ADD);
	net_mgmt_add_event_callback(&mgmt_cb);

	iface = net_if_get_default();

	net_dhcpv4_start(iface);
}
