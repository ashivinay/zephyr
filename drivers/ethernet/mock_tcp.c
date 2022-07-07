/* Mock TCP stack
 *
 *  Copyright 2022 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/ethernet.h>

LOG_MODULE_REGISTER(mock_tcp, CONFIG_NET_CORE_LOG_LEVEL);

enum tcp_state {
	TCP_LISTEN = 1,
	TCP_SYN_SENT,
	TCP_SYN_RECEIVED,
	TCP_ESTABLISHED,
	TCP_FIN_WAIT_1,
	TCP_FIN_WAIT_2,
	TCP_CLOSE_WAIT,
	TCP_CLOSING,
	TCP_LAST_ACK,
	TCP_TIME_WAIT,
	TCP_CLOSED
};

#define TCP_SYN BIT(1)
#define TCP_ACK BIT(4)
#define TCP_FIN BIT(0)

struct tcp_conn {
	enum tcp_state state;
	uint32_t syn;
	uint32_t ack;
	struct k_mutex lock;
};

struct enet_frame {
	struct net_eth_hdr l2;
	struct net_ipv4_hdr ip;
	struct net_tcp_hdr tcp;
} __packed;

/* Support one TCP connection */
static struct tcp_conn conn;

static struct k_sem tx_req;

static struct enet_frame frame;



/**
 * Process L2 ethernet header. Verifies that this header is an IPV4 packet.
 *
 * @return 0 if header appears valid, or negative errno on error
 */
static int process_enet_header(struct net_pkt *pkt)
{
	struct net_eth_hdr *hdr = NET_ETH_HDR(pkt);
	uint8_t type = ntohs(hdr->type);

	if (type != NET_ETH_PTYPE_IP) {
		return -ENOTSUP;
	}
	/* Strip the ethernet header */
	net_buf_pull(pkt->frags, sizeof(hdr));
	net_pkt_cursor_init(pkt);
	return 0;
}

/**
 * Process the IP header. Validate header IP version, length, and protocol.
 *
 * @return 0 if the header looks valid or negative errno if not
 */
static int process_ip_header(struct net_pkt *pkt)
{
	struct net_ipv4_hdr *hdr = NET_IPV4_HDR(pkt);
	uint8_t hdr_len;
	uint16_t pkt_len;

	if ((hdr->vhl & 0xf0) != 0x40) {
		return -ENOTSUP;
	}
	hdr_len = ((hdr->vhl & 0xf) * 4U);
	pkt_len = ntohs(hdr->len);
	if ((net_pkt_get_len(pkt) != pkt_len)) {
		return -EINVAL;
	}
	if (hdr->proto != IPPROTO_TCP) {
		return -ENOTSUP;
	}
	/* Strip the IP header */
	net_buf_pull(pkt->frags, hdr_len);
	net_pkt_cursor_init(pkt);
	return 0;
}

/**
 * Send TCP output packet.
 *
 * Sends a TCP output packet on a connection
 * @param flags: flags to set on TCP response
 */
static int tcp_out(uint8_t flags)
{
	return -ENOTSUP;
}



/**
 * Process TCP packet.
 *
 * Implements very basic TCP state machine
 * In the case of SYN, responds with SYN+ACK and updates TCP state
 * In the case of ACK, queues new ACK for TX thread
 * In the case of FIN, queues ACK and updates TCP state
 *
 * @return 0 on success, or negative errno on error
 */
static int tcp_in(struct net_pkt *pkt)
{
	NET_PKT_DATA_ACCESS_DEFINE(tcp_access, struct net_tcp_hdr);
	struct net_tcp_hdr *hdr =
		(struct net_tcp_hdr *)net_pkt_get_data(pkt, &tcp_access);

	if (hdr->flags & TCP_SYN) {
		/* SYN flag. Send SYN+ACK */
		k_mutex_lock(&conn.lock, K_FOREVER);
		if (conn.state != TCP_LISTEN) {
			return -ECONNREFUSED;
		}
		conn.state = TCP_SYN_RECEIVED;
		conn.syn = 0xDEADBEEF; /* Chosen at random by the dev team */
		conn.ack = ntohl(*((uint32_t *)hdr->ack)) + 1;
		if (tcp_out(TCP_SYN | TCP_ACK) < 0) {
			return -ECONNABORTED;
		}
		k_mutex_unlock(&conn.lock);
	} else if (hdr->flags & TCP_FIN) {
		/* FIN flag */
		k_mutex_lock(&conn.lock, K_FOREVER);
		if (conn.state != TCP_ESTABLISHED) {
			return -ECONNRESET;
		}
		conn.state = TCP_CLOSING;
		conn.ack = ntohl(*((uint32_t *)hdr->ack)) + 1;
		if (tcp_out(TCP_ACK | TCP_FIN) < 0) {
			return -ECONNABORTED;
		}
		k_mutex_unlock(&conn.lock);

	} else if (hdr->flags & TCP_ACK) {
		/* ACK flag. Queue ACK */
		k_mutex_lock(&conn.lock, K_FOREVER);
		if (conn.state == TCP_SYN_RECEIVED) {
			conn.state = TCP_ESTABLISHED;
			conn.syn += 1;
		} else if (conn.state == TCP_CLOSING) {
			/* Conn is closed */
			conn.state = TCP_LISTEN;
		} else {
			conn.ack += (net_pkt_get_len(pkt) - (hdr->offset * 5U));
			if (tcp_out(TCP_ACK) < 0) {
				return -ECONNABORTED;
			}
		}
		k_mutex_unlock(&conn.lock);
	} else {
		return -ENOTSUP;
	}
	return 0;
}
