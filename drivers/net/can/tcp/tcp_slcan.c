/*
 * tcp_slcan.c - serial line CAN over TCP interface driver
 *
 * This file is derived from linux/drivers/net/can/slcan.
 *
 * slcan Authors : Oliver Hartkopp <socketcan@hartkopp.net>
 *                 Dario Binacchi <dario.binacchi@amarulasolutions.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see http://www.gnu.org/licenses/gpl.html
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 */

#include <linux/module.h>

#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/kernel.h>
#include <linux/workqueue.h>
#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/skb.h>
#include <linux/miscdevice.h>
#include <net/sock.h>
#include <net/tcp.h>
#include <net/strparser.h>

MODULE_DESCRIPTION("serial line CAN over TCP interface");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Renjaya Raga Zenta <renjaya.zenta@formulatrix.com>");

/* maximum rx buffer len: extended CAN FD frame with timestamp */
#define SLCAN_FD_MTU (sizeof( \
			"T11112222F" \
			"1122334455667788" \
			"1122334455667788" \
			"1122334455667788" \
			"1122334455667788" \
			"1122334455667788" \
			"1122334455667788" \
			"1122334455667788" \
			"1122334455667788EA5F\r") + 1)

#define SLCAN_CMD_LEN 1
#define SLCAN_SFF_ID_LEN 3
#define SLCAN_EFF_ID_LEN 8
#define SLCAN_STATE_LEN 1
#define SLCAN_STATE_BE_RXCNT_LEN 3
#define SLCAN_STATE_BE_TXCNT_LEN 3
#define SLCAN_STATE_FRAME_LEN       (1 + SLCAN_CMD_LEN + \
				     SLCAN_STATE_BE_RXCNT_LEN + \
				     SLCAN_STATE_BE_TXCNT_LEN)

#define TCP_SLCAN_IOC_GET_NAME _IOR('S', 1, char[IFNAMSIZ])
#define TCP_SLCAN_IOC_ATTACH _IOW('S', 2, int)

struct tcp_slcan {
	struct can_priv		can;
	struct net_device	*dev;

	bool			connected;
	struct socket		*sock;
	void			(*orig_data_ready)(struct sock *sk);
	void			(*orig_state_change)(struct sock *sk);
	spinlock_t		lock;

	struct strparser	strp;

	unsigned char		rbuff[SLCAN_FD_MTU];
	int			rcount;
	unsigned char		xbuff[SLCAN_FD_MTU];

	wait_queue_head_t	event_wait;
	struct work_struct	cleanup_work;
};

/*************************************************************************
 *			SLCAN ENCAPSULATION FORMAT			 *
 *************************************************************************/

/* A CAN frame has a can_id (11 bit standard frame format OR 29 bit extended
 * frame format) a data length code (len) which can be from 0 to 8
 * and up to <len> data bytes as payload.
 * Additionally a CAN frame may become a remote transmission frame if the
 * RTR-bit is set. This causes another ECU to send a CAN frame with the
 * given can_id.
 *
 * The SLCAN ASCII representation of these different frame types is:
 * <type> <id> <dlc> <data>*
 *
 * Extended frames (29 bit) are defined by capital characters in the type.
 * RTR frames are defined as 'r' types - normal frames have 't' type:
 * t => 11 bit data frame
 * r => 11 bit RTR frame
 * T => 29 bit data frame
 * R => 29 bit RTR frame
 * tx => 11 bit data frame (CAN FD with BRS)
 * Tx => 29 bit data frame (CAN FD with BRS)
 * tX => 11 bit data frame (CAN FD without BRS)
 * TX => 29 bit data frame (CAN FD without BRS)
 *
 * The <id> is 3 (standard) or 8 (extended) bytes in ASCII Hex (base64).
 * The <dlc> is a one byte ASCII number ('0' - '8')
 * The <data> section has at much ASCII Hex bytes as defined by the <dlc>
 *
 * Examples:
 *
 * t1230 : can_id 0x123, len 0, no data
 * t4563112233 : can_id 0x456, len 3, data 0x11 0x22 0x33
 * T12ABCDEF2AA55 : extended can_id 0x12ABCDEF, len 2, data 0xAA 0x55
 * r1230 : can_id 0x123, len 0, no data, remote transmission request
 * tx456C112233445566778899AABBCC : can_id 0x456, len (dlc) 12, data 0x11 ... 0xCC, CAN FD with BRS
 * tX456C112233445566778899AABBCC : can_id 0x456, len (dlc) 12, data 0x11 ... 0xCC, CAN FD without BRS
 * Tx12ABCDEF2AA55 : extended can_id 0x12ABCDEF, len (dlc) 2, data 0xAA 0x55, CAN FD with BRS
 * TX12ABCDEF2AA55 : extended can_id 0x12ABCDEF, len (dlc) 2, data 0xAA 0x55, CAN FD without BRS
 *
 */

/*************************************************************************
 *			STANDARD SLCAN DECAPSULATION			 *
 *************************************************************************/

/* Send one completely decapsulated can_frame to the network layer */
static void slcan_bump_frame(struct tcp_slcan *sl)
{
	struct sk_buff *skb;
	struct can_frame *cf;
	int i, tmp;
	u32 tmpid;
	char *cmd = sl->rbuff;

	skb = alloc_can_skb(sl->dev, &cf);
	if (unlikely(!skb)) {
		sl->dev->stats.rx_dropped++;
		return;
	}

	switch (*cmd) {
	case 'r':
		cf->can_id = CAN_RTR_FLAG;
		fallthrough;
	case 't':
		/* store dlc ASCII value and terminate SFF CAN ID string */
		cf->len = sl->rbuff[SLCAN_CMD_LEN + SLCAN_SFF_ID_LEN];
		sl->rbuff[SLCAN_CMD_LEN + SLCAN_SFF_ID_LEN] = 0;
		/* point to payload data behind the dlc */
		cmd += SLCAN_CMD_LEN + SLCAN_SFF_ID_LEN + 1;
		break;
	case 'R':
		cf->can_id = CAN_RTR_FLAG;
		fallthrough;
	case 'T':
		cf->can_id |= CAN_EFF_FLAG;
		/* store dlc ASCII value and terminate EFF CAN ID string */
		cf->len = sl->rbuff[SLCAN_CMD_LEN + SLCAN_EFF_ID_LEN];
		sl->rbuff[SLCAN_CMD_LEN + SLCAN_EFF_ID_LEN] = 0;
		/* point to payload data behind the dlc */
		cmd += SLCAN_CMD_LEN + SLCAN_EFF_ID_LEN + 1;
		break;
	default:
		goto decode_failed;
	}

	if (kstrtou32(sl->rbuff + SLCAN_CMD_LEN, 16, &tmpid))
		goto decode_failed;

	cf->can_id |= tmpid;

	/* get len from sanitized ASCII value */
	if (cf->len >= '0' && cf->len < '9')
		cf->len -= '0';
	else
		goto decode_failed;

	/* RTR frames may have a dlc > 0 but they never have any data bytes */
	if (!(cf->can_id & CAN_RTR_FLAG)) {
		for (i = 0; i < cf->len; i++) {
			tmp = hex_to_bin(*cmd++);
			if (tmp < 0)
				goto decode_failed;

			cf->data[i] = (tmp << 4);
			tmp = hex_to_bin(*cmd++);
			if (tmp < 0)
				goto decode_failed;

			cf->data[i] |= tmp;
		}
	}

	sl->dev->stats.rx_packets++;
	if (!(cf->can_id & CAN_RTR_FLAG))
		sl->dev->stats.rx_bytes += cf->len;

	netif_rx(skb);
	return;

decode_failed:
	sl->dev->stats.rx_errors++;
	dev_kfree_skb(skb);
}

/* Send one completely decapsulated canfd_frame to the network layer */
static void slcan_bump_fd_frame(struct tcp_slcan *sl)
{
	struct sk_buff *skb;
	struct canfd_frame *cf;
	int i, tmp, len;
	u32 tmpid;
	char *cmd = sl->rbuff;

	skb = alloc_canfd_skb(sl->dev, &cf);
	if (unlikely(!skb)) {
		sl->dev->stats.rx_dropped++;
		return;
	}

	if (cmd[1] == 'x')
		cf->flags |= CANFD_BRS;

	switch (*cmd) {
	case 't':
		/* store dlc ASCII value and terminate SFF CAN ID string */
		len = sl->rbuff[1 + SLCAN_CMD_LEN + SLCAN_SFF_ID_LEN];
		sl->rbuff[1 + SLCAN_CMD_LEN + SLCAN_SFF_ID_LEN] = 0;
		/* point to payload data behind the dlc */
		cmd += 1 + SLCAN_CMD_LEN + SLCAN_SFF_ID_LEN + 1;
		break;
	case 'T':
		cf->can_id |= CAN_EFF_FLAG;
		/* store dlc ASCII value and terminate EFF CAN ID string */
		len = sl->rbuff[1 + SLCAN_CMD_LEN + SLCAN_EFF_ID_LEN];
		sl->rbuff[1 + SLCAN_CMD_LEN + SLCAN_EFF_ID_LEN] = 0;
		/* point to payload data behind the dlc */
		cmd += 1 + SLCAN_CMD_LEN + SLCAN_EFF_ID_LEN + 1;
		break;
	default:
		goto decode_failed;
	}

	if (kstrtou32(sl->rbuff + 1 + SLCAN_CMD_LEN, 16, &tmpid))
		goto decode_failed;

	cf->can_id |= tmpid;

	/* get len from sanitized ASCII value */
	tmp = hex_to_bin(len);
	if (unlikely(cf->len < 0))
		goto decode_failed;

	cf->len = can_fd_dlc2len(tmp);

	for (i = 0; i < cf->len; i++) {
		tmp = hex_to_bin(*cmd++);
		if (tmp < 0)
			goto decode_failed;

		cf->data[i] = (tmp << 4);
		tmp = hex_to_bin(*cmd++);
		if (tmp < 0)
			goto decode_failed;

		cf->data[i] |= tmp;
	}

	sl->dev->stats.rx_packets++;
	sl->dev->stats.rx_bytes += cf->len;

	netif_rx(skb);
	return;

decode_failed:
	sl->dev->stats.rx_errors++;
	dev_kfree_skb(skb);
}

/* A change state frame must contain state info and receive and transmit
 * error counters.
 *
 * Examples:
 *
 * sb256256 : state bus-off: rx counter 256, tx counter 256
 * sa057033 : state active, rx counter 57, tx counter 33
 */
static void slcan_bump_state(struct tcp_slcan *sl)
{
	struct net_device *dev = sl->dev;
	struct sk_buff *skb;
	struct can_frame *cf;
	char *cmd = sl->rbuff;
	u32 rxerr, txerr;
	enum can_state state, rx_state, tx_state;

	switch (cmd[1]) {
	case 'a':
		state = CAN_STATE_ERROR_ACTIVE;
		break;
	case 'w':
		state = CAN_STATE_ERROR_WARNING;
		break;
	case 'p':
		state = CAN_STATE_ERROR_PASSIVE;
		break;
	case 'b':
		state = CAN_STATE_BUS_OFF;
		break;
	default:
		return;
	}

	if (state == sl->can.state || sl->rcount < SLCAN_STATE_FRAME_LEN)
		return;

	cmd += SLCAN_STATE_BE_RXCNT_LEN + SLCAN_CMD_LEN + 1;
	cmd[SLCAN_STATE_BE_TXCNT_LEN] = 0;
	if (kstrtou32(cmd, 10, &txerr))
		return;

	*cmd = 0;
	cmd -= SLCAN_STATE_BE_RXCNT_LEN;
	if (kstrtou32(cmd, 10, &rxerr))
		return;

	skb = alloc_can_err_skb(dev, &cf);

	tx_state = txerr >= rxerr ? state : 0;
	rx_state = txerr <= rxerr ? state : 0;
	can_change_state(dev, cf, tx_state, rx_state);

	if (state == CAN_STATE_BUS_OFF) {
		can_bus_off(dev);
	} else if (skb) {
		cf->can_id |= CAN_ERR_CNT;
		cf->data[6] = txerr;
		cf->data[7] = rxerr;
	}

	if (skb)
		netif_rx(skb);
}

/* An error frame can contain more than one type of error.
 *
 * Examples:
 *
 * e1a : len 1, errors: ACK error
 * e3bcO: len 3, errors: Bit0 error, CRC error, Tx overrun error
 */
static void slcan_bump_err(struct tcp_slcan *sl)
{
	struct net_device *dev = sl->dev;
	struct sk_buff *skb;
	struct can_frame *cf;
	char *cmd = sl->rbuff;
	bool rx_errors = false, tx_errors = false, rx_over_errors = false;
	int i, len;

	/* get len from sanitized ASCII value */
	len = cmd[1];
	if (len >= '0' && len < '9')
		len -= '0';
	else
		return;

	if ((len + SLCAN_CMD_LEN + 1) > sl->rcount)
		return;

	skb = alloc_can_err_skb(dev, &cf);

	if (skb)
		cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;

	cmd += SLCAN_CMD_LEN + 1;
	for (i = 0; i < len; i++, cmd++) {
		switch (*cmd) {
		case 'a':
			netdev_dbg(dev, "ACK error\n");
			tx_errors = true;
			if (skb) {
				cf->can_id |= CAN_ERR_ACK;
				cf->data[3] = CAN_ERR_PROT_LOC_ACK;
			}

			break;
		case 'b':
			netdev_dbg(dev, "Bit0 error\n");
			tx_errors = true;
			if (skb)
				cf->data[2] |= CAN_ERR_PROT_BIT0;

			break;
		case 'B':
			netdev_dbg(dev, "Bit1 error\n");
			tx_errors = true;
			if (skb)
				cf->data[2] |= CAN_ERR_PROT_BIT1;

			break;
		case 'c':
			netdev_dbg(dev, "CRC error\n");
			rx_errors = true;
			if (skb) {
				cf->data[2] |= CAN_ERR_PROT_BIT;
				cf->data[3] = CAN_ERR_PROT_LOC_CRC_SEQ;
			}

			break;
		case 'f':
			netdev_dbg(dev, "Form Error\n");
			rx_errors = true;
			if (skb)
				cf->data[2] |= CAN_ERR_PROT_FORM;

			break;
		case 'o':
			netdev_dbg(dev, "Rx overrun error\n");
			rx_over_errors = true;
			rx_errors = true;
			if (skb) {
				cf->can_id |= CAN_ERR_CRTL;
				cf->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
			}

			break;
		case 'O':
			netdev_dbg(dev, "Tx overrun error\n");
			tx_errors = true;
			if (skb) {
				cf->can_id |= CAN_ERR_CRTL;
				cf->data[1] = CAN_ERR_CRTL_TX_OVERFLOW;
			}

			break;
		case 's':
			netdev_dbg(dev, "Stuff error\n");
			rx_errors = true;
			if (skb)
				cf->data[2] |= CAN_ERR_PROT_STUFF;

			break;
		default:
			if (skb)
				dev_kfree_skb(skb);

			return;
		}
	}

	if (rx_errors)
		dev->stats.rx_errors++;

	if (rx_over_errors)
		dev->stats.rx_over_errors++;

	if (tx_errors)
		dev->stats.tx_errors++;

	if (skb)
		netif_rx(skb);
}

static void slcan_bump(struct tcp_slcan *sl)
{
	switch (sl->rbuff[0]) {
	case 'r':
		fallthrough;
	case 't':
		fallthrough;
	case 'R':
		fallthrough;
	case 'T':
		if (sl->rcount >= 2 && (sl->rbuff[1] == 'x' || sl->rbuff[1] == 'X'))
			return slcan_bump_fd_frame(sl);

		return slcan_bump_frame(sl);
	case 'e':
		return slcan_bump_err(sl);
	case 's':
		return slcan_bump_state(sl);
	default:
		return;
	}
}

static int tcp_slcan_parse_msg(struct strparser *strp, struct sk_buff *skb)
{
	u8 b;
	int i;
	struct strp_msg *rxm = strp_msg(skb);
	int offset = rxm->offset;
	int len = skb->len - offset;
	for (i = 0; i < len; i++) {
		u8 *p = skb_header_pointer(skb, offset + i, 1, &b);
		if (!p)
			break;

		if (*p == '\r')
			return i + 1;
	}
	return 0; // need more data
}

static void tcp_slcan_rcv_msg(struct strparser *strp, struct sk_buff *skb)
{
	struct tcp_slcan *sl = container_of(strp, struct tcp_slcan, strp);
	struct strp_msg *rxm = strp_msg(skb);
	int offset = rxm->offset;
	int len = rxm->full_len;
	if (unlikely(len < 6 || len > SLCAN_FD_MTU - 1)) {
		sl->dev->stats.rx_errors++;
		goto out;
	}
	// strip the '\r' delimiter
	sl->rcount = len - 1;
	if (skb_copy_bits(skb, offset, sl->rbuff, sl->rcount)) {
		sl->dev->stats.rx_errors++;
		goto out;
	}
	slcan_bump(sl);
out:
	kfree_skb(skb);
}

static const struct strp_callbacks tcp_slcan_callbacks = {
	.parse_msg = tcp_slcan_parse_msg,
	.rcv_msg = tcp_slcan_rcv_msg,
};

/*************************************************************************
 *			STANDARD SLCAN ENCAPSULATION			 *
 *************************************************************************/

/* Encapsulate one can_frame and stuff into a TCP socket. */
static int tcp_slcan_encaps(struct tcp_slcan *sl, struct can_frame *cf)
{
	int actual_len, i, ret;
	unsigned char *pos;
	unsigned char *endpos;
	canid_t id = cf->can_id;
	struct kvec iov;
	struct msghdr msg = {0};

	pos = sl->xbuff;

	if (cf->can_id & CAN_RTR_FLAG)
		*pos = 'R'; /* becomes 'r' in standard frame format (SFF) */
	else
		*pos = 'T'; /* becomes 't' in standard frame format (SSF) */

	/* determine number of chars for the CAN-identifier */
	if (cf->can_id & CAN_EFF_FLAG) {
		id &= CAN_EFF_MASK;
		endpos = pos + SLCAN_EFF_ID_LEN;
	} else {
		*pos |= 0x20; /* convert R/T to lower case for SFF */
		id &= CAN_SFF_MASK;
		endpos = pos + SLCAN_SFF_ID_LEN;
	}

	/* build 3 (SFF) or 8 (EFF) digit CAN identifier */
	pos++;
	while (endpos >= pos) {
		*endpos-- = hex_asc_upper[id & 0xf];
		id >>= 4;
	}

	pos += (cf->can_id & CAN_EFF_FLAG) ?
		SLCAN_EFF_ID_LEN : SLCAN_SFF_ID_LEN;

	*pos++ = cf->len + '0';

	/* RTR frames may have a dlc > 0 but they never have any data bytes */
	if (!(cf->can_id & CAN_RTR_FLAG)) {
		for (i = 0; i < cf->len; i++)
			pos = hex_byte_pack_upper(pos, cf->data[i]);
	}

	*pos++ = '\r';
	actual_len = pos - sl->xbuff;

	iov.iov_base = sl->xbuff;
	iov.iov_len = actual_len;

	msg.msg_flags = MSG_DONTWAIT;

	ret = kernel_sendmsg(sl->sock, &msg, &iov, 1, actual_len);
	if (ret > 0) {
		sl->dev->stats.tx_packets++;
		sl->dev->stats.tx_bytes += cf->len;
		return 0;
	}
	if (ret == -EAGAIN || ret == -EWOULDBLOCK)
		return ret;

	sl->dev->stats.tx_errors++;
	return 0;
}

/* Encapsulate one canfd_frame and stuff into a TCP socket. */
static int tcp_slcan_fd_encaps(struct tcp_slcan *sl, struct canfd_frame *cf)
{
	int actual_len, i, ret;
	unsigned char *pos;
	unsigned char *endpos;
	canid_t id = cf->can_id;
	struct kvec iov;
	struct msghdr msg = {0};

	pos = sl->xbuff;

	if (unlikely(cf->can_id & CAN_RTR_FLAG))
		return -EINVAL;

	*pos = 'T'; /* becomes 't' in standard frame format (SSF) */
	*(pos + 1) = (cf->flags & CANFD_BRS) ? 'x' : 'X';

	/* determine number of chars for the CAN-identifier */
	if (cf->can_id & CAN_EFF_FLAG) {
		id &= CAN_EFF_MASK;
		endpos = pos + 1 + SLCAN_EFF_ID_LEN;
	} else {
		*pos |= 0x20; /* convert R/T to lower case for SFF */
		id &= CAN_SFF_MASK;
		endpos = pos + 1 + SLCAN_SFF_ID_LEN;
	}

	/* build 3 (SFF) or 8 (EFF) digit CAN identifier */
	pos += 2;
	while (endpos >= pos) {
		*endpos-- = hex_asc_upper[id & 0xf];
		id >>= 4;
	}

	pos += (cf->can_id & CAN_EFF_FLAG) ?
		SLCAN_EFF_ID_LEN : SLCAN_SFF_ID_LEN;

	*pos++ = hex_asc_upper[can_fd_len2dlc(cf->len) & 0xf];

	for (i = 0; i < cf->len; i++)
		pos = hex_byte_pack_upper(pos, cf->data[i]);

	*pos++ = '\r';
	actual_len = pos - sl->xbuff;

	iov.iov_base = sl->xbuff;
	iov.iov_len = actual_len;

	msg.msg_flags = MSG_DONTWAIT;

	ret = kernel_sendmsg(sl->sock, &msg, &iov, 1, actual_len);
	if (ret > 0) {
		sl->dev->stats.tx_packets++;
		sl->dev->stats.tx_bytes += cf->len;
		return 0;
	}
	if (ret == -EAGAIN || ret == -EWOULDBLOCK)
		return ret;

	sl->dev->stats.tx_errors++;
	return 0;
}

/* Send a can_frame to a TCP socket. */
static netdev_tx_t tcp_slcan_netdev_xmit(struct sk_buff *skb, struct net_device *dev)
{
	int err;
	struct tcp_slcan *sl = netdev_priv(dev);

	if (can_dev_dropped_skb(dev, skb))
		return NETDEV_TX_OK;

	spin_lock_bh(&sl->lock);
	if (!netif_running(dev))  {
		spin_unlock_bh(&sl->lock);
		netdev_warn(dev, "xmit: iface is down\n");
		goto out;
	}
	if (unlikely(!sl->connected || !sl->sock)) {
		spin_unlock_bh(&sl->lock);
		dev->stats.tx_dropped++;
		goto out;
	}
	spin_unlock_bh(&sl->lock);

	if (can_is_canfd_skb(skb))
		err = tcp_slcan_fd_encaps(sl, (struct canfd_frame *)skb->data);
	else
		err = tcp_slcan_encaps(sl, (struct can_frame *)skb->data);

	if (err)
		dev->stats.tx_dropped++;

	skb_tx_timestamp(skb);
out:
	kfree_skb(skb);
	return NETDEV_TX_OK;
}

/******************************************
 *   Routines looking at netdevice side.
 ******************************************/

/* Netdevice UP -> DOWN routine */
static int tcp_slcan_netdev_close(struct net_device *dev)
{
	struct tcp_slcan *sl = netdev_priv(dev);

	netif_stop_queue(dev);
	sl->rcount = 0;
	close_candev(dev);
	sl->can.state = CAN_STATE_STOPPED;
	return 0;
}

/* Netdevice DOWN -> UP routine */
static int tcp_slcan_netdev_open(struct net_device *dev)
{
	struct tcp_slcan *sl = netdev_priv(dev);
	int err;

	sl->can.bittiming.bitrate = 1000000;
	if (dev->mtu == CANFD_MTU)
		sl->can.data_bittiming.bitrate = 4000000;

	err = open_candev(dev);
	if (err) {
		netdev_err(dev, "failed to open can device\n");
		return err;
	}

	sl->can.state = CAN_STATE_ERROR_ACTIVE;
	sl->rcount = 0;
	netif_start_queue(dev);
	return 0;
}

static const struct net_device_ops tcp_slcan_netdev_ops = {
	.ndo_open               = tcp_slcan_netdev_open,
	.ndo_stop               = tcp_slcan_netdev_close,
	.ndo_start_xmit         = tcp_slcan_netdev_xmit,
	.ndo_change_mtu         = can_change_mtu,
};

static void tcp_slcan_data_ready(struct sock *sk)
{
	struct tcp_slcan *sl = sk->sk_user_data;
	if (sl) {
		strp_data_ready(&sl->strp);
		if (sl->orig_data_ready)
			sl->orig_data_ready(sk);
	}
}

static void tcp_slcan_state_change(struct sock *sk)
{
	struct tcp_slcan *sl = sk->sk_user_data;
	if (sl && (sk->sk_state != TCP_ESTABLISHED)) {
		pr_info("tcp_slcan: %s connection lost (state %d)\n", sl->dev->name, sk->sk_state);
		schedule_work(&sl->cleanup_work);
	}
	if (sl && sl->orig_state_change)
		sl->orig_state_change(sk);
}

static void tcp_slcan_connection_cleanup(struct work_struct *work)
{
	struct tcp_slcan *sl = container_of(work, struct tcp_slcan, cleanup_work);
	struct socket *sock;

	spin_lock_bh(&sl->lock);
	if (!sl->connected || !sl->sock) {
		spin_unlock_bh(&sl->lock);
		return;
	}

	sock = sl->sock;
	sl->sock = NULL;
	sl->connected = false;
	spin_unlock_bh(&sl->lock);

	strp_stop(&sl->strp);

	kernel_sock_shutdown(sock, SHUT_RDWR);

	lock_sock(sock->sk);
	sock->sk->sk_user_data = NULL;
	sock->sk->sk_data_ready = sl->orig_data_ready;
	sock->sk->sk_state_change = sl->orig_state_change;
	release_sock(sock->sk);

	strp_done(&sl->strp);

	sockfd_put(sock);

	wake_up_interruptible(&sl->event_wait);
	pr_info("tcp_slcan: %s connection cleanup completed\n", sl->dev->name);
}

static int tcp_slcan_misc_open(struct inode *inode, struct file *file)
{
	int err;
	struct net_device *dev;
	struct tcp_slcan *sl;

	dev = alloc_candev(sizeof(struct tcp_slcan), 1);
	if (!dev)
		return -ENOMEM;

	sl = netdev_priv(dev);
	sl->dev = dev;
	dev->netdev_ops = &tcp_slcan_netdev_ops;
	sl->can.ctrlmode_supported = CAN_CTRLMODE_FD;

	spin_lock_init(&sl->lock);
	init_waitqueue_head(&sl->event_wait);
	INIT_WORK(&sl->cleanup_work, tcp_slcan_connection_cleanup);

	err = register_candev(dev);
	if (err) {
		free_candev(dev);
		return err;
	}

	file->private_data = sl;
	return 0;
}

static ssize_t tcp_slcan_misc_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct tcp_slcan *sl = file->private_data;
	wait_event_interruptible(sl->event_wait, sl->connected == false);
	return 0;
}

static int tcp_slcan_misc_release(struct inode *inode, struct file *file)
{
	struct tcp_slcan *sl = file->private_data;

	tcp_slcan_connection_cleanup(&sl->cleanup_work);
	cancel_work_sync(&sl->cleanup_work);

	unregister_candev(sl->dev);
	free_candev(sl->dev);
	file->private_data = NULL;
	return 0;
}

static long tcp_slcan_misc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err, fd;
	struct tcp_slcan *sl;
	struct socket *sock;

	if (cmd != TCP_SLCAN_IOC_GET_NAME && cmd != TCP_SLCAN_IOC_ATTACH)
		return -EINVAL;

	sl = file->private_data;

	if (cmd == TCP_SLCAN_IOC_GET_NAME) {
		if (copy_to_user((char __user *)arg, sl->dev->name, IFNAMSIZ))
			return -EFAULT;

		return 0;
	}

	if (sl->connected)
		return -EBUSY;

	if (copy_from_user(&fd, (int __user *)arg, sizeof(int)))
		return -EFAULT;

	sock = sockfd_lookup(fd, &err);
	if (!sock)
		return err;

	sl->sock = sock;
	strp_init(&sl->strp, sock->sk, &tcp_slcan_callbacks);

	lock_sock(sock->sk);
	sl->connected = true;
	sock->sk->sk_user_data = sl;
	sl->orig_data_ready = sock->sk->sk_data_ready;
	sl->orig_state_change = sock->sk->sk_state_change;
	sock->sk->sk_data_ready = tcp_slcan_data_ready;
	sock->sk->sk_state_change = tcp_slcan_state_change;
	release_sock(sock->sk);

	strp_check_rcv(&sl->strp);
	netif_wake_queue(sl->dev);
	pr_info("tcp_slcan: %s attached to socket fd %d\n", sl->dev->name, fd);
	return 0;
}

static const struct file_operations tcp_slcan_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = tcp_slcan_misc_ioctl,
	.open = tcp_slcan_misc_open,
	.read = tcp_slcan_misc_read,
	.release = tcp_slcan_misc_release,
};

static struct miscdevice tcp_slcan_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "tcp_slcan",
	.fops = &tcp_slcan_fops,
};

module_misc_device(tcp_slcan_misc);
