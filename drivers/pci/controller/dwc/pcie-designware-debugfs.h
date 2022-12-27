/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Synopsys DesignWare PCIe controller debugfs driver
 *
 * Copyright (C) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Author: Shradha Todi <shradha.t@samsung.com>
 */

#ifndef _PCIE_DESIGNWARE_DEBUGFS_H
#define _PCIE_DESIGNWARE_DEBUGFS_H

#include "pcie-designware.h"

#define RAS_DES_EVENT_COUNTER_CTRL_REG	0x8
#define RAS_DES_EVENT_COUNTER_DATA_REG	0xC
#define SD_STATUS_L1LANE_REG		0xB0
#define ERR_INJ_ENABLE_REG		0x30
#define ERR_INJ0_OFF			0x34

#define LANE_DETECT_SHIFT		17
#define LANE_DETECT_MASK		0x1
#define PIPE_RXVALID_SHIFT		18
#define PIPE_RXVALID_MASK		0x1

#define LANE_SELECT_SHIFT		8
#define LANE_SELECT_MASK		0xF
#define EVENT_COUNTER_STATUS_SHIFT	7
#define EVENT_COUNTER_STATUS_MASK	0x1
#define EVENT_COUNTER_ENABLE		(0x7 << 2)
#define PER_EVENT_OFF			(0x1 << 2)
#define PER_EVENT_ON			(0x3 << 2)

#define EINJ_COUNT_MASK			0xFF
#define EINJ_TYPE_MASK			0xFFFFFF
#define EINJ_TYPE_SHIFT			8

enum event_numbers {
	ebuf_overflow		= 0x0,
	ebuf_underrun		= 0x1,
	decode_error		= 0x2,
	sync_header_error	= 0x5,
	receiver_error		= 0x106,
	framing_error		= 0x109,
	lcrc_error		= 0x201,
	ecrc_error		= 0x302,
	unsupp_req_error	= 0x303,
	cmpltr_abort_error	= 0x304,
	cmpltn_timeout_error	= 0x305,
	tx_l0s_entry		= 0x502,
	rx_l0s_entry		= 0x503,
	l1_entry		= 0x505,
	l1_1_entry		= 0x507,
	l1_2_entry		= 0x508,
	l2_entry		= 0x50B,
	speed_change		= 0x50C,
	width_chage		= 0x50D,
};

/*
 * struct event_counters: Struct to store event number
 *
 * @name: name of the event counter
 *        eg: ecrc_err count, l1 entry count
 * @event_num: Event number and group number
 * [16:8]: Group number
 * [7:0]: Event number
 */
struct event_counters {
	const char *name;
	u32 event_num;
};

enum error_inj_code {
	tx_lcrc			= 0x000,
	tx_ecrc			= 0x300,
	rx_lcrc			= 0x800,
	rx_ecrc			= 0xB00,
};

/*
 * struct error_injectionss: Struct to store error numbers
 *
 * @name: name of the error to be injected
 *        eg: ecrc_err, lcrc_err
 * @event_num: Error number and group number
 * [31:8]: Error type. This should be same as bits [31:8]
 *         in the EINJn_REG where n is group number
 * [7:0]: Error injection group
 *        0 - CRC
 *        1 - seq num
 *        2 - DLLP error
 *        3 - symbol error
 *        4 - FC error
 */
struct error_injections {
	const char *name;
	u32 type_of_err;
};

#define CREATE_RAS_EVENT_COUNTER_DEBUGFS(name)				\
do {									\
	sub_dir = debugfs_create_dir(#name, ras_des_event_counter);	\
	debugfs_create_file("counter_enable", 0644, sub_dir, pci,	\
				&cnt_en_ops);				\
	debugfs_create_file("lane_select", 0644, sub_dir, pci,		\
				&lane_sel_ops);				\
	debugfs_create_file("counter_value", 0444, sub_dir, pci,	\
				&cnt_val_ops);				\
} while (0)

#define CREATE_RAS_ERROR_INJECTION_DEBUGFS(name)			\
	debugfs_create_file(#name, 0644, ras_des_error_inj, pci,	\
				&inj_ops);

#ifdef CONFIG_PCIE_DW_DEBUGFS
int create_debugfs_files(struct dw_pcie *pci);
void remove_debugfs_files(void);
#else
int create_debugfs_files(struct dw_pcie *pci)
{
	/* No OP */
	return 0;
}

void remove_debugfs_files(void)
{
	/* No OP */
}
#endif

#endif
