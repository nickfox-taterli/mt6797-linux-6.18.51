/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __BTMT6797_COMPAT_H
#define __BTMT6797_COMPAT_H

#include <linux/types.h>

/* The two supplied 2016 ROMv3 releases accept the baseline LE commands but
 * omit their command bits. They reject LE event bits above 0x1f and the
 * mandatory Read Maximum Data Length command. Keep the standard host on the
 * usable LE interface; software privacy/ECC do not need those controller
 * extensions. The caller restricts this to the verified firmware versions
 * and leaves HCI USER-channel diagnostics completely unmodified.
 */
static void mt6797_bt_fixup_capabilities(u8 *event, size_t length)
{
	u16 opcode;
	u8 *commands;

	if (length < 6 || event[0] != 0x0e || event[1] + 2U != length || event[5])
		return;
	opcode = event[3] | (event[4] << 8);
	if (opcode == 0x1002 && length == 70) {
		commands = event + 6;
		commands[25] |= 0xf7;
		commands[26] |= 0xff;
		commands[27] |= 0xff;
		commands[28] |= 0x0f;
		/* These ECC completions cannot be enabled in the LE event mask.
		 * The standard host can compute P-256/DHKey in software instead.
		 */
		commands[34] &= ~0x06;
	} else if (opcode == 0x2003 && length == 14) {
		/* DLE, LL privacy and extended scanner policy require the missing
		 * event interface. Do not advertise working host support for them.
		 */
		event[6] &= ~0xe0;
	}
}

#endif
