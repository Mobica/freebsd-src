/*-
 * SPDX-License-Identifier: ISC
 *
 * Copyright (c) 2008-2009 Sam Leffler, Errno Consulting
 * Copyright (c) 2008 Atheros Communications, Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef	__AR9271_AN_H__
#define	__AR9271_AN_H__

#include "ar9002/ar9285an.h"

/* AR9271 Analog register definitions */

/* Bits for AR9271_AN_RF2G3. */
#define AR9271_AN_RF2G3_CCOMP		0x00000fff
#define AR9271_AN_RF2G3_CCOMP_S		0
#define AR9271_AN_RF2G3_OB_QAM		0x00007000
#define AR9271_AN_RF2G3_OB_QAM_S	12
#define AR9271_AN_RF2G3_OB_PSK		0x00038000
#define AR9271_AN_RF2G3_OB_PSK_S	15
#define AR9271_AN_RF2G3_OB_CCK		0x001c0000
#define AR9271_AN_RF2G3_OB_CCK_S	18
#define AR9271_AN_RF2G3_DB1		0x00e00000
#define AR9271_AN_RF2G3_DB1_S		21

/* Bits for AR9271_AN_RF2G4. */
#define AR9271_AN_RF2G4_DB2		0xe0000000
#define AR9271_AN_RF2G4_DB2_S		29

#endif	/* __AR9271_AN_H__ */
