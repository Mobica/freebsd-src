/*-
 * Copyright (c) 2011 Damien Bergamini <damien.bergamini@free.fr>
 * Copyright (c) 2018 Stefan Sperling <stsp@openbsd.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
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

#ifndef __IF_ATH_USB_DEVID_H__
#define __IF_ATH_USB_DEVID_H__

/* $OpenBSD: usbdevs.h,v 1.768 2023/05/10 18:26:43 miod Exp $ */
#define USB_VENDOR_ATHEROS2                 0x0cf3  /* Atheros Communications */
#define USB_VENDOR_AZUREWAVE                0x13d3  /* AsureWave */
#define USB_VENDOR_DLINK2                   0x07d1  /* D-Link */
#define USB_VENDOR_LITEON                   0x04ca  /* Lite-On Technology */
#define USB_VENDOR_NETGEAR                  0x0846  /* BayNETGEAR */
#define USB_VENDOR_VIA                      0x040d  /* VIA */
#define USB_VENDOR_ACCTON                   0x083a  /* Accton Technology */
#define USB_VENDOR_ACTIONTEC                0x1668  /* Actiontec Electronics */
#define USB_VENDOR_PANASONIC                0x04da  /* Panasonic (Matsushita) */
#define USB_VENDOR_MELCO                    0x0411  /* Melco */

#define USB_PRODUCT_ATHEROS2_AR9271_1       0x1006  /* AR9271 */
#define USB_PRODUCT_ATHEROS2_AR9271_2       0x9271  /* AR9271 */
#define USB_PRODUCT_ATHEROS2_AR9271_3       0xb003  /* AR9271 */
#define USB_PRODUCT_AZUREWAVE_AR9271_1      0x3327  /* AR9271 */
#define USB_PRODUCT_AZUREWAVE_AR9271_2      0x3328  /* AR9271 */
#define USB_PRODUCT_AZUREWAVE_AR9271_3      0x3346  /* AR9271 */
#define USB_PRODUCT_AZUREWAVE_AR9271_4      0x3348  /* AR9271 */
#define USB_PRODUCT_AZUREWAVE_AR9271_5      0x3349  /* AR9271 */
#define USB_PRODUCT_AZUREWAVE_AR9271_6      0x3350  /* AR9271 */
#define USB_PRODUCT_DLINK2_AR9271           0x3a10  /* AR9271 */
#define USB_PRODUCT_LITEON_AR9271           0x4605  /* AR9271 */
#define USB_PRODUCT_NETGEAR_WNA1100         0x9030  /* WNA1100 */
#define USB_PRODUCT_VIA_AR9271              0x3801  /* AR9271 */
#define USB_PRODUCT_ACCTON_AR9280           0xa704  /* AR9280+AR7010 */
#define USB_PRODUCT_ACTIONTEC_AR9287        0x1200  /* AR9287+AR7010 */
#define USB_PRODUCT_NETGEAR_WNDA3200        0x9018  /* WNDA3200 */
#define USB_PRODUCT_PANASONIC_N5HBZ0000055  0x3904  /* UB94 */
#define USB_PRODUCT_MELCO_UWABR100          0x017f  /* SONY UWA-BR100 */
#define USB_PRODUCT_ATHEROS2_AR9280         0x7010  /* AR9280+AR7010 */
#define USB_PRODUCT_ATHEROS2_AR9287         0x7015  /* AR9287+AR7010 */

#define ATHN_USB_FLAG_AR7010                0x01

#endif
