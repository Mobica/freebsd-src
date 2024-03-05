# $FreeBSD$

INCDIR=${SRCTOP}/sys/compat/linuxkpi/common/include

.PATH: ${SRCTOP}/sys/dev/usb/wlan ${SRCTOP}/sys/dev/ath ${SRCTOP}/sys/dev/ath/ath_hal

.include <src.opts.mk>

KMOD    = if_ath_usb

EXPORT_SYMS=    YES

SRCS=   if_ath_usb.c if_ath_usb_fw.c
SRCS+=	if_ath_usb_devlist.h
# SRCS+=  opt_inet.h opt_inet6.h opt_route.h opt_netlink.h

SRCS+=	${LINUXKPI_GENSRCS}

CFLAGS+=	-ferror-limit=0
CFLAGS+=	-I${INCDIR}

CFLAGS+=  -I. -I${SRCTOP}/sys/dev/ath -I${SRCTOP}/sys/dev/ath/ath_hal
CFLAGS+=  -I. -I${SRCTOP}/sys/contrib/dev/ath/ath_hal/

# CFLAGS+= 	-I${SRCTOP}/sys/dev/athn/headers

DEBUG_FLAGS+= -O0 -g

# CFLAGS+= 	-I${SRCTOP}/sys/dev/athn/include/amd64
# CFLAGS+= 	-I${SRCTOP}/sys/dev/athn/include/sys
# CFLAGS+= 	-I${SRCTOP}/sys/dev/athn/include/net80211


CWARNFLAGS+=	${NO_WUNUSED_BUT_SET_VARIABLE}
CWARNFLAGS+=	${NO_WUNUSED_PARAMETER}
CWARNFLAGS+=	${NO_WUNUSED_FUNCTION}

.include <bsd.kmod.mk>
