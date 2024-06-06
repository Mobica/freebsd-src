
DEVATH9KDIR=	${SRCTOP}/sys/contrib/dev/athk/ath9k

.PATH: ${DEVATH9KDIR}

# WITH_CONFIG_FWLOG=	1

KMOD=	if_ath9k

SRCS+=	beacon.c gpio.c init.c main.c 
# SRCS+=  debug.c
SRCS+=	recv.c xmit.c link.c antenna.c channel.c
SRCS+=	ar9002_hw.c ar9003_hw.c hw.c
SRCS+=	ar9002_phy.c ar9003_phy.c ar5008_phy.c
SRCS+=  ar9002_calib.c ar9003_calib.c calib.c
SRCS+=  eeprom.c eeprom_def.c eeprom_4k.c eeprom_9287.c
SRCS+=  ani.c mac.c ar9002_mac.c ar9003_mac.c ar9003_eeprom.c ar9003_paprd.c
SRCS+=  common.c common-init.c common-beacon.c
# SRCS+= 	dynack.c
SRCS+=  htc_hst.c hif_usb.c wmi.c htc_drv_txrx.c htc_drv_main.c htc_drv_beacon.c htc_drv_init.c htc_drv_gpio.c

SRCS+=	pci.c

# Other
SRCS+=	${LINUXKPI_GENSRCS}
SRCS+=	opt_wlan.h opt_inet6.h opt_inet.h opt_acpi.h

.if defined(WITH_CONFIG_FWLOG) && ${WITH_CONFIG_FWLOG} > 0
SRCS+=	fwlog.c
CFLAGS+=	-DCONFIG_FWLOG=${WITH_CONFIG_FWLOG}
.endif

CFLAGS+=	-DKBUILD_MODNAME='"ath9k"'

CFLAGS+=	-I${DEVATH9KDIR}
CFLAGS+=	-I${DEVATH9KDIR}/..
CFLAGS+=	${LINUXKPI_INCLUDES}
CFLAGS+=    -O0 -g
# Helpful after fresh imports.
#CFLAGS+=	-ferror-limit=0

# CFLAGS+=	-DCONFIG_ATH9K_DEBUG
CFLAGS+=	-DCONFIG_ATH9K_HW

.include <bsd.kmod.mk>
CFLAGS+= -Wno-cast-qual
