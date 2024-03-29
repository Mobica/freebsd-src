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

/*
 * USB front-end for Atheros AR9271 and AR7010 chipsets.
 */

// #include "bpfilter.h"
#define NBPFILTER 0 // TODO ?
#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/types.h>
#include <sys/param.h>
#include <sys/limits.h>
#include <sys/mutex.h>
#include <sys/module.h>
#include <sys/firmware.h>
#include <sys/sockio.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/time.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/linker.h>
#include <sys/kdb.h>
#include <sys/queue.h>
#include <sys/taskqueue.h>

#if NBPFILTER > 0
#include <net/bpf.h>
#endif
#include <machine/bus.h>
#include <machine/resource.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/ethernet.h>
#include <net/if_media.h>

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_amrr.h>
#include <net80211/ieee80211_radiotap.h>
#include <net80211/ieee80211_ratectl.h>

#include "if_athvar.h"
#include "if_ath_usb_devlist.h"

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usb_device.h>
#include <dev/usb/usbdi_util.h>

#include "openbsd_adapt.h"
#include "if_ath_usb_def.h"
#include "if_ath_usb_fw.h"
#include "if_ath_debug.h"
// TODO: DPRINTF from ATHN has different parameters than DPRINTF from ATH
#ifdef DPRINTF
	#undef DPRINTF
#endif
#ifdef ATHN_DEBUG
	#define DPRINTF(x)	do { if (athn_debug > 0) printf x; } while (0)
	#define DPRINTFN(n, x)	do { if (athn_debug >= (n)) printf x; } while (0)
	extern int athn_debug;
#else
	#define DPRINTF(x)
	#define DPRINTFN(n, x)
#endif


unsigned int ifq_oactive = 0;

#define ath_usb_lookup(uaa)	\
	(usbd_lookup_id_by_uaa(ath_usb_devs, sizeof(ath_usb_devs), uaa))

// TODO: Move it or delete if not needed
/* Tx queue software indexes. */
#define ATH_QID_AC_BE		0
#define ATH_QID_PSPOLL		1
#define ATH_QID_AC_BK		2
#define ATH_QID_AC_VI		3
#define ATH_QID_AC_VO		4
#define ATH_QID_UAPSD		5
#define ATH_QID_CAB		6
#define ATH_QID_BEACON		7
#define ATH_QID_COUNT		8

// TODO: Move it or delete if not needed
/* Bits for AR_RX_FILTER. */
#define AR_RX_FILTER_UCAST	0x00000001
#define AR_RX_FILTER_MCAST	0x00000002
#define AR_RX_FILTER_BCAST	0x00000004
#define AR_RX_FILTER_CONTROL	0x00000008
#define AR_RX_FILTER_BEACON	0x00000010
#define AR_RX_FILTER_PROM	0x00000020
#define AR_RX_FILTER_PROBEREQ	0x00000080
#define AR_RX_FILTER_MYBEACON	0x00000200
#define AR_RX_FILTER_COMPR_BAR	0x00000400
#define AR_RX_FILTER_PSPOLL	0x00004000

static device_probe_t	ath_usb_match;
static device_attach_t	ath_usb_attach;
static device_detach_t	ath_usb_detach;

// Temp
static void
ar9271_load_ani(struct ath_softc *sc)
{
#if Nath_USB > 0
	/* Write ANI registers. */
	AR_WRITE(sc, AR_PHY_DESIRED_SZ, 0x6d4000e2);
	AR_WRITE(sc, AR_PHY_AGC_CTL1,   0x3139605e);
	AR_WRITE(sc, AR_PHY_FIND_SIG,   0x7ec84d2e);
	AR_WRITE(sc, AR_PHY_SFCORR_LOW, 0x06903881);
	AR_WRITE(sc, AR_PHY_SFCORR,     0x5ac640d0);
	AR_WRITE(sc, AR_PHY_CCK_DETECT, 0x803e68c8);
	AR_WRITE(sc, AR_PHY_TIMING5,    0xd00a8007);
	AR_WRITE(sc, AR_PHY_SFCORR_EXT, 0x05eea6d4);
	AR_WRITE_BARRIER(sc);
#endif	/* Nath_USB */
}

void		ath_usb_attachhook(device_t self);
int		ath_usb_open_pipes(struct ath_usb_softc *);
void		ath_usb_close_pipes(struct ath_usb_softc *);
static int	ath_alloc_list(struct ath_usb_softc *sc, struct ath_usb_data data[],
				int ndata, int maxsz);
static int	ath_free_list(struct ath_usb_softc *sc, struct ath_usb_data data[],
						   int ndata);
int		ath_usb_alloc_rx_list(struct ath_usb_softc *);
void		ath_usb_free_rx_list(struct ath_usb_softc *);
int		ath_usb_alloc_tx_list(struct ath_usb_softc *);
void		ath_usb_free_tx_list(struct ath_usb_softc *);
int		ath_usb_alloc_tx_cmd(struct ath_usb_softc *);
void		ath_usb_free_tx_cmd(struct ath_usb_softc *);
void		ath_usb_task(void *, int);
void		ath_usb_do_async(struct ath_usb_softc *,
		    void (*)(struct ath_usb_softc *, void *), void *, int);
void		ath_usb_wait_async(struct ath_usb_softc *);
int		ath_usb_htc_msg(struct ath_usb_softc *, uint16_t, void *,
		    int);
int		ath_usb_htc_setup(struct ath_usb_softc *);
int		ath_usb_htc_connect_svc(struct ath_usb_softc *, uint16_t,
		    uint8_t, uint8_t, uint8_t *);
int		ath_usb_wmi_xcmd(struct ath_usb_softc *, uint16_t, void *,
		    int, void *);
int		ath_usb_read_rom(struct ath_softc *);
uint32_t	ath_usb_read(struct ath_softc *, uint32_t);
void		ath_usb_write(struct ath_softc *, uint32_t, uint32_t);
void		ath_usb_write_barrier(struct ath_softc *);
int		ath_usb_media_change(struct ifnet *);
void		ath_usb_next_scan(void *, int);
int		ath_usb_newstate(struct ieee80211vap *, enum ieee80211_state,
		    int);
//void		ath_usb_newstate_cb(struct ath_usb_softc *, void *);
void		ath_usb_newassoc(struct ieee80211com *,
		    struct ieee80211_node *, int);
void		ath_usb_newassoc_cb(struct ath_usb_softc *, void *);
struct ieee80211_node *ath_usb_node_alloc(struct ieee80211com *);
void		ath_usb_count_active_sta(void *, struct ieee80211_node *);
void		ath_usb_newauth_cb(struct ath_usb_softc *, void *);
int		ath_usb_newauth(struct ieee80211com *,
		    struct ieee80211_node *, int, uint16_t);
void		ath_usb_node_free(struct ieee80211com *,
		    struct ieee80211_node *);
void		ath_usb_node_free_cb(struct ath_usb_softc *, void *);
int		ath_usb_ampdu_tx_start(struct ieee80211com *,
		    struct ieee80211_node *, uint8_t);
void		ath_usb_ampdu_tx_start_cb(struct ath_usb_softc *, void *);
void		ath_usb_ampdu_tx_stop(struct ieee80211com *,
		    struct ieee80211_node *, uint8_t);
void		ath_usb_ampdu_tx_stop_cb(struct ath_usb_softc *, void *);
void		ath_usb_clean_nodes(void *, struct ieee80211_node *);
int		ath_usb_create_node(struct ath_usb_softc *,
		    struct ieee80211_node *);
int		ath_usb_node_set_rates(struct ath_usb_softc *,
		    struct ieee80211_node *);
int		ath_usb_remove_node(struct ath_usb_softc *,
		    struct ieee80211_node *);
void		ath_usb_rx_enable(struct ath_softc *);
int		ath_set_chan(struct ath_softc *, struct ieee80211_channel *,
		    struct ieee80211_channel *);
int		ath_usb_switch_chan(struct ath_softc *,
		    struct ieee80211_channel *, struct ieee80211_channel *);
void		ath_usb_updateedca(struct ieee80211com *);
void		ath_usb_updateedca_cb(struct ath_usb_softc *, void *);
void		ath_usb_updateslot(struct ieee80211com *);
void		ath_usb_updateslot_cb(struct ath_usb_softc *, void *);
int		ath_usb_set_key(struct ieee80211com *,
		    struct ieee80211_node *, struct ieee80211_key *);
void		ath_usb_set_key_cb(struct ath_usb_softc *, void *);
void		ath_usb_delete_key(struct ieee80211com *,
		    struct ieee80211_node *, struct ieee80211_key *);
void		ath_usb_delete_key_cb(struct ath_usb_softc *, void *);
void		ath_usb_bcneof(struct usb_xfer *xfer, void *priv,
			usb_error_t status);
void		ath_usb_swba(struct ath_usb_softc *);
void		ath_usb_tx_status(void *, struct ieee80211_node *);
void		ath_usb_rx_wmi_ctrl(struct ath_usb_softc *, uint8_t *, int);
void		ath_usb_intr(struct usb_xfer *, void *,
		    usb_error_t);
void		ath_usb_rx_radiotap(struct ath_softc *, struct mbuf *,
		    struct ar_rx_status *);
void		ath_usb_rx_frame(struct ath_usb_softc *, struct mbuf */*,
		    struct mbuf_list **/); //MichalP: Uses mbuf_list which doesn't exist in FreeBSD
void		ath_usb_rxeof(struct usb_xfer *, struct ath_usb_data*);
void		ath_usb_txeof(struct usb_xfer *, struct ath_usb_data *);
int		ath_usb_tx(struct ath_softc *, struct mbuf *,
		    struct ieee80211_node *);
void		ath_usb_start(struct ifnet *);
void		ath_usb_watchdog(struct ifnet *);
int		ath_usb_ioctl(struct ifnet *, u_long, caddr_t);
int		ath_usb_init(struct ifnet *);
void		ath_usb_stop(struct ifnet *);
void		ar9271_load_ani(struct ath_softc *);
int		ar5008_ccmp_decap(struct ath_softc *, struct mbuf *,
		    struct ieee80211_node *);
int		ar5008_ccmp_encap(struct mbuf *, u_int, struct ieee80211_key *);

/* Shortcut. */
#define ath_usb_wmi_cmd(sc, cmd_id)	\
	ath_usb_wmi_xcmd(sc, cmd_id, NULL, 0, NULL)

/* Extern functions. */
#if ATHN_API
void		ath_led_init(struct ath_softc *);
void		ath_set_led(struct ath_softc *, int);
void		ath_btcoex_init(struct ath_softc *);
void		ath_set_rxfilter(struct ath_softc *, uint32_t);
int		ath_reset(struct ath_softc *, int);

void		ath_init_pll(struct ath_softc *,
		    const struct ieee80211_channel *);
int		ath_set_power_awake(struct ath_softc *);
void		ath_set_power_sleep(struct ath_softc *);
void		ath_reset_key(struct ath_softc *, int);
int		ath_set_key(struct ieee80211com *, struct ieee80211_node *,
		    struct ieee80211_key *);
void		ath_delete_key(struct ieee80211com *, struct ieee80211_node *,
		    struct ieee80211_key *);
void		ath_rx_start(struct ath_softc *);
void		ath_set_sta_timers(struct ath_softc *);
void		ath_set_hostap_timers(struct ath_softc *);
void		ath_set_opmode(struct ath_softc *);
void		ath_set_bss(struct ath_softc *, struct ieee80211_node *);
int		ath_hw_reset(struct ath_softc *, struct ieee80211_channel *,
		    struct ieee80211_channel *, int);
void		ath_updateedca(struct ieee80211com *);
void		ath_updateslot(struct ieee80211com *);
#endif

static void ath_if_intr_rx_callback(struct usb_xfer *xfer, usb_error_t error);
static void ath_if_intr_tx_callback(struct usb_xfer *xfer, usb_error_t error);
static void ath_if_bulk_rx_callback(struct usb_xfer *xfer, usb_error_t error);
static void ath_if_bulk_tx_callback(struct usb_xfer *xfer, usb_error_t error);

static struct usb_config ath_if_config[ATH_N_XFER] = {
        [ath_BULK_TX] = { // MichalP: standard I/O
                .type = UE_BULK,
                .endpoint = AR_PIPE_TX_DATA,
                .direction = UE_DIR_OUT,
                .bufsize = 512,
                .flags = {.pipe_bof = 1,.force_short_xfer = 1,},
                .callback = ath_if_bulk_tx_callback,
				.timeout = ath_USB_TX_TIMEOUT
        }, // MichalP: standard I/O
        [ath_BULK_RX] = {
                .type = UE_BULK,
                .endpoint = AR_PIPE_RX_DATA,
                .direction = UE_DIR_IN, // .direction = UE_DIR_IN,
                .bufsize = 512,
                .flags = {.ext_buffer=1,.pipe_bof = 1,.short_xfer_ok = 1,},
                .callback = ath_if_bulk_rx_callback,
        }, // MichalP: what the device sends to us (CMD replies)
        [ath_BULK_IRQ] = {
                .type = UE_INTERRUPT,
                .endpoint = AR_PIPE_RX_INTR,
                .direction = UE_DIR_IN,
                .bufsize = 64,
                .flags = {.pipe_bof = 1,.short_xfer_ok = 1,},
                .callback = ath_if_intr_rx_callback,
				.interval = 1
        }, // MichalP: what we are sending to the device (CMDs)
		[ath_BULK_CMD] = {
                .type = UE_INTERRUPT,
                .endpoint = AR_PIPE_TX_INTR,
                .direction = UE_DIR_OUT,
                .bufsize = 64,
                .flags = {.pipe_bof = 1,.force_short_xfer = 1,},
                .callback = ath_if_intr_tx_callback,
			    .timeout = 5000,	/* ms */
				.interval = 1
        }
};

#define	INFSLP	UINT64_MAX
#define NSEC_PER_SEC 1000000000

// TODO MichalP: we use msleep with mtx so this could be obsolete
#if OpenBSD_ONLY
int
tsleep_nsec(const void *ident, int priority, const char *wmesg,
    uint64_t nsecs)
{
	uint64_t to_ticks;
	int tick_nsec = (NSEC_PER_SEC + hz / 2) / hz;

	if (nsecs == INFSLP)
		return tsleep(ident, priority, wmesg, 0);
#ifdef DIAGNOSTIC
	if (nsecs == 0) {
		log(LOG_WARNING,
		    "%s: %s[%d]: %s: trying to sleep zero nanoseconds\n",
		    __func__, curproc->p_p->ps_comm, curproc->p_p->ps_pid,
		    wmesg);
	}
#endif
	/*
	 * We want to sleep at least nsecs nanoseconds worth of ticks.
	 *
	 *  - Clamp nsecs to prevent arithmetic overflow.
	 *
	 *  - Round nsecs up to account for any nanoseconds that do not
	 *    divide evenly into tick_nsec, otherwise we'll lose them to
	 *    integer division in the next step.  We add (tick_nsec - 1)
	 *    to keep from introducing a spurious tick if there are no
	 *    such nanoseconds, i.e. nsecs % tick_nsec == 0.
	 *
	 *  - Divide the rounded value to a count of ticks.  We divide
	 *    by (tick_nsec + 1) to discard the extra tick introduced if,
	 *    before rounding, nsecs % tick_nsec == 1.
	 *
	 *  - Finally, add a tick to the result.  We need to wait out
	 *    the current tick before we can begin counting our interval,
	 *    as we do not know how much time has elapsed since the
	 *    current tick began.
	 */
	nsecs = MIN(nsecs, UINT64_MAX - tick_nsec);
	to_ticks = (nsecs + tick_nsec - 1) / (tick_nsec + 1) + 1;
	if (to_ticks > INT_MAX)
		to_ticks = INT_MAX;
	return tsleep(ident, priority, wmesg, (int)to_ticks);
}
#endif

static device_method_t ath_usb_methods[] = {
	DEVMETHOD(device_probe,		ath_usb_match),
	DEVMETHOD(device_attach,	ath_usb_attach),
	DEVMETHOD(device_detach,	ath_usb_detach),

	DEVMETHOD_END
};

static driver_t ath_usb_driver = {
	.name = "if_ath_usb",
	.methods = ath_usb_methods,
	.size = sizeof(struct ath_softc)
};

/* Temporary to nofiy that the module was loaded TODO MichalP: this can be removed at somepoint*/
static int
ath_usb_load(struct module *m, int what, void *arg)
{
	int error = 0;

	switch (what) {
	case MOD_LOAD:
		uprintf("[ath_usb] loaded\n");
		break;
	case MOD_UNLOAD:
		uprintf("[ath_usb] unloaded\n");
		break;
	default:
		error = EOPNOTSUPP;
		break;
	}
	return(error);
}

DRIVER_MODULE(if_ath_usb, uhub, ath_usb_driver, ath_usb_load, NULL);
MODULE_DEPEND(if_ath_usb, usb, 1, 1, 1);
MODULE_DEPEND(if_ath_usb, wlan, 1, 1, 1);
MODULE_DEPEND(if_ath_usb, ath_main, 1, 1, 1);
MODULE_DEPEND(if_ath_usb, ath_hal, 1, 1, 1);
MODULE_VERSION(if_ath_usb, 1);
USB_PNP_HOST_INFO(ath_usb_devs);

static int
ath_usb_match(device_t self)
{
	struct usb_attach_arg *uaa =  device_get_ivars(self);
	int result = 0;

	if (uaa->usb_mode != USB_MODE_HOST)
		return (ENXIO);
	if (uaa->info.bConfigIndex != 0)
		return (ENXIO);
	if (uaa->info.bIfaceIndex != 0)
		return (ENXIO);

	result = ath_usb_lookup(uaa);

	printf("ath_usb_lookup called with result %d \n", result);

	if (result == 0) {
		printf("serial: %s \n", usb_get_serial(uaa->device));
		printf("product: %s-%s \n", usb_get_manufacturer(uaa->device),
		    usb_get_product(uaa->device));
		printf("ProductID: 0x%x \n", uaa->info.idProduct);
		printf("VendorID: 0x%x \n", uaa->info.idVendor);
	}

	return result;
}

static int
ath_usb_attach(device_t self)
{
	struct usb_attach_arg *uaa = device_get_ivars(self);
	struct ath_softc *sc = device_get_softc(self);
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_usb_softc *usc;
	usb_error_t error;
	struct usb_endpoint *ep, *ep_end;

	device_set_usb_desc(self);
	usc = malloc(sizeof(struct ath_usb_softc), M_TEMP,
	      M_NOWAIT | M_ZERO);
	sc->usc = usc;
	usc->sc_sc = sc;
	usc->sc_udev = uaa->device;
	usc->no_buffer_write = TRUE;
	sc->sc_dev = self;
	ic->ic_name = device_get_nameunit(self);

	usc->flags = uaa->driver_info;
#ifdef notyet
	/* Check if it is a combo WiFi+Bluetooth (WB193) device. */
	if (strncmp(product, "wb193", 5) == 0)
		sc->flags |= ath_FLAG_BTCOEX3WIRE;
#endif

#if ATHN_API
	sc->ops.read = ath_usb_read;
	sc->ops.write = ath_usb_write;
	sc->ops.write_barrier = ath_usb_write_barrier;
#endif
	//mtx_init(&sc->sc_usb_mtx, device_get_nameunit(self), MTX_NETWORK_LOCK, MTX_DEF);

	ATH_LOCK_INIT(sc);
	ATH_USB_LOCK_INIT(sc);
 	ATH_PCU_LOCK_INIT(sc);
 	ATH_RX_LOCK_INIT(sc);
 	ATH_TX_LOCK_INIT(sc);
 	ATH_TXSTATUS_LOCK_INIT(sc);

#if ATHN_API
	// MichalP calib_to timeout missing don't know how to put it all together
	TIMEOUT_TASK_INIT(taskqueue_thread, &sc->scan_to, 0, ath_usb_next_scan, sc);
	TASK_INIT(&sc->sc_task, 0, ath_usb_task, sc);
#endif
	error = usbd_transfer_setup(uaa->device, &uaa->info.bIfaceIndex, usc->sc_xfer, ath_if_config,
								ATH_N_XFER, usc, &sc->sc_usb_mtx);
	if (error) {
		device_printf(sc->sc_dev,
					  "could not allocate USB transfers, err=%s\n",
					  usbd_errstr(error));
		ATH_TXSTATUS_LOCK_DESTROY(sc);
		ATH_USB_LOCK_DESTROY(sc);
		ATH_PCU_LOCK_DESTROY(sc);
		ATH_RX_LOCK_DESTROY(sc);
		ATH_TX_LOCK_DESTROY(sc);
		ATH_LOCK_DESTROY(sc);
		return error;
	}

	// TODO MichalP just for debug purposes can be removed
	ep = usc->sc_udev->endpoints;
	ep_end = usc->sc_udev->endpoints + usc->sc_udev->endpoints_max;
	for (; ep != ep_end; ep++) {
		uint8_t eaddr;

		eaddr = ep->edesc->bEndpointAddress;
		device_printf(sc->sc_dev, "%s: endpoint: addr %u, direction %s, endpoint: %p \n", __func__,
					 UE_GET_ADDR(eaddr), UE_GET_DIR(eaddr) == UE_DIR_OUT ?  "output" : "input", ep);
	}

	error = ath_usb_open_pipes(usc);
	if (error != 0)
		goto bad;

	/*
	 * Setup DMA descriptor area.
	 */
	if (bus_dma_tag_create(bus_get_dma_tag(sc->sc_dev),	/* parent */
			       1, 0,				/* alignment, bounds */
			       BUS_SPACE_MAXADDR_32BIT,		/* lowaddr */
			       BUS_SPACE_MAXADDR,		/* highaddr */
			       NULL, NULL,			/* filter, filterarg */
			       0x3ffff,				/* maxsize XXX */
			       ATH_MAX_SCATTER,			/* nsegments */
			       0x3ffff,				/* maxsegsize XXX */
			       BUS_DMA_ALLOCNOW,		/* flags */
			       NULL,				/* lockfunc */
			       NULL,				/* lockarg */
			       &sc->sc_dmat)) {
		device_printf(sc->sc_dev, "cannot allocate DMA tag\n");
		error = ENXIO;
		goto bad1;
	}

	/* Allocate xfer for firmware commands. */
	error = ath_usb_alloc_tx_cmd(usc);
	if (error != 0)
		goto bad2;

	ath_usb_attachhook(self);
	usc->no_buffer_write = FALSE;

	return 0;
bad2:
	bus_dma_tag_destroy(sc->sc_dmat);
bad1:
	ath_usb_close_pipes(usc);
bad:
	return error;
}

static int
ath_usb_detach(device_t self)
{
	struct ath_softc *sc = device_get_softc(self);
	struct ath_usb_softc *usc = sc->usc;

	if (usc->sc_ath_attached)
		ath_detach(sc);

	/* Wait for all async commands to complete. */
	ath_usb_wait_async(usc);
	
	usbd_transfer_unsetup(usc->sc_xfer, ATH_N_XFER);
	
	/* Abort and close Tx/Rx pipes. */
	ath_usb_close_pipes(usc);

	/* Free Tx/Rx buffers. */
	ath_usb_free_tx_cmd(usc);
	ath_usb_free_tx_list(usc);
	ath_usb_free_rx_list(usc);

	bus_dma_tag_destroy(sc->sc_dmat);
#if ATHN_API
	ath_usb_unload_firmware();
#endif
	ATH_TXSTATUS_LOCK_DESTROY(sc);
	ATH_USB_LOCK_DESTROY(sc);
	ATH_PCU_LOCK_DESTROY(sc);
	ATH_RX_LOCK_DESTROY(sc);
	ATH_TX_LOCK_DESTROY(sc);
	ATH_LOCK_DESTROY(sc);
	free(usc, M_TEMP);
	printf("ath_usb_detach called \n");
	return (0);
}

static int
ath_usb_get_fw_ver(struct ath_softc *sc, struct ar_wmi_fw_version *version)
{
	struct ath_usb_softc *usc = sc->usc;
	struct ar_wmi_fw_version cmd_rsp;
	int error;

	device_printf(sc->sc_dev, "%s: called \n", __func__);

	ATH_USB_LOCK(usc->sc_sc);
	error = ath_usb_wmi_xcmd(usc, AR_WMI_GET_FW_VERSION, NULL, 0, version);
	ATH_USB_UNLOCK(usc->sc_sc);

	version->major = bswap16(version->major);
	version->minor = bswap16(version->minor);

	if (error != 0) {
		device_printf(sc->sc_dev,  "%s: error = %d\n", __func__, error);
		version->major = 0;
		version->minor = 0;
	}

	return error;
}

/* Read fw version from module and compare it with passed image version */
static int
ath_usb_verify_fw_ver(struct ath_softc *sc, struct ar_wmi_fw_version *img_ver)
{
	struct ath_usb_softc *usc = sc->usc;
	struct ar_wmi_fw_version fw_ver;
	int error;

	device_printf(sc->sc_dev, "%s: called \n", __func__);

	error = ath_usb_get_fw_ver(sc, &fw_ver);

	if ((img_ver->major != fw_ver.major) ||
	    (img_ver->minor != fw_ver.minor)) {
		device_printf(sc->sc_dev, "%s: Image fw version %d.%d differs from module fw version %d.%d\n",
			      __func__, img_ver->major, img_ver->minor, fw_ver.major, fw_ver.minor);
		return -1;
	}
	return error;
}

void
ath_usb_attachhook(device_t self)
{
	struct ath_softc *sc = device_get_softc(self);
	struct ath_usb_softc *usc = sc->usc;
	struct ar_wmi_fw_version img_ver;
#if ATHN_API
	struct ath_ops *ops = &sc->ops;
#endif
	struct usb_attach_arg *uaa =  device_get_ivars(self);

	uint32_t val = 104;
#ifdef notyet
	struct ieee80211com *ic = &sc->sc_ic;
	struct ifnet *ifp = &ic->ic_if;
#endif
	int s, i, error;
	/* Load firmware. */
	error = ath_usb_load_firmware(usc, &img_ver);
	if (error != 0) {
		printf("Could not load firmware\n");
		return;
	}
	// TODO MichalP: this can be used as a starting point for echo command or firmware command
//	ATH_USB_LOCK(usc->sc_sc);
//	device_printf(sc->sc_dev, " %s:val = %d\n", __func__, val);
//  NOTE: command below is invalid because ath_usb_read has different arguments but let's keep that note about echo
//	val = ath_usb_read(sc, AR_WMI_CMD_ECHO);
//	device_printf(sc->sc_dev, "%s: returned val = %d\n", __func__, val);
//	val = *(uint32_t*)usc->obuf;
//	device_printf(sc->sc_dev, "%s: casted val = %d\n", __func__, val);
//	ATH_USB_UNLOCK(usc->sc_sc);
//
//	return;

	/* Setup the host transport communication interface. */
	error = ath_usb_htc_setup(usc);
	if (error != 0)
		return;

// TODO: MikolajF:I couldn't read a FW version before the firmware upload because 
// the module never responded to the WMI request, even if I moved the htc setup 
// before loading the FW. I don't know why, but checking it at this point is not
//  useful and adds to the initialization time. It's not critical and can be
// investigated or ignored in the future.
#if 0
	error = ath_usb_verify_fw_ver(sc, &img_ver);
	if (error != 0)
		return;
	extern void ath_get_chipid(struct ath_softc *sc);
	ath_get_chipid(sc);

	uint32_t vals[] = {0xffffffff, 0xDEADBEEF, 0x12345678, 0xDEADB11F, 0xDEADBEEE };
	extern int	ath_reset_power_on(struct ath_softc *);

	ath_reset_power_on(sc);
	for(int i=0; i<5; i++)
	{
		uint32_t val = vals[i];
		uint32_t r = AR_BSSMSKL;

		printf("Writing 0x%x to AR_BSSMSKL\n", val);
		ATH_USB_LOCK(sc);
		AR_WRITE(sc, r, val);
		ATH_USB_UNLOCK(sc);

		ATH_USB_LOCK(sc);
		uint32_t reg = AR_READ(sc, r);
		ATH_USB_UNLOCK(sc);
		printf("AR_BSSMSKL is: %x, expected: %x\n", reg, val);
	}

#endif

	/* We're now ready to attach the bus agnostic driver. */
	// TODO: MichalP needs proper FreeBSD adaptation because this uses code that is
	//  stubbed and/or commented

	error = ath_attach(uaa->info.idVendor, uaa->info.idProduct, sc);
	if (error != 0) {
		return;
	}

	usc->sc_ath_attached = 1;
#if OpenBSD_IEEE80211_API
	/* Override some operations for USB. */
	ifp->if_ioctl = ath_usb_ioctl;
	ifp->if_start = ath_usb_start;
	// TODO no member named 'if_watchdog' in 'struct ifnet'
	// ifp->if_watchdog = ath_usb_watchdog;
	ic->ic_node_alloc = ath_usb_node_alloc;
	ic->ic_newauth = ath_usb_newauth;
	ic->ic_newassoc = ath_usb_newassoc;
#ifndef IEEE80211_STA_ONLY
	usc->sc_node_free = ic->ic_node_free;
	ic->ic_node_free = ath_usb_node_free;
#endif
	ic->ic_updateslot = ath_usb_updateslot;
	ic->ic_updateedca = ath_usb_updateedca;
	ic->ic_set_key = ath_usb_set_key;
	ic->ic_delete_key = ath_usb_delete_key;
	ic->ic_vap_create = ath_vap_create; // This function exist in if_ath.c
	ic->ic_vap_delete = ath_vap_delete;
	ic->ic_ampdu_tx_start = ath_usb_ampdu_tx_start;
	ic->ic_ampdu_tx_stop = ath_usb_ampdu_tx_stop;
	ic->ic_newstate = ath_usb_newstate;

	ic->ic_media.ifm_change_cb = ath_usb_media_change;

	ops->rx_enable = ath_usb_rx_enable;

	/* Reset HW key cache entries. */
	for (i = 0; i < sc->kc_entries; i++)
		ath_reset_key(sc, i);

	ops->enable_antenna_diversity(sc);
#endif

#ifdef ath_BT_COEXISTENCE
	/* Configure bluetooth coexistence for combo chips. */
	if (sc->flags & ath_FLAG_BTCOEX)
		ath_btcoex_init(sc);
#endif
	/* Configure LED. */
#if OpenBSD_ONLY
	ath_led_init(sc);
#endif

}

int
ath_usb_open_pipes(struct ath_usb_softc *usc)
{
	struct ath_softc *sc = usc->sc_sc;
	usb_error_t error;
#if OpenBSD_ONLY
	/* Init host async commands ring. */
	usc->cmdq.cur = usc->cmdq.next = usc->cmdq.queued = 0;
#endif
	/* Allocate Tx/Rx buffers. */
	if((error = ath_usb_alloc_rx_list(usc)) != 0) {
		device_printf(sc->sc_dev, "%s: could not allocate Tx xfers\n",
					  __func__);
		goto fail;
	}
	if ((error = ath_usb_alloc_tx_list(usc)) != 0) {
		device_printf(sc->sc_dev, "%s: could not allocate Tx xfers\n",
					  __func__);
		goto fail;
	}
#if OpenBSD_ONLY
	/* Steal one buffer for beacons. */
	usc->tx_bcn = TAILQ_FIRST(&usc->tx_free_list);
	TAILQ_REMOVE(&usc->tx_free_list, usc->tx_bcn, next);
#endif

	ATH_USB_LOCK(sc);
	usbd_transfer_start(usc->sc_xfer[ath_BULK_RX]);
	usbd_transfer_start(usc->sc_xfer[ath_BULK_IRQ]);
	ATH_USB_UNLOCK(sc);
	return 0;

 fail:
	ath_usb_close_pipes(usc);
	return (error);
}

void
ath_usb_close_pipes(struct ath_usb_softc *usc)
{
	struct ath_softc *sc = usc->sc_sc;

	ATH_USB_LOCK(sc);
	ath_usb_free_rx_list(usc);
	ath_usb_free_tx_list(usc);
	ath_usb_free_tx_cmd(usc);
	ATH_USB_UNLOCK(sc);
}

static int
ath_alloc_list(struct ath_usb_softc *usc, struct ath_usb_data data[],
				int ndata, int maxsz)
{
	struct ath_softc *sc = usc->sc_sc;
	int i, error;

	for (i = 0; i < ndata; i++) {
		struct ath_usb_data *dp = &data[i];
		dp->usc = usc;
		// TODO MichalP might be needed not sure
//		dp->m = NULL;
		dp->buf = malloc(maxsz, M_USBDEV, M_NOWAIT | M_ZERO);
		if (dp->buf == NULL) {
			device_printf(sc->sc_dev,
						  "could not allocate buffer\n");
			error = ENOMEM;
			goto fail;
		}

		// TODO MichalP need IEEE80211 api implemented
		//dp->ni = NULL;
	}

	return (0);
fail:
	ath_free_list(usc, data, ndata);
	return (error);
}

static int
ath_free_list(struct ath_usb_softc *usc, struct ath_usb_data data[], int ndata)
{
	int i;

	for (i = 0; i < ndata; i++) {
		struct ath_usb_data *dp = &data[i];

		if (dp->buf != NULL) {
			free(dp->buf, M_USBDEV);
			dp->buf = NULL;
		}
// TODO MichalP need IEEE80211 api implemented
//		if (dp->ni != NULL) {
//			ieee80211_free_node(dp->ni);
//			dp->ni = NULL;
//		}
	}
	return 0;
}

int
ath_usb_alloc_rx_list(struct ath_usb_softc *usc)
{
	int i, error;

	error = ath_alloc_list(usc, usc->rx_data, ath_USB_RX_LIST_COUNT,
							ath_USB_RXBUFSZ);

	STAILQ_INIT(&usc->sc_rx_active);
	STAILQ_INIT(&usc->sc_rx_inactive);

	for (i = 0; i < ath_USB_RX_LIST_COUNT; i++)
		STAILQ_INSERT_HEAD(&usc->sc_rx_inactive, &usc->rx_data[i], next);

	return (error);
}

void
ath_usb_free_rx_list(struct ath_usb_softc *usc)
{
	int i;

	STAILQ_INIT(&usc->sc_rx_inactive);
	STAILQ_INIT(&usc->sc_rx_active);

	ath_free_list(usc, usc->rx_data, ath_USB_RX_LIST_COUNT);
}

int
ath_usb_alloc_tx_list(struct ath_usb_softc *usc)
{
	int i, error;

	error = ath_alloc_list(usc, usc->tx_data, ath_USB_TX_LIST_COUNT,
							ath_USB_TXBUFSZ);
	if (error != 0)
		return (error);

	STAILQ_INIT(&usc->sc_tx_inactive);

	for (i = 0; i != ATH_N_XFER; i++) {
		STAILQ_INIT(&usc->sc_tx_active[i]);
		STAILQ_INIT(&usc->sc_tx_pending[i]);
	}

	for (i = 0; i < ath_USB_TX_LIST_COUNT; i++) {
		STAILQ_INSERT_HEAD(&usc->sc_tx_inactive, &usc->tx_data[i], next);
	}

	return (0);
}

void
ath_usb_free_tx_list(struct ath_usb_softc *usc)
{
	int i;

	STAILQ_INIT(&usc->sc_tx_inactive);

	for (i = 0; i != ATH_N_XFER; i++) {
		STAILQ_INIT(&usc->sc_tx_active[i]);
		STAILQ_INIT(&usc->sc_tx_pending[i]);
	}

	ath_free_list(usc, usc->tx_data, ath_USB_TX_LIST_COUNT);
}

int
ath_usb_alloc_tx_cmd(struct ath_usb_softc *usc)
{
	struct ath_usb_data *data;
	int i, error = 0;
	struct ath_softc *sc = usc->sc_sc;

	error = ath_alloc_list(usc, usc->tx_cmd, ATH_USB_HOST_CMD_RING_COUNT,
							ATH_USB_TXCMDSZ);
	if (error != 0)
		return (error);

	STAILQ_INIT(&usc->sc_cmd_active);
	STAILQ_INIT(&usc->sc_cmd_inactive);
	STAILQ_INIT(&usc->sc_cmd_pending);
	STAILQ_INIT(&usc->sc_cmd_waiting);

	for (i = 0; i < ATH_USB_HOST_CMD_RING_COUNT; i++)
		STAILQ_INSERT_HEAD(&usc->sc_cmd_inactive, &usc->tx_cmd[i], next);

	return (0);
}

void
ath_usb_free_tx_cmd(struct ath_usb_softc *usc)
{
	STAILQ_INIT(&usc->sc_cmd_active);
	STAILQ_INIT(&usc->sc_cmd_inactive);
	STAILQ_INIT(&usc->sc_cmd_pending);
	STAILQ_INIT(&usc->sc_cmd_waiting);

	ath_free_list(usc, usc->tx_cmd, ATH_USB_HOST_CMD_RING_COUNT);
}

char *state2Str(int state) {
	switch (state) {
	case USB_ST_SETUP:
		return "USB_ST_SETUP";
	case USB_ST_TRANSFERRED:
		return "USB_ST_TRANSFERRED";
	case USB_ST_ERROR:
		return "USB_ST_ERROR";
	default:
		return "state2Str UNKNOWN";
	}
}


char * epType_str(uint8_t bmAttributes) {
	switch(bmAttributes) {
	case UE_CONTROL:
		return "UE_CONTROL";
	case UE_ISOCHRONOUS:
		return "UE_ISOCHRONOUS";
	case UE_BULK:
		return "UE_BULK";
	case UE_INTERRUPT:
		return "UE_INTERRUPT";
	default:
		return "UNKNOWN";
	}
}

static boolean_t 
ath_htc_rx_handle(struct ath_usb_softc *usc, uint8_t *buf, int actlen)
{
	struct ar_htc_msg_hdr *msg;
	uint16_t msg_id;

	// Endpoint 0 carries HTC messages.
	if (actlen < sizeof(*msg)) {
	#ifdef DEVELOPMENT
		// printf("TTTT: htc->flags & AR_HTC_FLAG_TRAILER\n");
	#endif
		return TRUE;
	}

	msg = (struct ar_htc_msg_hdr *)buf;
	msg_id = be16toh(msg->msg_id);
	#ifdef DEVELOPMENT
	// printf("TTTT: Rx HTC msg_id %d, wait_msg_id %d \n", msg_id, usc->wait_msg_id);
	#endif
	switch (msg_id) {
	case AR_HTC_MSG_READY:
		if (usc->wait_msg_id != msg_id)
			break;
		usc->wait_msg_id = 0;
		wakeup(&usc->wait_msg_id);
		break;
	case AR_HTC_MSG_CONN_SVC_RSP:
		if (usc->wait_msg_id != msg_id)
			break;
		if (usc->msg_conn_svc_rsp != NULL) {
			memcpy(usc->msg_conn_svc_rsp, &msg[1],
					sizeof(struct ar_htc_msg_conn_svc_rsp));
		}
		usc->wait_msg_id = 0;
		wakeup(&usc->wait_msg_id);
		break;
	case AR_HTC_MSG_CONF_PIPE_RSP:
		if (usc->wait_msg_id != msg_id)
			break;
		usc->wait_msg_id = 0;
		wakeup(&usc->wait_msg_id);
		break;
	default:
		printf("HTC message %d ignored\n", msg_id);
		break;
	}
	return FALSE;
}

static void
ath_if_intr_rx_callback(struct usb_xfer *xfer, usb_error_t error)
{
	struct ath_usb_softc *usc = usbd_xfer_softc(xfer);
	struct ath_softc *sc = usc->sc_sc;
	struct ar_htc_frame_hdr *htc;
	uint8_t *buf;

	int actlen, sumlen;
	int state = USB_GET_STATE(xfer);
	char *state_str = state2Str(state);

	usbd_xfer_status(xfer, &actlen, &sumlen, NULL, NULL);
	#ifdef DEVELOPMENT
	// printf("TTTT: INTR RX Xfer callback: error = %s, state = %s, actlen = %d, "
	// 	   "sumlen = %d, endpoint = 0x%lx\n",
	// 	   usbd_errstr(error), state_str, actlen, sumlen, (long)xfer->endpoint);
	#endif
	switch(state) {
	case USB_ST_TRANSFERRED:
	{
		#ifdef DEVELOPMENT
		// printf("TTTT: INTR RX Xfer state USB_ST_TRANSFERRED\n");
		#endif
		buf = usbd_xfer_get_frame_buffer(xfer, 0);
		if (actlen >= 4 && *(uint32_t *)buf == htobe32(0x00c60000)) {
			buf += 4;
			actlen -= 4;
		}
		if ((actlen < sizeof(*htc)))
			return;

		htc = (struct ar_htc_frame_hdr *)buf;
		// Skip HTC header.
		buf += sizeof(*htc);
		actlen -= sizeof(*htc);

		if (htc->endpoint_id == 0) {
			if (ath_htc_rx_handle(usc, buf, actlen))
				break;
		} else {
			#ifdef DEVELOPMENT
			// printf("TTTT: htc->endpoint_id != 0\n");
			#endif
			if (htc->endpoint_id != usc->ep_ctrl) {
				#ifdef DEVELOPMENT
				// printf("TTTT: htc->endpoint_id != usc->ep_ctrl\n");
				#endif
				return;
			}

			/// Remove trailer if present.
			if (htc->flags & AR_HTC_FLAG_TRAILER) {
				#ifdef DEVELOPMENT
				// printf("TTTT: htc->flags & AR_HTC_FLAG_TRAILER\n");
				#endif
				if (actlen < htc->control[0]) {
					#ifdef DEVELOPMENT
					// printf("TTTT: actlen < htc->control[0]\n");
					#endif
					return;
				}

				actlen -= htc->control[0];
			}
			ath_usb_rx_wmi_ctrl(usc, buf, actlen);
		}	
	}
	/* FALLTHROUGH */
	case USB_ST_SETUP:
		printf("USB_ST_SETUP called\n");
		usbd_xfer_set_frame_len(xfer, 0, usbd_xfer_max_len(xfer));
		usbd_transfer_submit(xfer);
		break;
	default: /* Error */
		#ifdef DEVELOPMENT
		// printf("TTTT: INTR RX Xfer: error\n");
		#endif
		break;
	}
	return;
}

static void
ath_if_intr_tx_callback(struct usb_xfer *xfer, usb_error_t error)
{
	struct ath_usb_data *cmd;
	struct ath_usb_softc *usc = usbd_xfer_softc(xfer);
	struct ath_softc *sc = usc->sc_sc;

	int actlen;
	int state = USB_GET_STATE(xfer);
	char *state_str = state2Str(state);

	ATH_USB_LOCK_ASSERT(sc);

	usbd_xfer_status(xfer, &actlen, NULL, NULL, NULL);
	#ifdef DEVELOPMENT
	// MichalP: Debuginfo
	// printf("TTTT: INTR TX Xfer callback: error = %s, state = %s, actlen = %d, endpoint = 0x%lx\n",
	// 	   usbd_errstr(error), state_str, actlen, (long)xfer->endpoint);
	#endif

	switch(state) {
	case USB_ST_TRANSFERRED:
		cmd = STAILQ_FIRST(&usc->sc_cmd_active);
		#ifdef DEVELOPMENT
		// device_printf(sc->sc_dev, "%s: continue with USB_ST_TRANSFERRED cmd: %p\n", __func__, cmd);
		#endif
		if (cmd == NULL)
			goto tr_setup;
		#ifdef DEVELOPMENT
		// device_printf(sc->sc_dev, "%s: transfer done cmd: %p\n", __func__, cmd);
		#endif
		STAILQ_REMOVE_HEAD(&usc->sc_cmd_active, next);
		// MichalP TODO: this still needs some thinking maybe separate function
		// different object that the thread sleeps on (cmd?)
		if ((usc->wait_msg_id == AR_HTC_MSG_CONN_SVC_RSP) ||
		    (usc->wait_msg_id == AR_WMI_CMD_MSG)) {
			#ifdef DEVELOPMENT
			// device_printf(sc->sc_dev, "%s: we are waiting for a response cmd: %p\n", __func__, cmd);
			#endif
			STAILQ_INSERT_TAIL(&usc->sc_cmd_waiting, cmd, next);
		} else {
			#ifdef DEVELOPMENT
			// device_printf(sc->sc_dev, "%s: we DONT wait for response cmd: %p\n", __func__, cmd);
			#endif
			wakeup(&usc->wait_msg_id);
			STAILQ_INSERT_TAIL(&usc->sc_cmd_inactive, cmd, next);
		}
		if (usc->wait_msg_id == AR_WMI_CMD_MSG)
			STAILQ_INSERT_TAIL(&usc->sc_cmd_inactive, cmd, next);
		
		/* FALLTHROUGH */
	case USB_ST_SETUP:
tr_setup:
		cmd = STAILQ_FIRST(&usc->sc_cmd_pending);
		if (cmd == NULL) {
			#ifdef DEVELOPMENT
			// device_printf(sc->sc_dev, "%s: empty pending queue cmd: %p\n", __func__, cmd);
			#endif
			return;
		}
		#ifdef DEVELOPMENT
		// device_printf(sc->sc_dev, "%s: continue with USB_ST_SETUP cmd: %p\n", __func__, cmd);
		#endif
		STAILQ_REMOVE_HEAD(&usc->sc_cmd_pending, next);
		STAILQ_INSERT_TAIL(&usc->sc_cmd_active, cmd, next);
		usbd_xfer_set_frame_data(xfer, 0, cmd->buf, cmd->buflen);
		#ifdef DEVELOPMENT
		// device_printf(sc->sc_dev, "%s: submitting transfer %p; buf=%p, buflen=%d\n",
		// 			  __func__, cmd, cmd->buf, cmd->buflen);
		#endif
		usbd_transfer_submit(xfer);
		break;
	default:
		cmd = STAILQ_FIRST(&usc->sc_cmd_active);
		#ifdef DEVELOPMENT
		// device_printf(sc->sc_dev, "%s: continue with default %p\n", __func__, usc);
		#endif
		if (cmd != NULL) {
			device_printf(sc->sc_dev, "%s: cmd not NULL %p\n", __func__, usc);
			STAILQ_REMOVE_HEAD(&usc->sc_cmd_active, next);
//			if (cmd->odata) {
//				STAILQ_INSERT_TAIL(&usc->sc_cmd_waiting, cmd, next_cmd);
//			} else {
//				wakeup(cmd);
//				otus_free_txcmd(sc, cmd);
//			}
		}
		if (error != USB_ERR_CANCELLED) {
			usbd_xfer_set_stall(xfer);
			goto tr_setup;
		}
		break;
	}
}

static void
ath_if_bulk_rx_callback(struct usb_xfer *xfer, usb_error_t error)
{
	struct ath_usb_softc *usc = usbd_xfer_softc(xfer);
	struct ath_softc *sc = usc->sc_sc;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_frame *wh;
	struct ieee80211_node *ni;
	struct ath_usb_data *data;

	int actlen;
	int state = USB_GET_STATE(xfer);
	char *state_str = state2Str(state);

	ATH_USB_LOCK_ASSERT(sc);

	usbd_xfer_status(xfer, &actlen, NULL, NULL, NULL);
#ifdef DEVELOPMENT
	// MichalP: Debuginfo
	// printf("TTTT: BULK RX Xfer callback: error = %s, state = %s, actlen = %d, endpoint = 0x%lx\n",
	// 	   usbd_errstr(error), state_str, actlen, (long)xfer->endpoint);
#endif
	switch(state) {
	case USB_ST_TRANSFERRED:
		device_printf(sc->sc_dev, "%s: USB_ST_TRANSFERRED called \n", __func__);
		data = STAILQ_FIRST(&usc->sc_rx_active);
		if (data == NULL)
			goto tr_setup;
		STAILQ_REMOVE_HEAD(&usc->sc_rx_active, next);
		ath_usb_rxeof(xfer, data);
		STAILQ_INSERT_TAIL(&usc->sc_rx_inactive, data, next);
		/* FALLTHROUGH */
	case USB_ST_SETUP:
		device_printf(sc->sc_dev, "%s: USB_ST_SETUP called \n", __func__);
	tr_setup:
		device_printf(sc->sc_dev, "%s: USB_ST_SETUP after goto called \n", __func__);
		data = STAILQ_FIRST(&usc->sc_rx_inactive);
		if (data == NULL) {
			//KASSERT(m == NULL, ("mbuf isn't NULL"));
			return;
		}
		STAILQ_REMOVE_HEAD(&usc->sc_rx_inactive, next);
		STAILQ_INSERT_TAIL(&usc->sc_rx_active, data, next);
		usbd_xfer_set_frame_data(xfer, 0, data->buf,
								 usbd_xfer_max_len(xfer));
		usbd_transfer_submit(xfer);
		break;
	default:
		device_printf(sc->sc_dev, "%s: default called \n", __func__);
		/* needs it to the inactive queue due to a error. */
		data = STAILQ_FIRST(&usc->sc_rx_active);
		if (data != NULL) {
			STAILQ_REMOVE_HEAD(&usc->sc_rx_active, next);
			STAILQ_INSERT_TAIL(&usc->sc_rx_inactive, data, next);
		}
		if (error != USB_ERR_CANCELLED) {
			usbd_xfer_set_stall(xfer);
			counter_u64_add(ic->ic_ierrors, 1);
			goto tr_setup;
		}
		break;
	}
}

static void
ath_if_bulk_tx_callback(struct usb_xfer *xfer, usb_error_t error)
{
	uint8_t which = ath_BULK_TX;
	struct ath_usb_softc *usc = usbd_xfer_softc(xfer);
	struct ath_softc *sc = usc->sc_sc;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_usb_data *data;

	ATH_USB_LOCK_ASSERT(sc);

	int actlen;
	int state = USB_GET_STATE(xfer);
	char *state_str = state2Str(state);

	usbd_xfer_status(xfer, &actlen, NULL, NULL, NULL);
#ifdef DEVELOPMENT
	// printf("TTTT: BULK TX Xfer callback: error = %s, state = %s, actlen = %d, endpoint = 0x%lx\n",
	// 	   usbd_errstr(error), state_str, actlen, (long)xfer->endpoint);
#endif
	switch(state) {
	case USB_ST_TRANSFERRED:
		data = STAILQ_FIRST(&usc->sc_tx_active[which]);
		if (data == NULL)
			goto tr_setup;
		device_printf(sc->sc_dev, "%s: transfer done %p\n", __func__, data);
		STAILQ_REMOVE_HEAD(&usc->sc_tx_active[which], next);
		ath_usb_txeof(xfer, data);
		STAILQ_INSERT_TAIL(&usc->sc_tx_inactive, data, next);
	case USB_ST_SETUP:
	tr_setup:
		data = STAILQ_FIRST(&usc->sc_tx_pending[which]);
		if (data == NULL) {
			device_printf(sc->sc_dev, "%s: empty pending queue sc %p\n", __func__, sc);
			usc->sc_tx_n_active = 0;
		}
		STAILQ_REMOVE_HEAD(&usc->sc_tx_pending[which], next);
		STAILQ_INSERT_TAIL(&usc->sc_tx_active[which], data, next);
		usbd_xfer_set_frame_data(xfer, 0, data->buf, data->buflen);
		device_printf(sc->sc_dev, "%s: submitting transfer %p\n", __func__, data);
		usbd_transfer_submit(xfer);
		usc->sc_tx_n_active++;
		break;
	default:
		data = STAILQ_FIRST(&usc->sc_tx_active[which]);
		if (data != NULL) {
			STAILQ_REMOVE_HEAD(&usc->sc_tx_active[which], next);
			ath_usb_txeof(xfer, data);
			STAILQ_INSERT_TAIL(&usc->sc_tx_inactive, data, next);
		}
		counter_u64_add(ic->ic_oerrors, 1);

		if (error != USB_ERR_CANCELLED) {
			usbd_xfer_set_stall(xfer);
			goto tr_setup;
		}
		break;
	}
#if ATHN_API
	taskqueue_enqueue(taskqueue_thread, &sc->sc_task);
#endif
}

void
ath_usb_task(void *arg, int pending)
{
#ifdef otus
	struct ath_usb_softc *usc = arg;
	struct ath_usb_host_cmd_ring *ring = &usc->cmdq;
	struct ath_usb_host_cmd *cmd;
	int s;

	// lock

	/* Process host commands. */
	while (ring->next != ring->cur) {
		cmd = &ring->cmd[ring->next];
		/* Invoke callback. */
		cmd->cb(usc, cmd->data);
		ring->queued--;
		ring->next = (ring->next + 1) % ATH_USB_HOST_CMD_RING_COUNT;
	}
#endif
}

void
ath_usb_do_async(struct ath_usb_softc *usc,
    void (*cb)(struct ath_usb_softc *, void *), void *arg, int len)
{
#if OpenBSD_ONLY
	struct ath_usb_host_cmd_ring *ring = &usc->cmdq;
	struct ath_usb_host_cmd *cmd;
	int s;

	if (ring->queued == ATH_USB_HOST_CMD_RING_COUNT) {
		printf("%s: host cmd queue overrun\n", device_get_name(usc->usb_dev));
		return;	/* XXX */
	}

	s = splusb();
	cmd = &ring->cmd[ring->cur];
	cmd->cb = cb;
	KASSERT(len <= sizeof(cmd->data), "ath_usb_do_async");
	memcpy(cmd->data, arg, len);
	ring->cur = (ring->cur + 1) % ATH_USB_HOST_CMD_RING_COUNT;

	/* If there is no pending command already, schedule a task. */
	if (++ring->queued == 1)
		usb_add_task(usc->sc_udev, &usc->sc_task);
	splx(s);
#endif
}

void
ath_usb_wait_async(struct ath_usb_softc *usc)
{
	/* Wait for all queued asynchronous commands to complete. */
#if OpenBSD_ONLY
	usb_wait_task(usc->sc_udev, &usc->sc_task);
#endif
}

void
ath_usb_verify_fw(struct ath_usb_softc *usc)
{
	struct ath_softc *sc = usc->sc_sc;
	struct ath_usb_data *data;

	ATH_USB_LOCK(sc);

	data = STAILQ_FIRST(&usc->sc_cmd_inactive);
	if (data == NULL) {
		device_printf(sc->sc_dev, "%s: no tx cmd buffers\n",
					  __func__);
		return;
	}
	STAILQ_REMOVE_HEAD(&usc->sc_cmd_inactive, next);
}

int
ath_usb_htc_msg(struct ath_usb_softc *usc, uint16_t msg_id, void *buf,
    int len)
{
	struct ath_softc *sc = usc->sc_sc;
	struct ath_usb_data *cmd;
	struct ar_htc_frame_hdr *htc;
	struct ar_htc_msg_hdr *msg;
	int error = 0;

	device_printf(sc->sc_dev, "%s: message id: %d\n",  __func__, msg_id);

	ATH_USB_LOCK_ASSERT(sc);

	cmd = STAILQ_FIRST(&usc->sc_cmd_inactive);
	if (cmd == NULL) {
		device_printf(sc->sc_dev, "%s: no tx cmd buffers\n",
					  __func__);
		return -1;
	}
	STAILQ_REMOVE_HEAD(&usc->sc_cmd_inactive, next);

	htc = (struct ar_htc_frame_hdr *)cmd->buf;
	memset(htc, 0, sizeof(*htc));
	htc->endpoint_id = 0;
	htc->payload_len = htobe16(sizeof(*msg) + len);

	msg = (struct ar_htc_msg_hdr *)&htc[1];
	msg->msg_id = htobe16(msg_id);

	memcpy(&msg[1], buf, len);

	cmd->buflen = sizeof(*htc) + sizeof(*msg) + len;
	device_printf(sc->sc_dev, "%s: prepare transfer %p; buf=%p, buflen=%d\n",
				  __func__, cmd, cmd->buf, cmd->buflen);

	STAILQ_INSERT_TAIL(&usc->sc_cmd_pending, cmd, next);
	usbd_transfer_start(usc->sc_xfer[ath_BULK_CMD]);

	return error;
}

int
ath_usb_htc_setup(struct ath_usb_softc *usc)
{
	struct ar_htc_msg_config_pipe cfg;
	int s, error;

	/*
	 * Connect WMI services to USB pipes.
	 */
	error = ath_usb_htc_connect_svc(usc, AR_SVC_WMI_CONTROL,
	    AR_PIPE_TX_INTR, AR_PIPE_RX_INTR, &usc->ep_ctrl);
	if (error != 0)
		return (error);
	error = ath_usb_htc_connect_svc(usc, AR_SVC_WMI_BEACON,
	    AR_PIPE_TX_DATA, AR_PIPE_RX_DATA, &usc->ep_bcn);
	if (error != 0)
		return (error);
	error = ath_usb_htc_connect_svc(usc, AR_SVC_WMI_CAB,
	    AR_PIPE_TX_DATA, AR_PIPE_RX_DATA, &usc->ep_cab);
	if (error != 0)
		return (error);
	error = ath_usb_htc_connect_svc(usc, AR_SVC_WMI_UAPSD,
	    AR_PIPE_TX_DATA, AR_PIPE_RX_DATA, &usc->ep_uapsd);
	if (error != 0)
		return (error);
	error = ath_usb_htc_connect_svc(usc, AR_SVC_WMI_MGMT,
	    AR_PIPE_TX_DATA, AR_PIPE_RX_DATA, &usc->ep_mgmt);
	if (error != 0)
		return (error);
	error = ath_usb_htc_connect_svc(usc, AR_SVC_WMI_DATA_BE,
	    AR_PIPE_TX_DATA, AR_PIPE_RX_DATA, &usc->ep_data[ATH_QID_AC_BE]);
	if (error != 0)
		return (error);
	error = ath_usb_htc_connect_svc(usc, AR_SVC_WMI_DATA_BK,
	    AR_PIPE_TX_DATA, AR_PIPE_RX_DATA, &usc->ep_data[ATH_QID_AC_BK]);
	if (error != 0)
		return (error);
	error = ath_usb_htc_connect_svc(usc, AR_SVC_WMI_DATA_VI,
	    AR_PIPE_TX_DATA, AR_PIPE_RX_DATA, &usc->ep_data[ATH_QID_AC_VI]);
	if (error != 0)
		return (error);
	error = ath_usb_htc_connect_svc(usc, AR_SVC_WMI_DATA_VO,
	    AR_PIPE_TX_DATA, AR_PIPE_RX_DATA, &usc->ep_data[ATH_QID_AC_VO]);
	if (error != 0)
		return (error);

	/* Set credits for WLAN Tx pipe. */
	memset(&cfg, 0, sizeof(cfg));
	cfg.pipe_id = UE_GET_ADDR(AR_PIPE_TX_DATA);
	cfg.credits = (usc->flags & ATH_USB_FLAG_AR7010) ? 45 : 33;

	ATH_USB_LOCK(usc->sc_sc);

	usc->wait_msg_id = AR_HTC_MSG_CONF_PIPE_RSP;
	error = ath_usb_htc_msg(usc, AR_HTC_MSG_CONF_PIPE, &cfg, sizeof(cfg));
	if (error == 0 && usc->wait_msg_id != 0)
		error = msleep(&usc->wait_msg_id, &usc->sc_sc->sc_usb_mtx, PCATCH, "athhtc",
					   hz);
	usc->wait_msg_id = 0;

	ATH_USB_UNLOCK(usc->sc_sc);

	if (error != 0) {
		printf("%s: could not configure pipe\n",
		    device_get_name(usc->usb_dev));
		return (error);
	}

	ATH_USB_LOCK(usc->sc_sc);

	error = ath_usb_htc_msg(usc, AR_HTC_MSG_SETUP_COMPLETE, NULL, 0);
	if (error != 0) {
		ATH_USB_UNLOCK(usc->sc_sc);
		printf("%s: could not complete setup\n",
		    device_get_name(usc->usb_dev));
		return (error);
	}

	ATH_USB_UNLOCK(usc->sc_sc);

	return (0);
}

int
ath_usb_htc_connect_svc(struct ath_usb_softc *usc, uint16_t svc_id,
    uint8_t ul_pipe, uint8_t dl_pipe, uint8_t *endpoint_id)
{
	struct ar_htc_msg_conn_svc msg;
	struct ar_htc_msg_conn_svc_rsp rsp;
	int s, error;

	struct ath_softc *sc = usc->sc_sc;
	device_printf(sc->sc_dev, "%s: configuring svc_id %d \n", __func__, svc_id);

	memset(&msg, 0, sizeof(msg));
	msg.svc_id = htobe16(svc_id);
	msg.dl_pipeid = UE_GET_ADDR(dl_pipe);
	msg.ul_pipeid = UE_GET_ADDR(ul_pipe);
	usc->msg_conn_svc_rsp = &rsp;

	ATH_USB_LOCK(usc->sc_sc);

	usc->wait_msg_id = AR_HTC_MSG_CONN_SVC_RSP;
	error = ath_usb_htc_msg(usc, AR_HTC_MSG_CONN_SVC, &msg, sizeof(msg));
	/* Wait at most 1 second for response. */
	if (error == 0 && usc->wait_msg_id != 0)
		error = msleep(&usc->wait_msg_id, &usc->sc_sc->sc_usb_mtx, PCATCH, "athhtc", hz * 2);
	usc->wait_msg_id = 0;

	ATH_USB_UNLOCK(usc->sc_sc);

	if (error != 0) {
		device_printf(sc->sc_dev, "%s: error waiting for service %d connection "
								  "error: %d \n", __func__, htobe16(svc_id), error);
		return (error);
	}
	if (rsp.status != AR_HTC_SVC_SUCCESS) {
		device_printf(sc->sc_dev, "%s: service %d connection failed, "
								  "error: %d\n", __func__, htobe16(svc_id), rsp.status);
		return (EIO);
	}

	device_printf(sc->sc_dev, "%s: service %d successfully connected to endpoint %d\n",
				  __func__, svc_id, rsp.endpoint_id);

	/* Return endpoint id. */
	*endpoint_id = rsp.endpoint_id;
	return (0);
}

int
ath_usb_wmi_xcmd(struct ath_usb_softc *usc, uint16_t cmd_id, void *ibuf,
    int ilen, void *obuf)
{
	struct ath_softc *sc = usc->sc_sc;
	struct ath_usb_data *data;
	struct ar_htc_frame_hdr *htc;
	struct ar_wmi_cmd_hdr *wmi;
	int xferlen, error;

	//ATH_USB_LOCK_ASSERT(sc);

	data = STAILQ_FIRST(&usc->sc_cmd_inactive);
	if (data == NULL) {
		device_printf(sc->sc_dev, "%s: no tx cmd buffers\n",
					  __func__);
		return -1;
	}
	STAILQ_REMOVE_HEAD(&usc->sc_cmd_inactive, next);
	ATH_USB_LOCK(sc);
	while (usc->wait_cmd_id) {
		/*
		 * The previous USB transfer is not done yet. We can't use
		 * data->xfer until it is done or we'll cause major confusion
		 * in the USB stack.
		 */
		msleep(&usc->wait_msg_id, &usc->sc_sc->sc_usb_mtx, PCATCH, "athwmx", hz);
	}
	xferlen = sizeof(*htc) + sizeof(*wmi) + ilen;
	data->buflen = xferlen;

	htc = (struct ar_htc_frame_hdr *)data->buf;
	memset(htc, 0, sizeof(*htc));
	htc->endpoint_id = usc->ep_ctrl;
	htc->payload_len = htobe16(sizeof(*wmi) + ilen);

	wmi = (struct ar_wmi_cmd_hdr *)&htc[1];
	wmi->cmd_id = htobe16(cmd_id);
	usc->wmi_seq_no++;
	wmi->seq_no = usc->wmi_seq_no;
	usc->wait_msg_id = AR_WMI_CMD_MSG;

	memcpy(&wmi[1], ibuf, ilen);

	usc->wait_cmd_id = cmd_id;
	usc->obuf = obuf;
	STAILQ_INSERT_TAIL(&usc->sc_cmd_pending, data, next);
	usbd_transfer_start(usc->sc_xfer[ath_BULK_CMD]);

	/*
	 * Wait for WMI command complete interrupt. In case it does not fire
	 * wait until the USB transfer times out to avoid racing the transfer.
	 */
	error = msleep(&usc->wait_cmd_id, &usc->sc_sc->sc_usb_mtx, PCATCH, "athwmi", 2*hz);
	if (error == EWOULDBLOCK) {
		printf("%s: firmware command 0x%x timed out\n",
			device_get_name(usc->usb_dev), cmd_id);
		error = ETIMEDOUT;
	}

	device_printf(sc->sc_dev, "%s: aftersleep\n", __func__);

	/*
	 * Both the WMI command and transfer are done or have timed out.
	 * Allow other threads to enter this function and use data->xfer.
	 */
	usc->obuf = NULL;
	usc->wait_msg_id = 0;
	usc->wait_cmd_id = 0;
	wakeup(&usc->wait_cmd_id);
	ATH_USB_UNLOCK(sc);
	return (error);
}

int
ath_usb_read_rom(struct ath_softc *sc)
{
	struct ath_usb_softc *usc = sc->usc;
	uint32_t addrs[8], vals[8], addr;
	uint16_t *eep;
	int i, j, error;
// no member named 'eep' in 'struct ath_softc'
#if ATHN_API
	/* Read EEPROM by blocks of 16 bytes. */
	eep = sc->eep;
	addr = AR_EEPROM_OFFSET(sc->eep_base);
	for (i = 0; i < sc->eep_size / 16; i++) {
		for (j = 0; j < 8; j++, addr += 4)
			addrs[j] = htobe32(addr);
		error = ath_usb_wmi_xcmd(usc, AR_WMI_CMD_REG_READ,
		    addrs, sizeof(addrs), vals);
		if (error != 0)
			break;
		for (j = 0; j < 8; j++)
			*eep++ = be32toh(vals[j]);
	}
#else
	return 0;
#endif
	return (error);
}

uint32_t
ath_usb_read(struct ath_softc *sc, uint32_t addr)
{
	struct ath_usb_softc *usc = sc->usc;
	uint32_t val;
	int error;

	device_printf(sc->sc_dev, "%s: called \n", __func__);

	/* Flush pending writes for strict consistency. */
	ath_usb_write_barrier(sc);

	addr = htobe32(addr);
	error = ath_usb_wmi_xcmd(usc, AR_WMI_CMD_REG_READ,
	    &addr, sizeof(addr), &val);
	if (error != 0)
		device_printf(sc->sc_dev,
					  "%s: error \n",
					  __func__);
	return be32toh(val);
}

void
ath_usb_write(struct ath_softc *sc, uint32_t addr, uint32_t val)
{
	struct ath_usb_softc *usc = sc->usc;

	usc->wbuf[usc->wcount].addr = htobe32(addr);
	usc->wbuf[usc->wcount].val  = htobe32(val);

	if ((++usc->wcount == AR_MAX_WRITE_COUNT) || (usc->no_buffer_write))
		ath_usb_write_barrier(sc);
}

void
ath_usb_write_barrier(struct ath_softc *sc)
{
	struct ath_usb_softc *usc = sc->usc;

	if (usc->wcount == 0)
		return;	/* Nothing to write. */

	(void)ath_usb_wmi_xcmd(usc, AR_WMI_CMD_REG_WRITE,
	    usc->wbuf, usc->wcount * sizeof(usc->wbuf[0]), NULL);
	usc->wcount = 0;	/* Always flush buffer. */
}

int
ath_usb_media_change(struct ifnet *ifp)
{
	int error;

	error = ieee80211_media_change(ifp);
	if (error != ENETRESET)
		return (error);

	if ((ifp->if_flags & (IFF_UP | IFF_RUNNING)) ==
	    (IFF_UP | IFF_RUNNING)) {
		ath_usb_stop(ifp);
		error = ath_usb_init(ifp);
	}

	return (error);
}

void
ath_usb_next_scan(void *arg, int pending)
{
	struct ath_usb_softc *usc = arg;
	struct ath_softc *sc = usc->sc_sc;
	struct ieee80211com *ic = &sc->sc_ic;
	int s;

#if OpenBSD_IEEE80211_API
	if (ic->ic_state == IEEE80211_S_SCAN)
		ieee80211_next_scan(&ic->ic_if);
#endif
}

// uncomment after investigation if it is needed to be async
// int
// ath_usb_newstate(struct ieee80211vap *vap, enum ieee80211_state nstate,
//     int arg)
// {
// 	struct ieee80211com *ic = vap->iv_ic;
// 	struct ath_usb_softc *usc = ic->ic_softc;
// 	struct ath_vap *uvp = ATH_VAP(vap);
// 	struct ath_usb_cmd_newstate cmd;

// 	/* Do it in a process context. */
// 	cmd.state = nstate;
// 	cmd.arg = arg;
// 	ath_usb_do_async(usc, ath_usb_newstate_cb, &cmd, sizeof(cmd));
// 	return (0);
// }
#if ATHN_API
int
ath_usb_newstate(struct ieee80211vap *vap, enum ieee80211_state nstate,
    int arg)
{
	struct ath_vap *uvp = ATH_VAP(vap);
	struct ieee80211com *ic = vap->iv_ic;
	struct ath_softc *sc = ic->ic_softc;
	struct ath_usb_softc *usc = sc->usc;
	struct ieee80211_node *ni = ieee80211_ref_node(vap->iv_bss);
	enum ieee80211_state ostate;
	uint32_t reg, imask;
	int s, error;
#if OpenBSD_ONLY
	timeout_del(&sc->calib_to);
#endif
	IEEE80211_UNLOCK(ic);
	ATH_USB_LOCK(sc);
	ostate = vap->iv_state;

	if (ostate == IEEE80211_S_RUN && ic->ic_opmode == IEEE80211_M_STA) {
		ath_usb_remove_node(usc, ni);
		reg = AR_READ(sc, AR_RX_FILTER);
		reg = (reg & ~AR_RX_FILTER_MYBEACON) |
		    AR_RX_FILTER_BEACON;
		AR_WRITE(sc, AR_RX_FILTER, reg);
		AR_WRITE_BARRIER(sc);
	}

	switch (nstate) {
	case IEEE80211_S_INIT:
		ath_set_led(sc, 0);
		break;
	case IEEE80211_S_SCAN:
		/* Make the LED blink while scanning. */
		ath_set_led(sc, !sc->led_state);
		error = ath_usb_switch_chan(sc, ni->ni_chan, NULL);
		if (error)
			printf("%s: could not switch to channel %d\n",
			    device_get_name(usc->usb_dev), 0);
			    ieee80211_chan2ieee(ic, ni->ni_chan);
		// uncomment after investigation if it is needed to be async
		// if (!usbd_is_dying(usc->sc_udev))
		// 	timeout_add_msec(&sc->scan_to, 200);
		break;
	case IEEE80211_S_AUTH:
		ath_set_led(sc, 0);
		error = ath_usb_switch_chan(sc, ni->ni_chan, NULL);
		if (error)
			printf("%s: could not switch to channel %d\n",
			    device_get_name(usc->usb_dev), 0);
			    ieee80211_chan2ieee(ic, ni->ni_chan);
		break;
	case IEEE80211_S_ASSOC:
		break;
	case IEEE80211_S_RUN:
		ath_set_led(sc, 1);

		if (ic->ic_opmode == IEEE80211_M_MONITOR)
			break;

		if (ic->ic_opmode == IEEE80211_M_STA) {
			/* Create node entry for our BSS */
			error = ath_usb_create_node(usc, ni);
			if (error)
				printf("%s: could not update firmware station "
				    "table\n", device_get_name(usc->usb_dev));
		}
		ath_set_bss(sc, ni);
		ath_usb_wmi_cmd(usc, AR_WMI_CMD_DISABLE_INTR);
#ifndef IEEE80211_STA_ONLY
		if (ic->ic_opmode == IEEE80211_M_HOSTAP) {
			ath_usb_switch_chan(sc, ni->ni_chan, NULL);
			ath_set_hostap_timers(sc);
			/* Enable software beacon alert interrupts. */
			imask = htobe32(HAL_INT_SWBA);
		} else
#endif
		{
			ath_set_sta_timers(sc);
			/* Enable beacon miss interrupts. */
			imask = htobe32(HAL_INT_BMISS);

			/* Stop receiving beacons from other BSS. */
			reg = AR_READ(sc, AR_RX_FILTER);
			reg = (reg & ~AR_RX_FILTER_BEACON) |
			    AR_RX_FILTER_MYBEACON;
			AR_WRITE(sc, AR_RX_FILTER, reg);
			AR_WRITE_BARRIER(sc);
		}
		ath_usb_wmi_xcmd(usc, AR_WMI_CMD_ENABLE_INTR,
		    &imask, sizeof(imask), NULL);
		break;
	default:
		break;
	}
#if OpenBSD_IEEE80211_API
	return sc->sc_newstate(ic, state, arg);
	splx(s);
	#endif
	ATH_USB_UNLOCK(sc);
	IEEE80211_LOCK(ic);
	return (uvp->newstate(vap, nstate, arg));
}
#endif

void
ath_usb_newassoc(struct ieee80211com *ic, struct ieee80211_node *ni,
    int isnew)
{
#ifndef IEEE80211_STA_ONLY
	struct ath_softc *sc = ic->ic_softc;
	struct ath_usb_softc *usc = sc->usc;
#if OpenBSD_IEEE80211_API
	if (ic->ic_opmode != IEEE80211_M_HOSTAP &&
	    ic->ic_state != IEEE80211_S_RUN)
		return;
#endif
	/* Update the node's supported rates in a process context. */
	ieee80211_ref_node(ni);
	ath_usb_do_async(usc, ath_usb_newassoc_cb, &ni, sizeof(ni));
#endif
}

#ifndef IEEE80211_STA_ONLY
void
ath_usb_newassoc_cb(struct ath_usb_softc *usc, void *arg)
{
	struct ieee80211com *ic = &usc->sc_sc->sc_ic;
	struct ieee80211_node *ni = *(void **)arg;
	struct ath_node *an = (struct ath_node *)ni;
	int s;
#if OpenBSD_IEEE80211_API
	if (ic->ic_state != IEEE80211_S_RUN)
		return;

	s = splnet();
	/* NB: Node may have left before we got scheduled. */
	if (an->sta_index != 0)
		(void)ath_usb_node_set_rates(usc, ni);
	ieee80211_release_node(ic, ni);
	splx(s);
#endif
}
#endif

struct ieee80211_node *
ath_usb_node_alloc(struct ieee80211com *ic)
{
	struct ath_node *an;

	an = malloc(sizeof(struct ath_node), M_USBDEV, M_NOWAIT | M_ZERO);
	return (struct ieee80211_node *)an;
}


#ifndef IEEE80211_STA_ONLY
void
ath_usb_count_active_sta(void *arg, struct ieee80211_node *ni)
{
	int *nsta = arg;
	struct ath_node *an = (struct ath_node *)ni;
#if ATHN_API
	if (an->sta_index == 0)
		return;
#endif
#if OpenBSD_IEEE80211_API
	if ((ni->ni_state == IEEE80211_STA_AUTH ||
	    ni->ni_state == IEEE80211_STA_ASSOC) &&
	    ni->ni_inact < IEEE80211_INACT_MAX)
		(*nsta)++;
#endif
}

struct ath_usb_newauth_cb_arg {
	struct ieee80211_node *ni;
	uint16_t seq;
};

void
ath_usb_newauth_cb(struct ath_usb_softc *usc, void *arg)
{
	struct ieee80211com *ic = &usc->sc_sc->sc_ic;
	struct ath_usb_newauth_cb_arg *a = arg;
	struct ieee80211_node *ni = a->ni;
	uint16_t seq = a->seq;
	struct ath_node *an = (struct ath_node *)ni;
	int s, error = 0;

#if OpenBSD_IEEE80211_API
	if (ic->ic_state != IEEE80211_S_RUN)
		return;
#endif

	s = splnet();
#if ATHN_API
	if (an->sta_index == 0) {
		error = ath_usb_create_node(usc, ni);
		if (error)
			printf("%s: could not add station %s to firmware "
			    "table\n", device_get_name(usc->usb_dev),
			    ether_sprintf(ni->ni_macaddr));
	}
#endif
#if OpenBSD_IEEE80211_API
	if (error == 0)
		ieee80211_auth_open_confirm(ic, ni, seq);
	ieee80211_unref_node(&ni);
#endif
	splx(s);
}
#endif

int
ath_usb_newauth(struct ieee80211com *ic, struct ieee80211_node *ni,
    int isnew, uint16_t seq)
{
#ifndef IEEE80211_STA_ONLY
	struct ath_softc *sc = ic->ic_softc;
	struct ath_usb_softc *usc = sc->usc;
#if OpenBSD_IEEE80211_API
	struct ifnet *ifp = &ic->ic_if;
#endif
	struct ath_node *an = (struct ath_node *)ni;
	int nsta;
	struct ath_usb_newauth_cb_arg arg;

	if (ic->ic_opmode != IEEE80211_M_HOSTAP)
		return 0;
#if ATHN_API
	if (!isnew && an->sta_index != 0) /* already in firmware table */
		return 0;
#endif
	/* Check if we have room in the firmware table. */
	nsta = 1; /* Account for default node. */
#if OpenBSD_IEEE80211_API
	ieee80211_iterate_nodes(ic, ath_usb_count_active_sta, &nsta);
	if (nsta >= AR_USB_MAX_STA) {
		if (ifp->if_flags & IFF_DEBUG)
			printf("%s: cannot authenticate station %s: firmware "
			    "table is full\n", device_get_name(usc->usb_dev),
			    ether_sprintf(ni->ni_macaddr));
		return ENOSPC;
	}
#endif

	/*
	 * In a process context, try to add this node to the
	 * firmware table and confirm the AUTH request.
	 */
	arg.ni = ieee80211_ref_node(ni);
	arg.seq = seq;
	ath_usb_do_async(usc, ath_usb_newauth_cb, &arg, sizeof(arg));
	return EBUSY;
#else
	return 0;
#endif /* IEEE80211_STA_ONLY */
}

#ifndef IEEE80211_STA_ONLY
void
ath_usb_node_free(struct ieee80211com *ic, struct ieee80211_node *ni)
{
	struct ath_softc *sc = ic->ic_softc;
	struct ath_usb_softc *usc = sc->usc;
	struct ath_node *an = (struct ath_node *)ni;

	/*
	 * Remove the node from the firmware table in a process context.
	 * Pass an index rather than the pointer which we will free.
	 */
#if ATHN_API
	if (an->sta_index != 0)
		ath_usb_do_async(usc, ath_usb_node_free_cb,
		    &an->sta_index, sizeof(an->sta_index));
#endif
	usc->sc_node_free(ic, ni);
}

void
ath_usb_node_free_cb(struct ath_usb_softc *usc, void *arg)
{
	struct ieee80211com *ic = &usc->sc_sc->sc_ic;
#if OpenBSD_IEEE80211_API
	struct ifnet *ifp = &ic->ic_if;
#endif
	uint8_t sta_index = *(uint8_t *)arg;
	int error;

	error = ath_usb_wmi_xcmd(usc, AR_WMI_CMD_NODE_REMOVE,
	    &sta_index, sizeof(sta_index), NULL);
	if (error) {
		printf("%s: could not remove station %u from firmware table\n",
		    device_get_name(usc->usb_dev), sta_index);
		return;
	}
	usc->free_node_slots |= (1 << sta_index);
#if OpenBSD_IEEE80211_API
	if (ifp->if_flags & IFF_DEBUG)
		printf("%s: station %u removed from firmware table\n",
		    device_get_name(usc->usb_dev), sta_index);
#endif
}
#endif /* IEEE80211_STA_ONLY */

int
ath_usb_ampdu_tx_start(struct ieee80211com *ic, struct ieee80211_node *ni,
    uint8_t tid)
{
	struct ath_softc *sc = ic->ic_softc;
	struct ath_usb_softc *usc = sc->usc;
	struct ath_node *an = (struct ath_node *)ni;
	struct ath_usb_aggr_cmd cmd;
#if ATHN_API
	/* Do it in a process context. */
	cmd.sta_index = an->sta_index;
#endif
	cmd.tid = tid;
	ath_usb_do_async(usc, ath_usb_ampdu_tx_start_cb, &cmd, sizeof(cmd));
	return (0);
}

void
ath_usb_ampdu_tx_start_cb(struct ath_usb_softc *usc, void *arg)
{
	struct ath_usb_aggr_cmd *cmd = arg;
	struct ar_htc_target_aggr aggr;

	memset(&aggr, 0, sizeof(aggr));
	aggr.sta_index = cmd->sta_index;
	aggr.tidno = cmd->tid;
	aggr.aggr_enable = 1;
	(void)ath_usb_wmi_xcmd(usc, AR_WMI_CMD_TX_AGGR_ENABLE,
	    &aggr, sizeof(aggr), NULL);
}

void
ath_usb_ampdu_tx_stop(struct ieee80211com *ic, struct ieee80211_node *ni,
    uint8_t tid)
{
	struct ath_softc *sc = ic->ic_softc;
	struct ath_usb_softc *usc = sc->usc;
	struct ath_node *an = (struct ath_node *)ni;
	struct ath_usb_aggr_cmd cmd;

	/* Do it in a process context. */
#if ATHN_API
	cmd.sta_index = an->sta_index;
#endif
	cmd.tid = tid;
	ath_usb_do_async(usc, ath_usb_ampdu_tx_stop_cb, &cmd, sizeof(cmd));
}

void
ath_usb_ampdu_tx_stop_cb(struct ath_usb_softc *usc, void *arg)
{
	struct ath_usb_aggr_cmd *cmd = arg;
	struct ar_htc_target_aggr aggr;

	memset(&aggr, 0, sizeof(aggr));
	aggr.sta_index = cmd->sta_index;
	aggr.tidno = cmd->tid;
	aggr.aggr_enable = 0;
	(void)ath_usb_wmi_xcmd(usc, AR_WMI_CMD_TX_AGGR_ENABLE,
	    &aggr, sizeof(aggr), NULL);
}

#ifndef IEEE80211_STA_ONLY
/* Try to find a node we can evict to make room in the firmware table. */
void
ath_usb_clean_nodes(void *arg, struct ieee80211_node *ni)
{
	struct ath_usb_softc *usc = arg;
	struct ieee80211com *ic = &usc->sc_sc->sc_ic;
	struct ath_node *an = (struct ath_node *)ni;

	/*
	 * Don't remove the default node (used for management frames).
	 * Nodes which are not in the firmware table also have index zero.
	 */
#if ATHN_API
	if (an->sta_index == 0)
		return;
#endif
#if OpenBSD_IEEE80211_API
	/* Remove non-associated nodes. */
	if (ni->ni_state != IEEE80211_STA_AUTH &&
	    ni->ni_state != IEEE80211_STA_ASSOC) {
		ath_usb_remove_node(usc, ni);
		return;
	}

	/*
	 * Kick off inactive associated nodes. This won't help
	 * immediately but will help if the new STA retries later.
	 */
	if (ni->ni_inact >= IEEE80211_INACT_MAX) {
		IEEE80211_SEND_MGMT(ic, ni, IEEE80211_FC0_SUBTYPE_DEAUTH,
		    IEEE80211_REASON_AUTH_EXPIRE);
		ieee80211_node_leave(ic, ni);
	}
#endif
}
#endif

int
ath_usb_create_node(struct ath_usb_softc *usc, struct ieee80211_node *ni)
{
	struct ath_node *an = (struct ath_node *)ni;
	struct ar_htc_target_sta sta;
	int error, sta_index;
#ifndef IEEE80211_STA_ONLY
	struct ieee80211com *ic = &usc->sc_sc->sc_ic;
#if OpenBSD_IEEE80211_API
	struct ifnet *ifp = &ic->ic_if;
#endif

	/* Firmware cannot handle more than 8 STAs. Try to make room first. */
#if OpenBSD_IEEE80211_API
	if (ic->ic_opmode == IEEE80211_M_HOSTAP)
		ieee80211_iterate_nodes(ic, ath_usb_clean_nodes, usc);
#endif
#endif
	if (usc->free_node_slots == 0x00)
		return ENOBUFS;

	sta_index = ffs(usc->free_node_slots) - 1;
	if (sta_index < 0 || sta_index >= AR_USB_MAX_STA)
		return ENOSPC;

	/* Create node entry on target. */
	memset(&sta, 0, sizeof(sta));
	IEEE80211_ADDR_COPY(sta.macaddr, ni->ni_macaddr);
	IEEE80211_ADDR_COPY(sta.bssid, ni->ni_bssid);
	sta.sta_index = sta_index;
	sta.maxampdu = 0xffff;
	if (ni->ni_flags & IEEE80211_NODE_HT)
		sta.flags |= htobe16(AR_HTC_STA_HT);
	error = ath_usb_wmi_xcmd(usc, AR_WMI_CMD_NODE_CREATE,
	    &sta, sizeof(sta), NULL);
	if (error != 0)
		return (error);
#if ATHN_API
	an->sta_index = sta_index;
	usc->free_node_slots &= ~(1 << an->sta_index);
#endif

#if OpenBSD_IEEE80211_API
#ifndef IEEE80211_STA_ONLY
	if (ifp->if_flags & IFF_DEBUG)
		printf("%s: station %u (%s) added to firmware table\n",
		    device_get_name(usc->usb_dev), sta_index,
		    ether_sprintf(ni->ni_macaddr));
#endif
#endif
	return ath_usb_node_set_rates(usc, ni);
}

int
ath_usb_node_set_rates(struct ath_usb_softc *usc, struct ieee80211_node *ni)
{
	struct ath_node *an = (struct ath_node *)ni;
	struct ar_htc_target_rate rate;
	int i, j;

	/* Setup supported rates. */
	memset(&rate, 0, sizeof(rate));
#if ATHN_API
	rate.sta_index = an->sta_index;
#endif
	rate.isnew = 1;
	rate.lg_rates.rs_nrates = ni->ni_rates.rs_nrates;
	memcpy(rate.lg_rates.rs_rates, ni->ni_rates.rs_rates,
	    ni->ni_rates.rs_nrates);
	if (ni->ni_flags & IEEE80211_NODE_HT) {
		rate.capflags |= htobe32(AR_RC_HT_FLAG);
		/* Setup HT rates. */
	#if OpenBSD_IEEE80211_API
		for (i = 0, j = 0; i < IEEE80211_HT_NUM_MCS; i++) {
			if (!isset(ni->ni_rxmcs, i))
				continue;
			if (j >= AR_HTC_RATE_MAX)
				break;
			rate.ht_rates.rs_rates[j++] = i;
		}
	#endif
		rate.ht_rates.rs_nrates = j;

#if OpenBSD_IEEE80211_API
		if (ni->ni_rxmcs[1]) /* dual-stream MIMO rates */
			rate.capflags |= htobe32(AR_RC_DS_FLAG);
#endif
#ifdef notyet
		if (ni->ni_htcaps & IEEE80211_HTCAP_CBW20_40)
			rate.capflags |= htobe32(AR_RC_40_FLAG);
		if (ni->ni_htcaps & IEEE80211_HTCAP_SGI40)
			rate.capflags |= htobe32(AR_RC_SGI_FLAG);
		if (ni->ni_htcaps & IEEE80211_HTCAP_SGI20)
			rate.capflags |= htobe32(AR_RC_SGI_FLAG);
#endif
	}

	return ath_usb_wmi_xcmd(usc, AR_WMI_CMD_RC_RATE_UPDATE,
	    &rate, sizeof(rate), NULL);
}

int
ath_usb_remove_node(struct ath_usb_softc *usc, struct ieee80211_node *ni)
{
	struct ath_node *an = (struct ath_node *)ni;
	int error;
#ifndef IEEE80211_STA_ONLY
	struct ieee80211com *ic = &usc->sc_sc->sc_ic;
#if OpenBSD_IEEE80211_API
	struct ifnet *ifp = &ic->ic_if;
#endif
#endif
#if ATHN_API
	error = ath_usb_wmi_xcmd(usc, AR_WMI_CMD_NODE_REMOVE,
	    &an->sta_index, sizeof(an->sta_index), NULL);
	if (error) {
		printf("%s: could not remove station %u (%s) from "
		    "firmware table\n", device_get_name(usc->usb_dev), an->sta_index,
		    ether_sprintf(ni->ni_macaddr));
		return error;
	}
#endif
#if OpenBSD_IEEE80211_API
#ifndef IEEE80211_STA_ONLY
	if (ifp->if_flags & IFF_DEBUG)
		printf("%s: station %u (%s) removed from firmware table\n",
		    device_get_name(usc->usb_dev), an->sta_index,
		    ether_sprintf(ni->ni_macaddr));
#endif
#endif
#if ATHN_API
	usc->free_node_slots |= (1 << an->sta_index);
	an->sta_index = 0;
#endif
	return 0;
}

#if ATHN_API
void
ath_usb_rx_enable(struct ath_softc *sc)
{
	AR_WRITE(sc, AR_CR, AR_CR_RXE);
	AR_WRITE_BARRIER(sc);
}

int
ath_usb_switch_chan(struct ath_softc *sc, struct ieee80211_channel *c,
    struct ieee80211_channel *extc)
{
	struct ath_usb_softc *usc = sc->usc;
	uint16_t mode;
	int error;

	/* Disable interrupts. */
	error = ath_usb_wmi_cmd(usc, AR_WMI_CMD_DISABLE_INTR);
	if (error != 0)
		goto reset;
	/* Stop all Tx queues. */
	error = ath_usb_wmi_cmd(usc, AR_WMI_CMD_DRAIN_TXQ_ALL);
	if (error != 0)
		goto reset;
	/* Stop Rx. */
	error = ath_usb_wmi_cmd(usc, AR_WMI_CMD_STOP_RECV);
	if (error != 0)
		goto reset;

	/* If band or bandwidth changes, we need to do a full reset. */
	if (c->ic_flags != sc->curchan->ic_flags ||
	    ((extc != NULL) ^ (sc->curchanext != NULL))) {
		DPRINTFN(2, ("channel band switch\n"));
		goto reset;
	}

	error = ath_set_chan(sc, c, extc);
	if (AR_SREV_9271(sc) && error == 0)
		ar9271_load_ani(sc);
	if (error != 0) {
 reset:		/* Error found, try a full reset. */
		DPRINTFN(3, ("needs a full reset\n"));
		error = ath_hw_reset(sc, c, extc, 0);
		if (error != 0)	/* Hopeless case. */
			return (error);

		error = ath_set_chan(sc, c, extc);
		if (AR_SREV_9271(sc) && error == 0)
			ar9271_load_ani(sc);
		if (error != 0)
			return (error);
	}

	sc->ops.set_txpower(sc, c, extc);

	error = ath_usb_wmi_cmd(usc, AR_WMI_CMD_START_RECV);
	if (error != 0)
		return (error);
	ath_rx_start(sc);

	mode = htobe16(IEEE80211_IS_CHAN_2GHZ(c) ?
	    AR_HTC_MODE_11NG : AR_HTC_MODE_11NA);
	error = ath_usb_wmi_xcmd(usc, AR_WMI_CMD_SET_MODE,
	    &mode, sizeof(mode), NULL);
	if (error != 0)
		return (error);

	/* Re-enable interrupts. */
	error = ath_usb_wmi_cmd(usc, AR_WMI_CMD_ENABLE_INTR);
	return (error);
}
#endif

void
ath_usb_updateedca(struct ieee80211com *ic)
{
	struct ath_softc *sc = ic->ic_softc;
	struct ath_usb_softc *usc = sc->usc;

	/* Do it in a process context. */
	ath_usb_do_async(usc, ath_usb_updateedca_cb, NULL, 0);
}

void
ath_usb_updateedca_cb(struct ath_usb_softc *usc, void *arg)
{
	int s;

	s = splnet();
#if ATHN_API	
	ath_updateedca(&usc->sc_sc->sc_ic);
#endif
	splx(s);
}

void
ath_usb_updateslot(struct ieee80211com *ic)
{
	struct ath_softc *sc = ic->ic_softc;
	struct ath_usb_softc *usc = sc->usc;

	return;	/* XXX */
	/* Do it in a process context. */
	ath_usb_do_async(usc, ath_usb_updateslot_cb, NULL, 0);
}

void
ath_usb_updateslot_cb(struct ath_usb_softc *usc, void *arg)
{
	int s;

	s = splnet();
#if ATHN_API	
	ath_updateslot(&usc->sc_sc->sc_ic);
#endif
	splx(s);
}

int
ath_usb_set_key(struct ieee80211com *ic, struct ieee80211_node *ni,
    struct ieee80211_key *k)
{
	struct ath_softc *sc = ic->ic_softc;
	struct ath_usb_softc *usc = sc->usc;
	struct ath_usb_cmd_key cmd;

	/* Defer setting of WEP keys until interface is brought up. */
#if OpenBSD_IEEE80211_API
	if ((ic->ic_if.if_flags & (IFF_UP | IFF_RUNNING)) !=
	    (IFF_UP | IFF_RUNNING))
		return (0);
#endif

	/* Do it in a process context. */
	cmd.ni = (ni != NULL) ? ieee80211_ref_node(ni) : NULL;
	cmd.key = k;
	ath_usb_do_async(usc, ath_usb_set_key_cb, &cmd, sizeof(cmd));
	usc->sc_key_tasks++;
	return EBUSY;
}

void
ath_usb_set_key_cb(struct ath_usb_softc *usc, void *arg)
{
	struct ieee80211com *ic = &usc->sc_sc->sc_ic;
	struct ath_usb_cmd_key *cmd = arg;
	int s;

	usc->sc_key_tasks--;

	s = splnet();
	ath_usb_write_barrier(usc->sc_sc);
#if ATHN_API	
	ath_set_key(ic, cmd->ni, cmd->key);
#endif
#if OpenBSD_IEEE80211_API
	if (usc->sc_key_tasks == 0) {
		DPRINTF(("marking port %s valid\n",
		    ether_sprintf(cmd->ni->ni_macaddr)));
		cmd->ni->ni_port_valid = 1;
		ieee80211_set_link_state(ic, LINK_STATE_UP);
	}
	if (cmd->ni != NULL)
		ieee80211_release_node(ic, cmd->ni);
	splx(s);
#endif
}

void
ath_usb_delete_key(struct ieee80211com *ic, struct ieee80211_node *ni,
    struct ieee80211_key *k)
{
	struct ath_softc *sc = ic->ic_softc;
	struct ath_usb_softc *usc = sc->usc;
	struct ath_usb_cmd_key cmd;
#if OpenBSD_IEEE80211_API
	if (!(ic->ic_if.if_flags & IFF_RUNNING) ||
	    ic->ic_state != IEEE80211_S_RUN)
		return;	/* Nothing to do. */
#endif
	/* Do it in a process context. */
	cmd.ni = (ni != NULL) ? ieee80211_ref_node(ni) : NULL;
	cmd.key = k;
	ath_usb_do_async(usc, ath_usb_delete_key_cb, &cmd, sizeof(cmd));
}

void
ath_usb_delete_key_cb(struct ath_usb_softc *usc, void *arg)
{
	struct ieee80211com *ic = &usc->sc_sc->sc_ic;
	struct ath_usb_cmd_key *cmd = arg;
	int s;

	s = splnet();
#if ATHN_API	
	ath_delete_key(ic, cmd->ni, cmd->key);
#endif
#if OpenBSD_IEEE80211_API
	if (cmd->ni != NULL)
		ieee80211_release_node(ic, cmd->ni);
	splx(s);
#endif
}

#ifndef IEEE80211_STA_ONLY
void
ath_usb_bcneof(struct usb_xfer *xfer, void *priv,
    usb_error_t status)
{
	struct ath_usb_data *data = priv;
#if OpenBSD_ONLY
	struct ath_usb_softc *usc = data->sc->usc;

	if (__predict_false(status == USB_ERR_STALLED))
		usbd_clear_endpoint_stall_async(usc->tx_data_pipe);
	usc->tx_bcn = data;
#endif
}

/*
 * Process Software Beacon Alert interrupts.
 */
void
ath_usb_swba(struct ath_usb_softc *usc)
{
	struct ath_softc *sc = usc->sc_sc;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ath_usb_data *data;
	struct ieee80211_frame *wh;
	struct ar_stream_hdr *hdr;
	struct ar_htc_frame_hdr *htc;
	struct ar_tx_bcn *bcn;
	struct mbuf *m;
	int error;
#if OpenBSD_IEEE80211_API
	if (ic->ic_dtim_count == 0)
		ic->ic_dtim_count = ic->ic_dtim_period - 1;
	else
		ic->ic_dtim_count--;
#endif
#if OpenBSD_ONLY
	/* Make sure previous beacon has been sent. */
	if (usc->tx_bcn == NULL)
		return;
	data = usc->tx_bcn;
#endif
#if OpenBSD_IEEE80211_API
	/* Get new beacon. */
	m = ieee80211_beacon_alloc(ic, ic->ic_bss);
	if (__predict_false(m == NULL))
		return;
	/* Assign sequence number. */
	wh = mtod(m, struct ieee80211_frame *);
	*(uint16_t *)&wh->i_seq[0] =
	    htole16(ic->ic_bss->ni_txseq << IEEE80211_SEQ_SEQ_SHIFT);
	ic->ic_bss->ni_txseq++;

	hdr = (struct ar_stream_hdr *)data->buf;
	hdr->tag = htole16(AR_USB_TX_STREAM_TAG);
	hdr->len = htole16(sizeof(*htc) + sizeof(*bcn) + m->m_pkthdr.len);

	htc = (struct ar_htc_frame_hdr *)&hdr[1];
	memset(htc, 0, sizeof(*htc));
	htc->endpoint_id = usc->ep_bcn;
	htc->payload_len = htobe16(sizeof(*bcn) + m->m_pkthdr.len);

	bcn = (struct ar_tx_bcn *)&htc[1];
	memset(bcn, 0, sizeof(*bcn));
	bcn->vif_idx = 0;

	m_copydata(m, 0, m->m_pkthdr.len, &bcn[1]);
#endif
#if OpenBSD_ONLY
	usbd_setup_xfer(data->xfer, usc->tx_data_pipe, data, data->buf,
	    sizeof(*hdr) + sizeof(*htc) + sizeof(*bcn) + m->m_pkthdr.len,
	    USBD_SHORT_XFER_OK | USBD_NO_COPY, ath_USB_TX_TIMEOUT,
	    ath_usb_bcneof);

	m_freem(m);
	usc->tx_bcn = NULL;
	error = usbd_transfer_start(data->xfer);
	if (__predict_false(error != USBD_IN_PROGRESS && error != 0))
		usc->tx_bcn = data;
#endif
}
#endif

/* Update current transmit rate for a node based on firmware Tx status. */
void
ath_usb_tx_status(void *arg, struct ieee80211_node *ni)
{
	struct ar_wmi_evt_txstatus *ts = arg;
	struct ath_node *an = (struct ath_node *)ni;
	uint8_t rate_index = (ts->rate & AR_HTC_TXSTAT_RATE);
#if ATHN_API
	if (an->sta_index != ts->cookie) /* Tx report for a different node */
		return;
#endif
#if OpenBSD_IEEE80211_API
	if (ts->flags & AR_HTC_TXSTAT_MCS) {
		if (isset(ni->ni_rxmcs, rate_index))
			ni->ni_txmcs = rate_index;
	} else if (rate_index < ni->ni_rates.rs_nrates)
		ni->ni_txrate = rate_index;
#endif
}

void
ath_usb_rx_wmi_ctrl(struct ath_usb_softc *usc, uint8_t *buf, int len)
{
	struct ar_wmi_cmd_hdr *wmi;
	uint16_t cmd_id;

	if (__predict_false(len < sizeof(*wmi)))
		return;
	wmi = (struct ar_wmi_cmd_hdr *)buf;
	cmd_id = be16toh(wmi->cmd_id);

	if (!(cmd_id & AR_WMI_EVT_FLAG)) {
		if (usc->wait_cmd_id != cmd_id)
			return;	/* Unexpected reply. */
		if (usc->obuf != NULL) {
			/* Copy answer into caller supplied buffer. */
			memcpy(usc->obuf, &wmi[1], len - sizeof(*wmi));
		}
		/* Notify caller of completion. */
		wakeup(&usc->wait_cmd_id);
		return;
	}
	switch (cmd_id & 0xfff) {
#ifndef IEEE80211_STA_ONLY
	case AR_WMI_EVT_SWBA:
		ath_usb_swba(usc);
		break;
#endif
	case AR_WMI_EVT_TXSTATUS: {
		struct ar_wmi_evt_txstatus_list *tsl;
		int i;

		tsl = (struct ar_wmi_evt_txstatus_list *)&wmi[1];
		for (i = 0; i < tsl->count && i < nitems(tsl->ts); i++) {
			struct ieee80211com *ic = &usc->sc_sc->sc_ic;
#if OpenBSD_IEEE80211_API
			struct ath_node *an = (struct ath_node *)ic->ic_bss;
#endif
			struct ar_wmi_evt_txstatus *ts = &tsl->ts[i];
			uint8_t qid;

			/* Skip the node we use to send management frames. */
			if (ts->cookie == 0)
				continue;

			/* Skip Tx reports for non-data frame endpoints. */
			qid = (ts->rate & AR_HTC_TXSTAT_EPID) >>
				AR_HTC_TXSTAT_EPID_SHIFT;
			if (qid != usc->ep_data[ATH_QID_AC_BE] &&
			    qid != usc->ep_data[ATH_QID_AC_BK-1] &&
			    qid != usc->ep_data[ATH_QID_AC_VI-1] &&
			    qid != usc->ep_data[ATH_QID_AC_VO-1])
				continue;
#if OpenBSD_IEEE80211_API
			if (ts->cookie == an->sta_index)
				ath_usb_tx_status(ts, ic->ic_bss);
			else
				ieee80211_iterate_nodes(ic, ath_usb_tx_status,
				    ts);
#endif
		}
		break;
	}
	case AR_WMI_EVT_FATAL:
		printf("%s: fatal firmware error\n", device_get_name(usc->usb_dev));
		break;
	default:
		DPRINTF(("WMI event %d ignored\n", cmd_id));
		break;
	}
}

#if NBPFILTER > 0
void
ath_usb_rx_radiotap(struct ath_softc *sc, struct mbuf *m,
    struct ar_rx_status *rs)
{
#define IEEE80211_RADIOTAP_F_SHORTGI	0x80	/* XXX from FBSD */

	struct ath_rx_radiotap_header *tap = &sc->sc_rxtap;
	struct ieee80211com *ic = &sc->sc_ic;
	struct mbuf mb;
	uint8_t rate;

	tap->wr_flags = IEEE80211_RADIOTAP_F_FCS;
	tap->wr_tsft = htole64(be64toh(rs->rs_tstamp));
	tap->wr_chan_freq = htole16(ic->ic_bss->ni_chan->ic_freq);
	tap->wr_chan_flags = htole16(ic->ic_bss->ni_chan->ic_flags);
	tap->wr_dbm_antsignal = rs->rs_rssi;
	/* XXX noise. */
	tap->wr_antenna = rs->rs_antenna;
	tap->wr_rate = 0;	/* In case it can't be found below. */
	rate = rs->rs_rate;
	if (rate & 0x80) {		/* HT. */
		/* Bit 7 set means HT MCS instead of rate. */
		tap->wr_rate = rate;
		if (!(rs->rs_flags & AR_RXS_FLAG_GI))
			tap->wr_flags |= IEEE80211_RADIOTAP_F_SHORTGI;

	} else if (rate & 0x10) {	/* CCK. */
		if (rate & 0x04)
			tap->wr_flags |= IEEE80211_RADIOTAP_F_SHORTPRE;
		switch (rate & ~0x14) {
		case 0xb: tap->wr_rate =   2; break;
		case 0xa: tap->wr_rate =   4; break;
		case 0x9: tap->wr_rate =  11; break;
		case 0x8: tap->wr_rate =  22; break;
		}
	} else {			/* OFDM. */
		switch (rate) {
		case 0xb: tap->wr_rate =  12; break;
		case 0xf: tap->wr_rate =  18; break;
		case 0xa: tap->wr_rate =  24; break;
		case 0xe: tap->wr_rate =  36; break;
		case 0x9: tap->wr_rate =  48; break;
		case 0xd: tap->wr_rate =  72; break;
		case 0x8: tap->wr_rate =  96; break;
		case 0xc: tap->wr_rate = 108; break;
		}
	}
	mb.m_data = (caddr_t)tap;
	mb.m_len = sc->sc_rxtap_len;
	mb.m_next = m;
	mb.m_nextpkt = NULL;
	mb.m_type = 0;
	mb.m_flags = 0;
	bpf_mtap(sc->sc_drvbpf, &mb, BPF_DIRECTION_IN);
}
#endif


void
ath_usb_rx_frame(struct ath_usb_softc *usc, struct mbuf *m/*,
    struct mbuf_list *ml*/)
{
#if OpenBSD_ONLY
	struct ath_softc *sc = usc->sc_sc;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ifnet *ifp = &ic->ic_if;
	struct ieee80211_frame *wh;
	struct ieee80211_node *ni;
	struct ieee80211_rxinfo rxi;

	struct ar_htc_frame_hdr *htc;
	struct ar_rx_status *rs;
	uint16_t datalen;
	int s;

	if (__predict_false(m->m_len < sizeof(*htc)))
		goto skip;
	htc = mtod(m, struct ar_htc_frame_hdr *);
	if (__predict_false(htc->endpoint_id == 0)) {
		DPRINTF(("bad endpoint %d\n", htc->endpoint_id));
		goto skip;
	}
	if (htc->flags & AR_HTC_FLAG_TRAILER) {
		if (m->m_len < htc->control[0])
			goto skip;
		m_adj(m, -(int)htc->control[0]);
	}
	m_adj(m, sizeof(*htc));	/* Strip HTC header. */

	if (__predict_false(m->m_len < sizeof(*rs)))
		goto skip;
	rs = mtod(m, struct ar_rx_status *);

	/* Make sure that payload fits. */
	datalen = be16toh(rs->rs_datalen);
	if (__predict_false(m->m_len < sizeof(*rs) + datalen))
		goto skip;

	if (__predict_false(datalen < sizeof(*wh) + IEEE80211_CRC_LEN))
		goto skip;

	if (rs->rs_status != 0) {
		if (rs->rs_status & AR_RXS_RXERR_DECRYPT)
			ic->ic_stats.is_ccmp_dec_errs++;
		goto skip;
	}

	m_adj(m, sizeof(*rs));	/* Strip Rx status. */

	s = splnet();

	/* Grab a reference to the source node. */

	wh = mtod(m, struct ieee80211_frame *);
	ni = ieee80211_find_rxnode(ic, wh);

	/* Remove any HW padding after the 802.11 header. */
	if (!(wh->i_fc[0] & IEEE80211_FC0_TYPE_CTL)) {

		u_int hdrlen = ieee80211_get_hdrlen(wh);
		if (hdrlen & 3) {
			memmove((caddr_t)wh + 2, wh, hdrlen);
			m_adj(m, 2);
		}
		wh = mtod(m, struct ieee80211_frame *);
	}

#if NBPFILTER > 0
	if (__predict_false(sc->sc_drvbpf != NULL))
		ath_usb_rx_radiotap(sc, m, rs);
#endif
	/* Trim 802.11 FCS after radiotap. */
	m_adj(m, -IEEE80211_CRC_LEN);

	/* Send the frame to the 802.11 layer. */

	memset(&rxi, 0, sizeof(rxi));
	rxi.rxi_rssi = rs->rs_rssi + AR_USB_DEFAULT_NF;
	rxi.rxi_tstamp = be64toh(rs->rs_tstamp);

	if (!(wh->i_fc[0] & IEEE80211_FC0_TYPE_CTL) &&
	    (wh->i_fc[1] & IEEE80211_FC1_PROTECTED) &&
	    (ic->ic_flags & IEEE80211_F_RSNON) &&
	    (ni->ni_flags & IEEE80211_NODE_RXPROT) &&
	    (ni->ni_rsncipher == IEEE80211_CIPHER_CCMP ||
	    (IEEE80211_IS_MULTICAST(wh->i_addr1) &&
	    ni->ni_rsngroupcipher == IEEE80211_CIPHER_CCMP))) {
		if (ar5008_ccmp_decap(sc, m, ni) != 0) {
			ieee80211_release_node(ic, ni);
			splx(s);
			goto skip;
		}
		rxi.rxi_flags |= IEEE80211_RXI_HWDEC;
	}
	ieee80211_inputm(ifp, m, ni, &rxi, ml);


	/* Node is no longer needed. */
	ieee80211_release_node(ic, ni);

	splx(s);
	return;
 skip:
	m_freem(m);
#endif
}

void
ath_usb_rxeof(struct usb_xfer *xfer, struct ath_usb_data *data)
{
#if OpenBSD_ONLY
	struct mbuf_list ml = MBUF_LIST_INITIALIZER();
#endif
	struct ath_usb_softc *usc = usbd_xfer_softc(xfer);
	struct ath_softc *sc = usc->sc_sc;
#if OpenBSD_IEEE80211_API
	struct ifnet *ifp = &sc->sc_ic.ic_if;
#endif
	struct ath_usb_rx_stream *stream = &usc->rx_stream;
	char *buf = data->buf;
	struct ar_stream_hdr *hdr;
	struct mbuf *m;
	uint16_t pktlen;
	int off;
	int32_t len;

#if OpenBSD_ONLY
	if (__predict_false(status != USBD_NORMAL_COMPLETION)) {
		DPRINTF(("RX status=%d\n", status));
		if (status == USBD_STALLED)
			usbd_clear_endpoint_stall_async(usc->rx_data_pipe);
		if (status != USBD_CANCELLED)
			goto resubmit;
		return;
	}
#endif
	usbd_xfer_status(xfer, NULL, NULL, &len, NULL);

	if (stream->left > 0) {
		if (len >= stream->left) {
			/* We have all our pktlen bytes now. */
			if (__predict_true(stream->m != NULL)) {
				memcpy(mtod(stream->m, uint8_t *) +
				    stream->moff, buf, stream->left);
				ath_usb_rx_frame(usc, stream->m/*, &ml*/); // MichalP: mbuf_list doesn't exist in FreeBSD
				stream->m = NULL;
			}
			/* Next header is 32-bit aligned. */
			off = (stream->left + 3) & ~3;
			buf += off;
			len -= off;
			stream->left = 0;
		} else {
			/* Still need more bytes, save what we have. */
			if (__predict_true(stream->m != NULL)) {
				memcpy(mtod(stream->m, uint8_t *) +
				    stream->moff, buf, len);
				stream->moff += len;
			}
			stream->left -= len;
			goto resubmit;
		}
	}
	KASSERT(stream->left == 0, "ath_usb_rxeof");
	while (len >= sizeof(*hdr)) {
		hdr = (struct ar_stream_hdr *)buf;
		if (hdr->tag != htole16(AR_USB_RX_STREAM_TAG)) {
			DPRINTF(("invalid tag 0x%x\n", hdr->tag));
			break;
		}
		pktlen = le16toh(hdr->len);
		buf += sizeof(*hdr);
		len -= sizeof(*hdr);
#if OpenBSD_ONLY // MichalP: mbuf_list operations needs proper refactoring
		if (__predict_true(pktlen <= MCLBYTES)) {
			/* Allocate an mbuf to store the next pktlen bytes. */
			MGETHDR(m, M_DONTWAIT, MT_DATA);
			if (__predict_true(m != NULL)) {
				m->m_pkthdr.len = m->m_len = pktlen;
				if (pktlen > MHLEN) {
					MCLGET(m, M_DONTWAIT);
					if (!(m->m_flags & M_EXT)) {
						m_free(m);
						m = NULL;
					}
				}
			}
		} else	/* Drop frames larger than MCLBYTES. */
			m = NULL;
#endif
		/*
		 * NB: m can be NULL, in which case the next pktlen bytes
		 * will be discarded from the Rx stream.
		 */
		if (pktlen > len) {
			/* Need more bytes, save what we have. */
			stream->m = m;	/* NB: m can be NULL. */
			if (__predict_true(stream->m != NULL)) {
				memcpy(mtod(stream->m, uint8_t *), buf, len);
				stream->moff = len;
			}
			stream->left = pktlen - len;
			goto resubmit;
		}
		if (__predict_true(m != NULL)) {
			/* We have all the pktlen bytes in this xfer. */
			memcpy(mtod(m, uint8_t *), buf, pktlen);
			ath_usb_rx_frame(usc, m/*, &ml*/); // MichalP: Another mbuf_list
		}

		/* Next header is 32-bit aligned. */
		off = (pktlen + 3) & ~3;
		buf += off;
		len -= off;
	}
#if OpenBSD_IEEE80211_API
	// TODO Refactor functions which use mbuf_list.
	// mbuf_list is OpenBSD specific and if_input uses the mbuf directly.
	// In the next steps, we should remove mbuf_list support from this driver.
	if_input(ifp, m);
#endif

	resubmit:
		printf("Resubmit called"); // MichalP: Temp printf call to maintain resubmit goto
	/* Setup a new transfer. */
#if OpenBSD_ONLY
	usbd_setup_xfer(xfer, usc->rx_data_pipe, data, data->buf,
	    ath_USB_RXBUFSZ, USBD_SHORT_XFER_OK | USBD_NO_COPY,
	    USBD_NO_TIMEOUT, ath_usb_rxeof);
	(void)usbd_transfer_start(xfer);
#endif
}

void
ath_usb_txeof(struct usb_xfer *xfer, struct ath_usb_data* data)
{
	struct ath_usb_softc *usc = usbd_xfer_softc(xfer);
	struct ath_softc *sc = usc->sc_sc;
#if OpenBSD_IEEE80211_API
	struct ifnet *ifp = &sc->sc_ic.ic_if;
#endif

	device_printf(sc->sc_dev, "%s: called; data=%p\n", __func__, data);

	ATH_USB_LOCK_ASSERT(sc);

	if (usc->sc_tx_n_active == 0) {
		device_printf(sc->sc_dev,
					  "%s: completed but tx_active=0\n",
					  __func__);
	} else {
		usc->sc_tx_n_active--;
	}

//	if (data->m) {
		/* XXX status? */
		/* XXX we get TX status via the RX path.. */
//		ieee80211_tx_complete(data->ni, data->m, 0);
//		data->m = NULL;
//		data->ni = NULL;
//	}
}

int
ath_usb_tx(struct ath_softc *sc, struct mbuf *m, struct ieee80211_node *ni)
{
	struct ath_usb_softc *usc = sc->usc;
	struct ath_node *an = (struct ath_node *)ni;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_frame *wh;
	struct ieee80211_key *k = NULL;
	struct ath_usb_data *data;
	struct ar_stream_hdr *hdr;
	struct ar_htc_frame_hdr *htc;
	struct ar_tx_frame *txf;
	struct ar_tx_mgmt *txm;
	caddr_t frm;
	uint16_t qos;
	uint8_t qid, tid = 0;
	int hasqos, xferlen, error;

	wh = mtod(m, struct ieee80211_frame *);
#if OpenBSD_IEEE80211_API
	if (wh->i_fc[1] & IEEE80211_FC1_PROTECTED) {
		k = ieee80211_get_txkey(ic, wh, ni);
		if (k->k_cipher == IEEE80211_CIPHER_CCMP) {
			u_int hdrlen = ieee80211_get_hdrlen(wh);
			if (ar5008_ccmp_encap(m, hdrlen, k) != 0)
				return (ENOBUFS);
		} else {
			if ((m = ieee80211_encrypt(ic, m, k)) == NULL)
				return (ENOBUFS);
			k = NULL; /* skip hardware crypto further below */
		}
		wh = mtod(m, struct ieee80211_frame *);
	}
	if ((hasqos = ieee80211_has_qos(wh))) {
		qos = ieee80211_get_qos(wh);
		tid = qos & IEEE80211_QOS_TID;
		qid = ieee80211_up_to_ac(ic, tid);
	} else
		qid = ATH_QID_AC_BE;
#endif
	/* Grab a Tx buffer from our free list. */
#if OpenBSD_ONLY
	data = TAILQ_FIRST(&usc->tx_free_list);
	TAILQ_REMOVE(&usc->tx_free_list, data, next);
#endif

#if NBPFILTER > 0
	/* XXX Change radiotap Tx header for USB (no txrate). */
	if (__predict_false(sc->sc_drvbpf != NULL)) {
		struct ath_tx_radiotap_header *tap = &sc->sc_txtap;
		struct mbuf mb;

		tap->wt_flags = 0;
		tap->wt_chan_freq = htole16(ic->ic_bss->ni_chan->ic_freq);
		tap->wt_chan_flags = htole16(ic->ic_bss->ni_chan->ic_flags);
		mb.m_data = (caddr_t)tap;
		mb.m_len = sc->sc_txtap_len;
		mb.m_next = m;
		mb.m_nextpkt = NULL;
		mb.m_type = 0;
		mb.m_flags = 0;
		bpf_mtap(sc->sc_drvbpf, &mb, BPF_DIRECTION_OUT);
	}
#endif

	/* NB: We don't take advantage of USB Tx stream mode for now. */
	hdr = (struct ar_stream_hdr *)data->buf;
	hdr->tag = htole16(AR_USB_TX_STREAM_TAG);

	htc = (struct ar_htc_frame_hdr *)&hdr[1];
	memset(htc, 0, sizeof(*htc));
	if ((wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) ==
	    IEEE80211_FC0_TYPE_DATA) {
		htc->endpoint_id = usc->ep_data[qid];

		txf = (struct ar_tx_frame *)&htc[1];
		memset(txf, 0, sizeof(*txf));
		txf->data_type = AR_HTC_NORMAL;
#if ATHN_API
		txf->node_idx = an->sta_index;
#endif
		txf->vif_idx = 0;
		txf->tid = tid;
#if OpenBSD_IEEE80211_API
		if (m->m_pkthdr.len + IEEE80211_CRC_LEN > ic->ic_rtsthreshold)
			txf->flags |= htobe32(AR_HTC_TX_RTSCTS);
		else if (ic->ic_flags & IEEE80211_F_USEPROT) {
			if (ic->ic_protmode == IEEE80211_PROT_CTSONLY)
				txf->flags |= htobe32(AR_HTC_TX_CTSONLY);
			else if (ic->ic_protmode == IEEE80211_PROT_RTSCTS)
				txf->flags |= htobe32(AR_HTC_TX_RTSCTS);
		}

		if (k != NULL) {
			/* Map 802.11 cipher to hardware encryption type. */
			if (k->k_cipher == IEEE80211_CIPHER_CCMP) {
				txf->key_type = AR_ENCR_TYPE_AES;
			} else
				panic("unsupported cipher");
			/*
			 * NB: The key cache entry index is stored in the key
			 * private field when the key is installed.
			 */
			txf->key_idx = (uintptr_t)k->k_priv;
		} else
			txf->key_idx = 0xff;
#endif
#if ATHN_API
		txf->cookie = an->sta_index;
#endif
		frm = (caddr_t)&txf[1];
	} else {
		htc->endpoint_id = usc->ep_mgmt;

		txm = (struct ar_tx_mgmt *)&htc[1];
		memset(txm, 0, sizeof(*txm));
#if ATHN_API
		txm->node_idx = an->sta_index;
#endif
		txm->vif_idx = 0;
		txm->key_idx = 0xff;
#if ATHN_API
		txm->cookie = an->sta_index;
#endif
		frm = (caddr_t)&txm[1];
	}
	/* Copy payload. */
	m_copydata(m, 0, m->m_pkthdr.len, frm);
	frm += m->m_pkthdr.len;
	m_freem(m);

	/* Finalize headers. */
	htc->payload_len = htobe16(frm - (caddr_t)&htc[1]);
	hdr->len = htole16(frm - (caddr_t)&hdr[1]);
	xferlen = frm - data->buf;

	data->buflen = xferlen;

	STAILQ_INSERT_TAIL(&usc->sc_tx_pending[ath_BULK_TX], data, next);
	usbd_transfer_start(usc->sc_xfer[ath_BULK_TX]);

#if OpenBSD_IEEE80211_API
	ieee80211_release_node(ic, ni);
#endif
	return (0);
}

void
ath_usb_start(struct ifnet *ifp)
{
	struct ath_softc *sc = ifp->if_softc;
	struct ath_usb_softc *usc = sc->usc;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_node *ni;
	struct mbuf *m;

	if (!(ifp->if_flags & IFF_RUNNING) || ifq_is_oactive())
		return;

	for (;;) {
#if OpenBSD_ONLY
		if (TAILQ_EMPTY(&usc->tx_free_list)) {
			ifq_set_oactive();
			break;
		}
#endif
		/* Send pending management frames first. */
#if OpenBSD_IEEE80211_API
		m = mq_dequeue(&ic->ic_mgtq);
		if (m != NULL) {
			ni = m->m_pkthdr.ph_cookie;
			goto sendit;
		}
		if (ic->ic_state != IEEE80211_S_RUN)
			break;
#endif

		/* Encapsulate and send data frames. */
		ALTQ_DEQUEUE(&ifp->if_snd, m);
		if (m == NULL)
			break;
#if NBPFILTER > 0
		f (ifp->if_bpf != NULL)
			bpf_mtap(ifp->if_bpf, m, BPF_DIRECTION_OUT);
#endif
#if OpenBSD_IEEE80211_API
		if ((m = ieee80211_encap(ifp, m, &ni)) == NULL)
			continue;
#endif
 sendit:
#if NBPFILTER > 0
		if (ic->ic_rawbpf != NULL)
			bpf_mtap(ic->ic_rawbpf, m, BPF_DIRECTION_OUT);
#endif
		if (ath_usb_tx(sc, m, ni) != 0) {
#if OpenBSD_IEEE80211_API
			ieee80211_release_node(ic, ni);
#endif
			continue;
		}
#if ATHN_API
		sc->sc_tx_timer = 5;
#endif
	}
}

// Comented because there is no sc_tx_timer in the ath_softc structure.
#if ATHN_API
void
ath_usb_watchdog(struct ifnet *ifp)
{
	struct ath_softc *sc = ifp->if_softc;

	if (sc->sc_tx_timer > 0) {
		if (--sc->sc_tx_timer == 0) {
			printf("%s: device timeout\n", device_get_name(sc->sc_dev));
			ath_usb_init(ifp);
			return;
		}
	}
#if OpenBSD_IEEE80211_API
	ieee80211_watchdog(ifp);
#endif
}
#endif

int
ath_usb_ioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
{
	struct ath_softc *sc = ifp->if_softc;
	struct ath_usb_softc *usc = sc->usc;
	struct ieee80211com *ic = &sc->sc_ic;
	int s, error = 0;

	switch (cmd) {
	case SIOCSIFADDR:
		ifp->if_flags |= IFF_UP;
		/* FALLTHROUGH */
	case SIOCSIFFLAGS:
		if (ifp->if_flags & IFF_UP) {
			if (!(ifp->if_flags & IFF_RUNNING))
				error = ath_usb_init(ifp);
		} else {
			if (ifp->if_flags & IFF_RUNNING)
				ath_usb_stop(ifp);
		}
		break;
#if OpenBSD_IEEE80211_API
	case SIOCS80211CHANNEL:
		error = ieee80211_ioctl(ifp, cmd, data);
		if (error == ENETRESET &&
		    ic->ic_opmode == IEEE80211_M_MONITOR) {
			if ((ifp->if_flags & (IFF_UP | IFF_RUNNING)) ==
			    (IFF_UP | IFF_RUNNING)) {
				ath_usb_switch_chan(sc, ic->ic_ibss_chan,
				    NULL);
			}
			error = 0;
		}
		break;
#endif
	default:
		error = ieee80211_ioctl(ifp, cmd, data);
	}

	if (error == ENETRESET) {
		error = 0;
		if ((ifp->if_flags & (IFF_UP | IFF_RUNNING)) ==
		    (IFF_UP | IFF_RUNNING)) {
			ath_usb_stop(ifp);
			error = ath_usb_init(ifp);
		}
	}
	return (error);
}


int
ath_usb_init(struct ifnet *ifp)
{
#if ATHN_API
	struct ath_softc *sc = ifp->if_softc;
	struct ath_usb_softc *usc = sc->usc;
	struct ath_ops *ops = &sc->ops;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_channel *c, *extc;
	struct ath_usb_data *data;
	struct ar_htc_target_vif hvif;
	struct ar_htc_target_sta sta;
	struct ar_htc_cap_target hic;
	uint16_t mode;
	int i, error;

#if OpenBSD_IEEE80211_API
	c = ic->ic_bss->ni_chan = ic->ic_ibss_chan;
	extc = NULL;

	/* In case a new MAC address has been configured. */
	IEEE80211_ADDR_COPY(ic->ic_myaddr, IF_LLADDR(ifp));
#endif
	error = ath_set_power_awake(sc);
	if (error != 0)
		goto fail;

	error = ath_usb_wmi_cmd(usc, AR_WMI_CMD_FLUSH_RECV);
	if (error != 0)
		goto fail;

	error = ath_hw_reset(sc, c, extc, 1);
	if (error != 0)
		goto fail;

	ops->set_txpower(sc, c, extc);

	mode = htobe16(IEEE80211_IS_CHAN_2GHZ(c) ?
	    AR_HTC_MODE_11NG : AR_HTC_MODE_11NA);
	error = ath_usb_wmi_xcmd(usc, AR_WMI_CMD_SET_MODE,
	    &mode, sizeof(mode), NULL);
	if (error != 0)
		goto fail;

	error = ath_usb_wmi_cmd(usc, AR_WMI_CMD_ATH_INIT);
	if (error != 0)
		goto fail;

	error = ath_usb_wmi_cmd(usc, AR_WMI_CMD_START_RECV);
	if (error != 0)
		goto fail;

	ath_rx_start(sc);

	/* Create main interface on target. */
	memset(&hvif, 0, sizeof(hvif));
	hvif.index = 0;
#if OpenBSD_IEEE80211_API
	IEEE80211_ADDR_COPY(hvif.myaddr, ic->ic_myaddr);
#endif
	switch (ic->ic_opmode) {
	case IEEE80211_M_STA:
		hvif.opmode = htobe32(AR_HTC_M_STA);
		break;
	case IEEE80211_M_MONITOR:
		hvif.opmode = htobe32(AR_HTC_M_MONITOR);
		break;
#ifndef IEEE80211_STA_ONLY
	case IEEE80211_M_IBSS:
		hvif.opmode = htobe32(AR_HTC_M_IBSS);
		break;
	case IEEE80211_M_AHDEMO:
		hvif.opmode = htobe32(AR_HTC_M_AHDEMO);
		break;
	case IEEE80211_M_HOSTAP:
		hvif.opmode = htobe32(AR_HTC_M_HOSTAP);
		break;
	case IEEE80211_M_MBSS:
		break;
	case IEEE80211_M_WDS:
		break;
#endif
	}
#if OpenBSD_IEEE80211_API
	hvif.rtsthreshold = htobe16(ic->ic_rtsthreshold);
#endif
	DPRINTF(("creating VAP\n"));
	error = ath_usb_wmi_xcmd(usc, AR_WMI_CMD_VAP_CREATE,
	    &hvif, sizeof(hvif), NULL);
	if (error != 0)
		goto fail;

	/* Create a fake node to send management frames before assoc. */
	memset(&sta, 0, sizeof(sta));
#if OpenBSD_IEEE80211_API
	IEEE80211_ADDR_COPY(sta.macaddr, ic->ic_myaddr);
#endif
	sta.sta_index = 0;
	sta.is_vif_sta = 1;
	sta.vif_index = hvif.index;
	sta.maxampdu = 0xffff;
	DPRINTF(("creating default node\n"));
	error = ath_usb_wmi_xcmd(usc, AR_WMI_CMD_NODE_CREATE,
	    &sta, sizeof(sta), NULL);
	if (error != 0)
		goto fail;
	usc->free_node_slots = ~(1 << sta.sta_index);

	/* Update target capabilities. */
	memset(&hic, 0, sizeof(hic));
	hic.ampdu_limit = htobe32(0x0000ffff);
	hic.ampdu_subframes = 20;
	hic.txchainmask = sc->txchainmask;
	DPRINTF(("updating target configuration\n"));
	error = ath_usb_wmi_xcmd(usc, AR_WMI_CMD_TARGET_IC_UPDATE,
	    &hic, sizeof(hic), NULL);
	if (error != 0)
		goto fail;

#if OpenBSD_ONLY
	/* Queue Rx xfers. */
	for (i = 0; i < ath_USB_RX_LIST_COUNT; i++) {
		data = &usc->rx_data[i];
		usbd_setup_xfer(data->xfer, usc->rx_data_pipe, data, data->buf,
		    ath_USB_RXBUFSZ, USBD_SHORT_XFER_OK | USBD_NO_COPY,
		    USBD_NO_TIMEOUT, ath_usb_rxeof);
		error = usbd_transfer_start(data->xfer);
		if (error != 0 && error != USBD_IN_PROGRESS)
			goto fail;
	}
#endif
	/* We're ready to go. */
	ifp->if_flags |= IFF_RUNNING;
	ifq_clr_oactive();

#ifdef notyet
	if (ic->ic_flags & IEEE80211_F_WEPON) {
		/* Install WEP keys. */
		for (i = 0; i < IEEE80211_WEP_NKID; i++)
			ath_usb_set_key(ic, NULL, &ic->ic_nw_keys[i]);
	}
#endif
#if OpenBSD_IEEE80211_API
	if (ic->ic_opmode == IEEE80211_M_MONITOR)
		ieee80211_new_state(ic, IEEE80211_S_RUN, -1);
	else
		ieee80211_new_state(ic, IEEE80211_S_SCAN, -1);
#endif
	ath_usb_wait_async(usc);
	return (0);
 fail:
	ath_usb_stop(ifp);
	return (error);
#else
	return 0;
#endif
}


void
ath_usb_stop(struct ifnet *ifp)
{
	struct ath_softc *sc = ifp->if_softc;
	struct ath_usb_softc *usc = sc->usc;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ar_htc_target_vif hvif;
	uint8_t sta_index;
	int s;
#if ATHN_API
	sc->sc_tx_timer = 0;
#endif
	ifp->if_flags &= ~IFF_RUNNING;
	ifq_clr_oactive();

	s = splusb();
#if OpenBSD_IEEE80211_API
	ieee80211_new_state(ic, IEEE80211_S_INIT, -1);
#endif

	/* Wait for all async commands to complete. */
	ath_usb_wait_async(usc);

#if OpenBSD_ONLY
	timeout_del(&sc->scan_to);
	timeout_del(&sc->calib_to);
#endif

	/* Remove all non-default nodes. */
	for (sta_index = 1; sta_index < AR_USB_MAX_STA; sta_index++) {
		if (usc->free_node_slots & (1 << sta_index))
			continue;
		(void)ath_usb_wmi_xcmd(usc, AR_WMI_CMD_NODE_REMOVE,
		    &sta_index, sizeof(sta_index), NULL);
	}

	/* Remove main interface. This also invalidates our default node. */
	memset(&hvif, 0, sizeof(hvif));
	hvif.index = 0;
#if OpenBSD_IEEE80211_API
	IEEE80211_ADDR_COPY(hvif.myaddr, ic->ic_myaddr);
#endif
	(void)ath_usb_wmi_xcmd(usc, AR_WMI_CMD_VAP_REMOVE,
	    &hvif, sizeof(hvif), NULL);

	usc->free_node_slots = 0xff;

	(void)ath_usb_wmi_cmd(usc, AR_WMI_CMD_DISABLE_INTR);
	(void)ath_usb_wmi_cmd(usc, AR_WMI_CMD_DRAIN_TXQ_ALL);
	(void)ath_usb_wmi_cmd(usc, AR_WMI_CMD_STOP_RECV);

#if ATHN_API
	ath_reset(sc, 0);
	ath_init_pll(sc, NULL);
	ath_set_power_awake(sc);
	ath_reset(sc, 1);
	ath_init_pll(sc, NULL);
	ath_set_power_sleep(sc);
#endif
#if OpenBSD_ONLY
	/* Abort Tx/Rx. */
	usbd_abort_pipe(usc->tx_data_pipe);
	usbd_abort_pipe(usc->rx_data_pipe);
#endif

	/* Free Tx/Rx buffers. */
	ath_usb_free_tx_list(usc);
	ath_usb_free_rx_list(usc);
	splx(s);

	/* Flush Rx stream. */
	m_freem(usc->rx_stream.m);
	usc->rx_stream.m = NULL;
	usc->rx_stream.left = 0;
}
