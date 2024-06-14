/*	$OpenBSD: athn.c,v 1.111 2021/04/15 18:25:43 stsp Exp $	*/

/*-
 * Copyright (c) 2009 Damien Bergamini <damien.bergamini@free.fr>
 * Copyright (c) 2008-2010 Atheros Communications Inc.
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

/*
 * Driver for Atheros 802.11a/g/n chipsets.
 */

#include <sys/param.h>
#include <sys/sockio.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/queue.h>
#include <sys/conf.h>
#include <sys/stdint.h>	/* uintptr_t */
#include <sys/endian.h>
#include <machine/bus.h>

// #if NBPFILTER > 0
// #include <net/bpf.h>
// #endif
#include <net/if.h>
#include <net/if_var.h>
#include <net/ethernet.h>
#include <net/if_media.h>

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_amrr.h>
#include <net80211/ieee80211_regdomain.h>
#include <net80211/ieee80211_radiotap.h>

#include "athnreg.h"
#include "athnvar.h"
#include "openbsd_adapt.h"

// TODO missing macro def
#define NATHN_USB 1

#ifdef ATHN_DEBUG
int athn_debug = 0;
#endif

void		athn_radiotap_attach(struct athn_softc *);
void		athn_get_chanlist(struct athn_softc *);
const char *	athn_get_mac_name(struct athn_softc *);
const char *	athn_get_rf_name(struct athn_softc *);
void		athn_led_init(struct athn_softc *);
void		athn_set_led(struct athn_softc *, int);
void		athn_btcoex_init(struct athn_softc *);
void		athn_btcoex_enable(struct athn_softc *);
void		athn_btcoex_disable(struct athn_softc *);
void		athn_set_rxfilter(struct athn_softc *, uint32_t);
void		athn_get_chipid(struct athn_softc *);
int		athn_reset_power_on(struct athn_softc *);
int		athn_reset(struct athn_softc *, int);
void		athn_init_pll(struct athn_softc *,
		    const struct ieee80211_channel *);
int		athn_set_power_awake(struct athn_softc *);
void		athn_set_power_sleep(struct athn_softc *);
void		athn_write_serdes(struct athn_softc *,
		    const struct athn_serdes *);
void		athn_config_pcie(struct athn_softc *);
void		athn_config_nonpcie(struct athn_softc *);
int		athn_set_chan(struct athn_softc *, struct ieee80211_channel *,
		    struct ieee80211_channel *);
int		athn_switch_chan(struct athn_softc *,
		    struct ieee80211_channel *, struct ieee80211_channel *);
void		athn_get_delta_slope(uint32_t, uint32_t *, uint32_t *);
void		athn_reset_key(struct athn_softc *, int);
int		athn_set_key(struct ieee80211com *, struct ieee80211_node *,
		    struct ieee80211_key *);
void		athn_delete_key(struct ieee80211com *, struct ieee80211_node *,
		    struct ieee80211_key *);
void		athn_iter_calib(void *, struct ieee80211_node *);
int		athn_cap_noisefloor(struct athn_softc *, int);
int		athn_nf_hist_mid(int *, int);
void		athn_filter_noisefloor(struct athn_softc *);
void		athn_start_noisefloor_calib(struct athn_softc *, int);
void		athn_calib_to(void *);
int		athn_init_calib(struct athn_softc *,
		    struct ieee80211_channel *, struct ieee80211_channel *);
uint8_t		athn_chan2fbin(struct ieee80211_channel *);
int		athn_interpolate(int, int, int, int, int);
void		athn_get_pier_ival(uint8_t, const uint8_t *, int, int *,
		    int *);
void		athn_init_dma(struct athn_softc *);
void		athn_rx_start(struct athn_softc *);
void		athn_inc_tx_trigger_level(struct athn_softc *);
int		athn_stop_rx_dma(struct athn_softc *);
int		athn_rx_abort(struct athn_softc *);
void		athn_tx_reclaim(struct athn_softc *, int);
int		athn_tx_pending(struct athn_softc *, int);
void		athn_stop_tx_dma(struct athn_softc *, int);
int		athn_txtime(struct athn_softc *, int, int, u_int);
void		athn_set_sta_timers(struct athn_softc *);
void		athn_set_hostap_timers(struct athn_softc *);
void		athn_set_opmode(struct athn_softc *);
void		athn_set_bss(struct athn_softc *, struct ieee80211_node *);
void		athn_enable_interrupts(struct athn_softc *);
void		athn_disable_interrupts(struct athn_softc *);
void		athn_init_qos(struct athn_softc *);
int		athn_hw_reset(struct athn_softc *, struct ieee80211_channel *,
		    struct ieee80211_channel *, int);
struct		ieee80211_node *athn_node_alloc(struct ieee80211com *);
void		athn_newassoc(struct ieee80211com *, struct ieee80211_node *,
		    int);
int		athn_media_change(struct ifnet *);
void		athn_next_scan(void *);
int		athn_newstate(struct ieee80211com *, enum ieee80211_state,
		    int);
void		athn_updateedca(struct ieee80211com *);
int		athn_clock_rate(struct athn_softc *);
int		athn_chan_sifs(struct ieee80211_channel *);
void		athn_setsifs(struct athn_softc *);
int		athn_acktimeout(struct ieee80211_channel *, int);
void		athn_setacktimeout(struct athn_softc *,
		    struct ieee80211_channel *, int);
void		athn_setctstimeout(struct athn_softc *,
		    struct ieee80211_channel *, int);
void		athn_setclockrate(struct athn_softc *);
void		athn_updateslot(struct ieee80211com *);
void		athn_start(struct ifnet *);
void		athn_watchdog(struct ifnet *);
void		athn_set_multi(struct athn_softc *);
int		athn_ioctl(struct ifnet *, u_long, caddr_t);
int		athn_init(struct ifnet *);
void		athn_stop(struct ifnet *, int);
void		athn_init_tx_queues(struct athn_softc *);
int32_t		athn_ani_get_rssi(struct athn_softc *);
void		athn_ani_ofdm_err_trigger(struct athn_softc *);
void		athn_ani_cck_err_trigger(struct athn_softc *);
void		athn_ani_lower_immunity(struct athn_softc *);
void		athn_ani_restart(struct athn_softc *);
void		athn_ani_monitor(struct athn_softc *);

/* Extern functions. */
int		ar9285_attach(struct athn_softc *);
int		ar9285_init_calib(struct athn_softc *,
		    struct ieee80211_channel *, struct ieee80211_channel *);
void		ar9271_pa_calib(struct athn_softc *);


// TODO missing in sys/device.h
/*
 * Minimal device structures.
 * Note that all ``system'' device types are listed here.
 */
// enum devclass {
// 	DV_DULL,		/* generic, no special info */
// 	DV_CPU,			/* CPU (carries resource utilization) */
// 	DV_DISK,		/* disk drive (label, etc) */
// 	DV_IFNET,		/* network interface */
// 	DV_TAPE,		/* tape device */
// 	DV_TTY			/* serial line interface (???) */
// };

#if 0
struct cfdriver athn_cd = {
	NULL, "athn", DV_IFNET
};
#endif


// TODO missing in net80211/ieee80211.h
#define IEEE80211_HTCAP_SMPS_DIS    3
#define IEEE80211_HTCAP_SMPS_SHIFT  2

static void
athn_config_ht(struct athn_softc *sc)
{
	__attribute__((unused)) struct ieee80211com *ic = &sc->sc_ic;
	int ntxstreams, nrxstreams;

	if ((sc->flags & ATHN_FLAG_11N) == 0)
		return;

	/* Set HT capabilities. */
	// ic->ic_htcaps = (IEEE80211_HTCAP_SMPS_DIS <<
	//     IEEE80211_HTCAP_SMPS_SHIFT);
#ifdef notyet
	ic->ic_htcaps |= IEEE80211_HTCAP_CBW20_40 |
	    IEEE80211_HTCAP_SGI40 |
	    IEEE80211_HTCAP_DSSSCCK40;
#endif
	// TODO missing field
	// ic->ic_htxcaps = 0;
#ifdef notyet
	if (AR_SREV_9271(sc) || AR_SREV_9287_10_OR_LATER(sc))
		ic->ic_htcaps |= IEEE80211_HTCAP_SGI20;
	if (AR_SREV_9380_10_OR_LATER(sc))
		ic->ic_htcaps |= IEEE80211_HTCAP_LDPC;
	if (AR_SREV_9280_10_OR_LATER(sc)) {
		ic->ic_htcaps |= IEEE80211_HTCAP_TXSTBC;
		ic->ic_htcaps |= 1 << IEEE80211_HTCAP_RXSTBC_SHIFT;
	}
#endif
	ntxstreams = sc->ntxchains;
	nrxstreams = sc->nrxchains;
	if (!AR_SREV_9380_10_OR_LATER(sc)) {
		ntxstreams = MIN(ntxstreams, 2);
		nrxstreams = MIN(nrxstreams, 2);
	}
	// TODO missing fields
	/* Set supported HT rates. */
	// if (ic->ic_userflags & IEEE80211_F_NOMIMO)
	// 	ntxstreams = nrxstreams = 1;
	// memset(ic->ic_sup_mcs, 0, sizeof(ic->ic_sup_mcs));
	// for (i = 0; i < nrxstreams; i++)
	// 	ic->ic_sup_mcs[i] = 0xff;
	// ic->ic_tx_mcs_set = IEEE80211_TX_MCS_SET_DEFINED;
	// if (ntxstreams != nrxstreams) {
	// 	ic->ic_tx_mcs_set |= IEEE80211_TX_RX_MCS_NOT_EQUAL;
	// 	ic->ic_tx_mcs_set |= (ntxstreams - 1) << 2;
	// }
}
// TODO: Move to header
#ifndef AR5416_OPFLAGS_11G
#define	AR5416_OPFLAGS_11A		0x01
#define	AR5416_OPFLAGS_11G		0x02
#endif

// Naming convention from otus.
// Function similar to athn_get_chanlist but with other parametrs so it can be used as a callback
static void
athn_getradiocaps(struct ieee80211com *ic,
    int maxchans, int *nchans, struct ieee80211_channel chans[])
{
	struct athn_softc *sc = ic->ic_softc;
	uint8_t bands[IEEE80211_MODE_BYTES];
	int cbw_flags;

	memset(bands, 0, sizeof(bands));


	cbw_flags = (ic->ic_htcaps & IEEE80211_HTCAP_CHWIDTH40) ?
	    NET80211_CBW_FLAG_HT40 : 0;
	/* Set supported rates. */

		// printf("ATHN_FLAG_11G");
		setbit(bands, IEEE80211_MODE_11B);
		setbit(bands, IEEE80211_MODE_11G);
		setbit(bands, IEEE80211_MODE_11NG);
		ieee80211_add_channels_default_2ghz(chans, maxchans, nchans,
	    bands, cbw_flags);
	if (sc->eeprom.baseEepHeader.opCapFlags & AR5416_OPFLAGS_11A) {
		printf("ATHN_FLAG_11A");
		setbit(bands, IEEE80211_MODE_11A);
		setbit(bands, IEEE80211_MODE_11NA);
		ieee80211_add_channel_list_5ghz(ic->ic_channels, IEEE80211_CHAN_MAX, nchans,
			athn_5ghz_chans, nitems(athn_5ghz_chans), bands, 0);
	}
}

#if 0
int
athn_attach(struct athn_softc *sc)
{
	__attribute__((unused)) struct ieee80211com *ic = &sc->sc_ic;
	// TODO missing field ic_if' in 'struct ieee80211com'
	// struct ifnet *ifp = &ic->ic_if;
	struct ifnet *ifp = NULL;
	int error;

	/* Read hardware revision. */
	athn_get_chipid(sc);

	device_printf(sc->sc_dev, "%s: chip id read\n",	__func__);

	if ((error = athn_reset_power_on(sc)) != 0) {
		device_printf(sc->sc_dev, "%s: could not reset chip\n",	__func__);
		return (error);
	}

	if ((error = athn_set_power_awake(sc)) != 0) {
		device_printf(sc->sc_dev, "%s: could not wakeup chip\n", __func__);
		return (error);
	}

#if OpenBSD_IEEE80211_API
	if (AR_SREV_9271(sc))
		error = ar9285_attach(sc);

	else
		error = ENOTSUP;
#endif

	if (error != 0) {
		// TOTO missing field 'dv_xname' in 'struct device'
		// printf("%s: could not attach chip\n", sc->sc_dev.dv_xname);
		return (error);
	}

	/* We can put the chip in sleep state now. */
	athn_set_power_sleep(sc);

	if (!(sc->flags & ATHN_FLAG_USB)) {
		error = sc->ops.dma_alloc(sc);
		if (error != 0) {
			// TOTO missing field 'dv_xname' in 'struct device'
			// printf("%s: could not allocate DMA resources\n",
			//     sc->sc_dev.dv_xname);
			return (error);
		}
		/* Steal one Tx buffer for beacons. */
		sc->bcnbuf = SIMPLEQ_FIRST(&sc->txbufs);
		SIMPLEQ_REMOVE_HEAD(&sc->txbufs, bf_list);
	}

	if (sc->flags & ATHN_FLAG_RFSILENT) {
		DPRINTF(("found RF switch connected to GPIO pin %d\n",
		    sc->rfsilent_pin));
	}
	DPRINTF(("%d key cache entries\n", sc->kc_entries));

	/*
	 * In HostAP mode, the number of STAs that we can handle is
	 * limited by the number of entries in the HW key cache.
	 * TKIP keys would consume 2 entries in this cache but we
	 * only use the hardware crypto engine for CCMP.
	 */
	// TODO missing field 'ic_max_nnodes' in 'struct ieee80211com'
	// ic->ic_max_nnodes = sc->kc_entries - IEEE80211_WEP_NKID;
	// if (ic->ic_max_nnodes > IEEE80211_CACHE_SIZE)
	// 	ic->ic_max_nnodes = IEEE80211_CACHE_SIZE;

	DPRINTF(("using %s loop power control\n",
	    (sc->flags & ATHN_FLAG_OLPC) ? "open" : "closed"));

	DPRINTF(("txchainmask=0x%x rxchainmask=0x%x\n",
	    sc->txchainmask, sc->rxchainmask));
	/* Count the number of bits set (in lowest 3 bits). */
	sc->ntxchains =
	    ((sc->txchainmask >> 2) & 1) +
	    ((sc->txchainmask >> 1) & 1) +
	    ((sc->txchainmask >> 0) & 1);
	sc->nrxchains =
	    ((sc->rxchainmask >> 2) & 1) +
	    ((sc->rxchainmask >> 1) & 1) +
	    ((sc->rxchainmask >> 0) & 1);

	if (AR_SINGLE_CHIP(sc)) {
		// TOTO missing field 'dv_xname' in 'struct device'
		// printf("%s: %s rev %d (%dT%dR), ROM rev %d, address %s\n",
		//     sc->sc_dev.dv_xname, athn_get_mac_name(sc), sc->mac_rev,
		//     sc->ntxchains, sc->nrxchains, sc->eep_rev,
		//     ether_sprintf(ic->ic_myaddr));
	} else {
		// TOTO missing field 'dv_xname' in 'struct device'
		// printf("%s: MAC %s rev %d, RF %s (%dT%dR), ROM rev %d, "
		//     "address %s\n",
		//     sc->sc_dev.dv_xname, athn_get_mac_name(sc), sc->mac_rev,
		//     athn_get_rf_name(sc), sc->ntxchains, sc->nrxchains,
		//     sc->eep_rev, ether_sprintf(ic->ic_myaddr));
	}
#if OpenBSD_IEEE80211_API
	// TODO: port everything below
#else
	device_printf(sc->sc_dev, "%s: Planned return on line: %d\n", __func__, __LINE__);
	return 0;
#endif


#if OpenBSD_ONLY
	timeout_set(&sc->scan_to, athn_next_scan, sc);
	timeout_set(&sc->calib_to, athn_calib_to, sc);
#endif
	sc->amrr.amrr_min_success_threshold =  1;
	sc->amrr.amrr_max_success_threshold = 15;

	ic->ic_phytype = IEEE80211_T_OFDM;	/* not only, but not used */
	ic->ic_opmode = IEEE80211_M_STA;	/* default to BSS mode */
	// TODO missing field 'ic_state' in 'struct ieee80211com'
	// ic->ic_state = IEEE80211_S_INIT;

	// TODO missing macros in ieee80211_var.h
	/* ic_caps */
#define	IEEE80211_C_WEP		0x00000001	/* CAPABILITY: WEP available */
#define	IEEE80211_C_APPMGT	0x00000020	/* CAPABILITY: AP power mgmt */
#define IEEE80211_C_SCANALL	0x00000400	/* CAPABILITY: scan all chan */
#define IEEE80211_C_QOS		0x00000800	/* CAPABILITY: QoS avail */
#define IEEE80211_C_RSN		0x00001000	/* CAPABILITY: RSN avail */
#define IEEE80211_C_MFP		0x00002000	/* CAPABILITY: MFP avail */
#define IEEE80211_C_RAWCTL	0x00004000	/* CAPABILITY: raw ctl */
#define IEEE80211_C_SCANALLBAND	0x00008000	/* CAPABILITY: scan all bands */
#define IEEE80211_C_TX_AMPDU	0x00010000	/* CAPABILITY: send A-MPDU */
#define IEEE80211_C_ADDBA_OFFLOAD 0x00020000	/* CAPABILITY: ADDBA offload */



	/* Set device capabilities. */
	ic->ic_caps =
	    IEEE80211_C_WEP |		/* WEP. */
	    IEEE80211_C_RSN |		/* WPA/RSN. */
#ifndef IEEE80211_STA_ONLY
	    IEEE80211_C_HOSTAP |	/* Host AP mode supported. */
	    IEEE80211_C_APPMGT |	/* Host AP power saving supported. */
#endif
	    IEEE80211_C_MONITOR |	/* Monitor mode supported. */
	    IEEE80211_C_SHSLOT |	/* Short slot time supported. */
	    IEEE80211_C_SHPREAMBLE |	/* Short preamble supported. */
	    IEEE80211_C_PMGT;		/* Power saving supported. */

	athn_config_ht(sc);

	/* Get the list of authorized/supported channels. */
	/* TODO: Is this function needed when athn_getradiocaps is available? */
	athn_get_chanlist(sc);

	athn_getradiocaps(ic, IEEE80211_CHAN_MAX, &ic->ic_nchans, ic->ic_channels);

	/* IBSS channel undefined for now. */
	// TODO missing 'ic_ibss_chan' in 'struct ieee80211com'
	// ic->ic_ibss_chan = &ic->ic_channels[0];
#if OpenBSD_IEEE80211_API
	ifp->if_softc = sc;
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_ioctl = athn_ioctl;
	ifp->if_start = athn_start;
#endif
	// TODO
	// ifp->if_watchdog = athn_watchdog;
	// TOTO missing field 'dv_xname' in 'struct device'
	// memcpy(ifp->if_xname, sc->sc_dev.dv_xname, IFNAMSIZ);

	// TODO missing members in 'struct ieee80211com'
	// if_attach(ifp);
	// ieee80211_ifattach(ifp);
	// ic->ic_node_alloc = athn_node_alloc;
	// ic->ic_newassoc = athn_newassoc;
	ic->ic_updateslot = athn_updateslot;
	ic->ic_getradiocaps = athn_getradiocaps;
	// ic->ic_updateedca = athn_updateedca;
	// ic->ic_set_key = athn_set_key;
	// ic->ic_delete_key = athn_delete_key;

	/* Override 802.11 state transition machine. */
	// sc->sc_newstate = ic->ic_newstate;
	// ic->ic_newstate = athn_newstate;
	// ieee80211_media_init(ifp, athn_media_change, ieee80211_media_status);

#if NBPFILTER > 0
	athn_radiotap_attach(sc);
#endif

	return (0);
}
#else
extern int		athn_usb_read_rom(struct athn_softc *);

static void
athn_scan_start(struct ieee80211com *ic)
{

//	printf("%s: TODO\n", __func__);
}

static void
athn_scan_end(struct ieee80211com *ic)
{

//	printf("%s: TODO\n", __func__);
}

// static int
// athn_raw_xmit(struct ieee80211_node *ni, struct mbuf *m,
//     const struct ieee80211_bpf_params *params)
// {
// // 	struct ieee80211com *ic= ni->ni_ic;
// // 	struct athn_softc *sc = ic->ic_softc;
// // 	struct otus_data *bf = NULL;
// // 	int error = 0;

// // 	/* Don't transmit if we're not running */
// // 	// OTUS_LOCK(sc);
// // 	// if (! sc->sc_running) {
// // 	// 	error = ENETDOWN;
// // 	// 	goto error;
// // 	// }

// // 	bf = otus_getbuf(sc);
// // 	if (bf == NULL) {
// // 		error = ENOBUFS;
// // 		goto error;
// // 	}

// // 	if (otus_tx(sc, ni, m, bf, params) != 0) {
// // 		error = EIO;
// // 		goto error;
// // 	}

// // 	OTUS_UNLOCK(sc);
// // 	return (0);
// // error:
// // 	if (bf)
// // 		otus_freebuf(sc, bf);
// // 	OTUS_UNLOCK(sc);
// // 	m_freem(m);
// // 	return (error);
// 	return 0;
// }

static void
athn_parent(struct ieee80211com *ic)
{
	struct otus_softc *sc = ic->ic_softc;
	// int startall = 0;
	ieee80211_start_all(ic);
}

int
athn_like_otus_attach(struct athn_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
	uint32_t in, out;
	uint8_t bands[IEEE80211_MODE_BYTES];
	int error;

	/* Read hardware revision. */
	athn_get_chipid(sc);

	ATHN_LOCK(sc);

	error = ar9285_attach(sc);


	sc->eep_base = AR9285_EEP_START_LOC;
	sc->eep_size = sizeof(struct ar9285_eeprom);

	if (error != 0) {
		device_printf(sc->sc_dev, "%s: could not attach chip\n", __func__);
		return (error);
	}

	sc->eep = &sc->eeprom;
	/* Read entire EEPROM. */
	if (athn_usb_read_rom(sc) != 0) {
		ATHN_UNLOCK(sc);
		device_printf(sc->sc_dev,
		    "%s: could not read EEPROM\n",
		    __func__);
		return (ENXIO);
	}

	ATHN_UNLOCK(sc);
	
	uint8_t txmask = sc->eeprom.baseEepHeader.txMask;
	uint8_t rxmask = sc->eeprom.baseEepHeader.rxMask;
	uint8_t capflags = sc->eeprom.baseEepHeader.opCapFlags;

	device_printf(sc->sc_dev,
		    "%s ic_macaddr = %s",
		    __func__, ether_sprintf(ic->ic_macaddr));

	IEEE80211_ADDR_COPY(ic->ic_macaddr, sc->eeprom.baseEepHeader.macAddr);

	if (txmask == 0x5)
		ic->ic_txstream = 2;
	else
		ic->ic_txstream = 1;

	if (rxmask == 0x5)
		ic->ic_rxstream = 2;
	else
		ic->ic_rxstream = 1;

#define AR5416_OPFLAGS_11A	0x01

	device_printf(sc->sc_dev,"%s: rx streams = %u, tx streams = %u\n", __func__, ic->ic_rxstream, ic->ic_txstream);

	device_printf(sc->sc_dev,
	    "MAC/BBP AR9271, RF AR%X, MIMO %dT%dR, address %s\n",
	    (capflags & AR5416_OPFLAGS_11A) ?
		0x9104 : ((txmask == 0x5) ? 0x9102 : 0x9101),
	    (txmask == 0x5) ? 2 : 1, (rxmask == 0x5) ? 2 : 1,
	    ether_sprintf(ic->ic_macaddr));

	ic->ic_softc = sc;
	ic->ic_name = device_get_nameunit(sc->sc_dev);
	ic->ic_phytype = IEEE80211_T_OFDM;	/* not only, but not used */
	ic->ic_opmode = IEEE80211_M_STA;	/* default to BSS mode */

	/* Set device capabilities. */
	ic->ic_caps =
	    IEEE80211_C_STA |		/* station mode */
#if 0
	    IEEE80211_C_BGSCAN |	/* Background scan. */
#endif
	    IEEE80211_C_SHPREAMBLE |	/* Short preamble supported. */
	    IEEE80211_C_WME |		/* WME/QoS */
	    IEEE80211_C_SHSLOT |	/* Short slot time supported. */
	    IEEE80211_C_FF |		/* Atheros fast-frames supported. */
	    IEEE80211_C_MONITOR |	/* Enable monitor mode */
	    IEEE80211_C_SWAMSDUTX |	/* Do software A-MSDU TX */
	    IEEE80211_C_WPA;		/* WPA/RSN. */

	ic->ic_htcaps =
	    IEEE80211_HTC_HT |
#if 0
	    IEEE80211_HTC_AMPDU |
#endif
	    IEEE80211_HTC_AMSDU |
	    IEEE80211_HTCAP_MAXAMSDU_3839 |
	    IEEE80211_HTCAP_SMPS_OFF;

	// athn_get_chanlist(sc);

	sc->flags |= ATHN_FLAG_11G;


	athn_getradiocaps(ic, IEEE80211_CHAN_MAX, &ic->ic_nchans,
	    ic->ic_channels);

	memset(bands, 0, sizeof(bands));
	setbit(bands, IEEE80211_MODE_11B);
	setbit(bands, IEEE80211_MODE_11G);

	ieee80211_init_channels(ic, NULL, bands);

	ieee80211_ifattach(ic);
	// ic->ic_raw_xmit = athn_raw_xmit;
	ic->ic_scan_start = athn_scan_start;
	ic->ic_scan_end = athn_scan_end;
	ic->ic_parent = athn_parent;
#if 0
	ic->ic_raw_xmit = otus_raw_xmit;
	ic->ic_scan_start = otus_scan_start;
	ic->ic_scan_end = otus_scan_end;
	ic->ic_set_channel = otus_set_channel;
#endif
	ic->ic_getradiocaps = athn_getradiocaps;
#if 0
	ic->ic_vap_create = otus_vap_create;
	ic->ic_vap_delete = otus_vap_delete;
	ic->ic_update_mcast = otus_update_mcast;
	ic->ic_update_promisc = otus_update_mcast;
	ic->ic_parent = otus_parent;
	ic->ic_transmit = otus_transmit;
	ic->ic_update_chw = otus_update_chw;
	ic->ic_ampdu_enable = otus_ampdu_enable;
	ic->ic_wme.wme_update = otus_updateedca;
	ic->ic_newassoc = otus_newassoc;
	ic->ic_node_alloc = otus_node_alloc;
	

#ifdef notyet
	ic->ic_set_key = otus_set_key;
	ic->ic_delete_key = otus_delete_key;
#endif

	ieee80211_radiotap_attach(ic, &sc->sc_txtap.wt_ihdr,
	    sizeof(sc->sc_txtap), OTUS_TX_RADIOTAP_PRESENT,
	    &sc->sc_rxtap.wr_ihdr, sizeof(sc->sc_rxtap),
	    OTUS_RX_RADIOTAP_PRESENT);
#endif
	sc->ntxchains =
	    ((sc->txchainmask >> 2) & 1) +
	    ((sc->txchainmask >> 1) & 1) +
	    ((sc->txchainmask >> 0) & 1);
	sc->nrxchains =
	    ((sc->rxchainmask >> 2) & 1) +
	    ((sc->rxchainmask >> 1) & 1) +
	    ((sc->rxchainmask >> 0) & 1);

	sc->amrr.amrr_min_success_threshold =  1;
	sc->amrr.amrr_max_success_threshold = 15;

	device_printf(sc->sc_dev,"%s: Planned end\n", __func__);

	return (0);
}
#endif

void
athn_detach(struct athn_softc *sc)
{
	// TODO no member named 'ic_if' in 'struct ieee80211com'
	// struct ifnet *ifp = &sc->sc_ic.ic_if;
	struct ifnet *ifp = NULL;
	int qid;
#if OpenBSD_ONLY
	timeout_del(&sc->scan_to);
	timeout_del(&sc->calib_to);
#endif
	if (!(sc->flags & ATHN_FLAG_USB)) {
		for (qid = 0; qid < ATHN_QID_COUNT; qid++)
			athn_tx_reclaim(sc, qid);

		/* Free Tx/Rx DMA resources. */
		sc->ops.dma_free(sc);
	}
	/* Free ROM copy. */
	if (sc->eep != NULL)
		// free(sc->eep, M_DEVBUF, 0);
		free(sc->eep, M_DEVBUF);

	// TODO
	// ieee80211_ifdetach(ifp);
#if OpenBSD_IEEE80211_API
	if_detach(ifp);
#endif
}

#if NBPFILTER > 0
/*
 * Attach the interface to 802.11 radiotap.
 */
void
athn_radiotap_attach(struct athn_softc *sc)
{
	bpfattach(&sc->sc_drvbpf, &sc->sc_ic.ic_if, DLT_IEEE802_11_RADIO,
	    sizeof(struct ieee80211_frame) + IEEE80211_RADIOTAP_HDRLEN);

	sc->sc_rxtap_len = sizeof(sc->sc_rxtapu);
	sc->sc_rxtap.wr_ihdr.it_len = htole16(sc->sc_rxtap_len);
	sc->sc_rxtap.wr_ihdr.it_present = htole32(ATHN_RX_RADIOTAP_PRESENT);

	sc->sc_txtap_len = sizeof(sc->sc_txtapu);
	sc->sc_txtap.wt_ihdr.it_len = htole16(sc->sc_txtap_len);
	sc->sc_txtap.wt_ihdr.it_present = htole32(ATHN_TX_RADIOTAP_PRESENT);
}
#endif

void
athn_get_chanlist(struct athn_softc *sc)
{
	__attribute__((unused)) struct ieee80211com *ic = &sc->sc_ic;
	uint8_t chan;
	int i;
	if (sc->flags & ATHN_FLAG_11G) {
		for (i = 1; i <= 14; i++) {
			chan = i;
			ic->ic_channels[chan].ic_freq =
			    ieee80211_ieee2mhz(chan, IEEE80211_CHAN_2GHZ);
			ic->ic_channels[chan].ic_flags =
			    IEEE80211_CHAN_CCK | IEEE80211_CHAN_OFDM |
			    IEEE80211_CHAN_DYN | IEEE80211_CHAN_2GHZ;
			if (sc->flags & ATHN_FLAG_11N)
				ic->ic_channels[chan].ic_flags |=
				    IEEE80211_CHAN_HT;
		}
	}
	if (sc->flags & ATHN_FLAG_11A) {
		for (i = 0; i < nitems(athn_5ghz_chans); i++) {
			chan = athn_5ghz_chans[i];
			ic->ic_channels[chan].ic_freq =
			    ieee80211_ieee2mhz(chan, IEEE80211_CHAN_5GHZ);
			ic->ic_channels[chan].ic_flags = IEEE80211_CHAN_A;
			if (sc->flags & ATHN_FLAG_11N)
				ic->ic_channels[chan].ic_flags |=
				    IEEE80211_CHAN_HT;
		}
	}
}

void
athn_rx_start(struct athn_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
	uint32_t rfilt;

	/* Setup Rx DMA descriptors. */
	sc->ops.rx_enable(sc);

	/* Set Rx filter. */
	rfilt = AR_RX_FILTER_UCAST | AR_RX_FILTER_BCAST | AR_RX_FILTER_MCAST;
	/* Want Compressed Block Ack Requests. */
	rfilt |= AR_RX_FILTER_COMPR_BAR;
	rfilt |= AR_RX_FILTER_BEACON;
	if (ic->ic_opmode != IEEE80211_M_STA) {
		rfilt |= AR_RX_FILTER_PROBEREQ;
		if (ic->ic_opmode == IEEE80211_M_MONITOR)
			rfilt |= AR_RX_FILTER_PROM;
#ifndef IEEE80211_STA_ONLY
		if (AR_SREV_9280_10_OR_LATER(sc) &&
		    ic->ic_opmode == IEEE80211_M_HOSTAP)
			rfilt |= AR_RX_FILTER_PSPOLL;
#endif
	}
	athn_set_rxfilter(sc, rfilt);

	/* Set BSSID mask. */
	AR_WRITE(sc, AR_BSSMSKL, 0xffffffff);
	AR_WRITE(sc, AR_BSSMSKU, 0xffff);

	athn_set_opmode(sc);

	/* Set multicast filter. */
	AR_WRITE(sc, AR_MCAST_FIL0, 0xffffffff);
	AR_WRITE(sc, AR_MCAST_FIL1, 0xffffffff);

	AR_WRITE(sc, AR_FILT_OFDM, 0);
	AR_WRITE(sc, AR_FILT_CCK, 0);
	AR_WRITE(sc, AR_MIBC, 0);
	AR_WRITE(sc, AR_PHY_ERR_MASK_1, AR_PHY_ERR_OFDM_TIMING);
	AR_WRITE(sc, AR_PHY_ERR_MASK_2, AR_PHY_ERR_CCK_TIMING);

	/* XXX ANI. */
	AR_WRITE(sc, AR_PHY_ERR_1, 0);
	AR_WRITE(sc, AR_PHY_ERR_2, 0);

	/* Disable HW crypto for now. */
	AR_SETBITS(sc, AR_DIAG_SW, AR_DIAG_ENCRYPT_DIS | AR_DIAG_DECRYPT_DIS);

	/* Start PCU Rx. */
	AR_CLRBITS(sc, AR_DIAG_SW, AR_DIAG_RX_DIS | AR_DIAG_RX_ABORT);
	AR_WRITE_BARRIER(sc);
}

void
athn_set_rxfilter(struct athn_softc *sc, uint32_t rfilt)
{
	AR_WRITE(sc, AR_RX_FILTER, rfilt);

#ifdef notyet
	reg = AR_READ(sc, AR_PHY_ERR);
	reg &= (AR_PHY_ERR_RADAR | AR_PHY_ERR_OFDM_TIMING |
	    AR_PHY_ERR_CCK_TIMING);
	AR_WRITE(sc, AR_PHY_ERR, reg);
	if (reg != 0)
		AR_SETBITS(sc, AR_RXCFG, AR_RXCFG_ZLFDMA);
	else
		AR_CLRBITS(sc, AR_RXCFG, AR_RXCFG_ZLFDMA);
#else
	AR_WRITE(sc, AR_PHY_ERR, 0);
	AR_CLRBITS(sc, AR_RXCFG, AR_RXCFG_ZLFDMA);
#endif
	AR_WRITE_BARRIER(sc);
}

int
athn_intr(void *xsc)
{
	struct athn_softc *sc = xsc;
	// TODO no member named 'ic_if' in 'struct ieee80211com'
	// struct ifnet *ifp = &sc->sc_ic.ic_if;
	struct ifnet *ifp = NULL;
#if OpenBSD_IEEE80211_API
	if ((ifp->if_flags & (IFF_UP | IFF_DRV_RUNNING)) !=
	    (IFF_UP | IFF_DRV_RUNNING))
		return (0);
#endif
	return (sc->ops.intr(sc));
}

void
athn_get_chipid(struct athn_softc *sc)
{
	uint32_t reg;

	ATHN_LOCK(sc);
	reg = AR_READ(sc, AR_SREV);
	ATHN_UNLOCK(sc);
	if (MS(reg, AR_SREV_ID) == 0xff) {
		sc->mac_ver = MS(reg, AR_SREV_VERSION2);
		sc->mac_rev = MS(reg, AR_SREV_REVISION2);
		if (!(reg & AR_SREV_TYPE2_HOST_MODE))
			sc->flags |= ATHN_FLAG_PCIE;
	} else {
		sc->mac_ver = MS(reg, AR_SREV_VERSION);
		sc->mac_rev = MS(reg, AR_SREV_REVISION);
		if (sc->mac_ver == AR_SREV_VERSION_5416_PCIE)
			sc->flags |= ATHN_FLAG_PCIE;
	}
}

const char *
athn_get_mac_name(struct athn_softc *sc)
{
	switch (sc->mac_ver) {
	case AR_SREV_VERSION_5416_PCI:
		return ("AR5416");
	case AR_SREV_VERSION_5416_PCIE:
		return ("AR5418");
	case AR_SREV_VERSION_9160:
		return ("AR9160");
	case AR_SREV_VERSION_9280:
		return ("AR9280");
	case AR_SREV_VERSION_9285:
		return ("AR9285");
	case AR_SREV_VERSION_9271:
		return ("AR9271");
	case AR_SREV_VERSION_9287:
		return ("AR9287");
	case AR_SREV_VERSION_9380:
		return ("AR9380");
	case AR_SREV_VERSION_9485:
		return ("AR9485");
	}
	return ("unknown");
}

/*
 * Return RF chip name (not for single-chip solutions).
 */
const char *
athn_get_rf_name(struct athn_softc *sc)
{
	// TODO improve output
	KASSERT(!AR_SINGLE_CHIP(sc), "athn_get_rf_name");

	switch (sc->rf_rev) {
	case AR_RAD5133_SREV_MAJOR:	/* Dual-band 3T3R. */
		return ("AR5133");
	case AR_RAD2133_SREV_MAJOR:	/* Single-band 3T3R. */
		return ("AR2133");
	case AR_RAD5122_SREV_MAJOR:	/* Dual-band 2T2R. */
		return ("AR5122");
	case AR_RAD2122_SREV_MAJOR:	/* Single-band 2T2R. */
		return ("AR2122");
	}
	return ("unknown");
}

int
athn_reset_power_on(struct athn_softc *sc)
{
	int ntries;

	ATHN_LOCK(sc);
	/* Set force wake. */
	AR_WRITE(sc, AR_RTC_FORCE_WAKE,
	    AR_RTC_FORCE_WAKE_EN | AR_RTC_FORCE_WAKE_ON_INT);

	if (!AR_SREV_9380_10_OR_LATER(sc)) {
		/* Make sure no DMA is active by doing an AHB reset. */
		AR_WRITE(sc, AR_RC, AR_RC_AHB);
	}
	/* RTC reset and clear. */
	AR_WRITE(sc, AR_RTC_RESET, 0);
	AR_WRITE_BARRIER(sc);
	DELAY(2);
	if (!AR_SREV_9380_10_OR_LATER(sc))
		AR_WRITE(sc, AR_RC, 0);
	AR_WRITE(sc, AR_RTC_RESET, 1);

	/* Poll until RTC is ON. */
	for (ntries = 0; ntries < 1000; ntries++) {
		if ((AR_READ(sc, AR_RTC_STATUS) & AR_RTC_STATUS_M) ==
		    AR_RTC_STATUS_ON)
			break;
		DELAY(10);
	}
	ATHN_UNLOCK(sc);
	if (ntries == 1000) {
		DPRINTF(("RTC not waking up\n"));
		return (ETIMEDOUT);
	}
	return (athn_reset(sc, 0));
}

int
athn_reset(struct athn_softc *sc, int cold)
{
	int ntries;

	ATHN_LOCK(sc);
	/* Set force wake. */
	AR_WRITE(sc, AR_RTC_FORCE_WAKE,
	    AR_RTC_FORCE_WAKE_EN | AR_RTC_FORCE_WAKE_ON_INT);

	if (AR_READ(sc, AR_INTR_SYNC_CAUSE) &
	    (AR_INTR_SYNC_LOCAL_TIMEOUT | AR_INTR_SYNC_RADM_CPL_TIMEOUT)) {
		AR_WRITE(sc, AR_INTR_SYNC_ENABLE, 0);
		AR_WRITE(sc, AR_RC, AR_RC_HOSTIF |
		    (!AR_SREV_9380_10_OR_LATER(sc) ? AR_RC_AHB : 0));
	} else if (!AR_SREV_9380_10_OR_LATER(sc))
		AR_WRITE(sc, AR_RC, AR_RC_AHB);

	AR_WRITE(sc, AR_RTC_RC, AR_RTC_RC_MAC_WARM |
	    (cold ? AR_RTC_RC_MAC_COLD : 0));
	AR_WRITE_BARRIER(sc);
	DELAY(50);
	AR_WRITE(sc, AR_RTC_RC, 0);
	for (ntries = 0; ntries < 1000; ntries++) {
		if (!(AR_READ(sc, AR_RTC_RC) &
		      (AR_RTC_RC_MAC_WARM | AR_RTC_RC_MAC_COLD)))
			break;
		DELAY(10);
	}
	if (ntries == 1000) {
		ATHN_UNLOCK(sc);
		DPRINTF(("RTC stuck in MAC reset\n"));
		return (ETIMEDOUT);
	}
	AR_WRITE(sc, AR_RC, 0);
	AR_WRITE_BARRIER(sc);
	ATHN_UNLOCK(sc);
	return (0);
}

int
athn_set_power_awake(struct athn_softc *sc)
{
	int ntries, error;

	ATHN_LOCK(sc);
	/* Do a Power-On-Reset if shutdown. */
	if ((AR_READ(sc, AR_RTC_STATUS) & AR_RTC_STATUS_M) ==
	    AR_RTC_STATUS_SHUTDOWN) {
		if ((error = athn_reset_power_on(sc)) != 0) {
			ATHN_UNLOCK(sc);
			return (error);
		}
		if (!AR_SREV_9380_10_OR_LATER(sc))
			athn_init_pll(sc, NULL);
	}
	AR_SETBITS(sc, AR_RTC_FORCE_WAKE, AR_RTC_FORCE_WAKE_EN);
	AR_WRITE_BARRIER(sc);
	DELAY(50);	/* Give chip the chance to awake. */

	/* Poll until RTC is ON. */
	for (ntries = 0; ntries < 4000; ntries++) {
		if ((AR_READ(sc, AR_RTC_STATUS) & AR_RTC_STATUS_M) ==
		    AR_RTC_STATUS_ON)
			break;
		DELAY(50);
		AR_SETBITS(sc, AR_RTC_FORCE_WAKE, AR_RTC_FORCE_WAKE_EN);
	}
	if (ntries == 4000) {
		DPRINTF(("RTC not waking up\n"));
		ATHN_UNLOCK(sc);
		return (ETIMEDOUT);
	}

	AR_CLRBITS(sc, AR_STA_ID1, AR_STA_ID1_PWR_SAV);
	AR_WRITE_BARRIER(sc);
	ATHN_UNLOCK(sc);
	return (0);
}

void
athn_set_power_sleep(struct athn_softc *sc)
{
	ATHN_LOCK(sc);
	AR_SETBITS(sc, AR_STA_ID1, AR_STA_ID1_PWR_SAV);
	/* Allow the MAC to go to sleep. */
	AR_CLRBITS(sc, AR_RTC_FORCE_WAKE, AR_RTC_FORCE_WAKE_EN);
	if (!AR_SREV_9380_10_OR_LATER(sc))
		AR_WRITE(sc, AR_RC, AR_RC_AHB | AR_RC_HOSTIF);
	/*
	 * NB: Clearing RTC_RESET_EN when setting the chip to sleep mode
	 * results in high power consumption on AR5416 chipsets.
	 */
	if (!AR_SREV_5416(sc) && !AR_SREV_9271(sc))
		AR_CLRBITS(sc, AR_RTC_RESET, AR_RTC_RESET_EN);
	AR_WRITE_BARRIER(sc);
	ATHN_UNLOCK(sc);
}

void
athn_init_pll(struct athn_softc *sc, const struct ieee80211_channel *c)
{
	uint32_t pll;

	ATHN_LOCK(sc);
	if (AR_SREV_9380_10_OR_LATER(sc)) {
		if (AR_SREV_9485(sc))
			AR_WRITE(sc, AR_RTC_PLL_CONTROL2, 0x886666);
		pll = SM(AR_RTC_9160_PLL_REFDIV, 0x5);
		pll |= SM(AR_RTC_9160_PLL_DIV, 0x2c);
	} else if (AR_SREV_9280_10_OR_LATER(sc)) {
		pll = SM(AR_RTC_9160_PLL_REFDIV, 0x05);
		if (c != NULL && IEEE80211_IS_CHAN_5GHZ(c)) {
			if (sc->flags & ATHN_FLAG_FAST_PLL_CLOCK)
				pll = 0x142c;
			else if (AR_SREV_9280_20(sc))
		 		pll = 0x2850;
			else
				pll |= SM(AR_RTC_9160_PLL_DIV, 0x28);
		} else
			pll |= SM(AR_RTC_9160_PLL_DIV, 0x2c);
	} else if (AR_SREV_9160_10_OR_LATER(sc)) {
		pll = SM(AR_RTC_9160_PLL_REFDIV, 0x05);
		if (c != NULL && IEEE80211_IS_CHAN_5GHZ(c))
			pll |= SM(AR_RTC_9160_PLL_DIV, 0x50);
		else
			pll |= SM(AR_RTC_9160_PLL_DIV, 0x58);
	} else {
		pll = AR_RTC_PLL_REFDIV_5 | AR_RTC_PLL_DIV2;
		if (c != NULL && IEEE80211_IS_CHAN_5GHZ(c))
			pll |= SM(AR_RTC_PLL_DIV, 0x0a);
		else
			pll |= SM(AR_RTC_PLL_DIV, 0x0b);
	}
	DPRINTFN(5, ("AR_RTC_PLL_CONTROL=0x%08x\n", pll));
	AR_WRITE(sc, AR_RTC_PLL_CONTROL, pll);
	if (AR_SREV_9271(sc)) {
		/* Switch core clock to 117MHz. */
		AR_WRITE_BARRIER(sc);
		DELAY(500);
		AR_WRITE(sc, AR9271_CLOCK_CONTROL, 0x304);
	}
	AR_WRITE_BARRIER(sc);
	DELAY(100);
	AR_WRITE(sc, AR_RTC_SLEEP_CLK, AR_RTC_FORCE_DERIVED_CLK);
	AR_WRITE_BARRIER(sc);
	ATHN_UNLOCK(sc);
}

void
athn_write_serdes(struct athn_softc *sc, const struct athn_serdes *serdes)
{
	int i;

	/* Write sequence to Serializer/Deserializer. */
	// ATHN_LOCK(sc);
	for (i = 0; i < serdes->nvals; i++)
		AR_WRITE(sc, serdes->regs[i], serdes->vals[i]);
	AR_WRITE_BARRIER(sc);
	// ATHN_UNLOCK(sc);
}

void
athn_config_pcie(struct athn_softc *sc)
{
	/* Disable PLL when in L0s as well as receiver clock when in L1. */
	athn_write_serdes(sc, sc->serdes);

	DELAY(1000);
	ATHN_LOCK(sc);
	/* Allow forcing of PCIe core into L1 state. */
	AR_SETBITS(sc, AR_PCIE_PM_CTRL, AR_PCIE_PM_CTRL_ENA);

#ifndef ATHN_PCIE_WAEN
	AR_WRITE(sc, AR_WA, sc->workaround);
#else
	AR_WRITE(sc, AR_WA, ATHN_PCIE_WAEN);
#endif
	AR_WRITE_BARRIER(sc);
	ATHN_UNLOCK(sc);
}

/*
 * Serializer/Deserializer programming for non-PCIe devices.
 */
static const uint32_t ar_nonpcie_serdes_regs[] = {
	AR_PCIE_SERDES,
	AR_PCIE_SERDES,
	AR_PCIE_SERDES,
	AR_PCIE_SERDES,
	AR_PCIE_SERDES,
	AR_PCIE_SERDES,
	AR_PCIE_SERDES,
	AR_PCIE_SERDES,
	AR_PCIE_SERDES,
	AR_PCIE_SERDES2,
};

static const uint32_t ar_nonpcie_serdes_vals[] = {
	0x9248fc00,
	0x24924924,
	0x28000029,
	0x57160824,
	0x25980579,
	0x00000000,
	0x1aaabe40,
	0xbe105554,
	0x000e1007,
	0x00000000
};

static const struct athn_serdes ar_nonpcie_serdes = {
	nitems(ar_nonpcie_serdes_vals),
	ar_nonpcie_serdes_regs,
	ar_nonpcie_serdes_vals
};

void
athn_config_nonpcie(struct athn_softc *sc)
{
	athn_write_serdes(sc, &ar_nonpcie_serdes);
}

int
athn_set_chan(struct athn_softc *sc, struct ieee80211_channel *c,
    struct ieee80211_channel *extc)
{
	struct athn_ops *ops = &sc->ops;
	int error, qid;

	/* Check that Tx is stopped, otherwise RF Bus grant will not work. */
	for (qid = 0; qid < ATHN_QID_COUNT; qid++)
		if (athn_tx_pending(sc, qid))
			return (EBUSY);

	/* Request RF Bus grant. */
	if ((error = ops->rf_bus_request(sc)) != 0)
		return (error);

	ops->set_phy(sc, c, extc);

	/* Change the synthesizer. */
	if ((error = ops->set_synth(sc, c, extc)) != 0)
		return (error);

	sc->curchan = c;
	sc->curchanext = extc;

	/* Set transmit power values for new channel. */
	// ops->set_txpower(sc, c, extc);

	/* Release the RF Bus grant. */
	// ops->rf_bus_release(sc);

	/* Write delta slope coeffs for modes where OFDM may be used. */
	// if (sc->sc_ic.ic_curmode != IEEE80211_MODE_11B)
	// 	ops->set_delta_slope(sc, c, extc);

	// ops->spur_mitigate(sc, c, extc);

	return (0);
}

int
athn_switch_chan(struct athn_softc *sc, struct ieee80211_channel *c,
    struct ieee80211_channel *extc)
{
	int error, qid;

	/* Disable interrupts. */
	athn_disable_interrupts(sc);

	/* Stop all Tx queues. */
	for (qid = 0; qid < ATHN_QID_COUNT; qid++)
		athn_stop_tx_dma(sc, qid);
	for (qid = 0; qid < ATHN_QID_COUNT; qid++)
		athn_tx_reclaim(sc, qid);

	/* Stop Rx. */
	AR_SETBITS(sc, AR_DIAG_SW, AR_DIAG_RX_DIS | AR_DIAG_RX_ABORT);
	AR_WRITE(sc, AR_MIBC, AR_MIBC_FMC);
	AR_WRITE(sc, AR_MIBC, AR_MIBC_CMC);
	AR_WRITE(sc, AR_FILT_OFDM, 0);
	AR_WRITE(sc, AR_FILT_CCK, 0);
	athn_set_rxfilter(sc, 0);
	error = athn_stop_rx_dma(sc);
	if (error != 0)
		goto reset;

#ifdef notyet
	/* AR9280 needs a full reset. */
	if (AR_SREV_9280(sc))
#endif
		goto reset;

	/* If band or bandwidth changes, we need to do a full reset. */
	if (c->ic_flags != sc->curchan->ic_flags ||
	    ((extc != NULL) ^ (sc->curchanext != NULL))) {
		DPRINTFN(2, ("channel band switch\n"));
		goto reset;
	}
	error = athn_set_power_awake(sc);
	if (error != 0)
		goto reset;

	error = athn_set_chan(sc, c, extc);
	if (error != 0) {
 reset:		/* Error found, try a full reset. */
		DPRINTFN(3, ("needs a full reset\n"));
		error = athn_hw_reset(sc, c, extc, 0);
		if (error != 0)	/* Hopeless case. */
			return (error);
	}
	athn_rx_start(sc);

	/* Re-enable interrupts. */
	athn_enable_interrupts(sc);
	return (0);
}

void
athn_get_delta_slope(uint32_t coeff, uint32_t *exponent, uint32_t *mantissa)
{
#define COEFF_SCALE_SHIFT	24
	uint32_t exp, man;

	/* exponent = 14 - floor(log2(coeff)) */
	for (exp = 31; exp > 0; exp--)
		if (coeff & (1 << exp))
			break;
	exp = 14 - (exp - COEFF_SCALE_SHIFT);

	/* mantissa = floor(coeff * 2^exponent + 0.5) */
	man = coeff + (1 << (COEFF_SCALE_SHIFT - exp - 1));

	*mantissa = man >> (COEFF_SCALE_SHIFT - exp);
	*exponent = exp - 16;
#undef COEFF_SCALE_SHIFT
}

void
athn_reset_key(struct athn_softc *sc, int entry)
{
	/*
	 * NB: Key cache registers access special memory area that requires
	 * two 32-bit writes to actually update the values in the internal
	 * memory.  Consequently, writes must be grouped by pair.
	 *
	 * All writes to registers with an offset of 0x0 or 0x8 write to a
	 * temporary register. A write to a register with an offset of 0x4
	 * or 0xc writes concatenates the written value with the value in
	 * the temporary register and writes the result to key cache memory.
	 * The actual written memory area is 50 bits wide.
	 */
	AR_WRITE(sc, AR_KEYTABLE_KEY0(entry), 0);
	AR_WRITE(sc, AR_KEYTABLE_KEY1(entry), 0);

	AR_WRITE(sc, AR_KEYTABLE_KEY2(entry), 0);
	AR_WRITE(sc, AR_KEYTABLE_KEY3(entry), 0);

	AR_WRITE(sc, AR_KEYTABLE_KEY4(entry), 0);
	AR_WRITE(sc, AR_KEYTABLE_TYPE(entry), AR_KEYTABLE_TYPE_CLR);

	AR_WRITE(sc, AR_KEYTABLE_MAC0(entry), 0);
	AR_WRITE(sc, AR_KEYTABLE_MAC1(entry), 0);

	AR_WRITE_BARRIER(sc);
}

int
athn_set_key(struct ieee80211com *ic, struct ieee80211_node *ni,
    struct ieee80211_key *k)
{
	struct athn_softc *sc = ic->ic_softc;
	const uint8_t *key = NULL;
	// const uint8_t *addr;
	uintptr_t entry;
	uint32_t lo, hi, unicast;
	lo = 0; hi = 0; // TODO remove

	// TODO no member named 'k_cipher' in 'struct ieee80211_key'
	// if (k->k_cipher != IEEE80211_CIPHER_CCMP) {
	// 	/* Use software crypto for ciphers other than CCMP. */
	// 	return ieee80211_set_key(ic, ni, k);
	// }

	// TODO no member named 'k_flags' in 'struct ieee80211_key'
// 	if (!(k->k_flags & IEEE80211_KEY_GROUP)) {
// #ifndef IEEE80211_STA_ONLY
// 		if (ic->ic_opmode == IEEE80211_M_HOSTAP)
// 			entry = IEEE80211_WEP_NKID + IEEE80211_AID(ni->ni_associd);
// 		else
// #endif
// 			entry = IEEE80211_WEP_NKID;
// 		if (entry >= sc->kc_entries - IEEE80211_WEP_NKID)
// 			return ENOSPC;
// 	} else {
// 		entry = k->k_id;
// 		if (entry >= IEEE80211_WEP_NKID)
// 			return ENOSPC;
// 	}
	// TODO no member named 'k_priv' in 'struct ieee80211_key'
	// k->k_priv = (void *)entry;

	/* NB: See note about key cache registers access above. */
	// TODO no member named 'k_key' in 'struct ieee80211_key'
	// key = k->k_key;
	entry = 0; // TODO remove

	AR_WRITE(sc, AR_KEYTABLE_KEY0(entry), LE_READ_4(&key[ 0]));
	AR_WRITE(sc, AR_KEYTABLE_KEY1(entry), LE_READ_2(&key[ 4]));

	AR_WRITE(sc, AR_KEYTABLE_KEY2(entry), LE_READ_4(&key[ 6]));
	AR_WRITE(sc, AR_KEYTABLE_KEY3(entry), LE_READ_2(&key[10]));

	AR_WRITE(sc, AR_KEYTABLE_KEY4(entry), LE_READ_4(&key[12]));
	AR_WRITE(sc, AR_KEYTABLE_TYPE(entry), AR_KEYTABLE_TYPE_CCM);

	unicast = AR_KEYTABLE_VALID;
	// TODO no member named 'k_flags' in 'struct ieee80211_key'
// 	if (!(k->k_flags & IEEE80211_KEY_GROUP)) {
// 		addr = ni->ni_macaddr;
// 		lo = LE_READ_4(&addr[0]);
// 		hi = LE_READ_2(&addr[4]);
// 		lo = lo >> 1 | hi << 31;
// 		hi = hi >> 1;
// 	} else {
// #ifndef IEEE80211_STA_ONLY
// 		if (ic->ic_opmode == IEEE80211_M_HOSTAP) {
// 			uint8_t groupaddr[ETHER_ADDR_LEN];
// 			IEEE80211_ADDR_COPY(groupaddr, ic->ic_myaddr);
// 			groupaddr[0] |= 0x01;
// 			lo = LE_READ_4(&groupaddr[0]);
// 			hi = LE_READ_2(&groupaddr[4]);
// 			lo = lo >> 1 | hi << 31;
// 			hi = hi >> 1;
// 			/*
// 			 * KEYTABLE_VALID indicates that the address
// 			 * is a unicast address which must match the
// 			 * transmitter address when decrypting frames.
// 			 * Not setting KEYTABLE_VALID allows hardware to
// 			 * use this key for multicast frame decryption.
// 			 */
// 			unicast = 0;
// 		} else
// #endif
// 			lo = hi = 0;
// 	}
	AR_WRITE(sc, AR_KEYTABLE_MAC0(entry), lo);
	AR_WRITE(sc, AR_KEYTABLE_MAC1(entry), hi | unicast);

	AR_WRITE_BARRIER(sc);

	/* Enable HW crypto. */
	AR_CLRBITS(sc, AR_DIAG_SW, AR_DIAG_ENCRYPT_DIS | AR_DIAG_DECRYPT_DIS);

	AR_WRITE_BARRIER(sc);
	return (0);
}

void
athn_delete_key(struct ieee80211com *ic, struct ieee80211_node *ni,
    struct ieee80211_key *k)
{
	// struct athn_softc *sc = ic->ic_softc;
	// uintptr_t entry;

	// TODO no member named 'k_cipher' in 'struct ieee80211_key'
	// if (k->k_cipher == IEEE80211_CIPHER_CCMP) {
	// 	entry = (uintptr_t)k->k_priv;
	// 	athn_reset_key(sc, entry);
	// 	explicit_bzero(k, sizeof(*k));
	// } else
	// 	ieee80211_delete_key(ic, ni, k);
}

void
athn_led_init(struct athn_softc *sc)
{
	struct athn_ops *ops = &sc->ops;

	ops->gpio_config_output(sc, sc->led_pin, AR_GPIO_OUTPUT_MUX_AS_OUTPUT);
	/* LED off, active low. */
	athn_set_led(sc, 0);
}

void
athn_set_led(struct athn_softc *sc, int on)
{
	struct athn_ops *ops = &sc->ops;

	// sc->led_state = on;
	// ops->gpio_write(sc, sc->led_pin, !sc->led_state);
}

#ifdef ATHN_BT_COEXISTENCE
void
athn_btcoex_init(struct athn_softc *sc)
{
	struct athn_ops *ops = &sc->ops;
	uint32_t reg;

	if (sc->flags & ATHN_FLAG_BTCOEX2WIRE) {
		/* Connect bt_active to baseband. */
		AR_CLRBITS(sc, sc->gpio_input_en_off,
		    AR_GPIO_INPUT_EN_VAL_BT_PRIORITY_DEF |
		    AR_GPIO_INPUT_EN_VAL_BT_FREQUENCY_DEF);
		AR_SETBITS(sc, sc->gpio_input_en_off,
		    AR_GPIO_INPUT_EN_VAL_BT_ACTIVE_BB);

		reg = AR_READ(sc, AR_GPIO_INPUT_MUX1);
		reg = RW(reg, AR_GPIO_INPUT_MUX1_BT_ACTIVE,
		    AR_GPIO_BTACTIVE_PIN);
		AR_WRITE(sc, AR_GPIO_INPUT_MUX1, reg);
		AR_WRITE_BARRIER(sc);

		ops->gpio_config_input(sc, AR_GPIO_BTACTIVE_PIN);
	} else {	/* 3-wire. */
		AR_SETBITS(sc, sc->gpio_input_en_off,
		    AR_GPIO_INPUT_EN_VAL_BT_PRIORITY_BB |
		    AR_GPIO_INPUT_EN_VAL_BT_ACTIVE_BB);

		reg = AR_READ(sc, AR_GPIO_INPUT_MUX1);
		reg = RW(reg, AR_GPIO_INPUT_MUX1_BT_ACTIVE,
		    AR_GPIO_BTACTIVE_PIN);
		reg = RW(reg, AR_GPIO_INPUT_MUX1_BT_PRIORITY,
		    AR_GPIO_BTPRIORITY_PIN);
		AR_WRITE(sc, AR_GPIO_INPUT_MUX1, reg);
		AR_WRITE_BARRIER(sc);

		ops->gpio_config_input(sc, AR_GPIO_BTACTIVE_PIN);
		ops->gpio_config_input(sc, AR_GPIO_BTPRIORITY_PIN);
	}
}

void
athn_btcoex_enable(struct athn_softc *sc)
{
	struct athn_ops *ops = &sc->ops;
	uint32_t reg;

	if (sc->flags & ATHN_FLAG_BTCOEX3WIRE) {
		AR_WRITE(sc, AR_BT_COEX_MODE,
		    SM(AR_BT_MODE, AR_BT_MODE_SLOTTED) |
		    SM(AR_BT_PRIORITY_TIME, 2) |
		    SM(AR_BT_FIRST_SLOT_TIME, 5) |
		    SM(AR_BT_QCU_THRESH, ATHN_QID_AC_BE) |
		    AR_BT_TXSTATE_EXTEND | AR_BT_TX_FRAME_EXTEND |
		    AR_BT_QUIET | AR_BT_RX_CLEAR_POLARITY);
		AR_WRITE(sc, AR_BT_COEX_WEIGHT,
		    SM(AR_BTCOEX_BT_WGHT, AR_STOMP_LOW_BT_WGHT) |
		    SM(AR_BTCOEX_WL_WGHT, AR_STOMP_LOW_WL_WGHT));
		AR_WRITE(sc, AR_BT_COEX_MODE2,
		    SM(AR_BT_BCN_MISS_THRESH, 50) |
		    AR_BT_HOLD_RX_CLEAR | AR_BT_DISABLE_BT_ANT);

		AR_SETBITS(sc, AR_QUIET1, AR_QUIET1_QUIET_ACK_CTS_ENABLE);
		AR_CLRBITS(sc, AR_PCU_MISC, AR_PCU_BT_ANT_PREVENT_RX);
		AR_WRITE_BARRIER(sc);

		ops->gpio_config_output(sc, AR_GPIO_WLANACTIVE_PIN,
		    AR_GPIO_OUTPUT_MUX_AS_RX_CLEAR_EXTERNAL);

	} else {	/* 2-wire. */
		ops->gpio_config_output(sc, AR_GPIO_WLANACTIVE_PIN,
		    AR_GPIO_OUTPUT_MUX_AS_TX_FRAME);
	}
	reg = AR_READ(sc, AR_GPIO_PDPU);
	reg &= ~(0x3 << (AR_GPIO_WLANACTIVE_PIN * 2));
	reg |= 0x2 << (AR_GPIO_WLANACTIVE_PIN * 2);
	AR_WRITE(sc, AR_GPIO_PDPU, reg);
	AR_WRITE_BARRIER(sc);

	/* Disable PCIe Active State Power Management (ASPM). */
	if (sc->sc_disable_aspm != NULL)
		sc->sc_disable_aspm(sc);

	/* XXX Start periodic timer. */
}

void
athn_btcoex_disable(struct athn_softc *sc)
{
	struct athn_ops *ops = &sc->ops;

	ops->gpio_write(sc, AR_GPIO_WLANACTIVE_PIN, 0);

	ops->gpio_config_output(sc, AR_GPIO_WLANACTIVE_PIN,
	    AR_GPIO_OUTPUT_MUX_AS_OUTPUT);

	if (sc->flags & ATHN_FLAG_BTCOEX3WIRE) {
		AR_WRITE(sc, AR_BT_COEX_MODE,
		    SM(AR_BT_MODE, AR_BT_MODE_DISABLED) | AR_BT_QUIET);
		AR_WRITE(sc, AR_BT_COEX_WEIGHT, 0);
		AR_WRITE(sc, AR_BT_COEX_MODE2, 0);
		/* XXX Stop periodic timer. */
	}
	AR_WRITE_BARRIER(sc);
	/* XXX Restore ASPM setting? */
}
#endif

void
athn_iter_calib(void *arg, struct ieee80211_node *ni)
{
	// struct athn_softc *sc = arg;
	// struct athn_node *an = (struct athn_node *)ni;

	// TODO implicit declaration of function 'ieee80211_amrr_choose'
	// if ((ni->ni_flags & IEEE80211_NODE_HT) == 0)
	// 	ieee80211_amrr_choose(&sc->amrr, ni, &an->amn);
}

int
athn_cap_noisefloor(struct athn_softc *sc, int nf)
{
	int16_t min, max;
	min = max = 0; //TODO remove

	if (nf == 0 || nf == -1) /* invalid measurement */
		return AR_DEFAULT_NOISE_FLOOR;

	// TODO no member named 'ic_bss' in 'struct ieee80211com'
	// if (IEEE80211_IS_CHAN_2GHZ(sc->sc_ic.ic_bss->ni_chan)) {
	// 	min = sc->cca_min_2g;
	// 	max = sc->cca_max_2g;
	// } else {
	// 	min = sc->cca_min_5g;
	// 	max = sc->cca_max_5g;
	// }

	if (nf < min)
		return min;
	if (nf > max)
		return max;

	return nf;
}

int
athn_nf_hist_mid(int *nf_vals, int nvalid)
{
	int nf_sorted[ATHN_NF_CAL_HIST_MAX];
	int i, j, nf;

	if (nvalid <= 1)
		return nf_vals[0];

	for (i = 0; i < nvalid; i++)
		nf_sorted[i] = nf_vals[i];

	for (i = 0; i < nvalid; i++) {
		for (j = 1; j < nvalid - i; j++) {
			if (nf_sorted[j] > nf_sorted[j - 1]) {
				nf = nf_sorted[j];
				nf_sorted[j] = nf_sorted[j - 1];
				nf_sorted[j - 1] = nf;
			}
		}
	}

	return nf_sorted[nvalid / 2];
}

void
athn_filter_noisefloor(struct athn_softc *sc)
{
	int nf_vals[ATHN_NF_CAL_HIST_MAX];
	int nf_ext_vals[ATHN_NF_CAL_HIST_MAX];
	int i, cur, n;

	for (i = 0; i < sc->nrxchains; i++) {
		if (sc->nf_hist_cur > 0)
			cur = sc->nf_hist_cur - 1;
		else
			cur = ATHN_NF_CAL_HIST_MAX - 1;
		for (n = 0; n < sc->nf_hist_nvalid; n++) {
			nf_vals[n] = sc->nf_hist[cur].nf[i];
			nf_ext_vals[n] = sc->nf_hist[cur].nf_ext[i];
			if (++cur >= ATHN_NF_CAL_HIST_MAX)
				cur = 0;
		}
		sc->nf_priv[i] = athn_cap_noisefloor(sc,
		    athn_nf_hist_mid(nf_vals, sc->nf_hist_nvalid));
		sc->nf_ext_priv[i] = athn_cap_noisefloor(sc,
		    athn_nf_hist_mid(nf_ext_vals, sc->nf_hist_nvalid));
	}
}

void
athn_start_noisefloor_calib(struct athn_softc *sc, int reset_history)
{
	extern volatile int ticks;

	if (reset_history)
		sc->nf_hist_nvalid = 0;

	sc->nf_calib_pending = 1;
	sc->nf_calib_ticks = ticks;

	sc->ops.noisefloor_calib(sc);
}

void
athn_calib_to(void *arg)
{
	extern volatile int ticks;
	struct athn_softc *sc = arg;
	struct athn_ops *ops = &sc->ops;
	__attribute__((unused)) struct ieee80211com *ic = &sc->sc_ic;
	int s;

	s = splnet();

	/* Do periodic (every 4 minutes) PA calibration. */
	if ((ticks - (sc->pa_calib_ticks + 240 * hz)) >= 0) {
		sc->pa_calib_ticks = ticks;
		if (AR_SREV_9271(sc))
			ar9271_pa_calib(sc);
	}

	/* Do periodic (every 4 minutes) NF calibration. */
	if (sc->nf_calib_pending && ops->get_noisefloor(sc)) {
		if (sc->nf_hist_nvalid < ATHN_NF_CAL_HIST_MAX)
			sc->nf_hist_nvalid++;
		athn_filter_noisefloor(sc);
		ops->apply_noisefloor(sc);
		sc->nf_calib_pending = 0;
	}
	if (ticks - (sc->nf_calib_ticks + 240 * hz) >= 0)
		athn_start_noisefloor_calib(sc, 0);

	/* Do periodic (every 30 seconds) temperature compensation. */
	if ((sc->flags & ATHN_FLAG_OLPC) &&
	    ticks >= sc->olpc_ticks + 30 * hz) {
		sc->olpc_ticks = ticks;
		ops->olpc_temp_compensation(sc);
	}

#ifdef notyet
	/* XXX ANI. */
	athn_ani_monitor(sc);
#endif

	/* Do periodic (every 30 seconds) ADC/IQ calibration. */
	if (sc->cur_calib_mask != 0) {
		ops->next_calib(sc);
		sc->iqcal_ticks = ticks;
	} else if (sc->sup_calib_mask != 0 &&
	    ticks >= sc->iqcal_ticks + 30 * hz) {
		memset(&sc->calib, 0, sizeof(sc->calib));
		sc->cur_calib_mask = sc->sup_calib_mask;
		ops->do_calib(sc);
		sc->iqcal_ticks = ticks;
	}

	// TODO no member named 'ic_fixed_rate' in 'struct ieee80211com'
	// if (ic->ic_fixed_rate == -1) {
	// 	if (ic->ic_opmode == IEEE80211_M_STA)
	// 		athn_iter_calib(sc, ic->ic_bss);
	// 	else
	// 		ieee80211_iterate_nodes(ic, athn_iter_calib, sc);
	// }
#if OpenBSD_ONLY
	timeout_add_msec(&sc->calib_to, 500);
#endif
	splx(s);
}

int
athn_init_calib(struct athn_softc *sc, struct ieee80211_channel *c,
    struct ieee80211_channel *extc)
{
	struct athn_ops *ops = &sc->ops;
	int error;

	if (AR_SREV_9285_10_OR_LATER(sc))
		error = ar9285_init_calib(sc, c, extc);
	if (error != 0)
		return (error);

	if (AR_SREV_9285_11_OR_LATER(sc)) {
		extern volatile int ticks;
		sc->pa_calib_ticks = ticks;
		if (AR_SREV_9271(sc))
			ar9271_pa_calib(sc);
		}

	/* Do noisefloor calibration. */
	ops->init_noisefloor_calib(sc);

	if (AR_SREV_9160_10_OR_LATER(sc)) {
		/* Support IQ calibration. */
		sc->sup_calib_mask = ATHN_CAL_IQ;
		if (AR_SREV_9380_10_OR_LATER(sc)) {
			/* Support temperature compensation calibration. */
			sc->sup_calib_mask |= ATHN_CAL_TEMP;
		} else if (IEEE80211_IS_CHAN_5GHZ(c) || extc != NULL) {
			/*
			 * ADC gain calibration causes uplink throughput
			 * drops in HT40 mode on AR9287.
			 */
			if (!AR_SREV_9287(sc)) {
				/* Support ADC gain calibration. */
				sc->sup_calib_mask |= ATHN_CAL_ADC_GAIN;
			}
			/* Support ADC DC offset calibration. */
			sc->sup_calib_mask |= ATHN_CAL_ADC_DC;
		}
	}
	return (0);
}

/*
 * Adaptive noise immunity.
 */
int32_t
athn_ani_get_rssi(struct athn_softc *sc)
{
	return (0);	/* XXX */
}

void
athn_ani_ofdm_err_trigger(struct athn_softc *sc)
{
	struct athn_ani *ani = &sc->ani;
	struct athn_ops *ops = &sc->ops;
	int32_t rssi;

	/* First, raise noise immunity level, up to max. */
	if (ani->noise_immunity_level < 4) {
		ani->noise_immunity_level++;
		ops->set_noise_immunity_level(sc, ani->noise_immunity_level);
		return;
	}

	/* Then, raise our spur immunity level, up to max. */
	if (ani->spur_immunity_level < 7) {
		ani->spur_immunity_level++;
		ops->set_spur_immunity_level(sc, ani->spur_immunity_level);
		return;
	}

#ifndef IEEE80211_STA_ONLY
	if (sc->sc_ic.ic_opmode == IEEE80211_M_HOSTAP) {
		if (ani->firstep_level < 2) {
			ani->firstep_level++;
			ops->set_firstep_level(sc, ani->firstep_level);
		}
		return;
	}
#endif
	rssi = athn_ani_get_rssi(sc);
	if (rssi > ATHN_ANI_RSSI_THR_HIGH) {
		/*
		 * Beacon RSSI is high, turn off OFDM weak signal detection
		 * or raise first step level as last resort.
		 */
		if (ani->ofdm_weak_signal) {
			ani->ofdm_weak_signal = 0;
			ops->disable_ofdm_weak_signal(sc);
			ani->spur_immunity_level = 0;
			ops->set_spur_immunity_level(sc, 0);
		} else if (ani->firstep_level < 2) {
			ani->firstep_level++;
			ops->set_firstep_level(sc, ani->firstep_level);
		}
	} else if (rssi > ATHN_ANI_RSSI_THR_LOW) {
		/*
		 * Beacon RSSI is in mid range, we need OFDM weak signal
		 * detection but we can raise first step level.
		 */
		if (!ani->ofdm_weak_signal) {
			ani->ofdm_weak_signal = 1;
			ops->enable_ofdm_weak_signal(sc);
		}
		if (ani->firstep_level < 2) {
			ani->firstep_level++;
			ops->set_firstep_level(sc, ani->firstep_level);
		}
	} // TODO no member named 'ic_bss' in 'struct ieee80211com'
	// } else if (IEEE80211_IS_CHAN_2GHZ(sc->sc_ic.ic_bss->ni_chan)) {
	// 	/*
	// 	 * Beacon RSSI is low, if in b/g mode, turn off OFDM weak
	// 	 * signal detection and zero first step level to maximize
	// 	 * CCK sensitivity.
	// 	 */
	// 	if (ani->ofdm_weak_signal) {
	// 		ani->ofdm_weak_signal = 0;
	// 		ops->disable_ofdm_weak_signal(sc);
	// 	}
	// 	if (ani->firstep_level > 0) {
	// 		ani->firstep_level = 0;
	// 		ops->set_firstep_level(sc, 0);
	// 	}
	// }
}

void
athn_ani_cck_err_trigger(struct athn_softc *sc)
{
	struct athn_ani *ani = &sc->ani;
	struct athn_ops *ops = &sc->ops;
	int32_t rssi;

	/* Raise noise immunity level, up to max. */
	if (ani->noise_immunity_level < 4) {
		ani->noise_immunity_level++;
		ops->set_noise_immunity_level(sc, ani->noise_immunity_level);
		return;
	}

#ifndef IEEE80211_STA_ONLY
	if (sc->sc_ic.ic_opmode == IEEE80211_M_HOSTAP) {
		if (ani->firstep_level < 2) {
			ani->firstep_level++;
			ops->set_firstep_level(sc, ani->firstep_level);
		}
		return;
	}
#endif
	rssi = athn_ani_get_rssi(sc);
	if (rssi > ATHN_ANI_RSSI_THR_LOW) {
		/*
		 * Beacon RSSI is in mid or high range, raise first step
		 * level.
		 */
		if (ani->firstep_level < 2) {
			ani->firstep_level++;
			ops->set_firstep_level(sc, ani->firstep_level);
		}
	} // TODO no member named 'ic_bss' in 'struct ieee80211com'
	// } else if (IEEE80211_IS_CHAN_2GHZ(sc->sc_ic.ic_bss->ni_chan)) {
	// 	/*
	// 	 * Beacon RSSI is low, zero first step level to maximize
	// 	 * CCK sensitivity.
	// 	 */
	// 	if (ani->firstep_level > 0) {
	// 		ani->firstep_level = 0;
	// 		ops->set_firstep_level(sc, 0);
	// 	}
	// }
}

void
athn_ani_lower_immunity(struct athn_softc *sc)
{
	struct athn_ani *ani = &sc->ani;
	struct athn_ops *ops = &sc->ops;
	int32_t rssi;

#ifndef IEEE80211_STA_ONLY
	if (sc->sc_ic.ic_opmode == IEEE80211_M_HOSTAP) {
		if (ani->firstep_level > 0) {
			ani->firstep_level--;
			ops->set_firstep_level(sc, ani->firstep_level);
		}
		return;
	}
#endif
	rssi = athn_ani_get_rssi(sc);
	if (rssi > ATHN_ANI_RSSI_THR_HIGH) {
		/*
		 * Beacon RSSI is high, leave OFDM weak signal detection
		 * off or it may oscillate.
		 */
	} else if (rssi > ATHN_ANI_RSSI_THR_LOW) {
		/*
		 * Beacon RSSI is in mid range, turn on OFDM weak signal
		 * detection or lower first step level.
		 */
		if (!ani->ofdm_weak_signal) {
			ani->ofdm_weak_signal = 1;
			ops->enable_ofdm_weak_signal(sc);
			return;
		}
		if (ani->firstep_level > 0) {
			ani->firstep_level--;
			ops->set_firstep_level(sc, ani->firstep_level);
			return;
		}
	} else {
		/* Beacon RSSI is low, lower first step level. */
		if (ani->firstep_level > 0) {
			ani->firstep_level--;
			ops->set_firstep_level(sc, ani->firstep_level);
			return;
		}
	}
	/*
	 * Lower spur immunity level down to zero, or if all else fails,
	 * lower noise immunity level down to zero.
	 */
	if (ani->spur_immunity_level > 0) {
		ani->spur_immunity_level--;
		ops->set_spur_immunity_level(sc, ani->spur_immunity_level);
	} else if (ani->noise_immunity_level > 0) {
		ani->noise_immunity_level--;
		ops->set_noise_immunity_level(sc, ani->noise_immunity_level);
	}
}

void
athn_ani_restart(struct athn_softc *sc)
{
	struct athn_ani *ani = &sc->ani;

	AR_WRITE(sc, AR_PHY_ERR_1, 0);
	AR_WRITE(sc, AR_PHY_ERR_2, 0);
	AR_WRITE(sc, AR_PHY_ERR_MASK_1, AR_PHY_ERR_OFDM_TIMING);
	AR_WRITE(sc, AR_PHY_ERR_MASK_2, AR_PHY_ERR_CCK_TIMING);
	AR_WRITE_BARRIER(sc);

	ani->listen_time = 0;
	ani->ofdm_phy_err_count = 0;
	ani->cck_phy_err_count = 0;
}

void
athn_ani_monitor(struct athn_softc *sc)
{
	struct athn_ani *ani = &sc->ani;
	uint32_t cyccnt, txfcnt, rxfcnt, phy1, phy2;
	int32_t cycdelta, txfdelta, rxfdelta;
	int32_t listen_time;

	txfcnt = AR_READ(sc, AR_TFCNT);	/* Tx frame count. */
	rxfcnt = AR_READ(sc, AR_RFCNT);	/* Rx frame count. */
	cyccnt = AR_READ(sc, AR_CCCNT);	/* Cycle count. */

	if (ani->cyccnt != 0 && ani->cyccnt <= cyccnt) {
		cycdelta = cyccnt - ani->cyccnt;
		txfdelta = txfcnt - ani->txfcnt;
		rxfdelta = rxfcnt - ani->rxfcnt;

		listen_time = (cycdelta - txfdelta - rxfdelta) /
		    (athn_clock_rate(sc) * 1000);
	} else
		listen_time = 0;

	ani->cyccnt = cyccnt;
	ani->txfcnt = txfcnt;
	ani->rxfcnt = rxfcnt;

	if (listen_time < 0) {
		athn_ani_restart(sc);
		return;
	}
	ani->listen_time += listen_time;

	phy1 = AR_READ(sc, AR_PHY_ERR_1);
	phy2 = AR_READ(sc, AR_PHY_ERR_2);

	if (phy1 < ani->ofdm_phy_err_base) {
		AR_WRITE(sc, AR_PHY_ERR_1, ani->ofdm_phy_err_base);
		AR_WRITE(sc, AR_PHY_ERR_MASK_1, AR_PHY_ERR_OFDM_TIMING);
	}
	if (phy2 < ani->cck_phy_err_base) {
		AR_WRITE(sc, AR_PHY_ERR_2, ani->cck_phy_err_base);
		AR_WRITE(sc, AR_PHY_ERR_MASK_2, AR_PHY_ERR_CCK_TIMING);
	}
	if (phy1 < ani->ofdm_phy_err_base || phy2 < ani->cck_phy_err_base) {
		AR_WRITE_BARRIER(sc);
		return;
	}
	ani->ofdm_phy_err_count = phy1 - ani->ofdm_phy_err_base;
	ani->cck_phy_err_count = phy2 - ani->cck_phy_err_base;

	if (ani->listen_time > 5 * ATHN_ANI_PERIOD) {
		/* Check to see if we need to lower immunity. */
		if (ani->ofdm_phy_err_count <=
		    ani->listen_time * ani->ofdm_trig_low / 1000 &&
		    ani->cck_phy_err_count <=
		    ani->listen_time * ani->cck_trig_low / 1000)
			athn_ani_lower_immunity(sc);
		athn_ani_restart(sc);

	} else if (ani->listen_time > ATHN_ANI_PERIOD) {
		/* Check to see if we need to raise immunity. */
		if (ani->ofdm_phy_err_count >
		    ani->listen_time * ani->ofdm_trig_high / 1000) {
			athn_ani_ofdm_err_trigger(sc);
			athn_ani_restart(sc);
		} else if (ani->cck_phy_err_count >
		    ani->listen_time * ani->cck_trig_high / 1000) {
			athn_ani_cck_err_trigger(sc);
			athn_ani_restart(sc);
		}
	}
}

uint8_t
athn_chan2fbin(struct ieee80211_channel *c)
{
	if (IEEE80211_IS_CHAN_2GHZ(c))
		return (c->ic_freq - 2300);
	else
		return ((c->ic_freq - 4800) / 5);
}

int
athn_interpolate(int x, int x1, int y1, int x2, int y2)
{
	if (x1 == x2)	/* Prevents division by zero. */
		return (y1);
	/* Linear interpolation. */
	return (y1 + ((x - x1) * (y2 - y1)) / (x2 - x1));
}

void
athn_get_pier_ival(uint8_t fbin, const uint8_t *pierfreq, int npiers,
    int *lo, int *hi)
{
	int i;

	for (i = 0; i < npiers; i++)
		if (pierfreq[i] == AR_BCHAN_UNUSED ||
		    pierfreq[i] > fbin)
			break;
	*hi = i;
	*lo = *hi - 1;
	if (*lo == -1)
		*lo = *hi;
	else if (*hi == npiers || pierfreq[*hi] == AR_BCHAN_UNUSED)
		*hi = *lo;
}

void
athn_init_dma(struct athn_softc *sc)
{
	uint32_t reg;

	if (!AR_SREV_9380_10_OR_LATER(sc)) {
		/* Set AHB not to do cacheline prefetches. */
		AR_SETBITS(sc, AR_AHB_MODE, AR_AHB_PREFETCH_RD_EN);
	}
	reg = AR_READ(sc, AR_TXCFG);
	/* Let MAC DMA reads be in 128-byte chunks. */
	reg = RW(reg, AR_TXCFG_DMASZ, AR_DMASZ_128B);

	/* Set initial Tx trigger level. */
	if (AR_SREV_9285(sc) || AR_SREV_9271(sc))
		reg = RW(reg, AR_TXCFG_FTRIG, AR_TXCFG_FTRIG_256B);
	else if (!AR_SREV_9380_10_OR_LATER(sc))
		reg = RW(reg, AR_TXCFG_FTRIG, AR_TXCFG_FTRIG_512B);
	AR_WRITE(sc, AR_TXCFG, reg);

	/* Let MAC DMA writes be in 128-byte chunks. */
	reg = AR_READ(sc, AR_RXCFG);
	reg = RW(reg, AR_RXCFG_DMASZ, AR_DMASZ_128B);
	AR_WRITE(sc, AR_RXCFG, reg);

	/* Setup Rx FIFO threshold to hold off Tx activities. */
	AR_WRITE(sc, AR_RXFIFO_CFG, 512);

	/* Reduce the number of entries in PCU TXBUF to avoid wrap around. */
	if (AR_SREV_9285(sc)) {
		AR_WRITE(sc, AR_PCU_TXBUF_CTRL,
		    AR9285_PCU_TXBUF_CTRL_USABLE_SIZE);
	} else if (!AR_SREV_9271(sc)) {
		AR_WRITE(sc, AR_PCU_TXBUF_CTRL,
		    AR_PCU_TXBUF_CTRL_USABLE_SIZE);
	}
	AR_WRITE_BARRIER(sc);
}

void
athn_inc_tx_trigger_level(struct athn_softc *sc)
{
	uint32_t reg, ftrig;

	reg = AR_READ(sc, AR_TXCFG);
	ftrig = MS(reg, AR_TXCFG_FTRIG);
	/*
	 * NB: The AR9285 and all single-stream parts have an issue that
	 * limits the size of the PCU Tx FIFO to 2KB instead of 4KB.
	 */
	if (ftrig == ((AR_SREV_9285(sc) || AR_SREV_9271(sc)) ? 0x1f : 0x3f))
		return;		/* Already at max. */
	reg = RW(reg, AR_TXCFG_FTRIG, ftrig + 1);
	AR_WRITE(sc, AR_TXCFG, reg);
	AR_WRITE_BARRIER(sc);
}

int
athn_stop_rx_dma(struct athn_softc *sc)
{
	int ntries;

	AR_WRITE(sc, AR_CR, AR_CR_RXD);
	/* Wait for Rx enable bit to go low. */
	for (ntries = 0; ntries < 100; ntries++) {
		if (!(AR_READ(sc, AR_CR) & AR_CR_RXE))
			return (0);
		DELAY(100);
	}
	DPRINTF(("Rx DMA failed to stop\n"));
	return (ETIMEDOUT);
}

int
athn_rx_abort(struct athn_softc *sc)
{
	int ntries;

	AR_SETBITS(sc, AR_DIAG_SW, AR_DIAG_RX_DIS | AR_DIAG_RX_ABORT);
	for (ntries = 0; ntries < 1000; ntries++) {
		if (MS(AR_READ(sc, AR_OBS_BUS_1), AR_OBS_BUS_1_RX_STATE) == 0)
			return (0);
		DELAY(10);
	}
	DPRINTF(("Rx failed to go idle in 10ms\n"));
	AR_CLRBITS(sc, AR_DIAG_SW, AR_DIAG_RX_DIS | AR_DIAG_RX_ABORT);
	AR_WRITE_BARRIER(sc);
	return (ETIMEDOUT);
}

void
athn_tx_reclaim(struct athn_softc *sc, int qid)
{
	struct athn_txq *txq = &sc->txq[qid];
	struct athn_tx_buf *bf;

	/* Reclaim all buffers queued in the specified Tx queue. */
	/* NB: Tx DMA must be stopped. */
	while ((bf = SIMPLEQ_FIRST(&txq->head)) != NULL) {
		SIMPLEQ_REMOVE_HEAD(&txq->head, bf_list);

		// TODO incomplete definition of type 'struct bus_dmamap'
		// bus_dmamap_sync(sc->sc_dmat, bf->bf_map, 0,
		//     bf->bf_map->dm_mapsize, BUS_DMASYNC_POSTWRITE);
		// bus_dmamap_unload(sc->sc_dmat, bf->bf_map);
		m_freem(bf->bf_m);
		bf->bf_m = NULL;
		bf->bf_ni = NULL;	/* Nodes already freed! */

		/* Link Tx buffer back to global free list. */
		SIMPLEQ_INSERT_TAIL(&sc->txbufs, bf, bf_list);
	}
}

int
athn_tx_pending(struct athn_softc *sc, int qid)
{
	return (MS(AR_READ(sc, AR_QSTS(qid)), AR_Q_STS_PEND_FR_CNT) != 0 ||
	    (AR_READ(sc, AR_Q_TXE) & (1 << qid)) != 0);
}

void
athn_stop_tx_dma(struct athn_softc *sc, int qid)
{
	uint32_t tsflo;
	int ntries, i;

	AR_WRITE(sc, AR_Q_TXD, 1 << qid);
	for (ntries = 0; ntries < 40; ntries++) {
		if (!athn_tx_pending(sc, qid))
			break;
		DELAY(100);
	}
	if (ntries == 40) {
		for (i = 0; i < 2; i++) {
			tsflo = AR_READ(sc, AR_TSF_L32) / 1024;
			AR_WRITE(sc, AR_QUIET2,
			    SM(AR_QUIET2_QUIET_DUR, 10));
			AR_WRITE(sc, AR_QUIET_PERIOD, 100);
			AR_WRITE(sc, AR_NEXT_QUIET_TIMER, tsflo);
			AR_SETBITS(sc, AR_TIMER_MODE, AR_QUIET_TIMER_EN);
			if (AR_READ(sc, AR_TSF_L32) / 1024 == tsflo)
				break;
		}
		AR_SETBITS(sc, AR_DIAG_SW, AR_DIAG_FORCE_CH_IDLE_HIGH);
		AR_WRITE_BARRIER(sc);
		DELAY(200);
		AR_CLRBITS(sc, AR_TIMER_MODE, AR_QUIET_TIMER_EN);
		AR_WRITE_BARRIER(sc);

		for (ntries = 0; ntries < 40; ntries++) {
			if (!athn_tx_pending(sc, qid))
				break;
			DELAY(100);
		}

		AR_CLRBITS(sc, AR_DIAG_SW, AR_DIAG_FORCE_CH_IDLE_HIGH);
	}
	AR_WRITE(sc, AR_Q_TXD, 0);
	AR_WRITE_BARRIER(sc);
}

int
athn_txtime(struct athn_softc *sc, int len, int ridx, u_int flags)
{
	__attribute__((unused)) struct ieee80211com *ic = &sc->sc_ic;
#define divround(a, b)	(((a) + (b) - 1) / (b))
	int txtime;

	if (athn_rates[ridx].hwrate & 0x80) { /* MCS */
	 	/* Assumes a 20MHz channel, HT-mixed frame format, no STBC. */
		txtime = 8 + 8 + 4 + 4 + 4 * 4 + 8 /* HT PLCP */
		    + 4 * ((8 * len + 16 + 6) / (athn_rates[ridx].rate * 2));
		// TODO no member named 'ic_bss' in 'struct ieee80211com'
		// if (IEEE80211_IS_CHAN_2GHZ(ic->ic_bss->ni_chan))
		// 	txtime += 6; /* aSignalExtension */
	} else if (athn_rates[ridx].phy == IEEE80211_T_OFDM) {
		txtime = divround(8 + 4 * len + 3, athn_rates[ridx].rate);
		/* SIFS is 10us for 11g but Signal Extension adds 6us. */
		txtime = 16 + 4 + 4 * txtime + 16;
	} else {
		txtime = divround(16 * len, athn_rates[ridx].rate);
		if (ridx != ATHN_RIDX_CCK1 && (flags & IEEE80211_F_SHPREAMBLE))
			txtime +=  72 + 24;
		else
			txtime += 144 + 48;
		txtime += 10;	/* 10us SIFS. */
	}
	return (txtime);
#undef divround
}

void
athn_init_tx_queues(struct athn_softc *sc)
{
	int qid;

	for (qid = 0; qid < ATHN_QID_COUNT; qid++) {
		SIMPLEQ_INIT(&sc->txq[qid].head);
		sc->txq[qid].lastds = NULL;
		sc->txq[qid].wait = NULL;
		sc->txq[qid].queued = 0;

		AR_WRITE(sc, AR_DRETRY_LIMIT(qid),
		    SM(AR_D_RETRY_LIMIT_STA_SH, 32) |
		    SM(AR_D_RETRY_LIMIT_STA_LG, 32) |
		    SM(AR_D_RETRY_LIMIT_FR_SH, 10));
		AR_WRITE(sc, AR_QMISC(qid),
		    AR_Q_MISC_DCU_EARLY_TERM_REQ);
		AR_WRITE(sc, AR_DMISC(qid),
		    SM(AR_D_MISC_BKOFF_THRESH, 2) |
		    AR_D_MISC_CW_BKOFF_EN | AR_D_MISC_FRAG_WAIT_EN);
	}

	/* Init beacon queue. */
	AR_SETBITS(sc, AR_QMISC(ATHN_QID_BEACON),
	    AR_Q_MISC_FSP_DBA_GATED | AR_Q_MISC_BEACON_USE |
	    AR_Q_MISC_CBR_INCR_DIS1);
	AR_SETBITS(sc, AR_DMISC(ATHN_QID_BEACON),
	    SM(AR_D_MISC_ARB_LOCKOUT_CNTRL,
	       AR_D_MISC_ARB_LOCKOUT_CNTRL_GLOBAL) |
	    AR_D_MISC_BEACON_USE |
	    AR_D_MISC_POST_FR_BKOFF_DIS);
	AR_WRITE(sc, AR_DLCL_IFS(ATHN_QID_BEACON),
	    SM(AR_D_LCL_IFS_CWMIN, 0) |
	    SM(AR_D_LCL_IFS_CWMAX, 0) |
	    SM(AR_D_LCL_IFS_AIFS,  1));

	/* Init CAB (Content After Beacon) queue. */
	AR_SETBITS(sc, AR_QMISC(ATHN_QID_CAB),
	    AR_Q_MISC_FSP_DBA_GATED | AR_Q_MISC_CBR_INCR_DIS1 |
	    AR_Q_MISC_CBR_INCR_DIS0);
	AR_SETBITS(sc, AR_DMISC(ATHN_QID_CAB),
	    SM(AR_D_MISC_ARB_LOCKOUT_CNTRL,
	       AR_D_MISC_ARB_LOCKOUT_CNTRL_GLOBAL));

	/* Init PS-Poll queue. */
	AR_SETBITS(sc, AR_QMISC(ATHN_QID_PSPOLL),
	    AR_Q_MISC_CBR_INCR_DIS1);

	/* Init UAPSD queue. */
	AR_SETBITS(sc, AR_DMISC(ATHN_QID_UAPSD),
	    AR_D_MISC_POST_FR_BKOFF_DIS);

	if (AR_SREV_9380_10_OR_LATER(sc)) {
		/* Enable MAC descriptor CRC check. */
		AR_WRITE(sc, AR_Q_DESC_CRCCHK, AR_Q_DESC_CRCCHK_EN);
	}
	/* Enable DESC interrupts for all Tx queues. */
	AR_WRITE(sc, AR_IMR_S0, 0x00ff0000);
	/* Enable EOL interrupts for all Tx queues except UAPSD. */
	AR_WRITE(sc, AR_IMR_S1, 0x00df0000);
	AR_WRITE_BARRIER(sc);
}

void
athn_set_sta_timers(struct athn_softc *sc)
{
	__attribute__((unused)) struct ieee80211com *ic = &sc->sc_ic;
	uint32_t tsfhi, tsflo, tsftu, reg;
	uint32_t intval = 0, next_tbtt, next_dtim;
	__attribute__((unused)) int dtim_period, dtim_count = 0, rem_dtim_count = 0;

	tsfhi = AR_READ(sc, AR_TSF_U32);
	tsflo = AR_READ(sc, AR_TSF_L32);
	tsftu = AR_TSF_TO_TU(tsfhi, tsflo) + AR_FUDGE;

	/* Beacon interval in TU. */
	// TODO no member named 'ic_bss' in 'struct ieee80211com'
	// intval = ic->ic_bss->ni_intval;

	next_tbtt = roundup(tsftu, intval);
#ifdef notyet
	dtim_period = ic->ic_dtim_period;
	if (dtim_period <= 0)
#endif
		dtim_period = 1;	/* Assume all TIMs are DTIMs. */

#ifdef notyet
	dtim_count = ic->ic_dtim_count;
	if (dtim_count >= dtim_period)	/* Should not happen. */
#endif
		dtim_count = 0;	/* Assume last TIM was a DTIM. */

	/* Compute number of remaining TIMs until next DTIM. */
	rem_dtim_count = 0;	/* XXX */
	next_dtim = next_tbtt + rem_dtim_count * intval;

	AR_WRITE(sc, AR_NEXT_TBTT_TIMER, next_tbtt * IEEE80211_DUR_TU);
	AR_WRITE(sc, AR_BEACON_PERIOD, intval * IEEE80211_DUR_TU);
	AR_WRITE(sc, AR_DMA_BEACON_PERIOD, intval * IEEE80211_DUR_TU);

	/*
	 * Set the number of consecutive beacons to miss before raising
	 * a BMISS interrupt to 10.
	 */
	reg = AR_READ(sc, AR_RSSI_THR);
	reg = RW(reg, AR_RSSI_THR_BM_THR, 10);
	AR_WRITE(sc, AR_RSSI_THR, reg);

	AR_WRITE(sc, AR_NEXT_DTIM,
	    (next_dtim - AR_SLEEP_SLOP) * IEEE80211_DUR_TU);
	AR_WRITE(sc, AR_NEXT_TIM,
	    (next_tbtt - AR_SLEEP_SLOP) * IEEE80211_DUR_TU);

	/* CAB timeout is in 1/8 TU. */
	AR_WRITE(sc, AR_SLEEP1,
	    SM(AR_SLEEP1_CAB_TIMEOUT, AR_CAB_TIMEOUT_VAL * 8) |
	    AR_SLEEP1_ASSUME_DTIM);
	AR_WRITE(sc, AR_SLEEP2,
	    SM(AR_SLEEP2_BEACON_TIMEOUT, AR_MIN_BEACON_TIMEOUT_VAL));

	AR_WRITE(sc, AR_TIM_PERIOD, intval * IEEE80211_DUR_TU);
	AR_WRITE(sc, AR_DTIM_PERIOD, dtim_period * intval * IEEE80211_DUR_TU);

	AR_SETBITS(sc, AR_TIMER_MODE,
	    AR_TBTT_TIMER_EN | AR_TIM_TIMER_EN | AR_DTIM_TIMER_EN);

	/* Set TSF out-of-range threshold (fixed at 16k us). */
	AR_WRITE(sc, AR_TSFOOR_THRESHOLD, 0x4240);

	AR_WRITE_BARRIER(sc);
}

#ifndef IEEE80211_STA_ONLY
void
athn_set_hostap_timers(struct athn_softc *sc)
{
	__attribute__((unused)) struct ieee80211com *ic = &sc->sc_ic;
	uint32_t intval = 0, next_tbtt;

	/* Beacon interval in TU. */
	// TODO no member named 'ic_bss' in 'struct ieee80211com'
	// intval = ic->ic_bss->ni_intval;
	next_tbtt = intval;

	AR_WRITE(sc, AR_NEXT_TBTT_TIMER, next_tbtt * IEEE80211_DUR_TU);
	AR_WRITE(sc, AR_NEXT_DMA_BEACON_ALERT,
	    (next_tbtt - AR_BEACON_DMA_DELAY) * IEEE80211_DUR_TU);
	AR_WRITE(sc, AR_NEXT_CFP,
	    (next_tbtt - AR_SWBA_DELAY) * IEEE80211_DUR_TU);

	AR_WRITE(sc, AR_BEACON_PERIOD, intval * IEEE80211_DUR_TU);
	AR_WRITE(sc, AR_DMA_BEACON_PERIOD, intval * IEEE80211_DUR_TU);
	AR_WRITE(sc, AR_SWBA_PERIOD, intval * IEEE80211_DUR_TU);
	AR_WRITE(sc, AR_NDP_PERIOD, intval * IEEE80211_DUR_TU);

	AR_WRITE(sc, AR_TIMER_MODE,
	    AR_TBTT_TIMER_EN | AR_DBA_TIMER_EN | AR_SWBA_TIMER_EN);

	AR_WRITE_BARRIER(sc);
}
#endif

void
athn_set_opmode(struct athn_softc *sc)
{
	uint32_t reg;

	switch (sc->sc_ic.ic_opmode) {
#ifndef IEEE80211_STA_ONLY
	case IEEE80211_M_HOSTAP:
		reg = AR_READ(sc, AR_STA_ID1);
		reg &= ~AR_STA_ID1_ADHOC;
		reg |= AR_STA_ID1_STA_AP | AR_STA_ID1_KSRCH_MODE;
		AR_WRITE(sc, AR_STA_ID1, reg);

		AR_CLRBITS(sc, AR_CFG, AR_CFG_AP_ADHOC_INDICATION);
		break;
	case IEEE80211_M_IBSS:
	case IEEE80211_M_AHDEMO:
		reg = AR_READ(sc, AR_STA_ID1);
		reg &= ~AR_STA_ID1_STA_AP;
		reg |= AR_STA_ID1_ADHOC | AR_STA_ID1_KSRCH_MODE;
		AR_WRITE(sc, AR_STA_ID1, reg);

		AR_SETBITS(sc, AR_CFG, AR_CFG_AP_ADHOC_INDICATION);
		break;
#endif
	default:
		reg = AR_READ(sc, AR_STA_ID1);
		reg &= ~(AR_STA_ID1_ADHOC | AR_STA_ID1_STA_AP);
		reg |= AR_STA_ID1_KSRCH_MODE;
		AR_WRITE(sc, AR_STA_ID1, reg);
		break;
	}
	AR_WRITE_BARRIER(sc);
}

void
athn_set_bss(struct athn_softc *sc, struct ieee80211_node *ni)
{
	const uint8_t *bssid = ni->ni_bssid;

	AR_WRITE(sc, AR_BSS_ID0, LE_READ_4(&bssid[0]));
	AR_WRITE(sc, AR_BSS_ID1, LE_READ_2(&bssid[4]) |
	    SM(AR_BSS_ID1_AID, IEEE80211_AID(ni->ni_associd)));
	AR_WRITE_BARRIER(sc);
}

void
athn_enable_interrupts(struct athn_softc *sc)
{
	uint32_t mask2;

	athn_disable_interrupts(sc);	/* XXX */

	AR_WRITE(sc, AR_IMR, sc->imask);

	mask2 = AR_READ(sc, AR_IMR_S2);
	mask2 &= ~(AR_IMR_S2_TIM | AR_IMR_S2_DTIM | AR_IMR_S2_DTIMSYNC |
	    AR_IMR_S2_CABEND | AR_IMR_S2_CABTO | AR_IMR_S2_TSFOOR);
	mask2 |= AR_IMR_S2_GTT | AR_IMR_S2_CST;
	AR_WRITE(sc, AR_IMR_S2, mask2);

	AR_CLRBITS(sc, AR_IMR_S5, AR_IMR_S5_TIM_TIMER);

	AR_WRITE(sc, AR_IER, AR_IER_ENABLE);

	AR_WRITE(sc, AR_INTR_ASYNC_ENABLE, AR_INTR_MAC_IRQ);
	AR_WRITE(sc, AR_INTR_ASYNC_MASK, AR_INTR_MAC_IRQ);

	AR_WRITE(sc, AR_INTR_SYNC_ENABLE, sc->isync);
	AR_WRITE(sc, AR_INTR_SYNC_MASK, sc->isync);
	AR_WRITE_BARRIER(sc);
}

void
athn_disable_interrupts(struct athn_softc *sc)
{
	AR_WRITE(sc, AR_IER, 0);
	(void)AR_READ(sc, AR_IER);

	AR_WRITE(sc, AR_INTR_ASYNC_ENABLE, 0);
	(void)AR_READ(sc, AR_INTR_ASYNC_ENABLE);

	AR_WRITE(sc, AR_INTR_SYNC_ENABLE, 0);
	(void)AR_READ(sc, AR_INTR_SYNC_ENABLE);

	AR_WRITE(sc, AR_IMR, 0);

	AR_CLRBITS(sc, AR_IMR_S2, AR_IMR_S2_TIM | AR_IMR_S2_DTIM |
	    AR_IMR_S2_DTIMSYNC | AR_IMR_S2_CABEND | AR_IMR_S2_CABTO |
	    AR_IMR_S2_TSFOOR | AR_IMR_S2_GTT | AR_IMR_S2_CST);

	AR_CLRBITS(sc, AR_IMR_S5, AR_IMR_S5_TIM_TIMER);
	AR_WRITE_BARRIER(sc);
}

void
athn_init_qos(struct athn_softc *sc)
{
	/* Initialize QoS settings. */
	AR_WRITE(sc, AR_MIC_QOS_CONTROL, 0x100aa);
	AR_WRITE(sc, AR_MIC_QOS_SELECT, 0x3210);
	AR_WRITE(sc, AR_QOS_NO_ACK,
	    SM(AR_QOS_NO_ACK_TWO_BIT, 2) |
	    SM(AR_QOS_NO_ACK_BIT_OFF, 5) |
	    SM(AR_QOS_NO_ACK_BYTE_OFF, 0));
	AR_WRITE(sc, AR_TXOP_X, AR_TXOP_X_VAL);
	/* Initialize TXOP for all TIDs. */
	AR_WRITE(sc, AR_TXOP_0_3,   0xffffffff);
	AR_WRITE(sc, AR_TXOP_4_7,   0xffffffff);
	AR_WRITE(sc, AR_TXOP_8_11,  0xffffffff);
	AR_WRITE(sc, AR_TXOP_12_15, 0xffffffff);
	AR_WRITE_BARRIER(sc);
}

int
athn_hw_reset(struct athn_softc *sc, struct ieee80211_channel *c,
    struct ieee80211_channel *extc, int init)
{
	__attribute__((unused)) struct ieee80211com *ic = &sc->sc_ic;
	struct athn_ops *ops = &sc->ops;
	uint32_t reg, def_ant, sta_id1, cfg_led, tsflo, tsfhi;
	int i, error;

	/* XXX not if already awake */
	if ((error = athn_set_power_awake(sc)) != 0) {
		// TOTO missing field 'dv_xname' in 'struct device'
		// printf("%s: could not wakeup chip\n", sc->sc_dev.dv_xname);
		return (error);
	}

	/* Preserve the antenna on a channel switch. */
	if ((def_ant = AR_READ(sc, AR_DEF_ANTENNA)) == 0)
		def_ant = 1;
	/* Preserve other registers. */
	sta_id1 = AR_READ(sc, AR_STA_ID1) & AR_STA_ID1_BASE_RATE_11B;
	cfg_led = AR_READ(sc, AR_CFG_LED) & (AR_CFG_LED_ASSOC_CTL_M |
	    AR_CFG_LED_MODE_SEL_M | AR_CFG_LED_BLINK_THRESH_SEL_M |
	    AR_CFG_LED_BLINK_SLOW);

	/* Mark PHY as inactive. */
	ops->disable_phy(sc);

	if (init && AR_SREV_9271(sc)) {
		AR_WRITE(sc, AR9271_RESET_POWER_DOWN_CONTROL,
		    AR9271_RADIO_RF_RST);
		DELAY(50);
	}
	if (AR_SREV_9280(sc) && (sc->flags & ATHN_FLAG_OLPC)) {
		/* Save TSF before it gets cleared. */
		tsfhi = AR_READ(sc, AR_TSF_U32);
		tsflo = AR_READ(sc, AR_TSF_L32);

		/* NB: RTC reset clears TSF. */
		error = athn_reset_power_on(sc);
	} else
		error = athn_reset(sc, 0);
	if (error != 0) {
		// TOTO missing field 'dv_xname' in 'struct device'
		// printf("%s: could not reset chip (error=%d)\n",
		//     sc->sc_dev.dv_xname, error);
		return (error);
	}

	/* XXX not if already awake */
	if ((error = athn_set_power_awake(sc)) != 0) {
		// TOTO missing field 'dv_xname' in 'struct device'
		// printf("%s: could not wakeup chip\n", sc->sc_dev.dv_xname);
		return (error);
	}

	athn_init_pll(sc, c);
	ops->set_rf_mode(sc, c);

	if (sc->flags & ATHN_FLAG_RFSILENT) {
		/* Check that the radio is not disabled by hardware switch. */
		reg = ops->gpio_read(sc, sc->rfsilent_pin);
		if (sc->flags & ATHN_FLAG_RFSILENT_REVERSED)
			reg = !reg;
		if (!reg) {
			// TOTO missing field 'dv_xname' in 'struct device'
			// printf("%s: radio is disabled by hardware switch\n",
			//     sc->sc_dev.dv_xname);
			return (EPERM);
		}
	}
	if (init && AR_SREV_9271(sc)) {
		AR_WRITE(sc, AR9271_RESET_POWER_DOWN_CONTROL,
		    AR9271_GATE_MAC_CTL);
		DELAY(50);
	}
	if (AR_SREV_9280(sc) && (sc->flags & ATHN_FLAG_OLPC)) {
		/* Restore TSF if it got cleared. */
		AR_WRITE(sc, AR_TSF_L32, tsflo);
		AR_WRITE(sc, AR_TSF_U32, tsfhi);
	}

	if (AR_SREV_9280_10_OR_LATER(sc))
		AR_SETBITS(sc, sc->gpio_input_en_off, AR_GPIO_JTAG_DISABLE);

	/* Write init values to hardware. */
	ops->hw_init(sc, c, extc);

	/*
	 * Only >=AR9280 2.0 parts are capable of encrypting unicast
	 * management frames using CCMP.
	 */
	if (AR_SREV_9280_20_OR_LATER(sc)) {
		reg = AR_READ(sc, AR_AES_MUTE_MASK1);
		/* Do not mask the subtype field in management frames. */
		reg = RW(reg, AR_AES_MUTE_MASK1_FC0_MGMT, 0xff);
		reg = RW(reg, AR_AES_MUTE_MASK1_FC1_MGMT,
		    ~(IEEE80211_FC1_RETRY | IEEE80211_FC1_PWR_MGT |
		      IEEE80211_FC1_MORE_DATA));
		AR_WRITE(sc, AR_AES_MUTE_MASK1, reg);
	} else if (AR_SREV_9160_10_OR_LATER(sc)) {
		/* Disable hardware crypto for management frames. */
		AR_CLRBITS(sc, AR_PCU_MISC_MODE2,
		    AR_PCU_MISC_MODE2_MGMT_CRYPTO_ENABLE);
		AR_SETBITS(sc, AR_PCU_MISC_MODE2,
		    AR_PCU_MISC_MODE2_NO_CRYPTO_FOR_NON_DATA_PKT);
	}

	if (ic->ic_curmode != IEEE80211_MODE_11B)
		ops->set_delta_slope(sc, c, extc);

	ops->spur_mitigate(sc, c, extc);
	ops->init_from_rom(sc, c, extc);

	/* XXX */
	// TODO no member named 'ic_myaddr' in 'struct ieee80211com'
	// AR_WRITE(sc, AR_STA_ID0, LE_READ_4(&ic->ic_myaddr[0]));
	// AR_WRITE(sc, AR_STA_ID1, LE_READ_2(&ic->ic_myaddr[4]) |
	//     sta_id1 | AR_STA_ID1_RTS_USE_DEF | AR_STA_ID1_CRPT_MIC_ENABLE);

	athn_set_opmode(sc);

	AR_WRITE(sc, AR_BSSMSKL, 0xffffffff);
	AR_WRITE(sc, AR_BSSMSKU, 0xffff);

	/* Restore previous antenna. */
	AR_WRITE(sc, AR_DEF_ANTENNA, def_ant);

	AR_WRITE(sc, AR_BSS_ID0, 0);
	AR_WRITE(sc, AR_BSS_ID1, 0);

	AR_WRITE(sc, AR_ISR, 0xffffffff);

	AR_WRITE(sc, AR_RSSI_THR, SM(AR_RSSI_THR_BM_THR, 7));

	if ((error = ops->set_synth(sc, c, extc)) != 0) {
		// TOTO missing field 'dv_xname' in 'struct device'
		// printf("%s: could not set channel\n", sc->sc_dev.dv_xname);
		return (error);
	}
	sc->curchan = c;
	sc->curchanext = extc;

	for (i = 0; i < AR_NUM_DCU; i++)
		AR_WRITE(sc, AR_DQCUMASK(i), 1 << i);

	athn_init_tx_queues(sc);

	/* Initialize interrupt mask. */
	sc->imask =
	    AR_IMR_TXDESC | AR_IMR_TXEOL |
	    AR_IMR_RXERR | AR_IMR_RXEOL | AR_IMR_RXORN |
	    AR_IMR_RXMINTR | AR_IMR_RXINTM |
	    AR_IMR_GENTMR | AR_IMR_BCNMISC;
	if (AR_SREV_9380_10_OR_LATER(sc))
		sc->imask |= AR_IMR_RXERR | AR_IMR_HP_RXOK;
#ifndef IEEE80211_STA_ONLY
	if (0 && ic->ic_opmode == IEEE80211_M_HOSTAP)
		sc->imask |= AR_IMR_MIB;
#endif
	AR_WRITE(sc, AR_IMR, sc->imask);
	AR_SETBITS(sc, AR_IMR_S2, AR_IMR_S2_GTT);
	AR_WRITE(sc, AR_INTR_SYNC_CAUSE, 0xffffffff);
	sc->isync = AR_INTR_SYNC_DEFAULT;
	if (sc->flags & ATHN_FLAG_RFSILENT)
		sc->isync |= AR_INTR_SYNC_GPIO_PIN(sc->rfsilent_pin);
	AR_WRITE(sc, AR_INTR_SYNC_ENABLE, sc->isync);
	AR_WRITE(sc, AR_INTR_SYNC_MASK, 0);
	if (AR_SREV_9380_10_OR_LATER(sc)) {
		AR_WRITE(sc, AR_INTR_PRIO_ASYNC_ENABLE, 0);
		AR_WRITE(sc, AR_INTR_PRIO_ASYNC_MASK, 0);
		AR_WRITE(sc, AR_INTR_PRIO_SYNC_ENABLE, 0);
		AR_WRITE(sc, AR_INTR_PRIO_SYNC_MASK, 0);
	}

	athn_init_qos(sc);

	AR_SETBITS(sc, AR_PCU_MISC, AR_PCU_MIC_NEW_LOC_ENA);

	athn_setsifs(sc);
	athn_updateslot(ic);
	athn_setclockrate(sc);

	/* Disable sequence number generation in hardware. */
	AR_SETBITS(sc, AR_STA_ID1, AR_STA_ID1_PRESERVE_SEQNUM);

	athn_init_dma(sc);

	/* Program observation bus to see MAC interrupts. */
	AR_WRITE(sc, sc->obs_off, 8);

	/* Setup Rx interrupt mitigation. */
	AR_WRITE(sc, AR_RIMT, SM(AR_RIMT_FIRST, 2000) | SM(AR_RIMT_LAST, 500));

	/* Setup Tx interrupt mitigation. */
	AR_WRITE(sc, AR_TIMT, SM(AR_TIMT_FIRST, 2000) | SM(AR_TIMT_LAST, 500));

	/* Set maximum interrupt rate threshold (in micro seconds). */
	AR_WRITE(sc, AR_MIRT, SM(AR_MIRT_RATE_THRES, 2000));

	ops->init_baseband(sc);

	if ((error = athn_init_calib(sc, c, extc)) != 0) {
		// TOTO missing field 'dv_xname' in 'struct device'
		// printf("%s: could not initialize calibration\n",
		//     sc->sc_dev.dv_xname);
		return (error);
	}

	ops->set_rxchains(sc);

	AR_WRITE(sc, AR_CFG_LED, cfg_led | AR_CFG_SCLK_32KHZ);

	if (sc->flags & ATHN_FLAG_USB) {
		if (AR_SREV_9271(sc))
			AR_WRITE(sc, AR_CFG, AR_CFG_SWRB | AR_CFG_SWTB);
		else
			AR_WRITE(sc, AR_CFG, AR_CFG_SWTD | AR_CFG_SWRD);
	}
#if BYTE_ORDER == BIG_ENDIAN
	else {
		/* Default is LE, turn on swapping for BE. */
		AR_WRITE(sc, AR_CFG, AR_CFG_SWTD | AR_CFG_SWRD);
	}
#endif
	AR_WRITE_BARRIER(sc);

	return (0);
}

struct ieee80211_node *
athn_node_alloc(struct ieee80211com *ic)
{
	struct athn_node *an;

	// TODO missing macro in ieee80211_var.h
	#define    IEEE80211_F_HTON        0x02000000      /* CONF: HT enabled */
#if OpenBSD_IEEE80211_API
	an = malloc(sizeof(struct athn_node), M_DEVBUF, M_NOWAIT | M_ZERO);
	if (an && (ic->ic_flags & IEEE80211_F_HTON))
		ieee80211_ra_node_init(&an->rn);
	return (struct ieee80211_node *)an;
#endif
	return NULL;
}

void
athn_newassoc(struct ieee80211com *ic, struct ieee80211_node *ni, int isnew)
{
	struct athn_softc *sc = ic->ic_softc;
	struct athn_node *an = (void *)ni;
	struct ieee80211_rateset *rs = &ni->ni_rates;
	uint8_t rate;
	int ridx, i, j;

	// TODO implicit declaration of function 'ieee80211_amrr_node_init'
#if OpenBSD_IEEE80211_API
	 if ((ni->ni_flags & IEEE80211_NODE_HT) == 0)
	 	ieee80211_amrr_node_init(&sc->amrr, &an->amn);
	else if (ic->ic_opmode == IEEE80211_M_STA)
		ieee80211_ra_node_init(&an->rn);
#endif

	/* Start at lowest available bit-rate, AMRR will raise. */
	ni->ni_txrate = 0;

	for (i = 0; i < rs->rs_nrates; i++) {
		rate = rs->rs_rates[i] & IEEE80211_RATE_VAL;

		/* Map 802.11 rate to HW rate index. */
		for (ridx = 0; ridx <= ATHN_RIDX_MAX; ridx++)
			if (athn_rates[ridx].rate == rate)
				break;
		an->ridx[i] = ridx;
		DPRINTFN(2, ("rate %d index %d\n", rate, ridx));

		/* Compute fallback rate for retries. */
		an->fallback[i] = i;
		for (j = i - 1; j >= 0; j--) {
			if (athn_rates[an->ridx[j]].phy ==
			    athn_rates[an->ridx[i]].phy) {
				an->fallback[i] = j;
				break;
			}
		}
		DPRINTFN(2, ("%d fallbacks to %d\n", i, an->fallback[i]));
	}

	/* In 11n mode, start at lowest available bit-rate, MiRA will raise. */
	// TODO no member named 'ni_txmcs' in 'struct ieee80211_node'
	// ni->ni_txmcs = 0;

	for (i = 0; i <= ATHN_MCS_MAX; i++) {
		/* Map MCS index to HW rate index. */
		ridx = ATHN_NUM_LEGACY_RATES + i;
		an->ridx[ridx] = ATHN_RIDX_MCS0 + i;

		DPRINTFN(2, ("mcs %d index %d ", i, ridx));
		/* Compute fallback rate for retries. */
		if (i == 0 || i == 8) {
		 	/* MCS 0 and 8 fall back to the lowest legacy rate. */
			if (IEEE80211_IS_CHAN_5GHZ(ni->ni_chan))
				an->fallback[ridx] = ATHN_RIDX_OFDM6;
			else
				an->fallback[ridx] = ATHN_RIDX_CCK1;
		} else {
			/* Other MCS fall back to next supported lower MCS. */
			an->fallback[ridx] = ATHN_NUM_LEGACY_RATES + i;
			for (j = i - 1; j >= 0; j--) {
#if OpenBSD_IEEE80211_API
				if (!isset(ni->ni_rxmcs, j))
					continue;
#endif
				an->fallback[ridx] = ATHN_NUM_LEGACY_RATES + j;
				break;
			}
		}
		DPRINTFN(2, (" fallback to %d\n", an->fallback[ridx]));
	}
}

int
athn_media_change(struct ifnet *ifp)
{
	struct athn_softc *sc = ifp->if_softc;
	__attribute__((unused)) struct ieee80211com *ic = &sc->sc_ic;
	uint8_t rate, ridx;
	int error;

	error = ieee80211_media_change(ifp);
	if (error != ENETRESET)
		return (error);
#if OpenBSD_IEEE80211_API
	if (ic->ic_fixed_rate != -1) {
		rate = ic->ic_sup_rates[ic->ic_curmode].
		    rs_rates[ic->ic_fixed_rate] & IEEE80211_RATE_VAL;
		/* Map 802.11 rate to HW rate index. */
		for (ridx = 0; ridx <= ATHN_RIDX_MAX; ridx++)
			if (athn_rates[ridx].rate == rate)
				break;
		sc->fixed_ridx = ridx;
	}
#endif
	if ((ifp->if_flags & (IFF_UP | IFF_DRV_RUNNING)) ==
	    (IFF_UP | IFF_DRV_RUNNING)) {
		athn_stop(ifp, 0);
		error = athn_init(ifp);
	}
	return (error);
}

void
athn_next_scan(void *arg)
{
	struct athn_softc *sc = arg;
	__attribute__((unused)) struct ieee80211com *ic = &sc->sc_ic;
	int s;

	s = splnet();
#if OpenBSD_IEEE80211_API
	if (ic->ic_state == IEEE80211_S_SCAN)
		ieee80211_next_scan(&ic->ic_if);
#endif
	splx(s);
}

int
athn_newstate(struct ieee80211com *ic, enum ieee80211_state nstate, int arg)
{
#if OpenBSD_IEEE80211_API
	struct ifnet *ifp = &ic->ic_if;
	struct athn_softc *sc = ifp->if_softc;
	uint32_t reg;
	int error = 0;

	timeout_del(&sc->calib_to);

	switch (nstate) {
	case IEEE80211_S_INIT:
		athn_set_led(sc, 0);
		break;
	case IEEE80211_S_SCAN:
		/* Make the LED blink while scanning. */
		athn_set_led(sc, !sc->led_state);
		// TODO no member named 'ic_bss' in 'struct ieee80211com'
		error = athn_switch_chan(sc, ic->ic_bss->ni_chan, NULL);
		if (error != 0)
			return (error);
		timeout_add_msec(&sc->scan_to, 200);
		break;
	case IEEE80211_S_AUTH:
		athn_set_led(sc, 0);
		// TODO no member named 'ic_bss' in 'struct ieee80211com'
		error = athn_switch_chan(sc, ic->ic_bss->ni_chan, NULL);
		if (error != 0)
			return (error);
		break;
	case IEEE80211_S_ASSOC:
		break;
	case IEEE80211_S_RUN:
		athn_set_led(sc, 1);
#ifndef IEEE80211_STA_ONLY
		if (ic->ic_opmode == IEEE80211_M_HOSTAP) {
			 error = athn_switch_chan(sc, ic->ic_bss->ni_chan, NULL);
			if (error != 0)
				return (error);
		} else
#endif
		if (ic->ic_opmode == IEEE80211_M_MONITOR) {
			 error = athn_switch_chan(sc, ic->ic_ibss_chan, NULL);
			if (error != 0)
				return (error);
			break;
		}

		/* Fake a join to initialize the Tx rate. */
		// TODO no member named 'ic_bss' in 'struct ieee80211com'
		 athn_newassoc(ic, ic->ic_bss, 1);

		 athn_set_bss(sc, ic->ic_bss);
		athn_disable_interrupts(sc);
#ifndef IEEE80211_STA_ONLY
		if (ic->ic_opmode == IEEE80211_M_HOSTAP) {
			athn_set_hostap_timers(sc);
			/* Enable software beacon alert interrupts. */
			sc->imask |= AR_IMR_SWBA;
		} else
#endif
		{
			athn_set_sta_timers(sc);
			/* Enable beacon miss interrupts. */
			sc->imask |= AR_IMR_BMISS;

			/* Stop receiving beacons from other BSS. */
			reg = AR_READ(sc, AR_RX_FILTER);
			reg = (reg & ~AR_RX_FILTER_BEACON) |
			    AR_RX_FILTER_MYBEACON;
			AR_WRITE(sc, AR_RX_FILTER, reg);
			AR_WRITE_BARRIER(sc);
		}
		athn_enable_interrupts(sc);

		if (sc->sup_calib_mask != 0) {
			memset(&sc->calib, 0, sizeof(sc->calib));
			sc->cur_calib_mask = sc->sup_calib_mask;
			sc->ops.do_calib(sc);
		}
		/* XXX Start ANI. */

		athn_start_noisefloor_calib(sc, 1);
		timeout_add_msec(&sc->calib_to, 500);
		break;
	}

	return (sc->sc_newstate(ic, nstate, arg));
#endif
	return 0;
}

void
athn_updateedca(struct ieee80211com *ic)
{
#define ATHN_EXP2(x)	((1 << (x)) - 1)	/* CWmin = 2^ECWmin - 1 */
	struct athn_softc *sc = ic->ic_softc;
	const struct ieee80211_edca_ac_params *ac;
	int aci, qid;
#if OpenBSD_IEEE80211_API
	for (aci = 0; aci < WME_NUM_AC; aci++) {
		ac = &ic->ic_edca_ac[aci];
		qid = athn_ac2qid[aci];

		AR_WRITE(sc, AR_DLCL_IFS(qid),
		    SM(AR_D_LCL_IFS_CWMIN, ATHN_EXP2(ac->ac_ecwmin)) |
		    SM(AR_D_LCL_IFS_CWMAX, ATHN_EXP2(ac->ac_ecwmax)) |
		    SM(AR_D_LCL_IFS_AIFS, ac->ac_aifsn));
		if (ac->ac_txoplimit != 0) {
			AR_WRITE(sc, AR_DCHNTIME(qid),
			    SM(AR_D_CHNTIME_DUR,
			       IEEE80211_TXOP_TO_US(ac->ac_txoplimit)) |
			    AR_D_CHNTIME_EN);
		} else
			AR_WRITE(sc, AR_DCHNTIME(qid), 0);
	}
	AR_WRITE_BARRIER(sc);
#undef ATHN_EXP2
#endif
}

int
athn_clock_rate(struct athn_softc *sc)
{
	__attribute__((unused)) struct ieee80211com *ic = &sc->sc_ic;
	int clockrate;	/* MHz. */
#if OpenBSD_IEEE80211_API
	/*
	 * AR9287 v1.3+ MAC runs at 117MHz (instead of 88/44MHz) when
	 * ASYNC FIFO is enabled.
	 */
	if (AR_SREV_9287_13_OR_LATER(sc) && !AR_SREV_9380_10_OR_LATER(sc))
		clockrate = 117;
	// TODO no member named 'ic_bss' in 'struct ieee80211com'
	else if (ic->ic_bss->ni_chan != IEEE80211_CHAN_ANYC &&
	    IEEE80211_IS_CHAN_5GHZ(ic->ic_bss->ni_chan)) {
		if (sc->flags & ATHN_FLAG_FAST_PLL_CLOCK)
			clockrate = AR_CLOCK_RATE_FAST_5GHZ_OFDM;
		else
			clockrate = AR_CLOCK_RATE_5GHZ_OFDM;
	} else if (ic->ic_curmode == IEEE80211_MODE_11B) {
		clockrate = AR_CLOCK_RATE_CCK;
	} else
		clockrate = AR_CLOCK_RATE_2GHZ_OFDM;
	if (sc->curchanext != NULL)
		clockrate *= 2;

	return (clockrate);
#endif
	return 0;
}

int
athn_chan_sifs(struct ieee80211_channel *c)
{
	return IEEE80211_IS_CHAN_2GHZ(c) ? IEEE80211_DUR_DS_SIFS : 16;
}

void
athn_setsifs(struct athn_softc *sc)
{
	int sifs = 0; //athn_chan_sifs(sc->sc_ic.ic_bss->ni_chan); // TODO no member named 'ic_bss' in 'struct ieee80211com'
	AR_WRITE(sc, AR_D_GBL_IFS_SIFS, (sifs - 2) * athn_clock_rate(sc));
	AR_WRITE_BARRIER(sc);
}

int
athn_acktimeout(struct ieee80211_channel *c, int slot)
{
	int sifs = athn_chan_sifs(c);
	int ackto = sifs + slot;

	/* Workaround for early ACK timeouts. */
	if (IEEE80211_IS_CHAN_2GHZ(c))
		ackto += 64 - sifs - slot;

	return ackto;
}

void
athn_setacktimeout(struct athn_softc *sc, struct ieee80211_channel *c, int slot)
{
	int ackto = athn_acktimeout(c, slot);
	uint32_t reg = AR_READ(sc, AR_TIME_OUT);
	reg = RW(reg, AR_TIME_OUT_ACK, ackto * athn_clock_rate(sc));
	AR_WRITE(sc, AR_TIME_OUT, reg);
	AR_WRITE_BARRIER(sc);
}

void
athn_setctstimeout(struct athn_softc *sc, struct ieee80211_channel *c, int slot)
{
	int ctsto = athn_acktimeout(c, slot);
	int sifs = athn_chan_sifs(c);
	uint32_t reg = AR_READ(sc, AR_TIME_OUT);

	/* Workaround for early CTS timeouts. */
	if (IEEE80211_IS_CHAN_2GHZ(c))
		ctsto += 48 - sifs - slot;

	reg = RW(reg, AR_TIME_OUT_CTS, ctsto * athn_clock_rate(sc));
	AR_WRITE(sc, AR_TIME_OUT, reg);
	AR_WRITE_BARRIER(sc);
}

void
athn_setclockrate(struct athn_softc *sc)
{
	int clockrate = athn_clock_rate(sc);
	uint32_t reg = AR_READ(sc, AR_USEC);
	reg = RW(reg, AR_USEC_USEC, clockrate - 1);
	AR_WRITE(sc, AR_USEC, reg);
	AR_WRITE_BARRIER(sc);
}

void
athn_updateslot(struct ieee80211com *ic)
{
	struct athn_softc *sc = ic->ic_softc;
	int slot;

#if OpenBSD_IEEE80211_API
	slot = (ic->ic_flags & IEEE80211_F_SHSLOT) ?
	    IEEE80211_DUR_DS_SHSLOT : IEEE80211_DUR_DS_SLOT;
	AR_WRITE(sc, AR_D_GBL_IFS_SLOT, slot * athn_clock_rate(sc));
	AR_WRITE_BARRIER(sc);

	// TODO no member named 'ic_bss' in 'struct ieee80211com'
	// athn_setacktimeout(sc, ic->ic_bss->ni_chan, slot);
	// athn_setctstimeout(sc, ic->ic_bss->ni_chan, slot);
#endif
}

void
athn_start(struct ifnet *ifp)
{
	struct athn_softc *sc = ifp->if_softc;
	__attribute__((unused)) struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_node *ni;
	struct mbuf *m;

	if (!(ifp->if_flags & IFF_DRV_RUNNING) || ifq_is_oactive())
		return;

#if OpenBSD_IEEE80211_API
	for (;;) {
		if (SIMPLEQ_EMPTY(&sc->txbufs)) {
			ifq_set_oactive();
			break;
		}
		/* Send pending management frames first. */
		//OpenBSD->FreeBSD prev code:
		//m = mq_dequeue(&ic->ic_mgtq);
//		m = ml_dequeue(&ic->ic_mgtq.mq_list);
		if (m != NULL) {
			ni = m->m_pkthdr.ph_cookie;
			goto sendit;
		}
		if (ic->ic_state != IEEE80211_S_RUN)
			break;

		//OpenBSD->FreeBSD prev code:
		//m = mq_dequeue(&ic->ic_pwrsaveq);
//		m = ml_dequeue(&ic->ic_pwrsaveq.mq_list);
		if (m != NULL) {
			ni = m->m_pkthdr.ph_cookie;
			goto sendit;
		}
		if (ic->ic_state != IEEE80211_S_RUN)
			break;

		/* Encapsulate and send data frames. */
		ALTQ_DEQUEUE(&ifp->if_snd, m);
		if (m == NULL)
			break;
#if NBPFILTER > 0
		if (ifp->if_bpf != NULL)
			bpf_mtap(ifp->if_bpf, m, BPF_DIRECTION_OUT);
#endif
		if ((m = ieee80211_encap(ifp, m, &ni)) == NULL)
			continue;
 sendit:
#if NBPFILTER > 0
		if (ic->ic_rawbpf != NULL)
			bpf_mtap(ic->ic_rawbpf, m, BPF_DIRECTION_OUT);
#endif
		if (sc->ops.tx(sc, m, ni, 0) != 0) {
			ieee80211_release_node(ic, ni);
			continue;
		}

		sc->sc_tx_timer = 5;
	}
#endif
}

void
athn_watchdog(struct ifnet *ifp)
{
	struct athn_softc *sc = ifp->if_softc;


	if (sc->sc_tx_timer > 0) {
		if (--sc->sc_tx_timer == 0) {
			// TOTO missing field 'dv_xname' in 'struct device'
			// printf("%s: device timeout\n", sc->sc_dev.dv_xname);
			athn_stop(ifp, 1);
			(void)athn_init(ifp);
			return;
		}
	}
#if OpenBSD_IEEE80211_API
	ieee80211_watchdog(ifp);
#endif
}

void
athn_set_multi(struct athn_softc *sc)
{
#if OpenBSD_IEEE80211_API
	struct arpcom *ac = &sc->sc_ic.ic_ac;
	struct ifnet *ifp = &ac->ac_if;
	struct ether_multi *enm;
	struct ether_multistep step;
	const uint8_t *addr;
	uint32_t val, lo, hi;
	uint8_t bit;

	if (ac->ac_multirangecnt > 0)
		ifp->if_flags |= IFF_ALLMULTI;

	if ((ifp->if_flags & (IFF_ALLMULTI | IFF_PROMISC)) != 0) {
		lo = hi = 0xffffffff;
		goto done;
	}
	lo = hi = 0;
	ETHER_FIRST_MULTI(step, ac, enm);
	while (enm != NULL) {
		addr = enm->enm_addrlo;
		/* Calculate the XOR value of all eight 6-bit words. */
		val = addr[0] | addr[1] << 8 | addr[2] << 16;
		bit  = (val >> 18) ^ (val >> 12) ^ (val >> 6) ^ val;
		val = addr[3] | addr[4] << 8 | addr[5] << 16;
		bit ^= (val >> 18) ^ (val >> 12) ^ (val >> 6) ^ val;
		bit &= 0x3f;
		if (bit < 32)
			lo |= 1 << bit;
		else
			hi |= 1 << (bit - 32);
		ETHER_NEXT_MULTI(step, enm);
	}
 done:
	AR_WRITE(sc, AR_MCAST_FIL0, lo);
	AR_WRITE(sc, AR_MCAST_FIL1, hi);
	AR_WRITE_BARRIER(sc);
#endif
}

int
athn_ioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
{
	struct athn_softc *sc = ifp->if_softc;
	__attribute__((unused)) struct ieee80211com *ic = &sc->sc_ic;
	struct ifreq *ifr;
	int s, error = 0;

	s = splnet();

	switch (cmd) {
	case SIOCSIFADDR:
		ifp->if_flags |= IFF_UP;
		/* FALLTHROUGH */
	case SIOCSIFFLAGS:
		if (ifp->if_flags & IFF_UP) {
			if ((ifp->if_flags & IFF_DRV_RUNNING) &&
			    ((ifp->if_flags ^ sc->sc_if_flags) &
			     (IFF_ALLMULTI | IFF_PROMISC)) != 0) {
				athn_set_multi(sc);
			} else if (!(ifp->if_flags & IFF_DRV_RUNNING))
				error = athn_init(ifp);
		} else {
			if (ifp->if_flags & IFF_DRV_RUNNING)
				athn_stop(ifp, 1);
		}
		sc->sc_if_flags = ifp->if_flags;
		break;

	case SIOCADDMULTI:
	case SIOCDELMULTI:
		ifr = (struct ifreq *)data;
#if OpenBSD_IEEE80211_API
		error = (cmd == SIOCADDMULTI) ?
		    ether_addmulti(ifr, &ic->ic_ac) :
		    ether_delmulti(ifr, &ic->ic_ac);

		if (error == ENETRESET) {
			athn_set_multi(sc);
			error = 0;
		}
#endif
		break;
#if OpenBSD_IEEE80211_API
	case SIOCS80211CHANNEL:
		error = ieee80211_ioctl(ifp, cmd, data);
		if (error == ENETRESET &&
		    ic->ic_opmode == IEEE80211_M_MONITOR) {
			if ((ifp->if_flags & (IFF_UP | IFF_DRV_RUNNING)) ==
			    (IFF_UP | IFF_DRV_RUNNING))
				athn_switch_chan(sc, ic->ic_ibss_chan, NULL);
			error = 0;
		}
		break;
#endif
	default:
		error = ieee80211_ioctl(ifp, cmd, data);
	}

	if (error == ENETRESET) {
		error = 0;
		if ((ifp->if_flags & (IFF_UP | IFF_DRV_RUNNING)) ==
		    (IFF_UP | IFF_DRV_RUNNING)) {
			athn_stop(ifp, 0);
			error = athn_init(ifp);
		}
	}

	splx(s);
	return (error);
}

int
athn_init(struct ifnet *ifp)
{
	struct athn_softc *sc = ifp->if_softc;
	struct athn_ops *ops = &sc->ops;
	__attribute__((unused)) struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_channel *c = NULL, *extc;
	int i, error;

	// TODO no member named 'ic_bss' in 'struct ieee80211com'
	// c = ic->ic_bss->ni_chan = ic->ic_ibss_chan;
	extc = NULL;

	/* In case a new MAC address has been configured. */
#if OpenBSD_IEEE80211_API
	IEEE80211_ADDR_COPY(ic->ic_myaddr, IF_LLADDR(ifp));
#endif
	/* For CardBus, power on the socket. */
	if (sc->sc_enable != NULL) {
		if ((error = sc->sc_enable(sc)) != 0) {
			// TOTO missing field 'dv_xname' in 'struct device'
			// printf("%s: could not enable device\n",
			//     sc->sc_dev.dv_xname);
			goto fail;
		}
		if ((error = athn_reset_power_on(sc)) != 0) {
			// TOTO missing field 'dv_xname' in 'struct device'
			// printf("%s: could not power on device\n",
			//     sc->sc_dev.dv_xname);
			goto fail;
		}
	}
	
	athn_config_nonpcie(sc);


	ops->enable_antenna_diversity(sc);

#ifdef ATHN_BT_COEXISTENCE
	/* Configure bluetooth coexistence for combo chips. */
	if (sc->flags & ATHN_FLAG_BTCOEX)
		athn_btcoex_init(sc);
#endif

	/* Configure LED. */
	athn_led_init(sc);

	/* Configure hardware radio switch. */
	if (sc->flags & ATHN_FLAG_RFSILENT)
		ops->rfsilent_init(sc);

	if ((error = athn_hw_reset(sc, c, extc, 1)) != 0) {
		// TOTO missing field 'dv_xname' in 'struct device'
		// printf("%s: unable to reset hardware; reset status %d\n",
		//     sc->sc_dev.dv_xname, error);
		goto fail;
	}

	athn_config_ht(sc);

	/* Enable Rx. */
	athn_rx_start(sc);

	/* Reset HW key cache entries. */
	for (i = 0; i < sc->kc_entries; i++)
		athn_reset_key(sc, i);

	/* Enable interrupts. */
	athn_enable_interrupts(sc);

#ifdef ATHN_BT_COEXISTENCE
	/* Enable bluetooth coexistence for combo chips. */
	if (sc->flags & ATHN_FLAG_BTCOEX)
		athn_btcoex_enable(sc);
#endif

	ifq_clr_oactive();
	ifp->if_flags |= IFF_DRV_RUNNING;

#ifdef notyet
	if (ic->ic_flags & IEEE80211_F_WEPON) {
		/* Configure WEP keys. */
		for (i = 0; i < IEEE80211_WEP_NKID; i++)
			athn_set_key(ic, NULL, &ic->ic_nw_keys[i]);
	}
#endif
#if OpenBSD_IEEE80211_API
	if (ic->ic_opmode == IEEE80211_M_MONITOR)
		ieee80211_new_state(ic, IEEE80211_S_RUN, -1);
	else
		ieee80211_new_state(ic, IEEE80211_S_SCAN, -1);
#endif
	return (0);
 fail:
	athn_stop(ifp, 1);
	return (error);
}

void
athn_stop(struct ifnet *ifp, int disable)
{
	struct athn_softc *sc = ifp->if_softc;
	__attribute__((unused)) struct ieee80211com *ic = &sc->sc_ic;
	int qid, i;

	ifp->if_flags &= ~IFF_DRV_RUNNING;
	ifq_clr_oactive();

//	timeout_del(&sc->scan_to);

//	ieee80211_new_state(ic, IEEE80211_S_INIT, -1);

#ifdef ATHN_BT_COEXISTENCE
	/* Disable bluetooth coexistence for combo chips. */
	if (sc->flags & ATHN_FLAG_BTCOEX)
		athn_btcoex_disable(sc);
#endif

	/* Disable interrupts. */
	athn_disable_interrupts(sc);
	/* Acknowledge interrupts (avoids interrupt storms). */
	AR_WRITE(sc, AR_INTR_SYNC_CAUSE, 0xffffffff);
	AR_WRITE(sc, AR_INTR_SYNC_MASK, 0);

	for (qid = 0; qid < ATHN_QID_COUNT; qid++)
		athn_stop_tx_dma(sc, qid);
	/* XXX call athn_hw_reset if Tx still pending? */
	for (qid = 0; qid < ATHN_QID_COUNT; qid++)
		athn_tx_reclaim(sc, qid);

	/* Stop Rx. */
	AR_SETBITS(sc, AR_DIAG_SW, AR_DIAG_RX_DIS | AR_DIAG_RX_ABORT);
	AR_WRITE(sc, AR_MIBC, AR_MIBC_FMC);
	AR_WRITE(sc, AR_MIBC, AR_MIBC_CMC);
	AR_WRITE(sc, AR_FILT_OFDM, 0);
	AR_WRITE(sc, AR_FILT_CCK, 0);
	AR_WRITE_BARRIER(sc);
	athn_set_rxfilter(sc, 0);
	athn_stop_rx_dma(sc);

	/* Reset HW key cache entries. */
	for (i = 0; i < sc->kc_entries; i++)
		athn_reset_key(sc, i);

	athn_reset(sc, 0);
	athn_init_pll(sc, NULL);
	athn_set_power_awake(sc);
	athn_reset(sc, 1);
	athn_init_pll(sc, NULL);

	athn_set_power_sleep(sc);

	/* For CardBus, power down the socket. */
	if (disable && sc->sc_disable != NULL)
		sc->sc_disable(sc);
}

void
athn_suspend(struct athn_softc *sc)
{
#if OpenBSD_IEEE80211_API
	struct ifnet *ifp = &sc->sc_ic.ic_if;

	if (ifp->if_flags & IFF_DRV_RUNNING)
		athn_stop(ifp, 1);
#endif
}

void
athn_wakeup(struct athn_softc *sc)
{
#if OpenBSD_IEEE80211_API
	struct ifnet *ifp = &sc->sc_ic.ic_if;

	if (ifp->if_flags & IFF_UP)
		athn_init(ifp);
#endif
}
