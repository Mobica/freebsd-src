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
 *
 * $FreeBSD$
 */
#include "opt_ah.h"

#include "ah.h"
#include "ah_internal.h"
#include "ah_devid.h"

#include "ah_eeprom_v4k.h"		/* XXX for tx/rx gain */

#include "ar9002/ar9280.h"
#include "ar9002/ar9271.h"
#include "ar5416/ar5416reg.h"
#include "ar5416/ar5416phy.h"

#include "ar9002/ar9285.ini"
#include "ar9002/ar9271.ini"
#include "ar9002/ar9280v2.ini"		/* XXX ini for tx/rx gain */

#include "ar9002/ar9271_cal.h"
#include "ar9002/ar9271_phy.h"
#include "ar9002/ar9271_diversity.h"

#include "if_ath_usb_devid.h"
#include "if_ath_usb.h"

static const HAL_PERCAL_DATA ar9280_iq_cal = {		/* single sample */
	.calName = "IQ", .calType = IQ_MISMATCH_CAL,
	.calNumSamples	= MIN_CAL_SAMPLES,
	.calCountMax	= PER_MAX_LOG_COUNT,
	.calCollect	= ar5416IQCalCollect,
	.calPostProc	= ar5416IQCalibration
};
static const HAL_PERCAL_DATA ar9280_adc_gain_cal = {	/* single sample */
	.calName = "ADC Gain", .calType = ADC_GAIN_CAL,
	.calNumSamples	= MIN_CAL_SAMPLES,
	.calCountMax	= PER_MIN_LOG_COUNT,
	.calCollect	= ar5416AdcGainCalCollect,
	.calPostProc	= ar5416AdcGainCalibration
};
static const HAL_PERCAL_DATA ar9280_adc_dc_cal = {	/* single sample */
	.calName = "ADC DC", .calType = ADC_DC_CAL,
	.calNumSamples	= MIN_CAL_SAMPLES,
	.calCountMax	= PER_MIN_LOG_COUNT,
	.calCollect	= ar5416AdcDcCalCollect,
	.calPostProc	= ar5416AdcDcCalibration
};
static const HAL_PERCAL_DATA ar9280_adc_init_dc_cal = {
	.calName = "ADC Init DC", .calType = ADC_DC_INIT_CAL,
	.calNumSamples	= MIN_CAL_SAMPLES,
	.calCountMax	= INIT_LOG_COUNT,
	.calCollect	= ar5416AdcDcCalCollect,
	.calPostProc	= ar5416AdcDcCalibration
};

static void ar9271ConfigPCIE(struct ath_hal *ah, HAL_BOOL restore,
		HAL_BOOL power_off);
static void ar9271DisablePCIE(struct ath_hal *ah);
static HAL_BOOL ar9271FillCapabilityInfo(struct ath_hal *ah);
static void ar9271WriteIni(struct ath_hal *ah,
	const struct ieee80211_channel *chan);

static void
ar9271AniSetup(struct ath_hal *ah)
{
	/*
	 * These are the parameters from the AR5416 ANI code;
	 * they likely need quite a bit of adjustment for the
	 * AR9271.
	 */
        static const struct ar5212AniParams aniparams = {
                .maxNoiseImmunityLevel  = 4,    /* levels 0..4 */
                .totalSizeDesired       = { -55, -55, -55, -55, -62 },
                .coarseHigh             = { -14, -14, -14, -14, -12 },
                .coarseLow              = { -64, -64, -64, -64, -70 },
                .firpwr                 = { -78, -78, -78, -78, -80 },
                .maxSpurImmunityLevel   = 7,
                .cycPwrThr1             = { 2, 4, 6, 8, 10, 12, 14, 16 },
                .maxFirstepLevel        = 2,    /* levels 0..2 */
                .firstep                = { 0, 4, 8 },
                .ofdmTrigHigh           = 500,
                .ofdmTrigLow            = 200,
                .cckTrigHigh            = 200,
                .cckTrigLow             = 100,
                .rssiThrHigh            = 40,
                .rssiThrLow             = 7,
                .period                 = 100,
        };
	/* NB: disable ANI noise immmunity for reliable RIFS rx */
	AH5416(ah)->ah_ani_function &= ~(1 << HAL_ANI_NOISE_IMMUNITY_LEVEL);

        ar5416AniAttach(ah, &aniparams, &aniparams, AH_TRUE);
}

static const char * ar9271_lna_conf[] = {
	"LNA1-LNA2",
	"LNA2",
	"LNA1",
	"LNA1+LNA2",
};

static void
ar9271_eeprom_print_diversity_settings(struct ath_hal *ah)
{
	const HAL_EEPROM_v4k *ee = AH_PRIVATE(ah)->ah_eeprom;
	const MODAL_EEP4K_HEADER *pModal = &ee->ee_base.modalHeader;

	ath_hal_printf(ah, "[ath] AR9271 Main LNA config: %s\n",
	    ar9271_lna_conf[(pModal->antdiv_ctl2 >> 2) & 0x3]);
	ath_hal_printf(ah, "[ath] AR9271 Alt LNA config: %s\n",
	    ar9271_lna_conf[pModal->antdiv_ctl2 & 0x3]);
	ath_hal_printf(ah, "[ath] LNA diversity %s, Diversity %s\n",
	    ((pModal->antdiv_ctl1 & 0x1) ? "enabled" : "disabled"),
	    ((pModal->antdiv_ctl1 & 0x8) ? "enabled" : "disabled"));
}

/*
 * Attach for an AR9271 part.
 */
static struct ath_hal *
ar9271Attach(uint16_t devid, HAL_SOFTC sc,
	HAL_BUS_TAG st, HAL_BUS_HANDLE sh, uint16_t *eepromdata,
	HAL_OPS_CONFIG *ah_config,
	HAL_STATUS *status)
{
	struct ath_hal_9271 *ahp9271;
	struct ath_hal_5212 *ahp;
	struct ath_hal *ah;
	uint32_t val;
	HAL_STATUS ecode;
	HAL_BOOL rfStatus;

	HALDEBUG(AH_NULL, HAL_DEBUG_ATTACH, "%s: sc %p st %p sh %p\n",
	    __func__, sc, (void*) st, (void*) sh);

	/* NB: memory is returned zero'd */
	ahp9271 = ath_hal_malloc(sizeof (struct ath_hal_9271));
	if (ahp9271 == AH_NULL) {
		HALDEBUG(AH_NULL, HAL_DEBUG_ANY,
		    "%s: cannot allocate memory for state block\n", __func__);
		*status = HAL_ENOMEM;
		return AH_NULL;
	}
	ahp = AH5212(ahp9271);
	ah = &ahp->ah_priv.h;

	ar5416InitState(AH5416(ah), devid, sc, st, sh, status);

	/*
	 * Use the "local" EEPROM data given to us by the higher layers.
	 * This is a private copy out of system flash. The Linux ath9k
	 * commit for the initial AR9130 support mentions MMIO flash
	 * access is "unreliable." -adrian
	 */
	if (eepromdata != AH_NULL) {
		AH_PRIVATE(ah)->ah_eepromRead = ath_hal_EepromDataRead;
		AH_PRIVATE(ah)->ah_eepromWrite = NULL;
		ah->ah_eepromdata = eepromdata;
	}

	/* override with 9271 specific state */
	AH5416(ah)->ah_initPLL = ar9280InitPLL;
	AH5416(ah)->ah_btCoexSetDiversity = ar9271BTCoexAntennaDiversity;

	ah->ah_setAntennaSwitch		= ar9271SetAntennaSwitch;
	ah->ah_configPCIE		= ar9271ConfigPCIE;
	ah->ah_disablePCIE		= ar9271DisablePCIE;
	ah->ah_setTxPower		= ar9271SetTransmitPower;
	ah->ah_setBoardValues		= ar9271SetBoardValues;
	ah->ah_btCoexSetParameter	= ar9271BTCoexSetParameter;
	ah->ah_divLnaConfGet		= ar9271_antdiv_comb_conf_get;
	ah->ah_divLnaConfSet		= ar9271_antdiv_comb_conf_set;

	AH5416(ah)->ah_cal.iqCalData.calData = &ar9280_iq_cal;
	AH5416(ah)->ah_cal.adcGainCalData.calData = &ar9280_adc_gain_cal;
	AH5416(ah)->ah_cal.adcDcCalData.calData = &ar9280_adc_dc_cal;
	AH5416(ah)->ah_cal.adcDcCalInitData.calData = &ar9280_adc_init_dc_cal;
	AH5416(ah)->ah_cal.suppCals = ADC_GAIN_CAL | ADC_DC_CAL | IQ_MISMATCH_CAL;

	AH5416(ah)->ah_spurMitigate	= ar9280SpurMitigate;
	AH5416(ah)->ah_writeIni		= ar9271WriteIni;
	AH5416(ah)->ah_rx_chainmask	= AR9271_DEFAULT_RXCHAINMASK;
	AH5416(ah)->ah_tx_chainmask	= AR9271_DEFAULT_TXCHAINMASK;

	ahp->ah_maxTxTrigLev		= MAX_TX_FIFO_THRESHOLD >> 1;

	AH_PRIVATE(ah)->ah_isusb	= TRUE;
	AH_PRIVATE(ah)->ah_usb_read	= (void *)ath_usb_read;
	AH_PRIVATE(ah)->ah_usb_write	= (void *)ath_usb_write;

	if (!ar5416SetResetReg(ah, HAL_RESET_POWER_ON)) {
		/* reset chip */
		HALDEBUG(ah, HAL_DEBUG_ANY, "%s: couldn't reset chip\n",
		    __func__);
		ecode = HAL_EIO;
		goto bad;
	}

	if (!ar5416SetPowerMode(ah, HAL_PM_AWAKE, AH_TRUE)) {
		HALDEBUG(ah, HAL_DEBUG_ANY, "%s: couldn't wakeup chip\n",
		    __func__);
		ecode = HAL_EIO;
		goto bad;
	}
	/* Read Revisions from Chips before taking out of reset */
	val = OS_REG_READ(ah, AR_SREV);
	HALDEBUG(ah, HAL_DEBUG_ATTACH,
	    "%s: ID 0x%x VERSION 0x%x TYPE 0x%x REVISION 0x%x\n",
	    __func__, MS(val, AR_XSREV_ID), MS(val, AR_XSREV_VERSION),
	    MS(val, AR_XSREV_TYPE), MS(val, AR_XSREV_REVISION));
	/* NB: include chip type to differentiate from pre-Sowl versions */
	AH_PRIVATE(ah)->ah_macVersion =
	    (val & AR_XSREV_VERSION) >> AR_XSREV_TYPE_S;
	AH_PRIVATE(ah)->ah_macRev = MS(val, AR_XSREV_REVISION);
	AH_PRIVATE(ah)->ah_ispcie = (val & AR_XSREV_TYPE_HOST_MODE) == 0;

	/* setup common ini data; rf backends handle remainder */
	HAL_INI_INIT(&ahp->ah_ini_modes, ar9271Modes, 6);
	HAL_INI_INIT(&ahp->ah_ini_common, ar9271Common, 2);
	HAL_INI_INIT(&AH5416(ah)->ah_ini_pcieserdes,
		ar9271NonPciePhy_clkreq, 2);
	ar5416AttachPCIE(ah);

	AH5416(ah)->ah_cal_initcal      = ar9271InitCalHardware;
	AH5416(ah)->ah_cal_pacal       	= ar9271_hw_pa_cal;

	ecode = ath_hal_v4kEepromAttach(ah);
	if (ecode != HAL_OK)
		goto bad;

	if (!ar5416ChipReset(ah, AH_NULL, HAL_RESET_NORMAL)) {	/* reset chip */
		HALDEBUG(ah, HAL_DEBUG_ANY, "%s: chip reset failed\n",
		    __func__);
		ecode = HAL_EIO;
		goto bad;
	}

	AH_PRIVATE(ah)->ah_phyRev = OS_REG_READ(ah, AR_PHY_CHIP_ID);

	if (!ar5212ChipTest(ah)) {
		HALDEBUG(ah, HAL_DEBUG_ANY, "%s: hardware self-test failed\n",
		    __func__);
		ecode = HAL_ESELFTEST;
		goto bad;
	}

	/*
	 * Set correct Baseband to analog shift
	 * setting to access analog chips.
	 */
	OS_REG_WRITE(ah, AR_PHY(0), 0x00000007);

	/* Read Radio Chip Rev Extract */
	AH_PRIVATE(ah)->ah_analog5GhzRev = ar5416GetRadioRev(ah);
	switch (AH_PRIVATE(ah)->ah_analog5GhzRev & AR_RADIO_SREV_MAJOR) {
        case AR_RAD2133_SREV_MAJOR:	/* Sowl: 2G/3x3 */
	case AR_RAD5133_SREV_MAJOR:	/* Sowl: 2+5G/3x3 */
		break;
	default:
		if (AH_PRIVATE(ah)->ah_analog5GhzRev == 0) {
			AH_PRIVATE(ah)->ah_analog5GhzRev =
				AR_RAD5133_SREV_MAJOR;
			break;
		}
#ifdef AH_DEBUG
		HALDEBUG(ah, HAL_DEBUG_ANY,
		    "%s: 5G Radio Chip Rev 0x%02X is not supported by "
		    "this driver\n", __func__,
		    AH_PRIVATE(ah)->ah_analog5GhzRev);
		ecode = HAL_ENOTSUPP;
		goto bad;
#endif
	}
	rfStatus = ar9271RfAttach(ah, &ecode);
	if (!rfStatus) {
		HALDEBUG(ah, HAL_DEBUG_ANY, "%s: RF setup failed, status %u\n",
		    __func__, ecode);
		goto bad;
	}

	HAL_INI_INIT(&ahp9271->ah_ini_rxgain, ar9280Modes_original_rxgain_v2,
	    6);

	/* setup txgain table */
	switch (ath_hal_eepromGet(ah, AR_EEP_TXGAIN_TYPE, AH_NULL)) {
	case AR5416_EEP_TXGAIN_HIGH_POWER:
		HAL_INI_INIT(&ahp9271->ah_ini_txgain,
			ar9271Modes_high_power_tx_gain, 6);	
		break;
	case AR5416_EEP_TXGAIN_ORIG:
		HAL_INI_INIT(&ahp9271->ah_ini_txgain,
			ar9271Modes_original_tx_gain, 6);
		break;
	default:
		HALASSERT(AH_FALSE);
		goto bad;		/* XXX ? try to continue */
	}

	/*
	 * Got everything we need now to setup the capabilities.
	 */
	if (!ar9271FillCapabilityInfo(ah)) {
		ecode = HAL_EEREAD;
		goto bad;
	}

	/*
	 * Print out the EEPROM antenna configuration mapping.
	 * Some devices have a hard-coded LNA configuration profile;
	 * others enable diversity.
	 */
	ar9271_eeprom_print_diversity_settings(ah);

	/* Print out whether the EEPROM settings enable AR9285 diversity */
	if (ar9271_check_div_comb(ah)) {
		ath_hal_printf(ah, "[ath] Enabling diversity for Kite\n");
	}

	/* Disable 11n for the AR2427 */
	if (devid == AR2427_DEVID_PCIE)
		AH_PRIVATE(ah)->ah_caps.halHTSupport = AH_FALSE;

	ecode = ath_hal_eepromGet(ah, AR_EEP_MACADDR, ahp->ah_macaddr);
	if (ecode != HAL_OK) {
		HALDEBUG(ah, HAL_DEBUG_ANY,
		    "%s: error getting mac address from EEPROM\n", __func__);
		goto bad;
        }
	/* XXX How about the serial number ? */
	/* Read Reg Domain */
	AH_PRIVATE(ah)->ah_currentRD =
	    ath_hal_eepromGet(ah, AR_EEP_REGDMN_0, AH_NULL);
	/*
         * For Kite and later chipsets, the following bits are not
	 * programmed in EEPROM and so are set as enabled always.
	 */
	AH_PRIVATE(ah)->ah_currentRDext = AR9285_RDEXT_DEFAULT;

	/*
	 * ah_miscMode is populated by ar5416FillCapabilityInfo()
	 * starting from griffin. Set here to make sure that
	 * AR_MISC_MODE_MIC_NEW_LOC_ENABLE is set before a GTK is
	 * placed into hardware.
	 */
	if (ahp->ah_miscMode != 0)
		OS_REG_WRITE(ah, AR_MISC_MODE, OS_REG_READ(ah, AR_MISC_MODE) | ahp->ah_miscMode);

	ar9271AniSetup(ah);			/* Anti Noise Immunity */

	/* Setup noise floor min/max/nominal values */
	AH5416(ah)->nf_2g.max = AR_PHY_CCA_MAX_GOOD_VAL_9271_2GHZ;
	AH5416(ah)->nf_2g.min = AR_PHY_CCA_MIN_GOOD_VAL_9271_2GHZ;
	AH5416(ah)->nf_2g.nominal = AR_PHY_CCA_NOM_VAL_9271_2GHZ;
	/* XXX no 5ghz values? */

	ar5416InitNfHistBuff(AH5416(ah)->ah_cal.nfCalHist);

	HALDEBUG(ah, HAL_DEBUG_ATTACH, "%s: return\n", __func__);

	return ah;
bad:
	if (ah != AH_NULL)
		ah->ah_detach(ah);
	if (status)
		*status = ecode;
	return AH_NULL;
}

static void
ar9271ConfigPCIE(struct ath_hal *ah, HAL_BOOL restore, HAL_BOOL power_off)
{
}

static void
ar9271DisablePCIE(struct ath_hal *ah)
{
}

static void
ar9271WriteIni(struct ath_hal *ah, const struct ieee80211_channel *chan)
{
	u_int modesIndex, freqIndex;
	int regWrites = 0;

	/* Setup the indices for the next set of register array writes */
	/* XXX Ignore 11n dynamic mode on the AR5416 for the moment */
	freqIndex = 2;
	if (IEEE80211_IS_CHAN_HT40(chan))
		modesIndex = 3;
	else if (IEEE80211_IS_CHAN_108G(chan))
		modesIndex = 5;
	else
		modesIndex = 4;

	/* Set correct Baseband to analog shift setting to access analog chips. */
	OS_REG_WRITE(ah, AR_PHY(0), 0x00000007);
	OS_REG_WRITE(ah, AR_PHY_ADC_SERIAL_CTL, AR_PHY_SEL_INTERNAL_ADDAC);
	regWrites = ath_hal_ini_write(ah, &AH5212(ah)->ah_ini_modes,
	    modesIndex, regWrites);
	if (AR_SREV_KITE_12_OR_LATER(ah)) {
		regWrites = ath_hal_ini_write(ah, &AH9271(ah)->ah_ini_txgain,
		    modesIndex, regWrites);
	}
	regWrites = ath_hal_ini_write(ah, &AH5212(ah)->ah_ini_common,
	    1, regWrites);
}

/*
 * Fill all software cached or static hardware state information.
 * Return failure if capabilities are to come from EEPROM and
 * cannot be read.
 */
static HAL_BOOL
ar9271FillCapabilityInfo(struct ath_hal *ah)
{
	HAL_CAPABILITIES *pCap = &AH_PRIVATE(ah)->ah_caps;

	if (!ar5416FillCapabilityInfo(ah))
		return AH_FALSE;
	pCap->halNumGpioPins = (AH_PRIVATE(ah)->ah_isusb) ? 16 : 12;
	/* Wake-on-Wireless */
	pCap->halWowSupport = AH_FALSE;
	pCap->halWowMatchPatternExact = AH_FALSE;

	/* AR9271 supports one transmit and one receive traffic stream */
	pCap->halTxStreams = 1;
	pCap->halRxStreams = 1;

	if (ar9271_check_div_comb(ah))
		pCap->halAntDivCombSupport = AH_TRUE;

	pCap->halCSTSupport = AH_TRUE;
	pCap->halRifsRxSupport = AH_TRUE;
	pCap->halRifsTxSupport = AH_TRUE;
	pCap->halRtsAggrLimit = 64*1024;	/* 802.11n max */
	pCap->halExtChanDfsSupport = AH_TRUE;
	pCap->halUseCombinedRadarRssi = AH_TRUE;
#if 1
	/* XXX bluetooth */
	pCap->halBtCoexSupport = AH_FALSE;
#endif
	pCap->halAutoSleepSupport = AH_FALSE;	/* XXX? */
	pCap->hal4kbSplitTransSupport = AH_FALSE;
	/* Disable this so Block-ACK works correctly */
	pCap->halHasRxSelfLinkedTail = AH_FALSE;
	pCap->halMbssidAggrSupport = AH_TRUE;  
	pCap->hal4AddrAggrSupport = AH_TRUE;
	pCap->halSpectralScanSupport = AH_TRUE;
	pCap->halRxUsingLnaMixing = AH_TRUE;

	/* TODO: Not sure. For AR9285 and AR9280 it's false for later revs */
	pCap->halPSPollBroken = AH_FALSE;

	/* Only RX STBC supported */
	pCap->halRxStbcSupport = 1;
	pCap->halTxStbcSupport = 0;

	return AH_TRUE;
}

static const char*
ar9271Probe(uint16_t vendorid, uint16_t devid)
{
	if (vendorid == USB_VENDOR_ATHEROS2) {
		if ((devid == USB_PRODUCT_ATHEROS2_AR9271_1) ||
		    (devid == USB_PRODUCT_ATHEROS2_AR9271_2) ||
		    (devid == USB_PRODUCT_ATHEROS2_AR9271_3))
			return "Atheros 9271";
	}
	if (vendorid == USB_VENDOR_AZUREWAVE) {
		if ((devid == USB_PRODUCT_AZUREWAVE_AR9271_1) ||
		    (devid == USB_PRODUCT_AZUREWAVE_AR9271_2) ||
		    (devid == USB_PRODUCT_AZUREWAVE_AR9271_3) ||
		    (devid == USB_PRODUCT_AZUREWAVE_AR9271_4) ||
		    (devid == USB_PRODUCT_AZUREWAVE_AR9271_5) ||
		    (devid == USB_PRODUCT_AZUREWAVE_AR9271_6))
			return "Atheros 9271 AZUREWAVE";
	}
	if (vendorid == USB_VENDOR_DLINK2) {
		if (devid == USB_PRODUCT_DLINK2_AR9271)
			return "Atheros 9271 DLINK2";
	}
	if (vendorid == USB_VENDOR_LITEON) {
		if (devid == USB_PRODUCT_LITEON_AR9271)
			return "Atheros 9271 LITEON";
	}
	if (vendorid == USB_VENDOR_NETGEAR) {
		if (devid == USB_PRODUCT_NETGEAR_WNA1100)
			return "Atheros 9271 NETGEAR";
	}
	if (vendorid == USB_VENDOR_VIA) {
		if (devid == USB_PRODUCT_VIA_AR9271)
			return "Atheros 9271 VIA";
	}
	return AH_NULL;
}
AH_CHIP(AR9271, ar9271Probe, ar9271Attach);
