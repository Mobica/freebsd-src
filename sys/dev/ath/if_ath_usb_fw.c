#include <sys/types.h>
#include <sys/param.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/firmware.h>
#include <sys/sockio.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/endian.h>

#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_media.h>

#include <net/if_var.h>

#include <net/if_arp.h>
#include <netinet/in.h>

#include <net/ethernet.h>
#include <net80211/ieee80211_freebsd.h>
#include <net80211/ieee80211_power.h>
#include <net80211/ieee80211_node.h>
#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_amrr.h>
#include <net80211/ieee80211_radiotap.h>
#include <openbsd_adapt.h>

#include "ar5212/ar5212reg.h"
#include "dev/ath/if_athvar.h"


// FreeBSD version
#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>

#include "if_ath_usb_def.h"

#define AR9271_CLOCK_CONTROL		0x050040
#define AR9271_RESET_POWER_DOWN_CONTROL	0x050044
#define AR9271_FIRMWARE			0x501000
#define AR9271_FIRMWARE_TEXT		0x903000
#define AR7010_FIRMWARE_TEXT		0x906000
static const struct firmware *fware = NULL;

extern int tsleep_nsec(const void *ident, int priority, const char *wmesg, uint64_t nsecs);

void
ath_usb_unload_firmware()
{
	if (fware == NULL) {
		printf("Fail: null firmware handler\n");
		return;
	}

	firmware_put(fware, FIRMWARE_UNLOAD);
	printf("Successully called firmware_put\n");
	return;
}

int
ath_usb_get_firmware(struct ath_usb_softc *usc, struct ar_wmi_fw_version *version)
{
	usb_device_descriptor_t *dd;
	const char *name;
	int error = ENXIO;

	/* Determine which firmware image to load. */
	if (usc->flags & ATH_USB_FLAG_AR7010) {
		dd = usbd_get_device_descriptor(usc->sc_udev);
		name = "ath-open-ar7010.bin";
	} else
		name = "athn-open-ar9271.bin";
	/* Read firmware image from the filesystem. */
	fware = firmware_get(name);
	if (fware == NULL) {
		printf("Failed firmware_get of file %s\n", name);
		return (error);
	} else {
		printf("Success firmware_get of file %s\n", name);
		printf("Firmware name: %s:\n", fware->name);
		printf("Firmware version: %u:\n", fware->version);
		printf("Firmware size: %zu:\n", fware->datasize);
		version->major = fware->version / 100;
		version->minor = (fware->version / 10) % 10;
		return 0;
	}
}

int
ath_usb_transfer_firmware(struct ath_usb_softc *usc)
{
	usb_device_request_t req;
	const char *name;
	void *ptr;
	size_t fwsize, size;
	uint32_t addr;
	int s, mlen;
	int error = 0;

	ATH_LOCK(&usc->sc_sc);

/* Load firmware image. */
	ptr = (void *)fware->data;
	addr = AR9271_FIRMWARE >> 8;
	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = AR_FW_DOWNLOAD;
	USETW(req.wIndex, 0);
	size = fware->datasize;

	while (size > 0) {
		printf("Uploading %zu bytes\n", size);
		mlen = MIN(size, 4096);

		USETW(req.wValue, addr);
		USETW(req.wLength, mlen);

		error = usbd_do_request(usc->sc_udev, &usc->sc_sc.sc_mtx, &req, ptr);
		if (error != 0) {
			ATH_UNLOCK(&usc->sc_sc);
			ath_usb_unload_firmware();
			return (error);
		}
		addr += mlen >> 8;
		ptr  += mlen;
		size -= mlen;
	}

	/* Start firmware. */
	if (usc->flags & ATH_USB_FLAG_AR7010)
		addr = AR7010_FIRMWARE_TEXT >> 8;
	else {
		addr = AR9271_FIRMWARE_TEXT >> 8;
		device_printf(usc->sc_sc.sc_dev, "%s: selected device: AR9271\n", __func__);
	}
	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = AR_FW_DOWNLOAD_COMP;
	USETW(req.wIndex, 0);
	USETW(req.wValue, addr);
	USETW(req.wLength, 0);

	usc->wait_msg_id = AR_HTC_MSG_READY;
	error = usbd_do_request(usc->sc_udev, &usc->sc_sc.sc_mtx, &req, NULL);

	/* Wait at most 1 second for firmware to boot. */
	device_printf(usc->sc_sc.sc_dev, "%s: Waiting for firmware to boot\n", __func__);
	if (error == 0 && usc->wait_msg_id != 0)
		error = msleep(&usc->wait_msg_id, &usc->sc_sc.sc_mtx, PCATCH, "athnfw", hz);

	if (usc->wait_msg_id == 0) {
		device_printf(usc->sc_sc.sc_dev, "%s: Firmware booted successfully!\n", __func__);
	}

	ATH_UNLOCK(&usc->sc_sc);

	ath_usb_unload_firmware();

	return (error);
}

int
ath_usb_load_firmware(struct ath_usb_softc *usc, struct ar_wmi_fw_version *img_version)
{
	int error = 0;
	
	error = ath_usb_get_firmware(usc, img_version);
	if (error) {
		printf("Failed to get firmware\n");
		return error;
	}

	error = ath_usb_transfer_firmware(usc);
	if (error) {
		printf("Failed to transfer firmware to the device\n");
	}

	return error;
}
