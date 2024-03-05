#ifndef IF_ATH_USB_FW_H
#define IF_ATH_USB_FW_H

struct ath_usb_softc;

int	 ath_usb_load_firmware(struct ath_usb_softc *, struct ar_wmi_fw_version *);
const struct firmware* ath_usb_unload_firmware();

#endif	/* IF_ATH_USB_FW_H */
