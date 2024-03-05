#ifndef IF_ATH_USB_H
#define IF_ATH_USB_H

struct ath_usb_softc;
struct ath_softc;

void
ath_usb_write(struct ath_softc *sc, uint32_t addr, uint32_t val);
uint32_t
ath_usb_read(struct ath_softc *sc, uint32_t addr);
#endif /* IF_ATH_USB_H */
