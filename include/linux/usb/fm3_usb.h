/*
 * board initialization should put one of these into dev->platform_data
 * and place the fm3 onto platform_bus named "fm3_usb-hcd".
 */

#ifndef __LINUX_USB_FM3_USB_H
#define __LINUX_USB_FM3_USB_H

struct fm3_usb_platform_data {
	unsigned	can_wakeup:1;
	/* given port_power, msec/2 after power on till power good */
	u8		potpg;
	/* mA/2 power supplied on this port (max = default = 250) */
	u8		power;
	/* fm3_usb relies on an external source of VBUS current */
	void		(*port_power)(struct device *dev, int is_on);
	/* pulse fm3_usb nRST (probably with a GPIO) */
	void		(*reset)(struct device *dev);
	/* some boards need something like these: */
	/* int		(*check_overcurrent)(struct device *dev); */
	/* void		(*clock_enable)(struct device *dev, int is_on); */
};

#endif /* __LINUX_USB_FM3_USB_H */
