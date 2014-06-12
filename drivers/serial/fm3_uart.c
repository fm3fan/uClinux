/*
 * (C) Copyright 2013
 * Kentaro Sekimoto
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/console.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/serial_reg.h>
#include <linux/tty.h>

#include <mach/fm3.h>
#include <mach/platform.h>
#include <mach/gpio.h>
#include <mach/uart.h>
#include <mach/clock.h>

#define FM3_UART_NUM	8
#define FM3_UART_NAME   "ttyS"
#define FM3_UART_PORT   "FM3 UART Port"

#define	PCLK2	72000000	// ToDo

struct fm3_uart_priv {
	volatile struct fm3_uart_regs *reg_uart_base;
	unsigned int	ch_mod;
	int uart_rx_irq;
    int uart_tx_irq;
};

#define fm3_drv_priv(port)  (struct fm3_uart_priv *)((port)->private_data)

static struct fm3_uart_regs *FM3_UART_REGS[] = {
    (struct fm3_uart_regs  *)FM3_MFS0_UART_BASE,
    (struct fm3_uart_regs  *)FM3_MFS1_UART_BASE,
    (struct fm3_uart_regs  *)FM3_MFS2_UART_BASE,
    (struct fm3_uart_regs  *)FM3_MFS3_UART_BASE,
    (struct fm3_uart_regs  *)FM3_MFS4_UART_BASE,
    (struct fm3_uart_regs  *)FM3_MFS5_UART_BASE,
    (struct fm3_uart_regs  *)FM3_MFS6_UART_BASE,
    (struct fm3_uart_regs  *)FM3_MFS7_UART_BASE
};

static struct uart_port FM3_PORTS[FM3_UART_NUM];
static struct fm3_uart_priv FM3_UART_PRIV[FM3_UART_NUM];

static void fm3_tx_char(volatile struct fm3_uart_regs *uart, unsigned char c);
static void fm3_transmit(struct uart_port *port);

static irqreturn_t fm3_uart_tx_isr(int irq, void *dev_id);
static irqreturn_t fm3_uart_rx_isr(int irq, void *dev_id);

struct fm3_uart_regs *uart_regs_by_port(struct uart_port *port)
{
    return FM3_UART_REGS[port->line];
}

static unsigned int fm3_port_tx_empty(struct uart_port *port)
{
	unsigned long flags;
    unsigned int ret;
    volatile struct fm3_uart_regs *uart_regs = uart_regs_by_port(port);
    spin_lock_irqsave(&port->lock, flags);
    ret = (uart_regs->SSR & SSR_TDRE) ? TIOCSER_TEMT : 0;
    spin_unlock_irqrestore(&port->lock, flags);
    return ret;
}

static void fm3_port_stop_tx(struct uart_port *port)
{
    unsigned long flags;
    volatile struct fm3_uart_regs *uart_regs = uart_regs_by_port(port);
    spin_lock_irqsave(&port->lock, flags);
    uart_regs->SCR  &= ~(SCR_TIE);
    spin_unlock_irqrestore(&port->lock, flags);
}

static void fm3_port_start_tx(struct uart_port *port)
{
    unsigned long flags;
    volatile struct fm3_uart_regs *uart_regs = uart_regs_by_port(port);
    spin_lock_irqsave(&port->lock, flags);
    uart_regs->SCR  |= (SCR_TIE);
    spin_unlock_irqrestore(&port->lock, flags);
    fm3_transmit(port);
}

static void fm3_port_stop_rx(struct uart_port *port)
{
    unsigned long flags;
    volatile struct fm3_uart_regs *uart_regs = uart_regs_by_port(port);
    spin_lock_irqsave(&port->lock, flags);
    uart_regs->SCR  &= ~(SCR_RXE | SCR_RIE);
    spin_unlock_irqrestore(&port->lock, flags);
}

static void fm3_port_start_rx(struct uart_port *port)
{
    unsigned long flags;
    volatile struct fm3_uart_regs *uart_regs = uart_regs_by_port(port);
    spin_lock_irqsave(&port->lock, flags);
    uart_regs->SCR  |= (SCR_RXE | SCR_RIE);
    spin_unlock_irqrestore(&port->lock, flags);
}

static void fm3_set_baud_rate(struct uart_port *port, int baudrate)
{
    unsigned long flags;
    volatile struct fm3_uart_regs *uart_regs = uart_regs_by_port(port);
    spin_lock_irqsave(&port->lock, flags);
    uart_regs->BGR = (uint16_t)(((PCLK2 + ((unsigned int)baudrate / 2)) / (unsigned int)baudrate) - 1);
    spin_unlock_irqrestore(&port->lock, flags);
}

static int fm3_port_startup(struct uart_port *port)
{
    volatile struct fm3_uart_regs *uart_regs = uart_regs_by_port(port);
    struct fm3_uart_priv *priv = fm3_drv_priv(port);
    int ret = 0;
    ret= request_irq(priv->uart_rx_irq, fm3_uart_rx_isr, IRQF_DISABLED | IRQF_SAMPLE_RANDOM,
            FM3_UART_PORT, port);
    if (ret) {
        printk(KERN_ERR "%s: request_irq (%d)failed (%d)\n", __func__, priv->uart_rx_irq, ret);
        return ret;
    }
    ret= request_irq(priv->uart_tx_irq, fm3_uart_tx_isr, IRQF_DISABLED | IRQF_SAMPLE_RANDOM,
            FM3_UART_PORT, port);
    if (ret) {
        printk(KERN_ERR "%s: request_irq (%d)failed (%d)\n", __func__, priv->uart_tx_irq, ret);
        return ret;
    }

	fm3_mfs_set_mode(port->line*3 + priv->ch_mod);
	uart_regs->SMR = (SMR_MD_UART | SMR_SOE);
	uart_regs->SCR = 0x00;
    fm3_set_baud_rate(port, 115200);
	uart_regs->SCR &= ~SCR_UPGL;
	uart_regs->SSR = 0x00;
	uart_regs->ESCR = 0;
	uart_regs->SCR  |= (SCR_TXE);
    fm3_port_start_rx(port);
    fm3_port_start_tx(port);
    return ret;
}

static void fm3_port_shutdown(struct uart_port *port)
{
    struct fm3_uart_priv *priv = fm3_drv_priv(port);
    fm3_port_stop_rx(port);
    fm3_port_stop_tx(port);
    free_irq(priv->uart_rx_irq, port);
    free_irq(priv->uart_tx_irq, port);
}

static const char *fm3_port_type(struct uart_port *port)
{
    return FM3_UART_PORT;
}

static void fm3_port_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old)
{
    // ToDo
}

static int fm3_port_verify_port(struct uart_port *port, struct serial_struct *ser)
{
    return -EINVAL;
}

static void fm3_port_release_port(struct uart_port *port)
{
    // Nothing to do
}

static int fm3_port_request_port(struct uart_port *port)
{
    // Nothing to do
    return 0;
}

static void fm3_port_config_port(struct uart_port *port, int flags)
{
    if (!fm3_port_request_port(port))
        port->type = PORT_FM3UART;
}

static unsigned int fm3_port_get_mctrl(struct uart_port *port)
{
    unsigned int ret = 0;
    return ret;
}

static void fm3_port_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
    // Nothing to do
}

static void fm3_port_enable_ms(struct uart_port *port)
{
    // Nothing to do
}

static void fm3_port_break_ctl(struct uart_port *port, int ctl)
{
    // Nothing to do
}

static struct uart_ops fm3_uart_ops = {
    .tx_empty   = fm3_port_tx_empty,
    .set_mctrl  = fm3_port_set_mctrl,
    .get_mctrl  = fm3_port_get_mctrl,
    .stop_tx    = fm3_port_stop_tx,
    .start_tx   = fm3_port_start_tx,
    .stop_rx    = fm3_port_stop_rx,
    .enable_ms  = fm3_port_enable_ms,
    .break_ctl  = fm3_port_break_ctl,
    .startup    = fm3_port_startup,
    .shutdown   = fm3_port_shutdown,
    .set_termios    = fm3_port_set_termios,
    .type       = fm3_port_type,
    .release_port   = fm3_port_release_port,
    .request_port   = fm3_port_request_port,
    .config_port    = fm3_port_config_port,
    .verify_port    = fm3_port_verify_port
};

#ifdef CONFIG_SERIAL_FM3_CONSOLE

static void fm3_console_putchar(struct uart_port *port, int ch)
{
    volatile struct fm3_uart_regs *uart_regs = uart_regs_by_port(port);
    fm3_tx_char(uart_regs, ch);
}

static void fm3_console_write(struct console *co, const char *s, unsigned int count)
{
    struct uart_port *port = &FM3_PORTS[co->index];
    unsigned long flags;
    int locked;

    if (oops_in_progress) {
        locked = spin_trylock_irqsave(&port->lock, flags);
    } else {
        locked = 1;
        spin_lock_irqsave(&port->lock, flags);
    }
    uart_console_write(port, s, count, fm3_console_putchar);

    if (locked)
        spin_unlock_irqrestore(&port->lock, flags);
}

static int __init fm3_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
    int ret = 0;
    int baud = 115200;
    int bits = 8;
    int parity = 'n';
    int flow = 'n';

    if (co->index < 0 || co->index >= FM3_UART_NUM) {
        ret = -EINVAL;
    } else {
        port = &FM3_PORTS[co->index];
        if (!port->private_data) {
            pr_debug("console on %s%i not present\n", FM3_UART_NAME, co->index);
            ret = -ENODEV;
        } else {
            if (options)
                uart_parse_options(options, &baud, &parity, &bits, &flow);
            ret = uart_set_options(port, co, baud, parity, bits, flow);
        }
    }
    return ret;
}

static struct uart_driver fm3_uart_driver;
static struct console fm3_console = {
    .name   = FM3_UART_NAME,
    .device = uart_console_device,
    .write  = fm3_console_write,
    .setup  = fm3_console_setup,
    .flags  = CON_PRINTBUFFER,
    .index  = -1,
    .data   = &fm3_uart_driver,
};

#endif
/* CONFIG_SERIAL_FM3_CONSOLE */

static struct uart_driver   fm3_uart_driver = {
    .owner      = THIS_MODULE,
    .driver_name    = FM3_UART_DRV_NAME,
    .dev_name   = FM3_UART_NAME,
    .major      = TTY_MAJOR,
    .minor      = 64,
    .nr     = FM3_UART_NUM,
#ifdef CONFIG_SERIAL_FM3_CONSOLE
    .cons       = &fm3_console,
#endif
};

static void fm3_tx_char(volatile struct fm3_uart_regs *uart_regs, unsigned char c)
{
    while ((uart_regs->SSR & SSR_TDRE) == 0) ;
    uart_regs->TDR = (unsigned short)c;
}

static void fm3_transmit(struct uart_port *port)
{
    struct circ_buf *xmit;
    volatile struct fm3_uart_regs *uart_regs = uart_regs_by_port(port);

    if (port->x_char) {
        fm3_tx_char(uart_regs, port->x_char);
        port->x_char = 0;
        port->icount.tx++;
        return;
    }
    xmit = &port->state->xmit;
    if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
        fm3_port_stop_tx(port);
        return;
    }
    fm3_tx_char(uart_regs, xmit->buf[xmit->tail]);
    xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
    port->icount.tx++;

    if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
        uart_write_wakeup(port);
    return;
}

static void fm3_recieve(struct uart_port *port)
{
    struct tty_struct *tty = port->state->port.tty;
    volatile struct fm3_uart_regs *uart_regs = uart_regs_by_port(port);
    unsigned char c;
    do {
    	if (uart_regs->SSR & SSR_ORE) {
    		uart_regs->SSR |= SSR_REC;
    		return;
    	}
        c = (unsigned char)uart_regs->RDR;
        port->icount.rx += 1;
		if (uart_handle_sysrq_char(port, c))
			;
		else
			tty_insert_flip_char(tty, c, TTY_NORMAL);
    } while (uart_regs->SSR & SSR_RDRF);
    tty_flip_buffer_push(tty);
}

static irqreturn_t fm3_uart_tx_isr(int irq, void *dev_id)
{
    fm3_transmit((struct uart_port *)dev_id);
    return IRQ_HANDLED;
}

static irqreturn_t fm3_uart_rx_isr(int irq, void *dev_id)
{
    fm3_recieve((struct uart_port *)dev_id);
    return IRQ_HANDLED;
}

static int __devexit fm3_release(struct device *dev)
{
    int ret = 0;
    return ret;
}

struct plat_uart_port {
    void __iomem    *membase;
    unsigned long   mapbase;
    unsigned int    irqs[2];
    unsigned int    type;
    char *clk;
};

static int __devinit fm3_probe(struct platform_device *pdev)
{
	struct fm3_uart_drvdata *d;
    struct fm3_uart_priv *priv;
    struct uart_port    *port;
    int ret;
    int id = pdev->id;
    struct device *dev = &pdev->dev;

    if (id >= FM3_UART_NUM) {
        dev_err(dev, "%s: bad port id %d\n", __func__, id);
        ret = -EINVAL;
        return ret;
    }
    if (id < 0) {
        for (id = 0; id < FM3_UART_NUM; id++) {
            if (!FM3_PORTS[id].mapbase)
                break;
        }
    }
    if (id == FM3_UART_NUM) {
        dev_err(dev, "%s: no free port\n", __func__);
        ret = -EBUSY;
        return ret;
    }

	priv = &FM3_UART_PRIV[id];
    priv->uart_rx_irq = MFS0RX_IRQn + id * 2;
    priv->uart_tx_irq = MFS0TX_IRQn + id * 2;
    priv->reg_uart_base = FM3_UART_REGS[id];
	d = (struct fm3_uart_drvdata *)platform_get_drvdata(pdev);
	priv->ch_mod = d->ch_mod;
    port = &FM3_PORTS[id];

    spin_lock_init(&port->lock);
    port->iotype  = SERIAL_IO_MEM,
    port->irq     = priv->uart_rx_irq;
    port->flags   = UPF_BOOT_AUTOCONF;
    port->line    = id;
    port->ops     = &fm3_uart_ops;
    port->dev     = dev;
    port->mapbase = (u32)priv->reg_uart_base;

    port->private_data = &FM3_UART_PRIV[id];
    dev_set_drvdata(dev, port);

    ret = uart_add_one_port(&fm3_uart_driver, port);
    if (ret) {
        dev_err(dev, "%s: uart_add_one_port failed (%d)\n", __func__, ret);
        dev_set_drvdata(dev, NULL);
        priv->reg_uart_base = NULL;
    }
    return ret;
}

static int __devexit fm3_remove(struct platform_device *pdev)
{
    return fm3_release(&pdev->dev);
}

static struct platform_driver fm3_platform_driver = {
    .probe  = fm3_probe,
    .remove = __devexit_p(fm3_remove),
    .driver = {
        .owner  = THIS_MODULE,
        .name   = FM3_UART_DRV_NAME,
    },
};

static int __init _fm3_uart_init(void)
{
    int ret;
    ret = uart_register_driver(&fm3_uart_driver);
    if (ret != 0)
        goto fail;
    ret = platform_driver_register(&fm3_platform_driver);
    if (ret != 0) {
        uart_unregister_driver(&fm3_uart_driver);
       goto fail;
    }
    pr_debug(FM3_UART_DRV_NAME " initialized OK\n");
    goto out;
fail:
    pr_debug(FM3_UART_DRV_NAME " initialized NG\n");
out:
    return ret;
}

static void __exit _fm3_uart_exit(void)
{
    platform_driver_unregister(&fm3_platform_driver);
    uart_unregister_driver(&fm3_uart_driver);
}

module_init(_fm3_uart_init);
module_exit(_fm3_uart_exit);

MODULE_AUTHOR("Kentaro Sekimoto");
MODULE_DESCRIPTION("FM3 UARTÅ@driver");
MODULE_LICENSE("GPL");
