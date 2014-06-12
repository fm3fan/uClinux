/*
 * (C) Copyright 2013
 * Kentaro Sekimoto
 *
 * Based on emcraft implementation.
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

#include <linux/clockchips.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <asm/hardware/cortexm3.h>
#include <mach/clock.h>
#include <mach/fm3.h>
#include <mach/timer.h>

extern void cortex_m3_register_systick_clocksource(u32 systick_clk);
static void tick_tmr_set_mode(enum clock_event_mode mode, struct clock_event_device *clk);
static irqreturn_t tick_tmr_irq_handler(int irq, void *dev_id);

static struct clock_event_device tick_tmr_clockevent = {
    .name       = "FM3 System Timer",
#if 0
    .rating     = 200,
#endif
    .irq        = DTIM_QDU_IRQn,
    .features   = CLOCK_EVT_FEAT_PERIODIC,
    .set_mode   = tick_tmr_set_mode,
    .cpumask    = cpu_all_mask,
};

static struct irqaction tick_tmr_irqaction = {
    .name       = "FM3 Kernel Time Tick",
    .flags      = IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
    .handler    = tick_tmr_irq_handler,
};

static void tick_tmr_set_mode(enum clock_event_mode mode, struct clock_event_device *clk)
{
    switch (mode) {
    case CLOCK_EVT_MODE_PERIODIC:
        FM3_DTIM->TIMER1CONTROL |= 0x80;
        break;
    case CLOCK_EVT_MODE_UNUSED:
    case CLOCK_EVT_MODE_SHUTDOWN:
    default:
        FM3_DTIM->TIMER1CONTROL &= ~0x80;
        break;
    }
}

#define MAX_TIMER_COUNT 0xffffffff

static irqreturn_t tick_tmr_irq_handler(int irq, void *dev_id)
{
    struct clock_event_device *clkevt = &tick_tmr_clockevent;

    FM3_DTIM->TIMER1INTCLR = 0;
    clkevt->event_handler(clkevt);
    return IRQ_HANDLED;
}

static void clocksource_tmr_init(void)
{
    cortex_m3_register_systick_clocksource(fm3_clock_get(CLOCK_SYSTICK));
}

#define FM3_PRESCALE    16

static void clockevents_tmr_init(void)
{
    unsigned int tick_tmr_clk;
    struct clock_event_device *clkevt = &tick_tmr_clockevent;
    tick_tmr_clk = fm3_clock_get(CLOCK_SYSCLK)/FM3_PRESCALE;
    FM3_DTIM->TIMER1INTCLR = 0;             // Clear interrupt flag with any value
    FM3_DTIM->TIMER1CONTROL = 0;            // Disable timer1 operation
    FM3_DTIM->TIMER1CONTROL =
            (DtimDev_CYCLE_MODE << 6) |     // cycle mode
            (DtimDev_INT_ENABLE  << 5) |    // enable timer interrupt
            (DtimDev_TIMER_PRE_16 << 2) |   // prescale 1/16 (FM3_PRESCALE)
            (DtimDev_TIMER_SIZE_32 << 1);   // size 32bit
    FM3_DTIM->TIMER1LOAD = (unsigned int)(tick_tmr_clk/HZ);
    setup_irq(DTIM_QDU_IRQn, &tick_tmr_irqaction);
#if 0
    clockevents_calc_mult_shift(clkevt, tick_tmr_clk/HZ, 5);
    clkevt->max_delta_ns = clockevent_delta2ns(0xFFFFFFF0, clkevt);
    clkevt->min_delta_ns = clockevent_delta2ns(0xF, clkevt);
#endif
    clockevents_register_device(clkevt);
    FM3_DTIM->TIMER1CONTROL |= (unsigned int)0x80;  // Enable timer1 operation
}

void __init fm3_timer_init(void)
{
    fm3_clock_init();
    clocksource_tmr_init();
    clockevents_tmr_init();
}
