#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]
#![allow(unused_imports)]

use core::sync::atomic::{AtomicUsize, Ordering};
use defmt_rtt as _;
use panic_probe as _;

#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

static COUNT: AtomicUsize = AtomicUsize::new(0);
defmt::timestamp!("{=usize}", {
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = COUNT.load(Ordering::Relaxed);
    COUNT.store(n + 1, Ordering::Relaxed);
    n
});

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

#[link_section = ".boot2"]
#[used]
pub static BOOT_LOADER: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

/* declare the RTIC application itself */
#[rtic::app(device = hal::pac, peripherals = true, dispatchers=[SPI0_IRQ])]
mod app {

    use defmt::*;

    use core::iter::once;

    use embedded_hal::timer::CountDown;
    use rp2040_hal as hal;
    use hal::{
        clocks::init_clocks_and_plls,
        clocks::Clock,
        pac,
        sio::Sio,
        watchdog::Watchdog,
        gpio::{Pin, FunctionPio0},
        gpio::pin::bank0::Gpio14,
        timer::Timer,
        pio::PIOExt,
    };
    use pico::XOSC_CRYSTAL_FREQ;
    use systick_monotonic::Systick;
    use rtic::time::duration::*;

    use ws2812_pio::Ws2812;
    use smart_leds::{brightness, SmartLedsWrite, RGB8};

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<100>;

    #[shared]
    struct Shared {
    }

    #[local]
    struct Local {
        leds: Ws2812<(pac::PIO0, hal::pio::SM0), hal::timer::CountDown<'static>>,
    }


    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
		info!("init");
        let mut resets = c.device.RESETS;
        let mut watchdog = Watchdog::new(c.device.WATCHDOG);
        let clocks = init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let sio = Sio::new(c.device.SIO);
        let pins = hal::gpio::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let mono = Systick::new(c.core.SYST, clocks.system_clock.freq().0);

        let timer = Timer::new(c.device.TIMER, &mut resets);

        let _leds_pin: Pin<_, FunctionPio0> = pins.gpio14.into_mode();
        let (mut pio, sm0, _, _, _) = c.device.PIO0.split(&mut resets);
        let mut leds = Ws2812::new(
            14, // Gpio14
            &mut pio,
            sm0,
            clocks.peripheral_clock.freq(),
            timer.count_down(),
        );

        turn_red::spawn_after(2.seconds()).unwrap();

        (
            Shared {
            },
            Local {
                leds,
            },
            init::Monotonics(mono),
        )
    }

    #[task(local=[leds])]
    fn turn_red(c: turn_red::Context)
    {
        info!("turn_red");
        c.local.leds.write(once((255, 0, 0))).unwrap();
    }
}
