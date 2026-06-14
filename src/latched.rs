//! ISR-driven HUB75 latched panel driver.
//!
//! Use the [`hub75_define!`] macro to create a per-instance module containing
//! the ISR state, DMA handler, and driver struct. Each invocation stamps out an
//! independent instance bound to specific timer and DMA channel types, allowing
//! multiple HUB75 panels to run simultaneously.
//!
//! Once [`Hub75::start()`] is called, DMA transfer-complete interrupts drive
//! the BCM refresh loop autonomously. The timer generates a PWM pixel clock on
//! CH1 and triggers DMA byte-transfers from the framebuffer to the GPIO ODR
//! byte. The ISR stops and resets the timer between planes for deterministic
//! clock alignment.
//!
//! Embassy's [`dma::InterruptHandler`] clears hardware flags portably; the
//! generated [`Hub75DmaHandler`] runs immediately after to advance the BCM
//! state machine and kick the next DMA transfer. Both are chained on the same
//! interrupt vector via [`bind_interrupts!`](embassy_stm32::bind_interrupts).

/// Define a HUB75 driver instance with its own ISR state and handler.
///
/// Each invocation creates a public module containing:
/// - `Hub75DmaHandler` — the DMA interrupt handler for `bind_interrupts!`
/// - `Hub75<'d, FB>` — the driver struct (idle and running typestates)
///
/// # Parameters
/// - `$mod_name` — name of the generated module
/// - `$timer` — the concrete timer peripheral type (e.g. `peripherals::TIM2`)
/// - `$dma_ch` — the concrete DMA channel type (e.g. `peripherals::DMA1_CH1`)
///
/// # Example
/// ```ignore
/// use embassy_stm32::{bind_interrupts, dma, peripherals};
/// use embassy_stm32_hub75::hub75_define;
///
/// hub75_define!(hub75, embassy_stm32::peripherals::TIM2, embassy_stm32::peripherals::DMA1_CH1);
///
/// bind_interrupts!(struct Irqs {
///     DMA1_CHANNEL1 =>
///         dma::InterruptHandler<peripherals::DMA1_CH1>,
///         hub75::Hub75DmaHandler;
/// });
/// ```
#[macro_export]
macro_rules! hub75_define {
    ($mod_name:ident, $timer:ty, $dma_ch:ty) => {
        #[allow(non_snake_case)]
        pub mod $mod_name {
            use core::cell::RefCell;
            use core::future::poll_fn;
            use core::marker::PhantomData;
            use core::mem::ManuallyDrop;
            use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
            use core::task::{Poll, Waker};

            use $crate::__macro_support::critical_section::Mutex;
            use $crate::__macro_support::embassy_stm32::dma::{
                self, Channel, Transfer, TransferOptions,
            };
            use $crate::__macro_support::embassy_stm32::gpio::{OutputType, Pin};
            use $crate::__macro_support::embassy_stm32::interrupt::typelevel::{
                Binding, Handler,
            };
            use $crate::__macro_support::embassy_stm32::pac;
            use $crate::__macro_support::embassy_stm32::time::Hertz;
            use $crate::__macro_support::embassy_stm32::timer::low_level::{
                CountingMode, OutputCompareMode, RoundTo, Timer,
            };
            use $crate::__macro_support::embassy_stm32::timer::simple_pwm::PwmPin;
            use $crate::__macro_support::embassy_stm32::timer::{
                Ch1, Channel as TimChannel, GeneralInstance4Channel, TimerPin,
                UpDma,
            };
            use $crate::__macro_support::embassy_stm32::Peri;

            use $crate::bcm::{planes_from_fb, BcmState, PlaneInfo};
            use $crate::framebuffer::FrameBuffer;
            use $crate::{Hub75Error, Hub75Pins8, Idle};

            // ---------------------------------------------------------------
            // ISR shared state
            // ---------------------------------------------------------------

            struct IsrState {
                timer: ManuallyDrop<Timer<'static, $timer>>,
                channel: Channel<'static>,
                transfer: Option<Transfer<'static>>,
                dma_request: dma::Request,
                odr_byte_addr: *mut u8,
                bcm: BcmState,
                current_fb_ptr: *const (),
                pending_planes: Option<PlaneInfo>,
                pending_fb_ptr: *const (),
                returned_fb_ptr: *const (),
            }

            // SAFETY: All access is serialised by `critical_section`.
            unsafe impl Send for IsrState {}

            static ISR_STATE: Mutex<RefCell<Option<IsrState>>> =
                Mutex::new(RefCell::new(None));
            static SWAP_DONE: AtomicBool = AtomicBool::new(false);
            static SWAP_WAKER: Mutex<RefCell<Option<Waker>>> =
                Mutex::new(RefCell::new(None));
            static FRAME_COUNT: AtomicU32 = AtomicU32::new(0);

            fn signal_swap_done(
                cs: $crate::__macro_support::critical_section::CriticalSection,
            ) {
                SWAP_DONE.store(true, Ordering::Release);
                if let Some(waker) = SWAP_WAKER.borrow_ref_mut(cs).take() {
                    waker.wake();
                }
            }

            // ---------------------------------------------------------------
            // DMA interrupt handler
            // ---------------------------------------------------------------

            /// HUB75 DMA interrupt handler for this instance.
            ///
            /// Must be bound to the same DMA channel interrupt as
            /// [`dma::InterruptHandler`], listed **after** it in
            /// [`bind_interrupts!`] so that embassy clears hardware flags first.
            pub struct Hub75DmaHandler;

            impl Handler<
                <$dma_ch as $crate::__macro_support::embassy_stm32::dma::ChannelInstance>::Interrupt,
            > for Hub75DmaHandler
            {
                unsafe fn on_interrupt() {
                    $crate::__macro_support::critical_section::with(|cs| {
                        let mut borrow = ISR_STATE.borrow_ref_mut(cs);
                        let state = match borrow.as_mut() {
                            Some(s) => s,
                            None => return,
                        };

                        // Stop and reset timer for deterministic clock alignment.
                        state.timer.stop();
                        state.timer.reset();

                        // Drop the completed transfer (instant — DMA already finished).
                        if let Some(transfer) = state.transfer.take() {
                            drop(transfer);
                        }

                        // Advance BCM state machine.
                        let frame_boundary = state.bcm.advance();

                        if frame_boundary {
                            FRAME_COUNT.fetch_add(1, Ordering::Relaxed);

                            if let Some(pending) = state.pending_planes.take() {
                                state.returned_fb_ptr = state.current_fb_ptr;
                                state.current_fb_ptr = state.pending_fb_ptr;
                                state.pending_fb_ptr = core::ptr::null();
                                state.bcm.update_planes(pending);
                                signal_swap_done(cs);
                            }
                        }

                        // Start next DMA transfer, then start timer.
                        let (ptr, len) = state.bcm.current_plane();
                        let buf = core::slice::from_raw_parts(ptr, len);
                        let new_transfer = state.channel.write_raw(
                            state.dma_request,
                            buf,
                            state.odr_byte_addr,
                            TransferOptions::default(),
                        );

                        // SAFETY: Transfer<'a> contains Channel<'a> which is just a u8 +
                        // PhantomData. The channel it borrows lives in this same static,
                        // so the referent outlives the reference.
                        state.transfer = Some(
                            core::mem::transmute::<Transfer<'_>, Transfer<'static>>(
                                new_transfer,
                            ),
                        );

                        // Start timer — first UEV fires after exactly ARR+1 ticks.
                        state.timer.start();
                    });
                }
            }

            // ---------------------------------------------------------------
            // Public driver handle
            // ---------------------------------------------------------------

            /// HUB75 LED matrix controller driven by an ISR-based BCM refresh
            /// loop.
            ///
            /// Uses a typestate pattern: `Hub75<'d, Idle>` is the idle state
            /// returned by [`Hub75::new()`]. Calling [`Hub75::start()`] consumes
            /// it and returns `Hub75<'d, FB>`, locking in the framebuffer type.
            /// [`Hub75::swap()`] is only available on the running state.
            pub struct Hub75<'d, FB: FrameBuffer + 'static = Idle> {
                _clock_pin: PwmPin<'d, $timer, Ch1>,
                _fb: PhantomData<&'static FB>,
            }

            impl<'d> Hub75<'d, Idle> {
                /// Create a new HUB75 driver and configure hardware.
                ///
                /// Sets up GPIO pins, timer PWM clock, and DMA channel. Does
                /// **not** start rendering — call [`Hub75::start()`] with a
                /// framebuffer to begin.
                pub fn new(
                    tim: Peri<'d, $timer>,
                    clock_pin: Peri<'d, impl TimerPin<$timer, Ch1>>,
                    dma_ch: Peri<'d, $dma_ch>,
                    dma_irq: impl Binding<
                            <$dma_ch as $crate::__macro_support::embassy_stm32::dma::ChannelInstance>::Interrupt,
                            dma::InterruptHandler<$dma_ch>,
                        > + Binding<
                            <$dma_ch as $crate::__macro_support::embassy_stm32::dma::ChannelInstance>::Interrupt,
                            Hub75DmaHandler,
                        > + 'd,
                    pins: Hub75Pins8,
                    frequency: Hertz,
                ) -> Self
                where
                    $dma_ch: UpDma<$timer>,
                {
                    let gpio = pins.pins[0].block();

                    for pin in &pins.pins {
                        let n = pin.pin() as usize;
                        gpio.moder().modify(|w| {
                            w.set_moder(n, pac::gpio::vals::Moder::OUTPUT)
                        });
                        gpio.ospeedr().modify(|w| {
                            w.set_ospeedr(n, pac::gpio::vals::Ospeedr::LOW_SPEED)
                        });
                        gpio.otyper().modify(|w| {
                            w.set_ot(n, pac::gpio::vals::Ot::PUSH_PULL)
                        });
                    }

                    // BLANK starts HIGH (display off during init)
                    gpio.bsrr()
                        .write(|w| w.set_bs(pins.blank_pin_num(), true));

                    // ODR byte address: base ODR + 0 for lower pins, + 1 for upper
                    let byte_offset: usize = if pins.base_pin == 0 { 0 } else { 1 };
                    let odr_byte_addr =
                        unsafe { (gpio.odr().as_ptr() as *mut u8).add(byte_offset) };

                    let clock_pin = PwmPin::new(clock_pin, OutputType::PushPull);

                    // Configure timer for PWM clock output on CH1
                    let timer = Timer::new(tim);

                    timer.set_counting_mode(CountingMode::EdgeAlignedUp);
                    timer.set_frequency(frequency, RoundTo::Slower);
                    timer.enable_outputs();

                    // PWM mode 2: CLK rises at CCR, DMA writes at update event.
                    timer.set_output_compare_mode(
                        TimChannel::Ch1,
                        OutputCompareMode::PwmMode2,
                    );
                    timer.set_output_compare_preload(TimChannel::Ch1, true);
                    timer.set_autoreload_preload(true);

                    let max: u32 = timer.get_max_compare_value().into();
                    timer.set_compare_value(
                        TimChannel::Ch1,
                        (max * 4 / 5).try_into().ok().unwrap(),
                    );

                    timer.enable_channel(TimChannel::Ch1, true);
                    timer.generate_update_event();

                    // Enable timer update-event DMA trigger
                    timer.enable_update_dma(true);

                    // Build DMA channel + request
                    let dma_request = <$dma_ch as UpDma<$timer>>::request(&*dma_ch);
                    let channel = Channel::new(dma_ch, dma_irq);

                    // Store state for the ISR. Timer is NOT started yet.
                    $crate::__macro_support::critical_section::with(|cs| {
                        // SAFETY: Timer<'d> → Timer<'static>. The peripheral is
                        // consumed by this driver and lives for the program's lifetime.
                        // Wrapped in ManuallyDrop to prevent rcc::disable on drop.
                        let timer: ManuallyDrop<Timer<'static, $timer>> =
                            ManuallyDrop::new(unsafe {
                                core::mem::transmute::<
                                    Timer<'_, $timer>,
                                    Timer<'static, $timer>,
                                >(timer)
                            });

                        // SAFETY: Channel<'d> → Channel<'static>. Same justification.
                        let channel: Channel<'static> = unsafe {
                            core::mem::transmute::<Channel<'_>, Channel<'static>>(
                                channel,
                            )
                        };

                        *ISR_STATE.borrow_ref_mut(cs) = Some(IsrState {
                            timer,
                            channel,
                            transfer: None,
                            dma_request,
                            odr_byte_addr,
                            bcm: BcmState::new(),
                            current_fb_ptr: core::ptr::null(),
                            pending_planes: None,
                            pending_fb_ptr: core::ptr::null(),
                            returned_fb_ptr: core::ptr::null(),
                        });
                    });

                    Self {
                        _clock_pin: clock_pin,
                        _fb: PhantomData,
                    }
                }

                /// Start display refresh, consuming the idle handle and returning
                /// a running handle typed to the framebuffer.
                ///
                /// The framebuffer must be `'static` because the ISR will
                /// continuously read from it until [`Hub75::swap()`] replaces it.
                ///
                /// # Panics
                /// Panics if called while a transfer is already in flight.
                pub fn start<FB: FrameBuffer>(
                    self,
                    fb: &'static mut FB,
                ) -> Result<Hub75<'d, FB>, Hub75Error> {
                    let (planes, plane_count) = planes_from_fb(fb);
                    $crate::__macro_support::critical_section::with(|cs| {
                        let mut borrow = ISR_STATE.borrow_ref_mut(cs);
                        let state =
                            borrow.as_mut().expect("Hub75 not initialised");

                        if state.transfer.is_some() {
                            panic!("start() called while already running");
                        }

                        state.bcm.reset_with_planes(planes, plane_count);
                        state.current_fb_ptr = fb as *const FB as *const ();
                        state.pending_planes = None;
                        state.pending_fb_ptr = core::ptr::null();
                        state.returned_fb_ptr = core::ptr::null();

                        SWAP_DONE.store(false, Ordering::Relaxed);
                        FRAME_COUNT.store(0, Ordering::Relaxed);

                        // Kick the first DMA transfer.
                        let (ptr, len) = state.bcm.current_plane();
                        let buf =
                            unsafe { core::slice::from_raw_parts(ptr, len) };
                        let transfer = unsafe {
                            state.channel.write_raw(
                                state.dma_request,
                                buf,
                                state.odr_byte_addr,
                                TransferOptions::default(),
                            )
                        };
                        // SAFETY: same transmute justification as in on_interrupt.
                        state.transfer = Some(unsafe {
                            core::mem::transmute::<Transfer<'_>, Transfer<'static>>(
                                transfer,
                            )
                        });

                        // Start the timer — update events will now trigger DMA.
                        state.timer.start();
                    });

                    Ok(Hub75 {
                        _clock_pin: self._clock_pin,
                        _fb: PhantomData,
                    })
                }
            }

            impl<'d, FB: FrameBuffer + 'static> Hub75<'d, FB> {
                /// Returns the number of complete BCM frames rendered since
                /// [`start()`](Hub75::start).
                pub fn frame_count(&self) -> u32 {
                    FRAME_COUNT.load(Ordering::Relaxed)
                }

                /// Replace the displayed framebuffer, returning the
                /// previously-displayed one.
                ///
                /// Queues `new_fb` for display and yields until the ISR reaches a
                /// BCM frame boundary, at which point plane pointers are swapped
                /// atomically. Returns an exclusive reference to the old
                /// framebuffer that is no longer being read by the ISR.
                pub async fn swap(
                    &self,
                    new_fb: &'static mut FB,
                ) -> Result<&'static mut FB, Hub75Error> {
                    let (new_planes, _) = planes_from_fb(new_fb);

                    $crate::__macro_support::critical_section::with(|cs| {
                        let mut borrow = ISR_STATE.borrow_ref_mut(cs);
                        let state =
                            borrow.as_mut().expect("Hub75 not initialised");
                        state.pending_planes = Some(new_planes);
                        state.pending_fb_ptr =
                            new_fb as *const FB as *const ();
                        SWAP_DONE.store(false, Ordering::Relaxed);
                    });

                    poll_fn(|cx| {
                        if SWAP_DONE.load(Ordering::Acquire) {
                            return Poll::Ready(());
                        }
                        $crate::__macro_support::critical_section::with(|cs| {
                            if SWAP_DONE.load(Ordering::Relaxed) {
                                return Poll::Ready(());
                            }
                            *SWAP_WAKER.borrow_ref_mut(cs) =
                                Some(cx.waker().clone());
                            Poll::Pending
                        })
                    })
                    .await;

                    $crate::__macro_support::critical_section::with(|cs| {
                        let borrow = ISR_STATE.borrow_ref(cs);
                        let state =
                            borrow.as_ref().expect("Hub75 not initialised");
                        let ptr = state.returned_fb_ptr;
                        // SAFETY: The ISR atomically swapped away from this buffer
                        // at the frame boundary — it is no longer being read.
                        Ok(unsafe { &mut *(ptr as *mut FB) })
                    })
                }
            }
        }
    };
}
