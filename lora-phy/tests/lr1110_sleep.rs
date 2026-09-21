//! Sleep/wake regression tests using the real LR1110 driver and a recording bus.
use core::{
    future::Future,
    pin::Pin,
    task::{Context, Poll},
};
use std::{
    cell::RefCell,
    rc::Rc,
    sync::Arc,
    task::{Wake, Waker},
};

use embedded_hal_async::{
    delay::DelayNs,
    spi::{ErrorKind, ErrorType, Operation, SpiDevice},
};
use lora_phy::{
    lr1110::{variant::Lr1110 as ChipKind, Config, Lr1110, TcxoCtrlVoltage},
    mod_params::{Bandwidth, CodingRate, DutyCycleParams, RadioError, RadioMode, RxMode, SpreadingFactor},
    mod_traits::{InterfaceVariant, RadioKind},
    LoRa,
};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum Mode {
    #[default]
    Standby,
    Sleeping,
    Receiving,
}
#[derive(Debug, PartialEq, Eq)]
enum Event {
    Command(Vec<u8>),
    Wake,
    Busy,
}
#[derive(Default)]
struct Chip {
    mode: Mode,
    events: Vec<Event>,
    fail_busy: bool,
    fail_wake: bool,
}
struct Bus(Rc<RefCell<Chip>>);
impl ErrorType for Bus {
    type Error = ErrorKind;
}
impl SpiDevice for Bus {
    async fn transaction(&mut self, ops: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
        let mut chip = self.0.borrow_mut();
        // The driver's wake helper performs a one-byte read without a command.
        if matches!(ops, [Operation::Read(bytes)] if bytes.len() == 1) {
            chip.events.push(Event::Wake);
            if chip.fail_wake {
                return Err(ErrorKind::Other);
            }
            chip.mode = Mode::Standby;
        }
        if let Some(Operation::Write(cmd)) = ops.first() {
            assert_ne!(chip.mode, Mode::Sleeping, "command issued before wake pulse");
            chip.events.push(Event::Command(cmd.to_vec()));
            match cmd.get(..2) {
                Some([0x01, 0x1b]) => chip.mode = Mode::Sleeping,
                Some([0x01, 0x1c]) => chip.mode = Mode::Standby,
                Some([0x02, 0x09]) => chip.mode = Mode::Receiving,
                _ => (),
            }
        }
        for op in ops {
            match op {
                Operation::Read(bytes) | Operation::Transfer(bytes, _) | Operation::TransferInPlace(bytes) => {
                    bytes.fill(0)
                }
                _ => (),
            }
        }
        Ok(())
    }
}
struct Pins(Rc<RefCell<Chip>>);
impl InterfaceVariant for Pins {
    async fn reset(&mut self, _: &mut impl DelayNs) -> Result<(), RadioError> {
        self.0.borrow_mut().mode = Mode::Standby;
        Ok(())
    }
    async fn wait_on_busy(&mut self) -> Result<(), RadioError> {
        let sleeping = {
            let mut chip = self.0.borrow_mut();
            chip.events.push(Event::Busy);
            if chip.fail_busy {
                return Err(RadioError::Busy);
            }
            chip.mode == Mode::Sleeping
        };
        if sleeping {
            core::future::pending::<()>().await;
        }
        Ok(())
    }
    async fn await_irq(&mut self) -> Result<(), RadioError> {
        core::future::pending().await
    }
    async fn enable_rf_switch_rx(&mut self) -> Result<(), RadioError> {
        Ok(())
    }
    async fn enable_rf_switch_tx(&mut self) -> Result<(), RadioError> {
        Ok(())
    }
    async fn disable_rf_switch(&mut self) -> Result<(), RadioError> {
        Ok(())
    }
}
struct Delay;
impl DelayNs for Delay {
    async fn delay_ns(&mut self, _: u32) {}
}
struct NoopWake;
impl Wake for NoopWake {
    fn wake(self: Arc<Self>) {}
}
fn poll<F: Future>(f: Pin<&mut F>) -> Poll<F::Output> {
    let waker = Waker::from(Arc::new(NoopWake));
    f.poll(&mut Context::from_waker(&waker))
}
fn ready<F: Future>(f: F) -> F::Output {
    let Poll::Ready(value) = poll(core::pin::pin!(f)) else {
        panic!("operation blocked, possibly on sleeping BUSY");
    };
    value
}
fn fixture(chip: Rc<RefCell<Chip>>) -> Lr1110<Bus, Pins, ChipKind> {
    Lr1110::new(
        Bus(chip.clone()),
        Pins(chip),
        Config {
            chip: ChipKind::default(),
            tcxo_ctrl: Some(TcxoCtrlVoltage::Ctrl1V6),
            use_dcdc: false,
            rx_boost: true,
            rf_switch: None,
        },
    )
}

#[test]
fn sleep_does_not_wait_for_busy_after_set_sleep() {
    for warm in [false, true] {
        let chip = Rc::new(RefCell::new(Chip::default()));
        let mut radio = fixture(chip.clone());
        ready(radio.set_sleep(warm, &mut Delay)).unwrap();
        assert_eq!(chip.borrow().mode, Mode::Sleeping);
        assert!(matches!(chip.borrow().events.last(), Some(Event::Command(cmd)) if cmd.starts_with(&[0x01, 0x1b])));
    }
}

#[test]
fn sleeping_and_duty_cycled_radios_wake_before_waiting_for_busy() {
    for mode in [
        RadioMode::Sleep,
        RadioMode::Receive(RxMode::DutyCycle(DutyCycleParams {
            rx_time: 10,
            sleep_time: 20,
        })),
    ] {
        let chip = Rc::new(RefCell::new(Chip {
            mode: Mode::Sleeping,
            ..Chip::default()
        }));
        let mut radio = fixture(chip.clone());
        ready(radio.ensure_ready(mode)).unwrap();
        assert_eq!(chip.borrow().mode, Mode::Standby);
        assert_eq!(chip.borrow().events, [Event::Wake, Event::Busy]);
    }
}

#[test]
fn awake_radio_waits_for_busy_without_a_wake_pulse() {
    let chip = Rc::new(RefCell::new(Chip::default()));
    let mut radio = fixture(chip.clone());
    ready(radio.ensure_ready(RadioMode::Standby)).unwrap();
    assert_eq!(chip.borrow().events, [Event::Busy]);
    chip.borrow_mut().fail_busy = true;
    assert_eq!(ready(radio.ensure_ready(RadioMode::Standby)), Err(RadioError::Busy));
}

#[test]
fn wake_errors_are_not_silently_accepted() {
    let chip = Rc::new(RefCell::new(Chip {
        mode: Mode::Sleeping,
        fail_wake: true,
        ..Chip::default()
    }));
    let mut radio = fixture(chip.clone());
    assert_eq!(ready(radio.ensure_ready(RadioMode::Sleep)), Err(RadioError::SPI));
    chip.borrow_mut().fail_wake = false;
    chip.borrow_mut().fail_busy = true;
    assert_eq!(ready(radio.ensure_ready(RadioMode::Sleep)), Err(RadioError::Busy));
}

#[test]
fn lora_restores_rx_after_repeated_cold_and_warm_sleep() {
    let chip = Rc::new(RefCell::new(Chip::default()));
    let mut lora = ready(LoRa::new(fixture(chip.clone()), false, Delay)).unwrap();
    let modulation = lora
        .create_modulation_params(SpreadingFactor::_7, Bandwidth::_62KHz, CodingRate::_4_5, 917_500_000)
        .unwrap();
    let packet = lora
        .create_rx_packet_params(16, false, 255, true, false, &modulation)
        .unwrap();
    for warm in [false, true, false] {
        ready(lora.enter_standby()).unwrap();
        ready(lora.sleep(warm)).unwrap();
        assert_eq!(chip.borrow().mode, Mode::Sleeping);
        chip.borrow_mut().events.clear();
        ready(lora.prepare_for_rx(RxMode::Continuous, &modulation, &packet)).unwrap();
        ready(lora.start_rx()).unwrap();
        let chip = chip.borrow();
        assert_eq!(chip.mode, Mode::Receiving);
        assert_eq!(&chip.events[..2], &[Event::Wake, Event::Busy]);
        // A cold restart restores the TCXO setup; a warm restart retains it.
        assert_eq!(
            chip.events
                .iter()
                .any(|e| matches!(e, Event::Command(cmd) if cmd.starts_with(&[0x01, 0x17]))),
            !warm
        );
    }
}
