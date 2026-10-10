pub use lora_modulation::{Bandwidth, CodingRate, SpreadingFactor};

/// Errors types reported during LoRa physical layer processing
#[allow(clippy::upper_case_acronyms)]
#[derive(Debug, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[allow(missing_docs)]
pub enum RadioError {
    SPI,
    Reset,
    RfSwitchRx,
    RfSwitchTx,
    Busy,
    Irq,
    DIO1,
    InvalidConfiguration,
    InvalidRadioMode,
    OpError(u8),
    InvalidBaseAddress(usize, usize),
    PayloadSizeUnexpected(usize),
    PayloadSizeMismatch(usize, usize),
    UnavailableSpreadingFactor,
    UnavailableBandwidth,
    InvalidBandwidthForFrequency,
    InvalidSF6ExplicitHeaderRequest,
    InvalidOutputPowerForFrequency,
    TransmitTimeout,
    ReceiveTimeout,
    /// LoRa header decoded but failed its CRC. Indicates the receiver was
    /// sync'd to an incoming frame but couldn't parse the header (likely
    /// a modulation/CR mismatch between RX and TX, or interference during
    /// the header). With explicit header, the chip aborts the packet — no
    /// RxDone fires.
    HeaderError,
    /// Packet payload completed but failed CRC. Use this to distinguish from
    /// a clean RX. The payload is still readable from the buffer.
    CrcError,
    DutyCycleUnsupported,
    RngUnsupported,
}

/// Status for a received packet
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct PacketStatus {
    /// Packet RSSI in dBm. On SX126x and LR11xx this is the total in-band
    /// power, signal plus noise.
    pub rssi: i16,
    /// SNR in dB, rounded to the nearest dB.
    pub snr: i16,
    /// SNR in quarter-dB steps, as measured by the radio.
    pub snr_quarter_db: i16,
    /// Estimated signal-only RSSI in dBm, for radios that measure it
    /// separately from [`rssi`](Self::rssi) (SX126x and LR11xx).
    pub signal_rssi: Option<i16>,
}

/// Round an SNR in quarter-dB steps to whole dB, halves rounding up.
pub(crate) const fn quarter_db_to_db(quarter_db: i16) -> i16 {
    (quarter_db + 2) >> 2
}

/// Parse the three-byte LoRa packet status shared by the SX126x
/// `GetPacketStatus` and the LR11xx `GetPacketStatus` commands:
/// `RssiPkt`, `SnrPkt`, `SignalRssiPkt`.
pub(crate) const fn lora_packet_status_from_bytes(bytes: [u8; 3]) -> PacketStatus {
    let snr_quarter_db = bytes[1] as i8 as i16;
    PacketStatus {
        rssi: half_db_to_dbm(bytes[0]),
        snr: quarter_db_to_db(snr_quarter_db),
        snr_quarter_db,
        signal_rssi: Some(half_db_to_dbm(bytes[2])),
    }
}

/// Convert a `-raw / 2` dBm reading to whole dBm, rounding down.
const fn half_db_to_dbm(raw: u8) -> i16 {
    (-(raw as i16)) >> 1
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quarter_db_rounds_to_nearest_db() {
        assert_eq!(quarter_db_to_db(0), 0);
        assert_eq!(quarter_db_to_db(1), 0);
        assert_eq!(quarter_db_to_db(2), 1);
        assert_eq!(quarter_db_to_db(3), 1);
        assert_eq!(quarter_db_to_db(-1), 0);
        assert_eq!(quarter_db_to_db(-2), 0);
        assert_eq!(quarter_db_to_db(-3), -1);
        assert_eq!(quarter_db_to_db(-30), -7);
        assert_eq!(quarter_db_to_db(-128), -32);
        assert_eq!(quarter_db_to_db(126), 32);
        assert_eq!(quarter_db_to_db(127), 32);
    }

    #[test]
    fn lora_packet_status_keeps_quarter_db_and_signal_rssi() {
        // RssiPkt 0xF0 = -120 dBm, SnrPkt 0xE2 = -30 steps = -7.5 dB,
        // SignalRssiPkt 0xF7 = -123.5 dBm.
        let status = lora_packet_status_from_bytes([0xF0, 0xE2, 0xF7]);
        assert_eq!(
            status,
            PacketStatus {
                rssi: -120,
                snr: -7,
                snr_quarter_db: -30,
                signal_rssi: Some(-124)
            }
        );
    }

    #[test]
    fn lora_packet_status_handles_extreme_snr() {
        assert_eq!(lora_packet_status_from_bytes([0, 0x7F, 0]).snr, 32);
        assert_eq!(lora_packet_status_from_bytes([0, 0x80, 0]).snr, -32);
    }
}

/// The state of the radio
#[derive(Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum RadioMode {
    /// Sleep mode
    Sleep,
    /// Standby mode
    Standby,
    /// Frequency synthesis mode
    FrequencySynthesis,
    /// Transmit (TX) mode
    Transmit,
    /// Receive (RX) mode
    Receive(RxMode),
    /// Listen mode
    Listen,
    /// Channel activity detection (CAD) mode
    ChannelActivityDetection,
}

impl From<RxMode> for RadioMode {
    fn from(rx_mode: RxMode) -> Self {
        RadioMode::Receive(rx_mode)
    }
}

/// Listening mode for LoRaWAN packet detection/reception
#[derive(Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum RxMode {
    /// Single shot Rx Mode to listen until packet preamble is detected or RxTimeout occurs.
    /// The device will stay in RX Mode until a packet is received.
    /// Preamble length as symbols is configured via following registers:
    /// * sx126x: uses `SetLoRaSymbNumTimeout(0 < n < 255)` + `SetStopRxTimerOnPreamble(1)`
    /// * sx127x: uses `RegSymbTimeout (4 < n < 1023)`
    // TODO: Single mode with time-based timeout is available on sx126x, but not sx127x
    Single(u16),
    /// Continuous Rx mode to listen for incoming packets continuously
    Continuous,
    /// Receive in Duty Cycle mode (NB! Not supported on sx127x)
    DutyCycle(DutyCycleParams),
}

/// Modulation parameters for a send and/or receive communication channel
pub struct ModulationParams {
    /// Spreading Factor: higher value improves sensitivity at the cost of time-on-air
    pub spreading_factor: SpreadingFactor,
    /// Signal Bandwidth: lower value improves sensitivity at the cost of time-on-air
    pub bandwidth: Bandwidth,
    /// Coding Rate: controls number of redundancy bits
    pub coding_rate: CodingRate,
    /// Set to 1 to improve reliability at the cost of time-on-air.
    /// LoRaWAN enable this mode for SF11/12 at bandwidth 125kHz and SF12 at bandwidth 250kHz
    pub low_data_rate_optimize: u8,
    /// Channel frequency in Hertz
    pub frequency_in_hz: u32,
}

/// Packet parameters for a send or receive communication channel
pub struct PacketParams {
    /// Number of LoRa symbols in the preamble (typical value are 12 for SF5/6 and 8 for SF7 to 12)
    pub preamble_length: u16,
    /// When true length, CodingRate and CRC must be known by the RX.
    /// When false, a header is automatically added to the packet allowing the RX to automatically discover the settings
    pub implicit_header: bool,
    /// Legth of the payload in number of bytes
    pub payload_length: u8,
    /// Enable CRC
    pub crc_on: bool,
    /// Use inverted chirp direction (generally used to distinguished uplink/downlink)
    pub iq_inverted: bool,
}

impl PacketParams {
    pub(crate) fn set_payload_length(&mut self, payload_length: usize) -> Result<(), RadioError> {
        if payload_length > 255 {
            return Err(RadioError::PayloadSizeUnexpected(payload_length));
        }
        self.payload_length = payload_length as u8;
        Ok(())
    }
}

/// Receive duty cycle parameters
#[derive(Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct DutyCycleParams {
    /// receive interval
    pub rx_time: u32,
    /// sleep interval
    pub sleep_time: u32,
}
