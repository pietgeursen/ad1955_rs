#![no_std]
use core::{fmt::Debug, marker::PhantomData};

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::spi::Transfer;
use embedded_hal::digital::v2::OutputPin;
use packed_struct::prelude::*;
use packed_struct::types::bits::Bits;

#[derive(PrimitiveEnum, Clone, Copy, PartialEq, Debug, Default)]
pub enum DataFormat {
    #[default]
    Pcm = 0b00,
    ExternalDf = 0b01,
    SacdSlave = 0b10,
    SacdMaster = 0b11,
}

#[derive(PrimitiveEnum, Clone, Copy, PartialEq, Debug, Default)]
pub enum OutputFormat {
    #[default]
    Stereo = 0b00,
    MonoLeft = 0b10,
    MonoRight = 0b11,
}

#[derive(PrimitiveEnum, Clone, Copy, PartialEq, Debug, Default)]
pub enum PcmSampleRate {
    #[default]
    _48kHz = 0b00,
    _96kHz = 0b01,
    _192kHz = 0b10,
}

#[derive(PrimitiveEnum, Clone, Copy, PartialEq, Debug, Default)]
pub enum DeEmphasis {
    #[default]
    None = 0b00,
    _44100Hz = 0b01,
    _32000Hz = 0b10,
    _48000Hz = 0b11,
}

#[derive(PrimitiveEnum, Clone, Copy, PartialEq, Debug, Default)]
pub enum PcmDataFormat {
    #[default]
    I2S = 0b00,
    RightJustified = 0b01,
    DSP = 0b10,
    LeftJustified = 0b11,
}

#[derive(PrimitiveEnum, Clone, Copy, PartialEq, Debug, Default)]
pub enum PcmDataWidth {
    #[default]
    _24bits = 0b00,
    _20bits = 0b01,
    _18bits = 0b10,
    _16bits = 0b11,
}

#[derive(Debug, Default, PackedStruct)]
#[packed_struct(bit_numbering = "lsb0", endian = "msb", size_bytes = "2")]
pub struct DacControl1 {
    #[packed_field(bits = "15")]
    pub is_power_down: bool,
    #[packed_field(bits = "14")]
    pub is_muted: bool,
    #[packed_field(bits = "12:13", ty = "enum")]
    pub data_format: DataFormat,
    #[packed_field(bits = "10:11", ty = "enum")]
    pub output_format: OutputFormat,
    #[packed_field(bits = "8:9", ty = "enum")]
    pub pcm_sample_rate: PcmSampleRate,
    #[packed_field(bits = "6:7", ty = "enum")]
    pub de_emphasis: DeEmphasis,
    #[packed_field(bits = "4:5", ty = "enum")]
    pub pcm_data_format: PcmDataFormat,
    #[packed_field(bits = "2:3", ty = "enum")]
    pub pcm_data_width: PcmDataWidth,
    #[packed_field(bits = "1:0")]
    _address: ReservedZero<Bits<2>>,
}

#[derive(PrimitiveEnum, Clone, Copy, PartialEq, Debug, Default)]
pub enum MclkMode {
    #[default]
    Fs256 = 0b00,
    Fs512 = 0b01,
    Fs768 = 0b10,
}

#[derive(PrimitiveEnum, Clone, Copy, PartialEq, Debug, Default)]
pub enum ZeroFlagPolarity {
    #[default]
    ActiveHigh = 0b0,
    ActiveLow = 0b1,
}

#[derive(PrimitiveEnum, Clone, Copy, PartialEq, Debug, Default)]
pub enum SacdBitRate {
    #[default]
    Fs64 = 0b0,
    Fs128 = 0b1,
}
#[derive(PrimitiveEnum, Clone, Copy, PartialEq, Debug, Default)]
pub enum SacdMode {
    #[default]
    Normal = 0b0,
    Phase = 0b1,
}

#[derive(PrimitiveEnum, Clone, Copy, PartialEq, Debug, Default)]
pub enum SacdBitInversion {
    #[default]
    Normal = 0b0,
    Inverted = 0b1,
}
#[derive(PrimitiveEnum, Clone, Copy, PartialEq, Debug, Default)]
pub enum SacdMclkToBclkPhase {
    #[default]
    RisingEdge = 0b0,
    FallingEdge = 0b1,
}

#[derive(PrimitiveEnum, Clone, Copy, PartialEq, Debug, Default)]
pub enum SacdPhaseSelect {
    #[default]
    Phase0 = 0b00,
    Phase1 = 0b01,
    Phase2 = 0b10,
    Phase3 = 0b11,
}

#[derive(Debug, Default, PackedStruct)]
#[packed_struct(bit_numbering = "lsb0", endian = "msb", size_bytes = "2")]
pub struct DacControl2 {
    #[packed_field(bits = "9:10", ty = "enum")]
    pub mclk_mode: MclkMode,
    #[packed_field(bits = "8", ty = "enum")]
    pub zero_flag_priority: ZeroFlagPolarity,
    #[packed_field(bits = "7", ty = "enum")]
    pub sacd_bit_rate: SacdBitRate,
    #[packed_field(bits = "6", ty = "enum")]
    pub sacd_mode: SacdMode,
    #[packed_field(bits = "4:5", ty = "enum")]
    pub sacd_phase_select: SacdPhaseSelect,
    #[packed_field(bits = "3", ty = "enum")]
    pub sacd_bit_inversion: SacdBitInversion,
    #[packed_field(bits = "2", ty = "enum")]
    pub sacd_mclk_to_bclk_phase: SacdMclkToBclkPhase,
    #[packed_field(bits = "1")]
    _address1: ReservedZero<Bits<1>>,
    #[packed_field(bits = "0")]
    _address0: ReservedOne<Bits<1>>,
}

#[derive(Debug, PackedStruct)]
#[packed_struct(bit_numbering = "lsb0", endian = "msb", size_bytes = "2")]
pub struct VolumeLeft {
    #[packed_field(bits = "2:15")]
    pub volume: Integer<u16, Bits<14>>,
    #[packed_field(bits = "1")]
    _address1: ReservedOne<Bits<1>>,
    #[packed_field(bits = "0")]
    _address0: ReservedZero<Bits<1>>,
}
impl Default for VolumeLeft {
    fn default() -> Self {
        Self {
            volume: 0x3FFF.into(),
            _address1: Default::default(),
            _address0: Default::default(),
        }
    }
}
#[derive(Debug, PackedStruct)]
#[packed_struct(bit_numbering = "lsb0", endian = "msb", size_bytes = "2")]
pub struct VolumeRight {
    #[packed_field(bits = "2:15")]
    pub volume: Integer<u16, Bits<14>>,
    #[packed_field(bits = "0:1")]
    _address: ReservedOnes<Bits<2>>,
}
impl Default for VolumeRight {
    fn default() -> Self {
        Self {
            volume: 0x3FFF.into(),
            _address: Default::default(),
        }
    }
}

pub struct Ad1955<P, SPI, E>
where
    P: OutputPin,
    <P as embedded_hal::digital::v2::OutputPin>::Error: Debug,
    SPI: Transfer<u8, Error = E>,
{
    chip_select: P,
    spi: PhantomData<SPI>,
    pub dac_control_1: DacControl1,
    pub dac_control_2: DacControl2,
    pub volume_left: VolumeLeft,
    pub volume_right: VolumeRight,
}

impl<P, SPI, E> Ad1955<P, SPI, E>
where
    P: OutputPin,
    <P as embedded_hal::digital::v2::OutputPin>::Error: Debug,
    SPI: Transfer<u8, Error = E>,
    E: Debug,
{
    pub fn new(chip_select: P) -> Self
    where
        SPI: Transfer<u8, Error = E>,
        E: Debug,
    {
        Self {
            chip_select,
            spi: PhantomData,
            dac_control_1: Default::default(),
            dac_control_2: Default::default(),
            volume_left: Default::default(),
            volume_right: Default::default(),
        }
    }

    pub fn update<D: DelayMs<u16>>(&mut self, spi: &mut SPI, delay: &mut D) {
        let mut dac_control_1_bytes = self.dac_control_1.pack().unwrap();
        let mut dac_control_2_bytes = self.dac_control_2.pack().unwrap();
        let mut volume_left_bytes = self.volume_left.pack().unwrap();
        let mut volume_right_bytes = self.volume_right.pack().unwrap();
        self.chip_select.set_low().unwrap();
        spi.transfer(&mut dac_control_1_bytes).unwrap();
        self.chip_select.set_high().unwrap();
        delay.delay_ms(1);

        self.chip_select.set_low().unwrap();
        spi.transfer(&mut dac_control_2_bytes).unwrap();
        self.chip_select.set_high().unwrap();
        delay.delay_ms(1);

        self.chip_select.set_low().unwrap();
        spi.transfer(&mut volume_left_bytes).unwrap();
        self.chip_select.set_high().unwrap();
        delay.delay_ms(1);

        self.chip_select.set_low().unwrap();
        spi.transfer(&mut volume_right_bytes).unwrap();
        self.chip_select.set_high().unwrap();
        delay.delay_ms(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pack_byte_order() {
        let dac_control_1 = DacControl1 {
            is_power_down: true,
            ..Default::default()
        };
        let packed = dac_control_1.pack().unwrap();
        assert_eq!(packed, [0b10000000u8, 0],)
    }
}
