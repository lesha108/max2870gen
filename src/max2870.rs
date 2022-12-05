extern crate embedded_hal;

use embedded_hal::blocking::spi::Write;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::blocking::delay::{DelayMs, DelayUs};

#[derive(Debug)]
pub enum DataError {
    /// An error occurred when working with SPI
    Spi,
    /// An error occurred when working with a PIN
    Pin,
}

// this parameters should be changed to suit the application
/*
If bit FB = 0, (DIVA is in PLL feedback loop) and DIVA
≤ 16:
N + (F/M) = (fVCO/fPFD)/DIVA
If bit FB = 0, (DIVA is in PLL feedback loop) and DIVA > 16:
N + (F/M) = (fVCO / fPFD)/16
*/
pub const N_VAL: u16 = 40; // (16 bit feedback divider factor in integer mode
pub const F_VAL: u16 = 0; // 12 bit  fractional divider
pub const M_VAL: u16 = 100; // 12 bit modulus value in fractional mode
                        // fPFD = fREF x [(1 + DBR)/(R x (1 + RDIV2))]
pub const R_VAL: u16 = 5; // 10 bit ref. frequency divider value
                      // fVCO = fRFOUTA x DIVA
pub const DIVA_VAL: u16 = 0b111; //Sets RFOUT_ output divider mode
                             // тут нужно смотреть на временную константу фильтра PLL tFAST-LOCK = M x CDIV/fPFD
pub const CDIV_VAL: u16 = 192; // 12 bit clock divide value
pub const B_POWER: u8 = 0b00; // Sets RFOUTB single-ended output power: 00 = -4dBm, 01 = -1dBm, 10 = +2dBm, 11 = +5dBm
pub const A_POWER: u8 = 0b11; // Sets RFOUTA single-ended output power: 00 = -4dBm, 01 = -1dBm, 10 = +2dBm, 11 = +5dBm

// less likely to get changed
pub const MUX: u8 = 0b0000;
pub const MUX_MSB: u8 = MUX >> 3; // MSB of MUX
pub const MUX_LSB: u8 = MUX & !(1 << 3); // lower 3 bits of MUX
pub const CPL_MODE: u8 = 0b00; //CPL linearity mode: 0b00in integer mode, 0b01 10% in frac mode, 0b10 for 20%, and 0b11 for 30%
pub const CPT_MODE: u8 = 0b00; // 00 normal mode, 01 long reset, 10 force into source, 11 force into sink
pub const CP_CURRENT: u8 = 0x0; // 4 bit CP current in mA
pub const P_VAL: u16 = 0x1; // 12-bit phase value for adjustment
pub const SD_VAL: u8 = 0b00; //sigma delta noise mode: 0b00 low noise mode, 0b01 res, 0b10 low spur 1 mode, 0b11 low spur 2 mode
pub const VCO: u8 = 0x0; // 6 bit VCO selection
pub const CDIV_MODE: u8 = 0b00; // clock divide mode: 0b00 mute until lock delay, 01 fast lock enable, 10 phase adjustment, 11 reserved
pub const BS: u16 = 100;
pub const BS_MSB_VAL: u16 = BS >> 8; // 2 MSBs of Band select clock divider
pub const BS_LSB_VAL: u16 = BS & !(11 << 8); // 8 LSBs of band select clock divider
pub const LD_VAL: u8 = 0b01; //  lock-detect pin function: 00 = Low, 01 = Digital lock detect, 10 = Analog lock detect, 11 = High
pub const ADC_MODE: u8 = 0b001; // ADC mode: 001 temperature, 100 tune pin,

// register 0 masks
pub const EN_INT: u32 = 1 << 31; // enables integer mode
pub const N_SET: u32 = (N_VAL as u32) << 15; // puts value N on its place
pub const N_MASK: u32 = 0xFFFF << 15; //16 bits at location 30:15
pub const F_SET: u32 = (F_VAL as u32) << 3;
pub const F_MASK: u32 = 0xFFF << 3; //12 bits at location 14:3
pub const REG_0: u32 = 0b000;

// register 1 masks
// перед заполением обнулить!
// нет константы 32 бита - CPOC. Предполагаем 0 всегда
pub const CPL: u32 = (CPL_MODE as u32) << 29; // Sets CP linearity mode
pub const CPT: u32 = (CPT_MODE as u32) << 27; // Sets CP test mode
pub const PHASE: u32 = (P_VAL as u32) << 15; // Sets phase adjustment
pub const M_SET: u32 = (M_VAL as u32) << 3; // sets modulus value
pub const M_MASK: u32 = 0xFFF << 3; // mask for M bits
pub const REG_1: u32 = 0b001;

// register 2 masks
pub const LDS: u32 = 1 << 31; //Lock detect speed adjustment: 0 fPFD < 32 MHz, 1 pPFD > 32 MHz
pub const SDN: u32 = (SD_VAL as u32) << 29; //sets sigma-delta noise
pub const MUX_2: u32 = (MUX_LSB as u32) << 26; //sets MUX bits
pub const DBR: u32 = 1 << 25; //sets reference doubler mode, 0 disable, 1 enable
pub const RDIV2: u32 = 1 << 24; //enable reference divide-by-2
pub const R_DIV: u32 = (R_VAL as u32) << 14; // set reference divider value
pub const R_MASK: u32 = 0x3FF << 14;
pub const REG4DB: u32 = 1 << 13; // sets double buffer mode
pub const CP_SET: u32 = (CP_CURRENT as u32) << 9; // sets CP current
pub const LDF: u32 = 1 << 8; // sets lock detecet in integer mode
pub const LDP: u32 = 1 << 7; //sets lock detect precision
pub const PDP: u32 = 1 << 6; // phase detect polarity
pub const SHDN: u32 = 1 << 5; // shutdown mode
pub const CP_HZ: u32 = 1 << 4; // sets CP to high Z mode
pub const RST: u32 = 1 << 3; // R and N counters reset
pub const REG_2: u32 = 0b010;

//register 3 masks
// перед заполением обнулить!
pub const VCO_SET: u32 = (VCO as u32) << 26; // Manual selection of VCO and VCO sub-band when VAS is disabled.
pub const VAS_SHDN: u32 = 1 << 25; // VAS shutdown mode
pub const VAS_TEMP: u32 = 1 << 24; // sets VAS temperature compensation
pub const CDM: u32 = (CDIV_MODE as u32) << 15; // sets clock divider mode
pub const CDIV: u32 = (CDIV_VAL as u32) << 3; // sets clock divider value
pub const REG_3: u32 = 0b011;

// register 4 masks
// перед заполением обнулить!
pub const REG4HEAD: u32 = 3 << 29; // Always program to 0b011
pub const BS_MSB: u32 = (BS_MSB_VAL as u32) << 24; // Sets band select 2 MSBs
pub const FB: u32 = 1 << 23; //Sets VCO to N counter feedback mode
pub const DIVA_MASK: u32 = 7 << 20; // 3 bits at 22:20
pub const DIVA: u32 = (DIVA_VAL as u32) << 20; // Sets RFOUT_ output divider mode. Double buffered by register 0 when REG4DB = 1.
pub const BS_LSB: u32 = (BS_LSB_VAL as u32) << 12; // Sets band select 8 LSBs
pub const BDIV: u32 = 1 << 9; // Sets RFOUTB output path select. 0 = VCO divided output, 1 = VCO fundamental frequency
pub const RFB_EN: u32 = 1 << 8; // Enable RFOUTB output
pub const BPWR: u32 = (B_POWER as u32) << 6; //RFOUTB Power
pub const RFA_EN: u32 = 1 << 5; // Enable RFOUTA output
pub const APWR: u32 = (A_POWER as u32) << 3; //RFOUTA Power
pub const REG_4: u32 = 0b100;

// register 5 masks
// перед заполением обнулить!
pub const F01: u32 = 1 << 24; // sets integer mode when F = 0
pub const LD: u32 = (LD_VAL as u32) << 22; // sets lock detection pin function
pub const MUX_5: u32 = (MUX_MSB as u32) << 18; // sets MSB of MUX bits
pub const REG_5: u32 = 0b101;

// register 6 masks (read only values) MAX2870
pub const POR: u32 = 1 << 23; // POR readback status
pub const VTUNE_ADC: u32 = 0b111 << 20; // ADC reading of the VTUNE
pub const VCO_N: u32 = 0b111111 << 3; // Current VCO
pub const REG_6: u32 = 0b110;

pub const MAX2870_REGS: usize = 7; // число регистров в MAX2870

// описание одного регистра
#[derive(Debug)]
struct Max2870Register {
    bits: u32,
}

impl Default for Max2870Register {
    fn default() -> Self {
        Max2870Register { bits: 0 }
    }
}

impl Max2870Register {
    fn new() -> Self {
        Default::default()
    }

    fn clear(&mut self) {
        self.bits = 0
    }
}

// описание набора регистров

#[derive(Debug)]
pub struct Max2870<SPI, LE>
where
    SPI: Write<u8>,
    LE: OutputPin,
{
    regs: [Max2870Register; MAX2870_REGS],
    spi: SPI,
    le: LE,
    buffer: [u8; 4],
}

impl<SPI, LE> Max2870<SPI, LE>
where
    SPI: Write<u8>,
    LE: OutputPin,
{
    pub fn new(spi: SPI, le: LE) -> Self {
        Max2870 {
            regs: [
                Max2870Register::new(), // REG 0
                Max2870Register::new(),
                Max2870Register::new(),
                Max2870Register::new(),
                Max2870Register::new(),
                Max2870Register::new(),
                Max2870Register::new(),
            ],
            spi,
            le,
            buffer: [0; 4],
        }
    }

    pub fn clear(&mut self) {
        for i in 0..MAX2870_REGS {
            self.regs[i].clear()
        }
    }

    pub fn set_reg(&mut self, r: usize, new_val: u32) {
        if r >= MAX2870_REGS { return }
        self.regs[r].bits = new_val;
    }
    pub fn get_reg(&mut self, r: usize) -> u32 {
        self.regs[r].bits
    }

    // дефолтные значения регистров из даташита
    pub fn set_defaults(&mut self) {
        self.regs[0].bits = 0x007d0000; // idiv = 250 , frac on = 0
        self.regs[1].bits = 0x2000fff9; // m=4095
        self.regs[2].bits = 0x00004042; // PDP - не меняем
        self.regs[3].bits = 0x0000000b; // VAS - не меняем
        self.regs[4].bits = 0x6180b23c; // div = 64 bs???
        self.regs[5].bits = 0x00400005; // Digital lock detect - не меняем? Lock LED
        self.regs[6].bits = 0x00000000; // не важно
    }

    // нужно писать в порт SPI от MSB к LSB побайтно
    fn write_reg<D: DelayUs<u16> + DelayMs<u8>>(&mut self, r: usize, delay: &mut D) -> Result<(), DataError> {
        let b = self.regs[r].bits;
        self.buffer[0] = ((0xFF000000 & b) >> 24) as u8;
        self.buffer[1] = ((0x00FF0000 & b) >> 16) as u8;
        self.buffer[2] = ((0x0000FF00 & b) >> 8) as u8;
        self.buffer[3] = (0x000000FF & b) as u8;
        self.le.set_low().map_err(|_| DataError::Pin)?;
        self.spi
            .write(&self.buffer[..])
            .map_err(|_| DataError::Spi)?;
        self.le.set_high().map_err(|_| DataError::Pin)?;
        delay.delay_us(100);
        Ok(())
    }

    // нужно после вкл питания 2 раза через 20мс записать
    pub fn write_all<D: DelayUs<u16> + DelayMs<u8>>(&mut self, delay: &mut D) -> Result<(), DataError> {
        self.write_reg(5, delay)?;
        self.write_reg(4, delay)?;
        self.write_reg(3, delay)?;
        self.write_reg(2, delay)?;
        self.write_reg(1, delay)?;
        self.write_reg(0, delay)
    }

    pub fn init<D: DelayUs<u16> + DelayMs<u8>>(&mut self, delay: &mut D) -> Result<(), DataError> {
        self.write_all(delay)?;
        delay.delay_ms(20);
        self.write_all(delay)
    }
}

