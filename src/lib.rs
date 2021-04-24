#![no_std]

extern crate byteorder;
extern crate embedded_hal as hal;

use byteorder::BigEndian;
use byteorder::ByteOrder;
use hal::blocking::spi::Transfer;
use hal::digital::v2::OutputPin;

const COMMAND_READ: u8 = 0x00 << 2;
const COMMAND_WRITE: u8 = 0x01 << 2;
const VARIABLE_DATA_LENGTH: u8 = 0b_00;

type Spi<E> = dyn Transfer<u8, Error = E>;

#[derive(Debug)]
pub enum Error<SpiError, PinError> {
    /// SPI communication error
    Spi(SpiError),
    /// CS output pin error
    Pin(PinError),
}

impl<SpiError, PinError> From<SpiError> for Error<SpiError, PinError> {
    fn from(err: SpiError) -> Self {
        Self::Spi(err)
    }
}

#[derive(Copy, Clone, PartialOrd, PartialEq, Default, Debug)]
pub struct IpAddress {
    pub address: [u8; 4],
}

impl IpAddress {
    pub fn new(a0: u8, a1: u8, a2: u8, a3: u8) -> IpAddress {
        IpAddress {
            address: [a0, a1, a2, a3],
        }
    }
}

impl ::core::fmt::Display for IpAddress {
    fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
        write!(
            f,
            "{}.{}.{}.{}",
            self.address[0], self.address[1], self.address[2], self.address[3],
        )
    }
}

#[derive(Copy, Clone, PartialOrd, PartialEq, Default, Debug)]
pub struct MacAddress {
    pub address: [u8; 6],
}

impl MacAddress {
    pub fn new(a0: u8, a1: u8, a2: u8, a3: u8, a4: u8, a5: u8) -> MacAddress {
        MacAddress {
            address: [a0, a1, a2, a3, a4, a5],
        }
    }
}

impl ::core::fmt::Display for MacAddress {
    fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
        write!(
            f,
            "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
            self.address[0],
            self.address[1],
            self.address[2],
            self.address[3],
            self.address[4],
            self.address[5],
        )
    }
}

pub struct W5500<CS> {
    cs: CS,
}

impl<CS, PinError> W5500<CS>
where
    CS: OutputPin<Error = PinError>,
{
    pub fn new(cs: CS) -> Self {
        W5500 { cs }
    }

    pub fn init<E>(&mut self, spi: &mut Spi<E>) -> Result<(), Error<E, PinError>> {
        self.reset(spi)?;
        self.set_mode(spi, false, false, false, false)?;
        Ok(())
    }

    pub fn reset<E>(&mut self, spi: &mut Spi<E>) -> Result<(), Error<E, PinError>> {
        self.write_to(
            spi,
            Register::CommonRegister(0x00_00_u16),
            &[
                0b1000_0000, // Mode Register (force reset)
            ],
        )
    }

    pub fn set_mode<E>(
        &mut self,
        spi: &mut Spi<E>,
        wol: bool,
        ping_block: bool,
        ppoe: bool,
        force_arp: bool,
    ) -> Result<(), Error<E, PinError>> {
        let mut mode = 0x00;

        if wol {
            mode |= 1 << 5;
        }

        if ping_block {
            mode |= 1 << 4;
        }

        if ppoe {
            mode |= 1 << 3;
        }

        if force_arp {
            mode |= 1 << 1;
        }

        self.write_to(spi, Register::CommonRegister(0x00_00_u16), &[mode])
    }

    pub fn set_interrupt_mask<E>(
        &mut self,
        spi: &mut Spi<E>,
        sockets: &[Socket],
    ) -> Result<(), Error<E, PinError>> {
        let mut mask = 0u8;
        for socket in sockets.iter() {
            mask |= match *socket {
                Socket::Socket0 => 1 << 0,
                Socket::Socket1 => 1 << 1,
                Socket::Socket2 => 1 << 2,
                Socket::Socket3 => 1 << 3,
                Socket::Socket4 => 1 << 4,
                Socket::Socket5 => 1 << 5,
                Socket::Socket6 => 1 << 6,
                Socket::Socket7 => 1 << 7,
            };
        }
        self.write_to(spi, Register::CommonRegister(0x00_17_u16), &[mask])
    }

    pub fn set_socket_interrupt_mask<E>(
        &mut self,
        spi: &mut Spi<E>,
        socket: Socket,
        interrupts: &[Interrupt],
    ) -> Result<(), Error<E, PinError>> {
        let mut mask = 0u8;
        for interrupt in interrupts.iter() {
            mask |= *interrupt as u8;
        }
        self.write_to(spi, socket.at(SocketRegister::InterruptMask), &[mask])
    }

    pub fn set_gateway<E>(
        &mut self,
        spi: &mut Spi<E>,
        gateway: &IpAddress,
    ) -> Result<(), Error<E, PinError>> {
        self.write_to(spi, Register::CommonRegister(0x00_01_u16), &gateway.address)
    }

    pub fn set_subnet<E>(
        &mut self,
        spi: &mut Spi<E>,
        subnet: &IpAddress,
    ) -> Result<(), Error<E, PinError>> {
        self.write_to(spi, Register::CommonRegister(0x00_05_u16), &subnet.address)
    }

    pub fn set_mac<E>(
        &mut self,
        spi: &mut Spi<E>,
        mac: &MacAddress,
    ) -> Result<(), Error<E, PinError>> {
        self.write_to(spi, Register::CommonRegister(0x00_09_u16), &mac.address)
    }

    pub fn get_mac<E>(&mut self, spi: &mut Spi<E>) -> Result<MacAddress, Error<E, PinError>> {
        let mut mac = MacAddress::default();
        self.read_from(spi, Register::CommonRegister(0x00_09_u16), &mut mac.address)?;
        Ok(mac)
    }

    pub fn set_ip<E>(
        &mut self,
        spi: &mut Spi<E>,
        ip: &IpAddress,
    ) -> Result<(), Error<E, PinError>> {
        self.write_to(spi, Register::CommonRegister(0x00_0F_u16), &ip.address)
    }

    pub fn is_interrupt_set<E>(
        &mut self,
        spi: &mut Spi<E>,
        socket: Socket,
        interrupt: Interrupt,
    ) -> Result<bool, Error<E, PinError>> {
        let mut state = [0u8; 1];
        self.read_from(spi, socket.at(SocketRegister::Interrupt), &mut state)?;
        Ok(state[0] & interrupt as u8 != 0)
    }

    pub fn reset_interrupt<E>(
        &mut self,
        spi: &mut Spi<E>,
        socket: Socket,
        interrupt: Interrupt,
    ) -> Result<(), Error<E, PinError>> {
        self.write_to(
            spi,
            socket.at(SocketRegister::Interrupt),
            &[interrupt as u8],
        )
    }

    pub fn close<E>(&mut self, spi: &mut Spi<E>, socket: Socket) -> Result<(), Error<E, PinError>> {
        self.write_u8(
            spi,
            socket.at(SocketRegister::Command),
            SocketCommand::Close as u8,
        )
    }

    pub fn dissconnect<E>(
        &mut self,
        spi: &mut Spi<E>,
        socket: Socket,
    ) -> Result<(), Error<E, PinError>> {
        self.write_u8(
            spi,
            socket.at(SocketRegister::Command),
            SocketCommand::Disconnect as u8,
        )
    }

    pub fn get_socket_status<E>(
        &mut self,
        spi: &mut Spi<E>,
        socket: Socket,
    ) -> Result<Option<SocketStatus>, Error<E, PinError>> {
        let status = self.read_u8(spi, socket.at(SocketRegister::Status))?;

        Ok(match status {
            status if status == SocketStatus::Closed as u8 => Some(SocketStatus::Closed),
            status if status == SocketStatus::Init as u8 => Some(SocketStatus::Init),
            status if status == SocketStatus::Listen as u8 => Some(SocketStatus::Listen),
            status if status == SocketStatus::Established as u8 => Some(SocketStatus::Established),
            status if status == SocketStatus::CloseWait as u8 => Some(SocketStatus::CloseWait),
            status if status == SocketStatus::Udp as u8 => Some(SocketStatus::Udp),
            status if status == SocketStatus::MacRaw as u8 => Some(SocketStatus::MacRaw),
            status if status == SocketStatus::SynSent as u8 => Some(SocketStatus::SynSent),
            status if status == SocketStatus::SynRecv as u8 => Some(SocketStatus::SynRecv),
            status if status == SocketStatus::FinWait as u8 => Some(SocketStatus::FinWait),
            status if status == SocketStatus::Closing as u8 => Some(SocketStatus::Closing),
            status if status == SocketStatus::TimeWait as u8 => Some(SocketStatus::TimeWait),
            status if status == SocketStatus::LastAck as u8 => Some(SocketStatus::LastAck),
            _ => None,
        })
    }

    // See pages 46 and 49
    pub fn read_registers<E>(
        &mut self,
        spi: &mut Spi<E>,
        socket: Socket,
    ) -> Result<(u8, u8, u8, u8, u16, u8), Error<E, PinError>> {
        let mode = self.read_u8(spi, socket.at(SocketRegister::Mode))?;
        let command = self.read_u8(spi, socket.at(SocketRegister::Command))?;
        let interrupt = self.read_u8(spi, socket.at(SocketRegister::Interrupt))?;
        let status = self.read_u8(spi, socket.at(SocketRegister::Status))?;
        let port = self.read_u16(spi, socket.at(SocketRegister::LocalPort))?;
        let interrupt_mask = self.read_u8(spi, socket.at(SocketRegister::InterruptMask))?;

        Ok((mode, command, status, interrupt, port, interrupt_mask))
    }

    pub fn send_udp<E>(
        &mut self,
        spi: &mut Spi<E>,
        socket: Socket,
        local_port: u16,
        host: &IpAddress,
        host_port: u16,
        data: &[u8],
    ) -> Result<(), Error<E, PinError>> {
        // TODO not always socket 0
        // TODO check if in use

        self.write_to(
            spi,
            socket.at(SocketRegister::Mode),
            &[
                Protocol::UDP as u8,       // Socket Mode Register
                SocketCommand::Open as u8, // Socket Command Regsiter
            ],
        )?;

        {
            let local_port = u16_to_be_bytes(local_port);
            let host_port = u16_to_be_bytes(host_port);

            self.write_to(
                spi,
                socket.at(SocketRegister::LocalPort),
                &[
                    local_port[0],
                    local_port[1], // local port u16
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x00,
                    0x00, // destination mac
                    host.address[0],
                    host.address[1],
                    host.address[2],
                    host.address[3], // target IP
                    host_port[0],
                    host_port[1], // destination port (5354)
                ],
            )?;
        }

        let data_length = data.len() as u16;
        {
            let data_length = u16_to_be_bytes(data_length);

            self.write_to(
                spi,
                socket.at(SocketRegister::TxReadPointer),
                &[0x00, 0x00, data_length[0], data_length[1]],
            )?;
        }

        self.write_to(
            spi,
            socket.tx_register_at(0x00_00),
            &data[..data_length as usize],
        )?;

        self.write_u8(
            spi,
            socket.at(SocketRegister::Command),
            SocketCommand::Send as u8,
        )
    }

    pub fn listen_udp<E>(
        &mut self,
        spi: &mut Spi<E>,
        socket: Socket,
        port: u16,
    ) -> Result<(), Error<E, PinError>> {
        self.write_u16(spi, socket.at(SocketRegister::LocalPort), port)?;
        self.write_u8(spi, socket.at(SocketRegister::Mode), Protocol::UDP as u8)?;
        self.write_u8(
            spi,
            socket.at(SocketRegister::Command),
            SocketCommand::Open as u8,
        )
    }

    pub fn set_protocol<E>(
        &mut self,
        spi: &mut Spi<E>,
        socket: Socket,
        protocol: Protocol,
    ) -> Result<(), Error<E, PinError>> {
        self.write_u8(spi, socket.at(SocketRegister::Mode), protocol as u8)?;
        Ok(())
    }

    pub fn connect<E>(
        &mut self,
        spi: &mut Spi<E>,
        socket: Socket,
        host_ip: &IpAddress,
        host_port: u16,
    ) -> Result<(), Error<E, PinError>> {
        self.write_to(
            spi,
            socket.at(SocketRegister::DestinationIp),
            &[
                host_ip.address[0],
                host_ip.address[1],
                host_ip.address[2],
                host_ip.address[3],
            ],
        )?;

        self.write_u16(spi, socket.at(SocketRegister::DestinationPort), host_port)?;

        self.write_u8(
            spi,
            socket.at(SocketRegister::Command),
            SocketCommand::Connect as u8,
        )?;

        Ok(())
    }

    pub fn open_tcp<E>(
        &mut self,
        spi: &mut Spi<E>,
        socket: Socket,
    ) -> Result<(), Error<E, PinError>> {
        self.write_u8(
            spi,
            socket.at(SocketRegister::Command),
            SocketCommand::Open as u8,
        )?;

        loop {
            let status = self.read_u8(spi, socket.at(SocketRegister::Status))?;
            if status == SocketStatus::Init as u8 {
                return Ok(());
            }
        }
    }

    pub fn send_tcp<E>(
        &mut self,
        spi: &mut Spi<E>,
        socket: Socket,
        data: &[u8],
    ) -> Result<usize, Error<E, PinError>> {
        let mut data_length = data.len() as u16;

        if data_length == 0 {
            return Ok(0);
        }

        let max_length = self.read_u16(spi, socket.at(SocketRegister::TransmitBuffer))?;

        data_length = if data_length < max_length {
            data_length
        } else {
            max_length
        };
        loop {
            let tx_free_size = self.read_u16_atomic(spi, socket.at(SocketRegister::TxFreeSize))?;
            if data_length < tx_free_size {
                break;
            }
        }

        // get the write pointer and write data at that address
        let ptr = self.read_u16_atomic(spi, socket.at(SocketRegister::TxWritePointer))?;
        self.write_to(
            spi,
            socket.tx_register_at(ptr),
            &data[..data_length as usize],
        )?;

        // update the write pointer
        self.write_u16(
            spi,
            socket.at(SocketRegister::TxWritePointer),
            ptr + data_length,
        )?;
        self.write_u8(
            spi,
            socket.at(SocketRegister::Command),
            SocketCommand::Send as u8,
        )?;

        // wait for bytes to be sent and clear the interrupt
        loop {
            let interrupt = self.read_u8(spi, socket.at(SocketRegister::Interrupt))?;
            if interrupt & 0b0001_0000 == 0b0001_0000 {
                self.write_u8(spi, socket.at(SocketRegister::Interrupt), 0b0001_0000)?;
                break;
            }
        }

        Ok(data_length as usize)
    }

    pub fn listen_tcp<E>(
        &mut self,
        spi: &mut Spi<E>,
        socket: Socket,
        port: u16,
    ) -> Result<(), Error<E, PinError>> {
        self.write_u16(spi, socket.at(SocketRegister::LocalPort), port)?;

        self.write_u8(
            spi,
            socket.at(SocketRegister::Command),
            SocketCommand::Listen as u8,
        )?;

        loop {
            let status = self.read_u8(spi, socket.at(SocketRegister::Status))?;
            if status == SocketStatus::Listen as u8 {
                return Ok(());
            }
        }
    }

    pub fn send_keep_alive_tcp<E>(
        &mut self,
        spi: &mut Spi<E>,
        socket: Socket,
        port: u16,
    ) -> Result<(), Error<E, PinError>> {
        self.write_u16(spi, socket.at(SocketRegister::LocalPort), port)?;

        self.write_u8(
            spi,
            socket.at(SocketRegister::Command),
            SocketCommand::Listen as u8,
        )?;

        loop {
            let status = self.read_u8(spi, socket.at(SocketRegister::Status))?;
            if status == SocketStatus::Listen as u8 {
                return Ok(());
            }
        }
    }

    pub fn try_receive_tcp<E>(
        &mut self,
        spi: &mut Spi<E>,
        socket: Socket,
        destination: &mut [u8],
    ) -> Result<Option<usize>, Error<E, PinError>> {
        if self.read_u8(spi, socket.at(SocketRegister::InterruptMask))? & 0x04 == 0 {
            return Ok(None);
        }

        let receive_size = loop {
            let s0 = self.read_u16(spi, socket.at(SocketRegister::RxReceivedSize))?;
            let s1 = self.read_u16(spi, socket.at(SocketRegister::RxReceivedSize))?;
            if s0 == s1 {
                break s0 as usize;
            }
        };

        let buffer_len = destination.len();
        if receive_size == 0 || buffer_len == 0 {
            return Ok(None);
        }

        let size = if receive_size > buffer_len {
            buffer_len
        } else {
            receive_size
        };

        let read_pointer = self.read_u16(spi, socket.at(SocketRegister::RxReadPointer))?;
        self.read_from(
            spi,
            socket.rx_register_at(read_pointer),
            &mut destination[..size],
        )?;

        // reset
        self.write_u16(
            spi,
            socket.at(SocketRegister::RxReadPointer),
            read_pointer + size as u16,
        )?;
        self.write_u8(
            spi,
            socket.at(SocketRegister::Command),
            SocketCommand::Recv as u8,
        )?;
        Ok(Some(size))
    }

    /// TODO destination buffer has to be as large as the receive buffer or complete read is not guaranteed
    pub fn try_receive_udp<E>(
        &mut self,
        spi: &mut Spi<E>,
        socket: Socket,
        destination: &mut [u8],
    ) -> Result<Option<(IpAddress, u16, usize)>, Error<E, PinError>> {
        if self.read_u8(spi, socket.at(SocketRegister::InterruptMask))? & 0x04 == 0 {
            return Ok(None);
        }
        let receive_size = loop {
            let s0 = self.read_u16(spi, socket.at(SocketRegister::RxReceivedSize))?;
            let s1 = self.read_u16(spi, socket.at(SocketRegister::RxReceivedSize))?;
            if s0 == s1 {
                break s0 as usize;
            }
        };
        if receive_size >= 8 {
            let read_pointer = self.read_u16(spi, socket.at(SocketRegister::RxReadPointer))?;

            // |<-- read_pointer                                read_pointer + received_size -->|
            // |Destination IP Address | Destination Port | Byte Size of DATA | Actual DATA ... |
            // |   --- 4 Bytes ---     |  --- 2 Bytes --- |  --- 2 Bytes ---  |      ....       |
            let ip = self.read_ip(spi, socket.rx_register_at(read_pointer))?;
            let port = self.read_u16(spi, socket.rx_register_at(read_pointer + 4))?;
            let data_length = destination
                .len()
                .min(self.read_u16(spi, socket.rx_register_at(read_pointer + 6))? as usize);

            self.read_from(
                spi,
                socket.rx_register_at(read_pointer + 8),
                &mut destination[..data_length],
            )?;

            // reset
            self.write_u16(
                spi,
                socket.at(SocketRegister::RxReadPointer),
                read_pointer + receive_size as u16,
            )?;
            self.write_u8(
                spi,
                socket.at(SocketRegister::Command),
                SocketCommand::Recv as u8,
            )?;

            Ok(Some((ip, port, data_length)))
        } else {
            Ok(None)
        }
    }

    pub fn read_u8<E>(
        &mut self,
        spi: &mut Spi<E>,
        register: Register,
    ) -> Result<u8, Error<E, PinError>> {
        let mut buffer = [0u8; 1];
        self.read_from(spi, register, &mut buffer)?;
        Ok(buffer[0])
    }

    pub fn read_u16<E>(
        &mut self,
        spi: &mut Spi<E>,
        register: Register,
    ) -> Result<u16, Error<E, PinError>> {
        let mut buffer = [0u8; 2];
        self.read_from(spi, register, &mut buffer)?;
        Ok(BigEndian::read_u16(&buffer))
    }

    fn read_u16_atomic<E>(
        &mut self,
        spi: &mut Spi<E>,
        register: Register,
    ) -> Result<u16, Error<E, PinError>> {
        loop {
            let s0 = self.read_u16(spi, register)?;
            let s1 = self.read_u16(spi, register)?;
            if s0 == s1 {
                return Ok(s0);
            }
        }
    }

    pub fn read_ip<E>(
        &mut self,
        spi: &mut Spi<E>,
        register: Register,
    ) -> Result<IpAddress, Error<E, PinError>> {
        let mut ip = IpAddress::default();
        self.read_from(spi, register, &mut ip.address)?;
        Ok(ip)
    }

    pub fn read_from<E>(
        &mut self,
        spi: &mut Spi<E>,
        register: Register,
        target: &mut [u8],
    ) -> Result<(), Error<E, PinError>> {
        self.chip_select().map_err(Error::Pin)?;
        let mut request = [
            0_u8,
            0_u8,
            register.control_byte() | COMMAND_READ | VARIABLE_DATA_LENGTH,
        ];
        BigEndian::write_u16(&mut request[..2], register.address());
        let result = self
            .write_bytes(spi, &request)
            .and_then(|_| self.read_bytes(spi, target));
        self.chip_deselect().map_err(Error::Pin)?;
        result
    }

    pub fn write_u8<E>(
        &mut self,
        spi: &mut Spi<E>,
        register: Register,
        value: u8,
    ) -> Result<(), Error<E, PinError>> {
        self.write_to(spi, register, &[value])
    }

    pub fn write_u16<E>(
        &mut self,
        spi: &mut Spi<E>,
        register: Register,
        value: u16,
    ) -> Result<(), Error<E, PinError>> {
        let mut data = [0u8; 2];
        BigEndian::write_u16(&mut data, value);
        self.write_to(spi, register, &data)
    }

    pub fn write_to<E>(
        &mut self,
        spi: &mut Spi<E>,
        register: Register,
        data: &[u8],
    ) -> Result<(), Error<E, PinError>> {
        self.chip_select().map_err(Error::Pin)?;
        let mut request = [
            0_u8,
            0_u8,
            register.control_byte() | COMMAND_WRITE | VARIABLE_DATA_LENGTH,
        ];
        BigEndian::write_u16(&mut request[..2], register.address());
        let result = self
            .write_bytes(spi, &request)
            .and_then(|_| self.write_bytes(spi, data));
        self.chip_deselect().map_err(Error::Pin)?;
        result
    }

    fn read_bytes<E>(
        &mut self,
        spi: &mut Spi<E>,
        bytes: &mut [u8],
    ) -> Result<(), Error<E, PinError>> {
        for i in 0..bytes.len() {
            bytes[i] = self.read(spi)?;
        }
        Ok(())
    }

    fn read<E>(&mut self, spi: &mut Spi<E>) -> Result<u8, Error<E, PinError>> {
        let command = &mut [0x00];
        let result = spi.transfer(command)?;
        Ok(result[0])
    }

    fn write_bytes<E>(&mut self, spi: &mut Spi<E>, bytes: &[u8]) -> Result<(), Error<E, PinError>> {
        for b in bytes {
            self.write(spi, *b)?;
        }
        Ok(())
    }

    fn write<E>(&mut self, spi: &mut Spi<E>, byte: u8) -> Result<(), Error<E, PinError>> {
        spi.transfer(&mut [byte])?;
        Ok(())
    }

    fn chip_select(&mut self) -> Result<(), PinError> {
        self.cs.set_low()
    }

    fn chip_deselect(&mut self) -> Result<(), PinError> {
        self.cs.set_high()
    }
}

fn u16_to_be_bytes(u16: u16) -> [u8; 2] {
    let mut bytes = [0u8; 2];
    BigEndian::write_u16(&mut bytes, u16);
    bytes
}

#[repr(u8)]
#[derive(Copy, Clone, PartialEq, Debug)]
pub enum SocketStatus {
    Closed = 0x0000,
    Init = 0x0013,
    Listen = 0x0014,
    Established = 0x0017,
    CloseWait = 0x001C,
    Udp = 0x0022,
    MacRaw = 0x0042,
    // the statuses below are temporary status indicated during changing the status of Socket n
    SynSent = 0x0015,
    SynRecv = 0x0016,
    FinWait = 0x0018,
    Closing = 0x001A,
    TimeWait = 0x001B,
    LastAck = 0x001D,
}

#[repr(u8)]
#[derive(Copy, Clone, PartialEq, Debug)]
pub enum SocketRegister {
    Mode = 0x0000,
    Command = 0x0001,
    Interrupt = 0x0002,
    Status = 0x0003,
    LocalPort = 0x0004,
    DestinationMac = 0x0006,
    DestinationIp = 0x000C,
    DestinationPort = 0x0010,
    MaxSegmentSize = 0x0012,
    // Reserved 0x0014
    TypeOfService = 0x0015,
    TimeToLive = 0x0016,
    // Reserved 0x0017 - 0x001D
    ReceiveBuffer = 0x001E,
    TransmitBuffer = 0x001F,
    TxFreeSize = 0x0020,
    TxReadPointer = 0x0022,
    TxWritePointer = 0x0024,
    RxReceivedSize = 0x0026,
    RxReadPointer = 0x0028,
    RxWritePointer = 0x002A,
    InterruptMask = 0x002C,
    FragmentOffset = 0x002D,
    KeepAliveTimer = 0x002F,
    // Reserved 0x0030 - 0xFFFF
}

#[repr(u8)]
#[derive(Copy, Clone, PartialEq, Debug)]
pub enum Interrupt {
    SendOk = 1 << 4,
    Timeout = 1 << 3,
    Received = 1 << 2,
    Disconnected = 1 << 1,
    Connected = 1 << 0,
}

#[repr(u8)]
#[derive(Copy, Clone, PartialEq, Debug)]
pub enum Protocol {
    TCP = 0b0001,
    UDP = 0b0010,
    MACRAW = 0b0100,
}

#[repr(u8)]
#[derive(Copy, Clone, PartialEq, Debug)]
pub enum SocketCommand {
    Open = 0x01,
    Listen = 0x02,
    Connect = 0x04,
    Disconnect = 0x08,
    Close = 0x10,
    Send = 0x20,
    SendMac = 0x21,
    SendKeep = 0x22,
    Recv = 0x40,
}

#[derive(Copy, Clone, PartialEq, PartialOrd, Debug)]
pub enum Socket {
    Socket0,
    Socket1,
    Socket2,
    Socket3,
    Socket4,
    Socket5,
    Socket6,
    Socket7,
}

impl Socket {
    pub fn number(&self) -> usize {
        match *self {
            Socket::Socket0 => 0,
            Socket::Socket1 => 1,
            Socket::Socket2 => 2,
            Socket::Socket3 => 3,
            Socket::Socket4 => 4,
            Socket::Socket5 => 5,
            Socket::Socket6 => 6,
            Socket::Socket7 => 7,
        }
    }

    fn tx_register_at(&self, address: u16) -> Register {
        match *self {
            Socket::Socket0 => Register::Socket0TxBuffer(address),
            Socket::Socket1 => Register::Socket1TxBuffer(address),
            Socket::Socket2 => Register::Socket2TxBuffer(address),
            Socket::Socket3 => Register::Socket3TxBuffer(address),
            Socket::Socket4 => Register::Socket4TxBuffer(address),
            Socket::Socket5 => Register::Socket5TxBuffer(address),
            Socket::Socket6 => Register::Socket6TxBuffer(address),
            Socket::Socket7 => Register::Socket7TxBuffer(address),
        }
    }

    fn rx_register_at(&self, address: u16) -> Register {
        match *self {
            Socket::Socket0 => Register::Socket0RxBuffer(address),
            Socket::Socket1 => Register::Socket1RxBuffer(address),
            Socket::Socket2 => Register::Socket2RxBuffer(address),
            Socket::Socket3 => Register::Socket3RxBuffer(address),
            Socket::Socket4 => Register::Socket4RxBuffer(address),
            Socket::Socket5 => Register::Socket5RxBuffer(address),
            Socket::Socket6 => Register::Socket6RxBuffer(address),
            Socket::Socket7 => Register::Socket7RxBuffer(address),
        }
    }

    fn register_at(&self, address: u16) -> Register {
        match *self {
            Socket::Socket0 => Register::Socket0Register(address),
            Socket::Socket1 => Register::Socket1Register(address),
            Socket::Socket2 => Register::Socket2Register(address),
            Socket::Socket3 => Register::Socket3Register(address),
            Socket::Socket4 => Register::Socket4Register(address),
            Socket::Socket5 => Register::Socket5Register(address),
            Socket::Socket6 => Register::Socket6Register(address),
            Socket::Socket7 => Register::Socket7Register(address),
        }
    }

    fn at(&self, register: SocketRegister) -> Register {
        self.register_at(register as u16)
    }
}

#[derive(Copy, Clone, PartialEq, Debug)]
pub enum Register {
    CommonRegister(u16),

    Socket0Register(u16),
    Socket0TxBuffer(u16),
    Socket0RxBuffer(u16),

    Socket1Register(u16),
    Socket1TxBuffer(u16),
    Socket1RxBuffer(u16),

    Socket2Register(u16),
    Socket2TxBuffer(u16),
    Socket2RxBuffer(u16),

    Socket3Register(u16),
    Socket3TxBuffer(u16),
    Socket3RxBuffer(u16),

    Socket4Register(u16),
    Socket4TxBuffer(u16),
    Socket4RxBuffer(u16),

    Socket5Register(u16),
    Socket5TxBuffer(u16),
    Socket5RxBuffer(u16),

    Socket6Register(u16),
    Socket6TxBuffer(u16),
    Socket6RxBuffer(u16),

    Socket7Register(u16),
    Socket7TxBuffer(u16),
    Socket7RxBuffer(u16),
}

impl Register {
    fn control_byte(&self) -> u8 {
        match *self {
            Register::CommonRegister(_) => 0b00000_000,

            Register::Socket0Register(_) => 0b00001_000,
            Register::Socket0TxBuffer(_) => 0b00010_000,
            Register::Socket0RxBuffer(_) => 0b00011_000,

            Register::Socket1Register(_) => 0b00101_000,
            Register::Socket1TxBuffer(_) => 0b00110_000,
            Register::Socket1RxBuffer(_) => 0b00111_000,

            Register::Socket2Register(_) => 0b01001_000,
            Register::Socket2TxBuffer(_) => 0b01010_000,
            Register::Socket2RxBuffer(_) => 0b01011_000,

            Register::Socket3Register(_) => 0b01101_000,
            Register::Socket3TxBuffer(_) => 0b01110_000,
            Register::Socket3RxBuffer(_) => 0b01111_000,

            Register::Socket4Register(_) => 0b10001_000,
            Register::Socket4TxBuffer(_) => 0b10010_000,
            Register::Socket4RxBuffer(_) => 0b10011_000,

            Register::Socket5Register(_) => 0b10101_000,
            Register::Socket5TxBuffer(_) => 0b10110_000,
            Register::Socket5RxBuffer(_) => 0b10111_000,

            Register::Socket6Register(_) => 0b11001_000,
            Register::Socket6TxBuffer(_) => 0b11010_000,
            Register::Socket6RxBuffer(_) => 0b11011_000,

            Register::Socket7Register(_) => 0b11101_000,
            Register::Socket7TxBuffer(_) => 0b11110_000,
            Register::Socket7RxBuffer(_) => 0b11111_000,
        }
    }

    fn address(&self) -> u16 {
        match *self {
            Register::CommonRegister(address) => address,

            Register::Socket0Register(address) => address,
            Register::Socket0TxBuffer(address) => address,
            Register::Socket0RxBuffer(address) => address,

            Register::Socket1Register(address) => address,
            Register::Socket1TxBuffer(address) => address,
            Register::Socket1RxBuffer(address) => address,

            Register::Socket2Register(address) => address,
            Register::Socket2TxBuffer(address) => address,
            Register::Socket2RxBuffer(address) => address,

            Register::Socket3Register(address) => address,
            Register::Socket3TxBuffer(address) => address,
            Register::Socket3RxBuffer(address) => address,

            Register::Socket4Register(address) => address,
            Register::Socket4TxBuffer(address) => address,
            Register::Socket4RxBuffer(address) => address,

            Register::Socket5Register(address) => address,
            Register::Socket5TxBuffer(address) => address,
            Register::Socket5RxBuffer(address) => address,

            Register::Socket6Register(address) => address,
            Register::Socket6TxBuffer(address) => address,
            Register::Socket6RxBuffer(address) => address,

            Register::Socket7Register(address) => address,
            Register::Socket7TxBuffer(address) => address,
            Register::Socket7RxBuffer(address) => address,
        }
    }
}
