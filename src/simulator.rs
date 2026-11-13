//! UDP link with the imu-sensor-stream program: send "READY", receive the
//! measurements frame by frame, answer each frame with a position estimate.

use crate::utils::{format_position, parse_message, DataKind, Message};
use std::io::ErrorKind;
use std::net::{ToSocketAddrs, UdpSocket};
use std::time::Duration;

/// One measurement frame (everything received between MSG_START and MSG_END).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Frame
{
    /// Seconds since the start of the trajectory.
    pub time: f64,
    /// Only in the first frame (and in every frame when the stream runs with --debug).
    pub true_position: Option<[f64; 3]>,
    /// km/h, only in the first frame (and with --debug).
    pub speed_kmh: Option<f64>,
    /// m/s², XYZ frame, every 10 ms.
    pub acceleration: Option<[f64; 3]>,
    /// Euler angles (roll, pitch, yaw), every 10 ms.
    pub direction: Option<[f64; 3]>,
    /// GPS position, every 3 s.
    pub gps: Option<[f64; 3]>,
}

pub struct Simulator
{
    socket: UdpSocket,
    buf: Vec<u8>,
    first_timeout: Duration,
    timeout: Duration,
    started: bool,
}

impl Simulator
{
    pub fn connect(address: &str, port: u16) -> Result<Simulator, String>
    {
        let target = (address, port)
            .to_socket_addrs()
            .map_err(|e| format!("Cannot resolve '{}:{}': {}", address, port, e))?
            .next()
            .ok_or_else(|| format!("No address found for '{}:{}'", address, port))?;
        let local = if target.is_ipv4() { "0.0.0.0:0" } else { "[::]:0" };
        let socket = UdpSocket::bind(local).map_err(|e| format!("Cannot open UDP socket: {}", e))?;

        socket.connect(target).map_err(|e| format!("Cannot connect to {}: {}", target, e))?;
        Ok(Simulator
        {
            socket,
            buf: vec![0u8; 65536],
            first_timeout: Duration::from_secs(30),
            timeout: Duration::from_secs(5),
            started: false,
        })
    }

    pub fn set_timeouts(&mut self, first: Duration, next: Duration)
    {
        self.first_timeout = first;
        self.timeout = next;
    }

    /// Ask the stream to start the generation of the trajectory.
    pub fn ready(&self) -> Result<(), String>
    {
        self.socket.send(b"READY").map_err(|e| Self::net_error("send READY", e))?;
        Ok(())
    }

    pub fn send_estimate(&self, position: [f64; 3]) -> Result<(), String>
    {
        if position.iter().any(|v| !v.is_finite())
        {
            return Err("Refusing to send a non finite estimate".to_string());
        }
        self.socket
            .send(format_position(position).as_bytes())
            .map_err(|e| Self::net_error("send estimate", e))?;
        Ok(())
    }

    fn net_error(what: &str, e: std::io::Error) -> String
    {
        match e.kind()
        {
            ErrorKind::WouldBlock | ErrorKind::TimedOut => format!("Timeout ({}): the sensor stream does not answer", what),
            ErrorKind::ConnectionRefused => format!("Cannot {}: no sensor stream listening at this address/port", what),
            _ => format!("Cannot {}: {}", what, e),
        }
    }

    fn receive(&mut self) -> Result<Message, String>
    {
        let timeout = if self.started { self.timeout } else { self.first_timeout };

        self.socket.set_read_timeout(Some(timeout)).map_err(|e| format!("Socket error: {}", e))?;
        let n = self.socket.recv(&mut self.buf).map_err(|e| Self::net_error("receive", e))?;
        self.started = true;
        parse_message(&String::from_utf8_lossy(&self.buf[..n]))
    }

    /// Next complete frame. `Ok(None)` when the stream says goodbye (end of the trajectory).
    pub fn next_frame(&mut self) -> Result<Option<Frame>, String>
    {
        let mut frame: Option<Frame> = None;

        loop
        {
            match self.receive()?
            {
                Message::Start => frame = Some(Frame::default()),
                Message::End =>
                {
                    if let Some(f) = frame.take()
                    {
                        return Ok(Some(f));
                    }
                }
                Message::Goodbye => return Ok(None),
                Message::Error(e) => return Err(format!("Sensor stream error: {}", e)),
                Message::Info(_) => {}
                Message::Data { time, kind, values } =>
                {
                    let f = frame.get_or_insert_with(Frame::default);
                    f.time = time;
                    let v3 = if values.len() == 3 { Some([values[0], values[1], values[2]]) } else { None };
                    match kind
                    {
                        DataKind::TruePosition => f.true_position = v3,
                        DataKind::Speed => f.speed_kmh = values.first().copied(),
                        DataKind::Acceleration => f.acceleration = v3,
                        DataKind::Direction => f.direction = v3,
                        DataKind::Position => f.gps = v3,
                    }
                }
            }
        }
    }
}
