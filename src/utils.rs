//! Parsing of the sensor stream messages, command line parsing and small helpers.

pub const DEFAULT_ADDRESS: &str = "127.0.0.1";
pub const DEFAULT_PORT: u16 = 4242;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DataKind
{
    TruePosition,
    Speed,
    Acceleration,
    Direction,
    Position,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Message
{
    Start,
    End,
    Goodbye,
    Data { time: f64, kind: DataKind, values: Vec<f64> },
    Info(String),
    Error(String),
}

pub fn kmh_to_ms(v: f64) -> f64
{
    v / 3.6
}

/// "[HH:MM:SS.mmm]" (without the brackets) -> seconds.
pub fn parse_time(s: &str) -> Result<f64, String>
{
    let parts: Vec<&str> = s.trim().split(':').collect();

    if parts.len() != 3
    {
        return Err(format!("Invalid timestamp '{}'", s));
    }
    let h: f64 = parts[0].parse().map_err(|_| format!("Invalid timestamp '{}'", s))?;
    let m: f64 = parts[1].parse().map_err(|_| format!("Invalid timestamp '{}'", s))?;
    let sec: f64 = parts[2].parse().map_err(|_| format!("Invalid timestamp '{}'", s))?;
    let t = h * 3600.0 + m * 60.0 + sec;

    if !t.is_finite() || h < 0.0 || m < 0.0 || sec < 0.0
    {
        return Err(format!("Invalid timestamp '{}'", s));
    }
    Ok(t)
}

fn parse_kind(name: &str) -> Option<DataKind>
{
    match name.trim()
    {
        "TRUE POSITION" => Some(DataKind::TruePosition),
        "SPEED" => Some(DataKind::Speed),
        "ACCELERATION" => Some(DataKind::Acceleration),
        "DIRECTION" => Some(DataKind::Direction),
        "POSITION" => Some(DataKind::Position),
        _ => None,
    }
}

/// Parse one datagram sent by the sensor stream.
pub fn parse_message(text: &str) -> Result<Message, String>
{
    let t = text.trim();

    if t == "MSG_START"
    {
        return Ok(Message::Start);
    }
    if t == "MSG_END"
    {
        return Ok(Message::End);
    }
    if t.starts_with("GOODBYE")
    {
        return Ok(Message::Goodbye);
    }
    if t.to_ascii_lowercase().starts_with("error")
    {
        return Ok(Message::Error(t.to_string()));
    }
    if !t.starts_with('[')
    {
        return Ok(Message::Info(t.to_string()));
    }

    let mut lines = t.lines();
    let head = lines.next().unwrap_or("");
    let close = head.find(']').ok_or_else(|| format!("Malformed message '{}'", head))?;
    let time = parse_time(&head[1..close])?;
    let kind = match parse_kind(&head[close + 1..])
    {
        Some(k) => k,
        None => return Ok(Message::Info(t.to_string())),
    };

    let mut values = Vec::new();
    for l in lines
    {
        let l = l.trim();
        if l.is_empty()
        {
            continue;
        }
        let v: f64 = l.parse().map_err(|_| format!("Invalid number '{}'", l))?;
        if !v.is_finite()
        {
            return Err(format!("Non finite number '{}'", l));
        }
        values.push(v);
    }

    let expected = if kind == DataKind::Speed { 1 } else { 3 };
    if values.len() != expected
    {
        return Err(format!("Expected {} values in '{}', got {}", expected, head, values.len()));
    }
    Ok(Message::Data { time, kind, values })
}

/// Format required by the sensor stream: "X Y Z".
pub fn format_position(p: [f64; 3]) -> String
{
    format!("{} {} {}", p[0], p[1], p[2])
}

pub fn distance(a: [f64; 3], b: [f64; 3]) -> f64
{
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

#[derive(Clone, Debug, PartialEq)]
pub struct Config
{
    pub address: String,
    pub port: u16,
    pub visual: Option<String>,
    pub meanspeed: bool,
    pub delta: bool,
}

#[derive(Debug, PartialEq)]
pub enum Command
{
    Help,
    Run(Config),
}

pub fn usage() -> String
{
    "ft_kalman - Kalman filter for the imu-sensor-stream

USAGE:
    ft_kalman [OPTIONS]

OPTIONS:
    -a, --address <host>     Address of the sensor stream (default 127.0.0.1)
    -p, --port <port>        Port of the sensor stream (default 4242)
    -v, --visual <file>      Write an interactive 3D trajectory viewer (HTML) with the variance
    -m, --meanspeed          Print the mean response time of the filter at the end
    -d, --delta              Print the error against the true position at the end
                             (needs imu-sensor-stream started with --debug)
    -h, --help               Print this help"
        .to_string()
}

pub fn parse_args(args: &[String]) -> Result<Command, String>
{
    let mut cfg = Config { address: DEFAULT_ADDRESS.to_string(), port: DEFAULT_PORT, visual: None, meanspeed: false, delta: false };
    let mut i = 0;

    while i < args.len()
    {
        let a = args[i].as_str();
        match a
        {
            "-h" | "--help" => return Ok(Command::Help),
            "-m" | "--meanspeed" | "--filterspeed" => cfg.meanspeed = true,
            "-d" | "--delta" => cfg.delta = true,
            "-a" | "--address" | "-p" | "--port" | "-v" | "--visual" =>
            {
                i += 1;
                let value = args.get(i).ok_or_else(|| format!("Missing value for '{}'", a))?;
                match a
                {
                    "-a" | "--address" =>
                    {
                        if value.is_empty()
                        {
                            return Err("Empty address".to_string());
                        }
                        cfg.address = value.clone();
                    }
                    "-p" | "--port" =>
                    {
                        let p: u16 = value.parse().map_err(|_| format!("Invalid port '{}'", value))?;
                        if p == 0
                        {
                            return Err("Port must be between 1 and 65535".to_string());
                        }
                        cfg.port = p;
                    }
                    _ =>
                    {
                        if value.is_empty()
                        {
                            return Err("Empty file name".to_string());
                        }
                        cfg.visual = Some(value.clone());
                    }
                }
            }
            _ => return Err(format!("Unknown argument '{}'", a)),
        }
        i += 1;
    }
    Ok(Command::Run(cfg))
}
