use ft_kalman::utils::*;

fn args(v: &[&str]) -> Vec<String>
{
    v.iter().map(|s| s.to_string()).collect()
}

#[test]
fn parses_the_messages_of_the_stream()
{
    assert_eq!(parse_message("MSG_START").unwrap(), Message::Start);
    assert_eq!(parse_message("MSG_END").unwrap(), Message::End);
    assert_eq!(parse_message("GOODBYE.").unwrap(), Message::Goodbye);
    assert!(matches!(parse_message("Trajectory Generated!\nSending Info. . .\n").unwrap(), Message::Info(_)));
    assert!(matches!(parse_message("Error: Delta is too high").unwrap(), Message::Error(_)));

    let m = parse_message("[00:01:02.500]ACCELERATION\n0.5\n-1.5\n2\n").unwrap();
    assert_eq!(m, Message::Data { time: 62.5, kind: DataKind::Acceleration, values: vec![0.5, -1.5, 2.0] });
    let m = parse_message("[00:00:00.000]SPEED\n58.8\n").unwrap();
    assert_eq!(m, Message::Data { time: 0.0, kind: DataKind::Speed, values: vec![58.8] });
    assert!(matches!(parse_message("[01:30:00.000]TRUE POSITION\n1\n2\n3\n").unwrap(), Message::Data { kind: DataKind::TruePosition, .. }));
    assert!(matches!(parse_message("[00:00:03.000]POSITION\n1\n2\n3\n").unwrap(), Message::Data { kind: DataKind::Position, .. }));
    assert!(matches!(parse_message("[00:00:03.000]DIRECTION\n1\n2\n3\n").unwrap(), Message::Data { kind: DataKind::Direction, .. }));
}

#[test]
fn rejects_malformed_messages_without_panic()
{
    assert!(parse_message("[00:00:00.000]ACCELERATION\n1\n2\n").is_err()); // missing value
    assert!(parse_message("[00:00:00.000]ACCELERATION\n1\n2\nabc\n").is_err());
    assert!(parse_message("[00:00:00.000]ACCELERATION\n1\n2\nNaN\n").is_err());
    assert!(parse_message("[00:00:00.000]ACCELERATION\n1\n2\ninf\n").is_err());
    assert!(parse_message("[00:00:00.000]SPEED\n1\n2\n").is_err());
    assert!(parse_message("[00:00]SPEED\n1\n").is_err());
    assert!(parse_message("[xx:00:00.000]SPEED\n1\n").is_err());
    assert!(parse_message("[00:00:00.000 SPEED\n1\n").is_err());
    assert!(parse_message("[-1:00:00.000]SPEED\n1\n").is_err());
    assert!(matches!(parse_message("").unwrap(), Message::Info(_)));
    assert!(matches!(parse_message("[00:00:00.000]UNKNOWN\n1\n").unwrap(), Message::Info(_)));
}

#[test]
fn formats_the_position_like_the_subject()
{
    assert_eq!(format_position([1.5, -2.0, 0.25]), "1.5 -2 0.25");
    let back: Vec<f64> = format_position([1.7325073314060224, -2.2213777837034083, 0.49999962025821726])
        .split(' ').map(|v| v.parse().unwrap()).collect();
    assert_eq!(back, vec![1.7325073314060224, -2.2213777837034083, 0.49999962025821726]);
}

#[test]
fn conversions()
{
    assert!((kmh_to_ms(36.0) - 10.0).abs() < 1e-12);
    assert!((distance([0.0, 0.0, 0.0], [3.0, 4.0, 0.0]) - 5.0).abs() < 1e-12);
}

#[test]
fn command_line()
{
    assert_eq!(parse_args(&args(&["-h"])).unwrap(), Command::Help);
    assert_eq!(parse_args(&args(&["--help"])).unwrap(), Command::Help);

    match parse_args(&args(&[])).unwrap()
    {
        Command::Run(c) => assert_eq!((c.address.as_str(), c.port, c.visual, c.meanspeed, c.delta), ("127.0.0.1", 4242, None, false, false)),
        _ => panic!(),
    }
    match parse_args(&args(&["-p", "5000", "-a", "localhost", "-v", "t.html", "-m", "-d"])).unwrap()
    {
        Command::Run(c) => assert_eq!((c.address.as_str(), c.port, c.visual, c.meanspeed, c.delta), ("localhost", 5000, Some("t.html".to_string()), true, true)),
        _ => panic!(),
    }
    assert!(parse_args(&args(&["--filterspeed"])).is_ok());
}

#[test]
fn command_line_errors()
{
    for bad in [
        vec!["-p"], vec!["-p", "abc"], vec!["-p", "0"], vec!["-p", "70000"], vec!["-p", "-1"],
        vec!["-a"], vec!["-a", ""], vec!["-v"], vec!["-v", ""], vec!["--nope"], vec!["extra"],
    ]
    {
        assert!(parse_args(&args(&bad)).is_err(), "{:?} should be refused", bad);
    }
    assert!(usage().contains("USAGE"));
}
