use ft_kalman::simulator::Simulator;
use std::net::{SocketAddr, UdpSocket};
use std::thread;
use std::time::Duration;

/// Fake sensor stream: waits for READY, then runs `script`.
fn fake_stream<F>(script: F) -> u16
where
    F: FnOnce(UdpSocket, SocketAddr) + Send + 'static,
{
    let sock = UdpSocket::bind("127.0.0.1:0").unwrap();
    let port = sock.local_addr().unwrap().port();

    thread::spawn(move || {
        let mut buf = [0u8; 64];
        let (n, client) = sock.recv_from(&mut buf).unwrap();
        assert_eq!(&buf[..n], b"READY");
        script(sock, client);
    });
    port
}

fn send(sock: &UdpSocket, to: SocketAddr, text: &str)
{
    sock.send_to(text.as_bytes(), to).unwrap();
}

fn recv_text(sock: &UdpSocket) -> String
{
    let mut buf = [0u8; 256];
    sock.set_read_timeout(Some(Duration::from_secs(5))).unwrap();
    let (n, _) = sock.recv_from(&mut buf).unwrap();
    String::from_utf8_lossy(&buf[..n]).to_string()
}

#[test]
fn full_dialog()
{
    let port = fake_stream(|sock, c| {
        send(&sock, c, "Trajectory Generation. . .\n");
        send(&sock, c, "Trajectory Generated!\nSending Info. . .\n");
        send(&sock, c, "MSG_START");
        send(&sock, c, "[00:00:00.000]TRUE POSITION\n1.5\n-2.5\n0.5\n");
        send(&sock, c, "[00:00:00.000]SPEED\n36\n");
        send(&sock, c, "[00:00:00.000]ACCELERATION\n0.1\n0.2\n0.3\n");
        send(&sock, c, "[00:00:00.000]DIRECTION\n0.01\n0.02\n0.03\n");
        send(&sock, c, "MSG_END");
        assert_eq!(recv_text(&sock), "1 2 3");
        send(&sock, c, "MSG_START");
        send(&sock, c, "[00:00:03.010]POSITION\n9\n8\n7\n");
        send(&sock, c, "[00:00:03.010]ACCELERATION\n0\n0\n0\n");
        send(&sock, c, "MSG_END");
        assert_eq!(recv_text(&sock), "4.5 5.5 6.5");
        send(&sock, c, "GOODBYE.");
    });

    let mut sim = Simulator::connect("127.0.0.1", port).unwrap();
    sim.ready().unwrap();

    let f = sim.next_frame().unwrap().unwrap();
    assert_eq!(f.true_position, Some([1.5, -2.5, 0.5]));
    assert_eq!(f.speed_kmh, Some(36.0));
    assert_eq!(f.acceleration, Some([0.1, 0.2, 0.3]));
    assert_eq!(f.direction, Some([0.01, 0.02, 0.03]));
    assert_eq!(f.gps, None);
    assert_eq!(f.time, 0.0);
    sim.send_estimate([1.0, 2.0, 3.0]).unwrap();

    let f = sim.next_frame().unwrap().unwrap();
    assert_eq!(f.gps, Some([9.0, 8.0, 7.0]));
    assert!((f.time - 3.01).abs() < 1e-9);
    assert_eq!(f.true_position, None);
    sim.send_estimate([4.5, 5.5, 6.5]).unwrap();

    assert!(sim.next_frame().unwrap().is_none());
}

#[test]
fn server_error_is_reported()
{
    let port = fake_stream(|sock, c| send(&sock, c, "Error: Delta is too high"));
    let mut sim = Simulator::connect("127.0.0.1", port).unwrap();
    sim.ready().unwrap();

    let e = sim.next_frame().unwrap_err();
    assert!(e.contains("Delta is too high"), "{}", e);
}

#[test]
fn silent_server_gives_a_timeout_error()
{
    let port = fake_stream(|_sock, _| thread::sleep(Duration::from_millis(1500)));
    let mut sim = Simulator::connect("127.0.0.1", port).unwrap();
    sim.set_timeouts(Duration::from_millis(200), Duration::from_millis(200));
    sim.ready().unwrap();

    assert!(sim.next_frame().unwrap_err().contains("Timeout"));
}

#[test]
fn nobody_listening_is_an_error_not_a_hang()
{
    let port = UdpSocket::bind("127.0.0.1:0").unwrap().local_addr().unwrap().port(); // closed right away
    let mut sim = Simulator::connect("127.0.0.1", port).unwrap();
    sim.set_timeouts(Duration::from_secs(3), Duration::from_secs(3));
    let _ = sim.ready();

    assert!(sim.next_frame().is_err());
}

#[test]
fn malformed_data_is_an_error()
{
    let port = fake_stream(|sock, c| {
        send(&sock, c, "MSG_START");
        send(&sock, c, "[00:00:00.000]ACCELERATION\n0.1\nabc\n0.3\n");
    });
    let mut sim = Simulator::connect("127.0.0.1", port).unwrap();
    sim.ready().unwrap();

    assert!(sim.next_frame().is_err());
}

#[test]
fn refuses_to_send_a_non_finite_estimate_and_bad_addresses()
{
    let port = fake_stream(|_, _| {});
    let sim = Simulator::connect("127.0.0.1", port).unwrap();

    assert!(sim.send_estimate([f64::NAN, 0.0, 0.0]).is_err());
    assert!(sim.send_estimate([0.0, f64::INFINITY, 0.0]).is_err());
    assert!(Simulator::connect("not an address !", 4242).is_err());
}
