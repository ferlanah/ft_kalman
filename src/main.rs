use ft_kalman::simulator::{Frame, Simulator};
use ft_kalman::tracker::Tracker;
use ft_kalman::utils::{distance, parse_args, usage, Command, Config};
use ft_kalman::visual::Recorder;
use std::process::ExitCode;
use std::time::{Duration, Instant};

struct Stats
{
    count: u64,
    total: Duration,
    worst: Duration,
    delta_count: u64,
    delta_sum: f64,
    delta_max: f64,
}

fn run(cfg: &Config, recorder: &mut Recorder, stats: &mut Stats) -> Result<(), String>
{
    let mut sim = Simulator::connect(&cfg.address, cfg.port)?;
    sim.ready()?;

    let first = match sim.next_frame()?
    {
        Some(f) => f,
        None => return Err("The sensor stream ended before sending any data".to_string()),
    };
    let mut tracker = Tracker::new(&first)?;
    let mut frame: Frame = first;

    loop
    {
        let start = Instant::now();

        if let Some(gps) = frame.gps
        {
            recorder.record_gps(gps);
        }
        tracker.step(&frame)?;
        let estimate = tracker.position();
        sim.send_estimate(estimate)?;

        let elapsed = start.elapsed();
        stats.count += 1;
        stats.total += elapsed;
        stats.worst = stats.worst.max(elapsed);
        if let Some(truth) = frame.true_position
        {
            let d = distance(estimate, truth);
            stats.delta_count += 1;
            stats.delta_sum += d;
            stats.delta_max = stats.delta_max.max(d);
        }
        recorder.record(frame.time, estimate, tracker.position_sigma(), frame.true_position);

        frame = match sim.next_frame()?
        {
            Some(f) => f,
            None => return Ok(()),
        };
    }
}

fn main() -> ExitCode
{
    let args: Vec<String> = std::env::args().skip(1).collect();
    let cfg = match parse_args(&args)
    {
        Ok(Command::Help) =>
        {
            println!("{}", usage());
            return ExitCode::SUCCESS;
        }
        Ok(Command::Run(cfg)) => cfg,
        Err(e) =>
        {
            eprintln!("ft_kalman: {}\n\n{}", e, usage());
            return ExitCode::from(2);
        }
    };

    let mut recorder = Recorder::new(100);
    recorder.set_enabled(cfg.visual.is_some());
    let mut stats = Stats { count: 0, total: Duration::ZERO, worst: Duration::ZERO, delta_count: 0, delta_sum: 0.0, delta_max: 0.0 };
    let result = run(&cfg, &mut recorder, &mut stats);
    let mut code: u8 = 0;

    if let Err(e) = &result
    {
        eprintln!("ft_kalman: {}", e);
        code = 1;
    }
    if cfg.meanspeed && stats.count > 0
    {
        let mean = stats.total.as_secs_f64() * 1000.0 / stats.count as f64;
        println!("Mean response time: {:.6} ms over {} estimates (worst: {:.6} ms)", mean, stats.count, stats.worst.as_secs_f64() * 1000.0);
    }
    if cfg.delta && stats.delta_count > 0
    {
        println!("Delta vs true position: mean {:.4} m, max {:.4} m over {} estimates", stats.delta_sum / stats.delta_count as f64, stats.delta_max, stats.delta_count);
    }
    else if cfg.delta
    {
        println!("Delta: no true position received (start imu-sensor-stream with --debug)");
    }
    if let Some(path) = &cfg.visual
    {
        match recorder.write(path)
        {
            Ok(()) => println!("Trajectory viewer written to {} ({} points)", path, recorder.len()),
            Err(e) =>
            {
                eprintln!("ft_kalman: {}", e);
                code = 1;
            }
        }
    }
    ExitCode::from(code)
}
