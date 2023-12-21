#[derive(Debug)]
pub struct PIDController {
    kp: f64,
    ki: f64,
    kd: f64,
    dt: f64,
    output_multiplier: f64,
    integral: f64,
    previous_error: f64,
}

impl PIDController {
    fn new(kp: f64, ki: f64, kd: f64, dt: f64, output_multiplier: f64) -> PIDController {
        PIDController {
            kp,
            ki,
            kd,
            dt,
            output_multiplier,
            integral: 0.0,
            previous_error: 0.0,
        }
    }

    pub fn update(&mut self, setpoint: f64, actual: f64) -> f64 {
        let error = setpoint - actual;
        self.integral += error * self.dt;
        let derivative = (error - self.previous_error) / self.dt;
        let output = self.kp * error + self.ki * self.integral + self.kd * derivative;
        self.previous_error = error;
        self.output_multiplier * output
    }
}

struct RControl {
    throttle: f64,
    acceleration: f64,
    velocity: f64,
    location: f64,
}

impl RControl {
    fn new() -> RControl {
        RControl {
            throttle: 0.0,
            acceleration: 0.0,
            velocity: 0.0,
            location: 0.0,
        }
    }

    fn update(&mut self, throttle: f64) -> f64 {
        self.throttle = throttle;
        self.acceleration += self.throttle;
        self.velocity += self.acceleration;
        self.location += self.velocity;

        self.location
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let throttle_kp: f64 = 0.01;
    let throttle_ki: f64 = 0.0001;
    let throttle_kd: f64 = 0.6;
    let throttle_dt: f64 = 1.0;
    let throttle_output_multiplier: f64 = 1.;

    let mut throttle_controller = PIDController::new(
        throttle_kp,
        throttle_ki,
        throttle_kd,
        throttle_dt,
        throttle_output_multiplier,
    );

    let mut rcontrol = RControl::new();
    let mut location = 0.;
    let setpoint = 100.;
    let mut break_count = 0;

    for i in 0..10000 {
        let throttle = throttle_controller.update(setpoint, location);
        location = rcontrol.update(throttle);
        println!("{i} -> throttle: {throttle}; location: {location}");
        std::thread::sleep(std::time::Duration::from_millis(100));
        if location * 1.1 < setpoint && location * 0.9 > setpoint {
            break_count += 1;
            if break_count > 10 {
                println!("Reached goal");
                return Ok(());
            }
        } else {
            break_count = 0;
        }
        if location.abs() > setpoint * 100. {
            return Err("Overshot goal".into());
        }
    }

    Err("Ran out of battery".into())
}
