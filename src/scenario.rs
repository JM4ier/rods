use super::*;

pub fn scenarios() -> Vec<(&'static str, Armature)> {
    vec![
        ("pendulum", pendulum()),
        ("elastic-rod", elastic_rod()),
        ("stable-circle", stable_circle()),
        ("weird-circle", weird_circle()),
        ("soft-circle", soft_circle()),
    ]
}

fn pendulum() -> Armature {
    let config = Config {
        stiffness: 100.0,
        angle_stiffness: 0.0,
        damping: 1.0,
    };
    let mut arm = Armature::from_config(config);

    let origin = Vector2::new(320.0, 80.0);
    let n = 20;

    let mut point = arm.add_joint(origin);
    arm.add_constraint(FixPoint {
        point,
        position: origin,
    });

    let distance = 14.0;

    for i in 1..=n {
        let new_point = arm.add_joint(origin + Vector2::new(i as Float * distance, 0.0));
        arm.add_constraint(DistanceConstraint {
            from: point,
            to: new_point,
            distance,
            weight: 0.1,
        });
        point = new_point;
    }

    arm.add_constraint(BoxConstraint {
        min: Vector2::new(10.0, 10.0),
        max: Vector2::new(630.0, 470.0),
    });

    arm
}

fn elastic_rod() -> Armature {
    let n = 20;
    let config = Config {
        stiffness: 300.0,
        damping: 0.03,
        angle_stiffness: 5000.0,
    };
    let mut arm = Armature::from_config(config);
    let origin = Vector2::new(40.0, 440.0);
    let distance = 20.0;

    for i in 0..=n {
        arm.add_joint(origin + Vector2::new(distance * i as Float, 0.0).rotate(-1.2));
    }
    for i in 0..n {
        arm.add_constraint(DistanceConstraint {
            from: i,
            to: i + 1,
            distance,
            weight: 1.0,
        });
    }
    for i in 1..n {
        arm.add_constraint(AngularConstraint {
            a: i - 1,
            pivot: i,
            b: i + 1,
            target_angle: PI,
        });
    }
    for i in 0..2 {
        arm.fixate(i);
    }

    arm.add_constraint(Damping);

    arm
}

fn stable_circle() -> Armature {
    circle_gen(300, 2, 1)
}

fn soft_circle() -> Armature {
    circle_gen(1000, 2, 1)
}

fn weird_circle() -> Armature {
    circle_gen(1000, 4, 2)
}

fn circle_gen(circle_len: usize, off_1: usize, off_2: usize) -> Armature {
    let config = Config {
        stiffness: 300.0,
        damping: 100.0,
        angle_stiffness: 80_000.0,
    };

    let mut arm = Armature::from_config(config);

    for i in 0..circle_len {
        let angle = (i as f32) / (circle_len as f32) * 2.0 * PI;
        let dir = Vector2::new(0.0, 150.0).rotate(angle);
        let pos = Vector2::new(300.0, 200.0) + dir;

        let dist = DistanceConstraint {
            from: i,
            to: (i + 1) % circle_len,
            distance: 1000.0 / circle_len as Float,
            weight: 1.0,
        };

        let ang = AngularConstraint {
            a: i,
            b: (i + off_1) % circle_len,
            pivot: (i + off_2) % circle_len,
            target_angle: 1.0 * PI,
        };

        arm.add_constraint(dist);
        arm.add_constraint(ang);
        arm.add_joint(pos);
    }

    arm.add_constraint(BoxConstraint {
        min: Vector2::new(10.0, 10.0),
        max: Vector2::new(630.0, 470.0),
    });

    arm
}