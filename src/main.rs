mod phys;
mod prelude;
use phys::constraint::*;
use phys::*;
use prelude::*;

fn main() {
    let (mut rl, thread) = raylib::init().size(640, 480).title("Hello, World").build();

    let config = Config {
        stiffness: 300.0,
        damping: 100.0,
        angle_stiffness: 1000.0,
    };

    let mut arm = Armature::from_config(config);
    let circle_len = 10;
    let mut connections = Vec::new();

    for i in 0..circle_len {
        let angle = (i as f32) / (circle_len as f32) * 2.0 * PI;
        let dir = Vector2::new(0.0, 150.0).rotate(angle);
        let pos = Vector2::new(300.0, 200.0) + dir;

        let dist = DistanceConstraint {
            from: i,
            to: (i + 1) % circle_len,
            distance: 60.0,
            weight: 1.0,
        };

        let ang = AngularConstraint {
            a: i,
            b: (i + 2) % circle_len,
            pivot: (i + 1) % circle_len,
            target_angle: 2.0 * PI,
        };

        arm.add_constraint(dist.clone());
        arm.add_constraint(ang);
        arm.add_joint(pos);
        connections.push(dist);
    }

    let bounds = BoxConstraint {
        min: Vector2::new(10.0, 10.0),
        max: Vector2::new(630.0, 470.0),
    };
    arm.add_constraint(bounds.clone());

    rl.set_target_fps(240);

    let mut running = false;

    while !rl.window_should_close() {
        if rl.is_key_released(KeyboardKey::KEY_SPACE) {
            running = !running;
        }

        let mut d = rl.begin_drawing(&thread);

        d.clear_background(Color::WHITE);

        fn tp(pos: Vector2) -> Vector2 {
            Vector2::new(pos.x, 480.0 - pos.y)
        }

        let BoxConstraint { min, max } = bounds;
        let mm_0 = Vector2::new(min.x, max.y);
        let mm_1 = Vector2::new(max.x, min.y);
        d.draw_line_v(tp(min), tp(mm_0), Color::BLACK);
        d.draw_line_v(tp(min), tp(mm_1), Color::BLACK);
        d.draw_line_v(tp(max), tp(mm_0), Color::BLACK);
        d.draw_line_v(tp(max), tp(mm_1), Color::BLACK);

        for con in connections.iter() {
            let from = tp(arm.joints[con.from].position);
            let to = tp(arm.joints[con.to].position);
            d.draw_line_v(from, to, Color::BLACK);
        }

        for joint in arm.joints.iter() {
            let pos = tp(joint.position);
            d.draw_circle_v(pos, 3.0, Color::RED);
        }

        let running_text;
        if running {
            for _ in 0..100 {
                arm.apply_forces((0.2 * d.get_frame_time()).min(0.002));
            }
            running_text = "running";
        } else {
            running_text = "paused";
        }

        d.draw_fps(15, 15);
        d.draw_text(running_text, 15, 30, 22, Color::RED);
    }
}
