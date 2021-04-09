mod physics;
mod prelude;
mod scenario;
use physics::*;
use prelude::*;

fn main() {
    let scenarios = scenario::scenarios();
    let names = format!("{:?}", scenarios.iter().map(|(a, _)| a).collect::<Vec<_>>());

    let name = match std::env::args().nth(1) {
        None => {
            println!("Need name of demo. Possible demos are {}.", names);
            println!("To pass a command line argument, use `cargo run -- <ARG>`");
            return;
        }
        Some(n) => n,
    };

    for (title, scenario) in scenarios.into_iter() {
        if title.to_lowercase() != name.to_lowercase() {
            continue;
        }
        run(title, scenario);
        return;
    }
    println!(
        "No demo with name \"{}\". Possible demos are {}.",
        name, names
    );
}

fn run(title: &str, arm_fn: fn() -> World) {
    let (mut rl, thread) = raylib::init().size(640, 480).title(title).build();

    let mut world = arm_fn();

    rl.set_target_fps(240);

    let mut running = false;

    loop {
        if rl.window_should_close() {
            return;
        }

        let key = rl.get_key_pressed();
        let mut d = rl.begin_drawing(&thread);

        if let Some(key) = key {
            use KeyboardKey::*;
            match key {
                KEY_SPACE => running = !running,
                KEY_R => world = arm_fn(),
                _ => {}
            }
        }

        if d.is_key_down(KeyboardKey::KEY_C) {
            d.clear_background(Color::BLACK);
        }

        d.draw_rectangle(
            0,
            0,
            640,
            480,
            Color {
                a: 10,
                ..Color::BLACK
            },
        );

        world.visualize(&mut d);

        let running_text;
        if running {
            for _ in 0..100 {
                let dt = (0.2 * d.get_frame_time()).min(0.001);
                world.update(dt);
            }
            running_text = "running";
        } else {
            running_text = "paused";
        }

        d.draw_fps(15, 15);
        d.draw_text(running_text, 15, 30, 22, Color::RED);
    }
}
