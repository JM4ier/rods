mod physics;
mod prelude;
mod scenario;
use physics::*;
use prelude::*;
use scenario::*;

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

fn run(title: &str, generator: Generator) {
    let (mut rl, thread) = raylib::init().size(640, 480).title(title).build();

    let (mut world, mut gfx) = generator();

    rl.set_target_fps(240);

    let mut running = false;
    let mut draw_phys = true;
    let mut clear = true;

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
                KEY_C => clear = !clear,
                KEY_P => draw_phys = !draw_phys,
                KEY_R => {
                    let gen = generator();
                    world = gen.0;
                    gfx = gen.1;
                }
                _ => {}
            }
        }

        if clear {
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

        if let Some(gfx) = gfx.as_ref() {
            gfx(&world, &mut d);
        }
        if draw_phys {
            world.visualize(&mut d);
        }

        let running_text;
        if running {
            for _ in 0..100 {
                let dt = (0.01 * d.get_frame_time()).min(0.001);
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
