use super::*;

pub type Generated = (World, Option<Box<dyn Fn(&World, &mut RaylibDrawHandle)>>);
pub type Generator = fn() -> Generated;

macro_rules! make_scenarios {
    ($(fn $name:ident() $body:block)*) => {
        pub fn scenarios() -> Vec<(&'static str, Generator)> {
            vec![$((stringify!($name), $name as Generator)),*]
        }
        $(fn $name() -> Generated $body)*
    }
}

make_scenarios! {
    fn pendulum() {
        let wind = WindConfig {
            dir: Vector2::new(1.0, 0.0),
            speed: 250.0,
            viscosity: 0.2,
            low: 300.0,
            high: 60.0,
        };
        let config = WorldConfig {
            rod_stiffness: 10_000.0,
            angle_stiffness: 200.0,
            general_damping: 0.02,
            rod_damping: 1.0,
            wind: vec![wind],
            ..Default::default()
        };

        let mut world = World::from_config(config);

        let origin = Vector2::new(320.0, 200.0);
        let n = 50;

        let mut point = world.add_joint(origin);
        world.fix(point);

        let distance = 5.0;

        for i in 1..=n {
            let new_point = world.add_joint(origin + Vector2::new(i as Float * distance, 0.0));
            world.add_rod([point, new_point], 1.0);
            point = new_point;
        }

        for i in 1..n {
            world.keep_angle([i-1, i, i+1]);
        }

        world.add_bounds(Bounds {
            min: Vector2::new(10.0, 10.0),
            max: Vector2::new(630.0, 470.0),
        });

        (world, None)
    }
    fn elastic_rod() {
        let n = 20;
        let config = WorldConfig {
            rod_stiffness: 300.0,
            rod_damping: 1.0,
            general_damping: 0.03,
            angle_stiffness: 5000.0,
            ..Default::default()
        };

        let mut world = World::from_config(config);
        let origin = Vector2::new(40.0, 440.0);
        let distance = 20.0;

        for i in 0..=n {
            world.add_joint(origin + Vector2::new(distance * i as Float, 0.0).rotate(-1.2));
        }
        for i in 0..n {
            world.add_rod([i, i+1], 1.0);
        }
        for i in 1..n {
            world.keep_angle([i-1, i, i+1]);
        }
        for i in 0..2 {
            world.fix(i);
        }

        (world, None)
    }

    fn tree() {
        let wind = WindConfig {
            dir: Vector2::new(1.0, 0.0),
            speed: 50.0,
            viscosity: 0.01,
            low: 300.0,
            high: 60.0,
        };
        let other_wind = WindConfig {
            low: 567.0,
            speed: 93.0,
            ..wind
        };
        let config = WorldConfig {
            angle_stiffness: 500_000.0,
            rod_stiffness: 10_000.0,
            rod_damping: 50.0,
            general_damping: 0.002,
            wind: vec![wind, other_wind],
            time_scale: Some(1.5),
            ..Default::default()
        };
        let mut world = World::from_config(config);

        let root = world.add_joint(Vector2::new(320.0, 479.0));
        let origin = world.add_joint(Vector2::new(320.0, 478.0));
        world.fix(root);
        world.fix(origin);

        world.add_rod([root, origin], 1.0);

        fn rand_float(from: Float, to: Float) -> Float {
            rand::random::<Float>() * (to - from) + from
        }
        fn len() -> Float {
            rand_float(0.7, 0.9)
        }
        fn ang() -> Float {
            rand_float(0.0, 0.8)
        }

        fn col_gen(col: &str) -> Color {
            let mut col = Color::from_hex(col).unwrap();
            let rnd = 15;
            if col.r > rnd && col.r < 255-rnd {
                col.r -= rnd;
                col.r += rand::random::<u8>() % (2*rnd);
            }
            if col.g > rnd && col.g < 255-rnd {
                col.g -= rnd;
                col.g += rand::random::<u8>() % (2*rnd);
            }
            if col.b > rnd && col.b < 255-rnd {
                col.b -= rnd;
                col.b += rand::random::<u8>() % (2*rnd);
            }
            col
        }

        struct Square {
            rod: RodId,
            pos: Vector2,
            size: Float,
            col: Color
        }

        fn generate_tree(
            depth: usize,
            world: &mut World,
            mut prev: JointId,
            mut knot: JointId,
            mut dir: Vector2,
            squares: &mut Vec<Square>,
        ) {
            let weight = dir.length() * 0.01;
            dir = dir * len();

            let mut early_branch = None;
            if rand_float(0.0, 1.0) < 0.6 && depth > 0 {
                early_branch = Some(rand::random::<usize>() % 3);
            }

            let ldir = dir.rotate(-ang());
            let rdir = dir.rotate(ang());

            let mut add_branch = |world: &mut World, from: JointId, to: JointId, squares: &mut Vec<Square>| {
                let rod = world.add_rod([from, to], weight);

                let steps = 60;
                for i in 0..steps {
                    let i = i as Float / steps as Float;
                    let pos = Vector2::new(i, rand_float(-0.3, 0.3));
                    let mut col = col_gen("885E48");

                    if pos.y < -0.1 {
                        let sub = 1.0 - 3.0 * (-0.1 - pos.y);
                        col.r = ((col.r as Float) * sub) as u8;
                        col.g = ((col.g as Float) * sub) as u8;
                        col.b = ((col.b as Float) * sub) as u8;
                    }

                    squares.push(Square {
                        rod, pos, col, size: weight * 20.0
                    });
                }

                if weight < 0.15 {
                    for _ in 0..100 {
                        let side = 4.0;
                        squares.push(Square {
                            rod,
                            pos: Vector2::new(rand_float(0.0, 1.0), rand_float(-side, side)),
                            col: col_gen("e22a00").fade(rand::random()),
                            size: rand_float(3.0, 10.0),
                        });
                    }
                }

                rod
            };

            let mut make_child = |world: &mut World, prev, knot: JointId, dir, weight, squares: &mut Vec<_>| {
                let child = world.add_joint(world.joints[knot].position + dir);
                world.keep_angle([prev, knot, child]);
                add_branch(world, knot, child, squares);
                generate_tree(depth - 1, world, knot, child, dir, squares);
            };

            let bend = rand_float(-0.1, 0.1);
            for i in 0..4 {

                if Some(i) == early_branch {
                    let dir = dir.rotate(rand_float(-0.8, 0.8));
                    make_child(world, prev, knot, dir, weight, squares);
                }

                let new = world.add_joint(world.joints[knot].position + dir);
                add_branch(world, knot, new, squares);
                world.keep_angle([prev, knot, new]);
                prev = knot;
                knot = new;
                dir = dir.rotate(bend);
            }

            if depth == 0 {
                return;
            }

            make_child(world, prev, knot, ldir, weight, squares);
            make_child(world, prev, knot, rdir, weight, squares);
        }

        let mut squares = Vec::new();
        generate_tree(4, &mut world, root, origin, Vector2::new(0.0, -30.0), &mut squares);

        world.add_bounds(Bounds {
            min: Vector2::new(10.0, 10.0),
            max: Vector2::new(630.0, 480.0),
        });

        (world, Some(Box::new(move |world, draw|{
            draw.clear_background(Color::from_hex("A1D9E8").unwrap());

            for square in squares.iter() {
                let a = world.joints[world.rods[square.rod].ends[0]].position;
                let b = world.joints[world.rods[square.rod].ends[1]].position;
                let dir = b-a;
                let pos = a + dir * square.pos.x + dir.rotate(-0.5 * PI) * square.pos.y - square.size * 0.5;

                draw.draw_rectangle_v(pos, Vector2::one() * square.size, square.col);
            }
        })))
    }


    fn stable_circle() {
        (circle_gen(300, 2, 1), None)
    }
    fn soft_circle() {
        (circle_gen(1000, 2, 1), None)
    }
    fn weird_circle() {
        (circle_gen(1000, 4, 2), None)
    }
}

fn circle_gen(circle_len: usize, off_1: usize, off_2: usize) -> World {
    let config = WorldConfig {
        rod_stiffness: 300.0,
        rod_damping: 100.0,
        general_damping: 0.0,
        angle_stiffness: 80_000.0,
        ..Default::default()
    };

    let mut world = World::from_config(config);

    for i in 0..circle_len {
        let angle = (i as f32) / (circle_len as f32) * 2.0 * PI;
        let dir = Vector2::new(0.0, 150.0).rotate(angle);
        let pos = Vector2::new(300.0, 200.0) + dir;
        world.add_joint(pos);
    }

    for i in 0..circle_len {
        world.add_rod([i, (i + 1) % circle_len], 1.0);
        world.keep_angle([i, (i + off_2) % circle_len, (i + off_1) % circle_len]);
    }

    world.add_bounds(Bounds {
        min: Vector2::new(10.0, 10.0),
        max: Vector2::new(630.0, 470.0),
    });

    world
}
