use super::*;

macro_rules! make_scenarios {
    ($(fn $name:ident() -> World $body:block)*) => {
        pub fn scenarios() -> Vec<(&'static str, fn() -> World)> {
            let mut funs = Vec::new();
            $(
                funs.push((stringify!($name), $name as fn() -> World));
            )*
                funs
        }
        $(
            fn $name() -> World $body
        )*
    }
}

make_scenarios! {
    fn pendulum() -> World {
        let config = WorldConfig {
            rod_stiffness: 100.0,
            angle_stiffness: 0.0,
            rod_damping: 1.0,
            general_damping: 0.0,
        };

        let mut world = World::from_config(config);

        let origin = Vector2::new(320.0, 80.0);
        let n = 20;

        let mut point = world.add_joint(origin);
        world.fix(point);

        let distance = 14.0;

        for i in 1..=n {
            let new_point = world.add_joint(origin + Vector2::new(i as Float * distance, 0.0));
            world.add_rod([point, new_point], 0.1);
            point = new_point;
        }

        world.add_bounds(Bounds {
            min: Vector2::new(10.0, 10.0),
            max: Vector2::new(630.0, 470.0),
        });

        world
    }
    fn elastic_rod() -> World {
        let n = 20;
        let config = WorldConfig {
            rod_stiffness: 300.0,
            rod_damping: 1.0,
            general_damping: 0.03,
            angle_stiffness: 5000.0,
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

        world
    }

    fn trees() -> World {
        let config = WorldConfig {
            angle_stiffness: 50_000.0,
            general_damping: 0.08,
            rod_stiffness: 100.0,
            rod_damping: 50.0,
        };
        let mut world = World::from_config(config);

        let root = world.add_joint(Vector2::new(320.0, 470.0));
        let origin = world.add_joint(Vector2::new(320.0, 400.0));
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

        fn generate_tree(
            depth: usize,
            world: &mut World,
            mut prev: JointId,
            mut knot: JointId,
            mut dir: Vector2,

        ) {
            let weight = dir.length() * 0.1;
            dir = dir * len();

            let bend = rand_float(-0.1, 0.1);
            for _ in 0..3 {
                let new = world.add_joint(world.joints[knot].position + dir);
                world.add_rod([knot, new], weight);
                world.keep_angle([prev, knot, new]);
                prev = knot;
                knot = new;
                dir = dir.rotate(bend);

            }

            if depth == 0 {
                return;

            }

            let ldir = dir.rotate(-ang());
            let rdir = dir.rotate(ang());

            let lchild = world.add_joint(world.joints[knot].position + ldir);
            let rchild = world.add_joint(world.joints[knot].position + rdir);

            world.keep_angle([prev, knot, lchild]);
            world.keep_angle([prev, knot, rchild]);

            world.add_rod([knot, lchild], weight);
            world.add_rod([knot, rchild], weight);

            generate_tree(depth - 1, world, knot, lchild, ldir);
            generate_tree(depth - 1, world, knot, rchild, rdir);

        }

        generate_tree(5, &mut world, root, origin, Vector2::new(0.0, -30.0));

        world.add_bounds(Bounds {
            min: Vector2::new(10.0, 10.0),
            max: Vector2::new(630.0, 470.0),
        });

        world
    }


    fn stable_circle() -> World {
        circle_gen(300, 2, 1)
    }
    fn soft_circle() -> World {
        circle_gen(1000, 2, 1)
    }
    fn weird_circle() -> World {
        circle_gen(1000, 4, 2)
    }
}

fn circle_gen(circle_len: usize, off_1: usize, off_2: usize) -> World {
    let config = WorldConfig {
        rod_stiffness: 300.0,
        rod_damping: 100.0,
        general_damping: 0.0,
        angle_stiffness: 80_000.0,
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