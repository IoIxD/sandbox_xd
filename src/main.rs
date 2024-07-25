use std::{sync::Arc, time::SystemTime};

use rapier2d::prelude::*;
use raylib::prelude::*;
use state::State;

pub mod state;

#[tokio::main]
async fn main() {
    let state = Arc::new(State::new());

    let s1 = state.clone();
    let s2 = state.clone();

    tokio::spawn(async move {
        let mut time = SystemTime::now();
        loop {
            if time.elapsed().unwrap().as_secs_f64() >= 1.0 / 480.0 {
                s1.step();
                time = SystemTime::now();
            }
        }
    });

    let (mut rl, thread) = raylib::init()
        .width(320)
        .height(240)
        .title("sandbox_xd")
        .resizable()
        .build();

    let mut last_x = 0.0;
    let mut last_y = 0.0;
    let mut last_width = 0;
    let mut last_height = 0;
    let wpos = rl.get_window_position();

    while !rl.window_should_close() {
        let wpos = rl.get_window_position();
        let size = (rl.get_screen_width(), rl.get_screen_height());
        if wpos.x != last_x || wpos.y != last_y || size.0 != last_width || size.1 != last_height {
            s2.resize(wpos.x, wpos.y, size.0 as f32, size.1 as f32);
            last_x = wpos.x;
            last_y = wpos.y;
            last_width = size.0;
            last_height = size.1;
        }
        if rl.is_mouse_button_down(MouseButton::MOUSE_BUTTON_LEFT) {
            let pos = rl.get_mouse_position();

            s2.insert_particle(pos.x + last_x, pos.y + last_y);
        }

        let mut d = rl.begin_drawing(&thread);
        d.clear_background(Color::WHITE);

        s2.for_each_cube(|x, y, width, height, rand| {
            d.draw_rectangle(
                x as i32 - last_x as i32,
                y as i32 - last_y as i32,
                width as i32,
                height as i32,
                Color::new(
                    (rand ^ 0xFF0000) as u8,
                    (rand ^ 0x00FF00) as u8,
                    (rand ^ 0x0000FF) as u8,
                    255,
                ),
            );
        })
    }
}
