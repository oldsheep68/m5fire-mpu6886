//! SPI loopback test
//!
//! Folowing pins are used:
//! SCLK    GPIO19
//! MISO    GPIO25 (D)
//! MOSI    GPIO23 (SDO)
//! CS      GPIO22
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! This example transfers data via SPI.
//! Connect MISO and MOSI pins to see the outgoing data is read as incoming
//! data.
//! 
//! =============================================================
//! Versuch mit einfachstem kalman filter auf den einzelnen x und z acceleration werten
//! 

#![no_std]
#![no_main]

// use std::{thread, time::Duration};

use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle, StrokeAlignment},
    text::{Alignment, Text},
};
use hal::{
    clock::ClockControl,
    gpio::IO,
    gpio::{Event, Input, Pin, PullDown, PullUp},
    peripherals::Peripherals,
    prelude::*,
    spi::{Spi, SpiMode},
    i2c,
    // systimer::SystemTimer,
    timer::TimerGroup,
    Delay,
    Rtc,
};
use esp_backtrace as _;
use esp_println::println;
// use hx711_spi::Hx711;
use ili9341::DisplayError;
use nb::block;
// use xtensa_lx_rt::entry;

use mpu6886::{*, device::AccelRange};
pub use mpu6886::{
    config::{AccelBw, }, //GyroBw
    //error::Error,
};
use nalgebra::{Vector3, Vector2};
use libm::*;
use biquad::*;
use lexical_core::BUFFER_SIZE;


fn increment(current: i8) -> i8 {
    current.wrapping_add(1)
}

fn draw_sample<T>(lcd: &mut T)
where
    T: DrawTarget<Color = Rgb565, Error = DisplayError>,
{
    // Create a border style
    let border_stroke = PrimitiveStyleBuilder::new()
        .stroke_color(embedded_graphics::pixelcolor::Rgb565::BLUE)
        .stroke_width(8)
        .stroke_alignment(StrokeAlignment::Inside)
        .build();

    // Draw border around the screen
    let _ = lcd.bounding_box().into_styled(border_stroke).draw(lcd);

    // Create text style
    let character_style =
        MonoTextStyle::new(&FONT_10X20, embedded_graphics::pixelcolor::Rgb565::BLACK);
    let text = "Rotation Angle in °";

    // Draw text
    let textdrawable = Text::with_alignment(
        text,
        lcd.bounding_box().center() + Point::new(0, 15),
        character_style,
        Alignment::Center,
    );
    let _ = textdrawable.draw(lcd);
}

fn draw_single_btn_status<T>(lcd: &mut T, label: &str, status: bool, point: Point)
where
    T: DrawTarget<Color = Rgb565>,
{
    let character_style_normal = MonoTextStyle::new(
        &FONT_10X20,
        embedded_graphics::pixelcolor::Rgb565::new(96, 96, 96),
    );
    let character_style_pressed =
        MonoTextStyle::new(&FONT_10X20, embedded_graphics::pixelcolor::Rgb565::YELLOW);

    let size = Size::new(64, 24);
    let topleft = point;
    let center = topleft + (size / 2) + Point::new(0, 4);

    let chstyle: &MonoTextStyle<Rgb565> = if status {
        &character_style_pressed
    } else {
        &character_style_normal
    };
    let bgcolor = if status { Rgb565::RED } else { Rgb565::BLACK };

    let _ = lcd.fill_solid(&mut Rectangle::new(topleft, size), bgcolor);

    let textdrawable = Text::with_alignment(label, center, *chstyle, Alignment::Center);
    let _ = textdrawable.draw(lcd);
}

fn draw_btn_status<T>(lcd: &mut T, btn_a: bool, btn_b: bool, btn_c: bool)
where
    T: DrawTarget<Color = Rgb565>,
{
    let pos2 = Point::new(320 / 2 - 32, 240 - 32);
    let pos1 = pos2 - Point::new(64 + 32, 0);
    let pos3 = pos2 + Point::new(64 + 32, 0);

    draw_single_btn_status(lcd, "Btn A", btn_a, pos1);
    draw_single_btn_status(lcd, "Btn B", btn_b, pos2);
    draw_single_btn_status(lcd, "Btn C", btn_c, pos3);
}

fn draw_angle<T>(lcd: &mut T, angle: &str)
where
    T: DrawTarget<Color = Rgb565>,
{
    let size = Size::new(128, 24);
    let topleft = Point::new(320 / 2 - 60, 240/2 +30);
    // let character_style = MonoTextStyle::new(
    //     &FONT_10X20,
    //     embedded_graphics::pixelcolor::Rgb565::new(96, 96, 96),
    // );
    // Create text style
    let character_style =
        MonoTextStyle::new(&FONT_10X20, embedded_graphics::pixelcolor::Rgb565::BLACK);

    let _ = lcd.fill_solid(&mut Rectangle::new(topleft, size), embedded_graphics::pixelcolor::Rgb565::WHITE);
    
    let textdrawable = Text::with_alignment(
        angle,
        lcd.bounding_box().center() + Point::new(0, 45),
        character_style,
        Alignment::Center,
    );
    let _ = textdrawable.draw(lcd);
}

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the watchdog timers. For the ESP32-C3, this includes the Super WDT,
    // the RTC WDT, and the TIMG WDTs.
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt = timer_group0.wdt;

    wdt.disable();
    rtc.rwdt.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    // let sclk = io.pins.gpio19;
    // let miso = io.pins.gpio25;
    // let mosi = io.pins.gpio23;
    // let cs = io.pins.gpio22;

    let mut pin_btn_a = io.pins.gpio39.into_floating_input();
    let mut pin_btn_b = io.pins.gpio38.into_floating_input();
    let mut pin_btn_c = io.pins.gpio37.into_floating_input();
    // let mut button = io.pins.gpio0.into_pull_down_input();

    let mut pin_lcd_blk = io.pins.gpio32.into_push_pull_output();
    pin_lcd_blk.set_high().unwrap();
    let pin_sclk = io.pins.gpio18.into_push_pull_output();
    let pin_sdo = io.pins.gpio23.into_push_pull_output();
    let pin_cs = io.pins.gpio14.into_push_pull_output();
    let pin_dc = io.pins.gpio27.into_push_pull_output();
    let mut lcd_reset_pin = io.pins.gpio33.into_push_pull_output();
    let miso = io.pins.gpio25;

    // let pin_scl = io.pins.gpio22.into_push_pull_output();
    // let pin_sda = io.pins.gpio21

    let mut delay = Delay::new(&clocks);
    // let mut delay2 = Delay::new(&clocks);

    println!("Issue LCD Reset by GPIO pin");
    lcd_reset_pin.set_low().unwrap();
    delay.delay_ms(100u32);
    lcd_reset_pin.set_high().unwrap();
    delay.delay_ms(2000u32);

    let i2c = i2c::I2C::new(
        peripherals.I2C0,
        io.pins.gpio21, // sda
        io.pins.gpio22, // scl
        400u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let bus = shared_bus::BusManagerSimple::new(i2c);

    let mut mpu = Mpu6886::new(bus.acquire_i2c());
    
    let _ret = mpu.init(&mut delay);
    let _ret = mpu.set_temp_enabled(true);
    // get roll and pitch estimate
    let acc = mpu.get_acc_angles();
        println!("r/p: {:?}", acc);
    // get sensor temp
    let temp = mpu.get_temp();
    println!("temp: {:?}c", temp);
    // get gyro data, scaled with sensitivity 
    let gyro = mpu.get_gyro();
    println!("gyro: {:?}", gyro);
    // get accelerometer data, scaled with sensitivity
    let acc = mpu.get_acc();
    println!("acc: {:?}", acc);

    // biquad filter initialization
    // Cutoff and sampling frequencies
    let f100 = 500.hz();
    let fs = 4.khz();

    let f10 = 500.hz();


    // Create coefficients for the biquads
    let coeffs100 = Coefficients::<f32>::from_params(Type::LowPass, fs, f100, Q_BUTTERWORTH_F32).unwrap();
    let coeffs10 = Coefficients::<f32>::from_params(Type::LowPass, fs, f10, Q_BUTTERWORTH_F32).unwrap();

    // Create two different biquads
    let mut biquad1_x = DirectForm1::<f32>::new(coeffs100);
    let mut biquad1_y = DirectForm1::<f32>::new(coeffs100);
    let mut biquad2_x = DirectForm1::<f32>::new(coeffs100);
    let mut biquad2_y = DirectForm1::<f32>::new(coeffs100);
    

    println!("SPI Master");
    let mut lcd_spi_master = Spi::new(
        peripherals.SPI2,
        pin_sclk,
        pin_sdo,
        miso,
        pin_cs,
        10u32.MHz(),
        SpiMode::Mode0,
        &mut system.peripheral_clock_control,
        &clocks,
    );

    println!("SPI Display interface");

    let spidisplayinterface = display_interface_spi::SPIInterfaceNoCS::new(lcd_spi_master, pin_dc);

    println!("ILI9341");

    let mut lcd = ili9341::Ili9341::new(
        spidisplayinterface,
        lcd_reset_pin,
        &mut delay,
        ili9341::Orientation::Landscape,
        ili9341::DisplaySize240x320,
    )
    .expect("Failed to initialize LCD ILI9341.");

    println!("ILI9341 display: {}x{}", lcd.width(), lcd.height());



    println!("Custom configuration");
    lcd.command(ili9341::Command::DisplayInvertionOn, &[])
        .expect("Failed to issue Display Invertion ON command");
    lcd.command(ili9341::Command::MemoryAccessControl, &[0x00 | 0x08])
        .expect("Failed to issue MemoryAccessControl command");
    let _ = lcd.fill_solid(
        &mut Rectangle::new(Point::new(0, 0), Size::new(320, 240)),
        embedded_graphics::pixelcolor::Rgb565::new(0, 255, 255),
    );

    draw_sample(&mut lcd);
    draw_btn_status(&mut lcd, false, false, false);

    let mut counter: i8 = 0;

    let mut prev_btn_a = false;
    let mut prev_btn_b = false;
    let mut prev_btn_c = false;

    let mut btnstatusupdate_timer_sec: f32 = 0.0f32;
    let mut serialout_timer_sec: f32 = 0.0f32;

    // angle calculatoin from inclinatoin initialisation
    const K:f32 = 180.0/PI;
    const AVERAGE_NUMBER: usize = 200;

    let mut count: usize = 0;
    let mut xy_angle_min: f32 = 180.0;
    let mut xy_angle_max: f32 = -180.0;
    let mut x_angle_min: f32 = 180.0;
    let mut x_angle_max: f32 = -180.0;
    let mut y_angle_min: f32 = 91.0;
    let mut y_angle_max: f32 = -91.0;
    // let mut x_averagef = 0f32;
    // let mut y_averagef = 0f32;
    let mut angle360: f32 = 0f32;

    let mut accel_data_arry: [Vector3<f32>; AVERAGE_NUMBER] = [Vector3::<f32>::new(0f32,0f32,0f32) ; AVERAGE_NUMBER];
    let mut last_w_position: usize = 0;
    let mut initi_done: bool = false;

    let _ret = mpu.set_accel_range(AccelRange::G2);
    
    //let _ret = mpu.set_accel_bw(AccelBw::Hz1046);
    let mut buffer = [b'0'; BUFFER_SIZE];

    let mut display_cnt = 0;

    //let mut mpu_temp:f32 = 0.0;

    // =================================================
    // kalman filter variables
    let measurement_error:f32 = 0.002;  // 2mg as of datasheet at 100Hz (may needt to be incread)
    let mut estimate_error_x:f32 = 0.001; // 1mg initial will be recalculated at every round
    let mut kalman_gain_x:f32 = estimate_error_x / (estimate_error_x + measurement_error);
    let mut estimate_error_y:f32 = 0.001; // 1mg initial will be recalculated at every round
    let mut kalman_gain_y:f32 = estimate_error_y / (estimate_error_y + measurement_error);

    let acc_initial = mpu.get_acc().unwrap();
    let mut estimate_x = acc_initial.x as f32;
    let mut estimate_y = acc_initial.y as f32;

    // kalman on angle directly
    let mut mess_x = acc_initial.x as f32;
    let mut mess_y = acc_initial.y as f32;
    let phi_measurement_error = 0.6;
    let mut phi_estimate_error = 0.5;
    let mut phi_kalman_gain = phi_estimate_error /(phi_estimate_error + phi_measurement_error);
    let mut phi_estimate: f32 = K*atan2f(mess_y/16_384.0, mess_x/16_384.0);


    loop {
        let acc = mpu.get_acc().unwrap();
        accel_data_arry[last_w_position] = acc;
        last_w_position += 1;
        if last_w_position == AVERAGE_NUMBER {
            last_w_position = 0;
        }

        count = count + 1;
        if count == AVERAGE_NUMBER || initi_done {
                initi_done = true;

                
                // if display_cnt == 0 {
                //     mpu_temp = mpu.get_temp().unwrap();
                // }
                // // count = 0;
                // //println!("arry accel data: {:?}",accel_data_arry);
                // if count % 1000 == 0 {
                //     xy_angle_min = 180.0;
                //     xy_angle_max = -180.0;
                // }

                // let mut x_average = 0f64;
                // let mut y_average = 0f64;
                // //let mut z_average = 0f32;
                // for data in accel_data_arry {
                //     x_average += data.x as f64;
                //     y_average += data.y as f64;
                //     // z_average += data.z;
                // }
                // let x_averagef = x_average / AVERAGE_NUMBER as f64;
                // let y_averagef = y_average / AVERAGE_NUMBER as f64;

                
                // // let z_average = z_average as f32 / AVERAGE_NUMBER as f32;
                // let filtered_x1 = biquad1_x.run(x_averagef as f32);
                // let filtered_y1 = biquad1_y.run(y_averagef as f32);

                // let filtered_x = biquad2_x.run(filtered_x1 as f32);
                // let filtered_y = biquad2_y.run(filtered_y1 as f32);

                // if fabsf(x_averagef as f32 - filtered_x) < 0.3 {
                //     biquad1_x.update_coefficients(coeffs10);
                //     biquad2_x.update_coefficients(coeffs10);
                //     // filtered_y1.update_coefficients(coeffs10);
                //     // filtered_y2.update_coefficients(coeffs10);
                // } else {
                //     biquad1_x.update_coefficients(coeffs100);
                //     biquad2_x.update_coefficients(coeffs100);
                // }
                // if fabsf(y_averagef as f32 - filtered_y) < 0.3 {
                //     biquad1_y.update_coefficients(coeffs10);
                //     biquad2_y.update_coefficients(coeffs10);
                //     // filtered_y1.update_coefficients(coeffs10);
                //     // filtered_y2.update_coefficients(coeffs10);
                // } else {
                //     biquad1_y.update_coefficients(coeffs100);
                //     biquad2_y.update_coefficients(coeffs100);
                // }
                // kalman filter implementation
                let accx = acc.x as f32;
                let accy = acc.y as f32;
                // estimate_error corresponds to last estimate_error
                kalman_gain_x = estimate_error_x / (estimate_error_x + measurement_error);
                kalman_gain_y = estimate_error_y / (estimate_error_y + measurement_error);

                // new estiamtes (output value of kalman fiter)
                if fabsf(K*atan2f(estimate_y/16_384.0, estimate_x/16_384.0)
                        - K*atan2f(accy/16_384.0, accx/16_384.0)) > 1.0 {
                    estimate_x = accx;
                    estimate_y = accy;
                }
                // if fabsf(accy - estimate_y) > 0.1 {
                    
                // }
                estimate_x = estimate_x + kalman_gain_x * (accx - estimate_x);
                estimate_y = estimate_y + kalman_gain_y * (accy - estimate_y);

                // calculate estimate errors with new measurement values for next round
                estimate_error_x = (1.0 - kalman_gain_x) * estimate_error_x;
                estimate_error_y = (1.0 - kalman_gain_y) * estimate_error_y;

                // kalman phi prediction
                let phi_measurement = K*atan2f(accy/16_384.0, accx/16_384.0);
                phi_kalman_gain = phi_estimate_error / (phi_estimate_error + phi_measurement_error);
                phi_estimate = phi_estimate + phi_kalman_gain * (phi_measurement - phi_estimate);
                phi_estimate_error = (1.0 - phi_kalman_gain) * phi_estimate_error;
                if fabsf(phi_estimate - phi_measurement) > 0.5 {
                    phi_estimate_error = phi_estimate - phi_measurement;
                }
                println!("phi_estimate_error {:.5} phi_kalman_gain {:.5} phi_measurement {:.5}",
                                phi_estimate_error,
                                phi_kalman_gain, 
                                phi_measurement, 

                            );


                //let xy_avarage_a = K*atanf(x_averagef/y_averagef); // scale: 16_384.0
                //let xy_avarage_a = K*atan2f(y_averagef/16_384.0, x_averagef/16_384.0); // scale: 16_384.0
                let xy_avarage_a = K*atan2f(estimate_y/16_384.0, estimate_x/16_384.0);
                angle360 = xy_avarage_a;
                if xy_avarage_a > xy_angle_max {
                    xy_angle_max = xy_avarage_a;
                }

                if xy_avarage_a < xy_angle_min {
                    xy_angle_min = xy_avarage_a;
                }

                // if (x_averagef as f32) > x_angle_max {
                //     x_angle_max = x_averagef as f32;
                // }

                // if (x_averagef as f32) < x_angle_min  {
                //     x_angle_min = x_averagef as f32;
                // }

                // if (y_averagef as f32) > y_angle_max {
                //     y_angle_max = y_averagef as f32;
                // }

                // if (y_averagef as f32) < y_angle_min {
                //     y_angle_min = y_averagef as f32;
                // }

                
                // if x_averagef > 0.0 && y_averagef > 0.0 {
                //     angle360 = xy_avarage_a;
                // } else if x_averagef < 0.0 && y_averagef < 0.0 {
                //     angle360 = xy_avarage_a - 180.0;
                // } else if x_averagef < 0.0 && y_averagef > 0.0 {
                //     angle360 = xy_avarage_a;
                // } else if x_averagef > 0.0 && y_averagef < 0.0 {
                //     angle360 = xy_avarage_a + 180.0;
                // }


              
                // println!("x_average: {:?}, y_average: {:?}, z_average: {:?}", x_average, y_average, z_average);
                // println!("x_average_g: {:?}, y_average_g: {:?}, z_average_g: {:?}", x_average/16_384.0, y_average/16_384.0, z_average/16_384.0);
                
                // println!("x_angle: {:?}, y_angle_g: {:?}, z_anglee_g: {:?}", K*asinf(x_average/16_384.0), K*asinf(y_average/16_384.0), K*asinf(z_average/16_384.0));
                println!("{:.3} x_angle: {:.3} {:.3} {:.3} {:.3} x: {:.5} {:.5} y: {:.5} {:.5}",// T: {:.2}", 
                                                            phi_estimate,
                                                            angle360, 
                                                            xy_angle_min, 
                                                            xy_angle_max, 
                                                            xy_angle_max-xy_angle_min,
                                                            accx, estimate_x,
                                                            accy, estimate_y,
                                                            // x_averagef, x_angle_max-x_angle_min,
                                                            // y_averagef, y_angle_max-y_angle_min,
                                                            // mpu_temp 
                                                        );
                
                display_cnt += 1;
                if display_cnt == 50 {                                        
                    let angle_slice = lexical_core::write(phi_estimate, &mut buffer);
                    let mut point_pos: usize = 0;
                    for i in angle_slice.iter() {
                        if  *i == b"."[0] {
                            point_pos = point_pos + 3;
                            break;
                        } else {
                            point_pos += 1;
                        }

                    } 
                    let angle_str = core::str::from_utf8(&angle_slice[..point_pos]).unwrap();
                    draw_angle(&mut lcd, angle_str);
                    display_cnt = 0;
                }
        }
        


        // let now = SystemTimer::now();
        let delta_sec: f32 = 0.0;
        let btn_a = pin_btn_a.is_low().unwrap();
        let btn_b = pin_btn_b.is_low().unwrap();
        let btn_c = pin_btn_c.is_low().unwrap();

        if btn_a {
            println!("btn_a {:?}", btn_a);
        }

        btnstatusupdate_timer_sec += delta_sec;

        if btnstatusupdate_timer_sec > 0.025f32 {
            btnstatusupdate_timer_sec = 0.0f32;

            if btn_a != prev_btn_a || btn_b != prev_btn_b || btn_c != prev_btn_c {
                draw_btn_status(&mut lcd, btn_a, btn_b, btn_c);
                prev_btn_a = btn_a;
                prev_btn_b = btn_b;
                prev_btn_c = btn_c;
            }
        }

        serialout_timer_sec += delta_sec;

        if serialout_timer_sec > 1.0 {
            serialout_timer_sec = 0.0f32;

            if btn_a {
                counter = counter.wrapping_add(10);
            }
            if btn_c {
                counter = counter.wrapping_sub(16);
            }

            if !btn_a {
                // BtnA not pressed.
                println!("Hello world counter={}", counter);
            } else {
                // BtnA pressed.
                println!("BtnA Pressed !! counter={}", counter);
            }

            counter = increment(counter);

            delay.delay_us(200u32);
            //delay.delay_ms(10u32);
        }
    }
}
