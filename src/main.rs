#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]
#![allow(incomplete_features)]

use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_net::dns::DnsSocket;
use embassy_net::tcp::client::{TcpClient, TcpClientState};
//use embassy_net::tcp::TcpSocket;
use embassy_net::{Config, EthernetAddress, HardwareAddress, Stack, StackResources};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH0, PIN_23, PIN_25, PIO0, PIO1};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::spi;
use embassy_rp::spi::Spi;
use embassy_time::{Delay, Duration, Timer};
use embedded_graphics::mono_font::ascii::FONT_8X13 as font_size;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::pixelcolor::{self, BinaryColor};
use embedded_graphics::primitives::{PrimitiveStyleBuilder, Rectangle, StyledDrawable};
use embedded_graphics::text::Text;
use embedded_graphics::{
    image::Image,
    prelude::*,
    primitives::{Line, PrimitiveStyle},
};
use epd_waveshare::{color::*, epd3in7::*, prelude::*};
use heapless::{String, Vec};
use static_cell::make_static;
use {defmt_rtt as _, panic_probe as _};

use reqwless::request::RequestBuilder;

bind_interrupts!(struct WIrqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

bind_interrupts!(struct DIrqs {
    PIO1_IRQ_0 => InterruptHandler<PIO1>;
});

const WIFI_NETWORK: &'static str = core::env!("WIFI_SSID");
const WIFI_PASSWORD: &'static str = core::env!("WIFI_PASSWORD");

#[embassy_executor::task]
async fn wifi_task(
    runner: cyw43::Runner<
        'static,
        Output<'static, PIN_23>,
        PioSpi<'static, PIN_25, PIO0, 0, DMA_CH0>,
    >,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<cyw43::NetDriver<'static>>) -> ! {
    stack.run().await
}

#[derive(Format, Debug, serde::Serialize)]
pub struct Capabilities {
    pub produces: &'static [&'static str], // list of names
    pub consumes: &'static [&'static str], // list of names
}

/* this device has no sensors yet */
const PRODUCES: &[&str] = &[];

/* this device has a display and it has these regions to display values */
const CONSUMES: &[&str] = &["top", "middle", "bottom"];

#[derive(Format, Debug, serde::Serialize)]
pub struct Device {
    pub hwaddr: [u8; 6],
    pub capabilities: Capabilities,
}

#[derive(Format, Debug, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Value {
    Signed(i32),
    Unsigned(u32),
    Real(f32),
}

#[derive(Format, Debug, serde::Serialize, serde::Deserialize)]
pub struct Resource {
    pub what: String<32>,
    pub value: Value,
}

#[derive(Format, Debug, serde::Serialize, serde::Deserialize)]
struct ContextResource {
    pub context: String<32>,
    pub resource: Resource,
}

#[derive(Format, Debug, serde::Serialize, serde::Deserialize)]
pub struct DeviceResources {
    pub hwaddr: [u8; 6],
    pub resources: Resource,
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let fw = include_bytes!("/home/avlec/rp2040/embassy/cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("/home/avlec/rp2040/embassy/cyw43-firmware/43439A0_clm.bin");

    // To make flashing faster for development, you may want to flash the firmwares independently
    // at hardcoded addresses, instead of baking them into the program with `include_bytes!`:
    //     probe-rs download 43439A0.bin --format bin --chip RP2040 --base-address 0x10100000
    //     probe-rs download 43439A0_clm.bin --format bin --chip RP2040 --base-address 0x10140000
    //let fw = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 224190) };
    //let clm = unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) };

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs_w = Output::new(p.PIN_25, Level::High);

    let mut pio_w = Pio::new(p.PIO0, WIrqs);
    let spi_w = PioSpi::new(
        &mut pio_w.common,
        pio_w.sm0,
        pio_w.irq0,
        cs_w,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );

    let mut spi_d_cfg = spi::Config::default();
    spi_d_cfg.frequency = 4000000;
    let mut spi_d = Spi::new_blocking_txonly(p.SPI1, p.PIN_10, p.PIN_11, spi_d_cfg);

    let cs = Output::new(p.PIN_9, Level::Low);
    let busy = Input::new(p.PIN_13, Pull::None);
    let dc = Output::new(p.PIN_8, Level::Low);
    let rst = Output::new(p.PIN_12, Level::Low);
    let mut delay = Delay;
    let mut epd = EPD3in7::new(&mut spi_d, cs, busy, dc, rst, &mut delay, None).unwrap();

    let mut display = Display3in7::default();

    let splash_bytes = include_bytes!("../splashnewbw.bmp");
    let splash_bmp = tinybmp::Bmp::<BinaryColor>::from_slice(splash_bytes).unwrap();
    match Image::new(&splash_bmp, Point::new(0, 0)).draw(&mut display.color_converted()) {
        Ok(_) => info!("rendered splash screen"),
        Err(e) => error!("failed to render splash screen {:?}", e),
    }

    epd.clear_frame(&mut spi_d, &mut delay).unwrap();
    epd.update_and_display_frame(&mut spi_d, &display.buffer(), &mut delay)
        .unwrap();

    info!("displayed...");

    match epd.sleep(&mut spi_d, &mut delay) {
        Ok(_) => (),
        Err(e) => error!("can't sleep display {}", e),
    }

    let state = make_static!(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi_w, fw).await;
    unwrap!(spawner.spawn(wifi_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    let config = Config::dhcpv4(Default::default());

    // Generate random seed
    let seed = 0x0123_4567_89ab_cdef; // chosen by fair dice roll. guarenteed to be random.

    // Init network stack
    let stack = &*make_static!(Stack::new(
        net_device,
        config,
        make_static!(StackResources::<8>::new()),
        seed
    ));
    let client_state = &*make_static!(TcpClientState::<4, 1024, 1024>::new());

    let tcp = &*make_static!(TcpClient::new(&stack, &client_state));
    let dns = &*make_static!(DnsSocket::new(&stack));
    let client = &mut *make_static!(reqwless::client::HttpClient::new(tcp, dns));

    let dev = Device {
        hwaddr: match stack.hardware_address() {
            HardwareAddress::Ethernet(EthernetAddress(e)) => e,
        },
        capabilities: Capabilities {
            produces: PRODUCES,
            consumes: CONSUMES,
        },
    };
    info!("{:?}", dev);

    unwrap!(spawner.spawn(net_task(stack)));

    info!("network stack init'd");

    loop {
        match control.join_wpa2(WIFI_NETWORK, WIFI_PASSWORD).await {
            Ok(_) => break,
            Err(err) => {
                info!("join failed with status={}", err.status);
            }
        }
    }

    loop {
        match stack.config_v4() {
            Some(_) => break,
            None => Timer::after(Duration::from_millis(100)).await,
        }
    }

    info!("connected");

    let mut rx_buffer = [0; 4096];

    info!("httpclient sending the message");
    let url = "http://192.168.1.156:1234/v1/device";
    // register device on startup
    match client.request(reqwless::request::Method::POST, &url).await {
        Ok(request) => match serde_json_core::to_string::<Device, 128>(&dev) {
            Ok(json) => match request
                .content_type(reqwless::headers::ContentType::ApplicationJson)
                .body(json.as_bytes())
                .send(&mut rx_buffer)
                .await
            {
                Ok(response) => info!("recieved response {}", response.body().unwrap().body_buf),
                Err(_) => error!("sending request"),
            },
            Err(_) => error!("formatting the request failed..."),
        },
        // from my experiementing this happened when the server was offline
        Err(_) => error!("creating request"),
    };

    let mut loopctr = 0;

    // TODO repr these in the device structure
    // have these values being present there translate into
    // sending consumes to the server
    // and have the values get updated by the responses recieved
    // from the server
    let mut value1: Value = Value::Real(42.0);
    let mut value2: Value = Value::Real(100.0);
    let mut value3: Value = Value::Real(-32.0);

    // let style = PrimitiveStyleBuilder::new()
    //     .stroke_color(Color::Black)
    //     .stroke_width(1)
    //     .fill_color(Color::White)
    //     .build();

    let text_height = font_size.character_size.height;
    // let _ = Rectangle::new(
    //     Point {
    //         x: 0,
    //         y: 200 - text_height as i32,
    //     },
    //     Size {
    //         width: 280,
    //         height: 3 * text_height * 2,
    //     },
    // )
    // .draw_styled(&style, &mut display);

    loop {
        // TODO produce values first
        // request updates on consumed values
        match client.request(reqwless::request::Method::GET, &url).await {
            Ok(request) => match serde_json_core::to_string::<[u8; 6], 128>(&dev.hwaddr) {
                Ok(json) => match request
                    .content_type(reqwless::headers::ContentType::ApplicationJson)
                    .body(json.as_bytes())
                    .send(&mut rx_buffer)
                    .await
                {
                    Ok(response) => {
                        // TODO json to Resources
                        let length = response.content_length.unwrap_or(0);
                        match serde_json_core::from_slice::<Vec<ContextResource, 8>>(
                            &response.body().unwrap().body_buf[0..length],
                        ) {
                            Ok(t) => info!("recieved response {:?}", t),
                            Err(e) => info!("{:?}", e),
                        }
                    }
                    Err(_) => error!("sending request"),
                },
                Err(_) => error!("formatting the request failed..."),
            },
            // from my experiementing this happened when the server was offline
            Err(_) => error!("creating request"),
        };
        // TODO update display

        // let _ = display.clear(Color::White);

        let text_style = MonoTextStyleBuilder::new()
            .font(&font_size)
            .text_color(Color::Black)
            .background_color(Color::White)
            .build();

        let mut data: String<64> = String::from("Top value: ");
        let formatted_data = serde_json_core::to_string::<Value, 32>(&value1).unwrap_or_default();
        let _ = data.push_str(formatted_data.as_str());
        let _ = Text::new(data.as_str(), Point::new(10, 200), text_style).draw(&mut display);
        let mut data: String<64> = String::from("Middle value: ");
        let _ =
            data.push_str(&serde_json_core::to_string::<Value, 32>(&value2).unwrap_or_default());
        let _ = Text::new(
            data.as_str(),
            Point::new(10, 200 + (1 * text_height * 2) as i32),
            text_style,
        )
        .draw(&mut display);

        let mut data: String<64> = String::from("Bottom value: ");
        let _ =
            data.push_str(&serde_json_core::to_string::<Value, 32>(&value3).unwrap_or_default());
        let _ = Text::new(
            data.as_str(),
            Point::new(10, 200 + (2 * text_height * 2) as i32),
            text_style,
        )
        .draw(&mut display);

        match epd.wake_up(&mut spi_d, &mut delay) {
            Ok(_) => (),
            Err(e) => error!("can't wake up display {}", e),
        }

        match epd.clear_frame(&mut spi_d, &mut delay) {
            Ok(_) => (),
            Err(e) => error!("can't clear display {}", e),
        }

        match epd.update_and_display_frame(&mut spi_d, &display.buffer(), &mut delay) {
            Ok(_) => (),
            Err(e) => error!("can't update and display {}", e),
        };

        match epd.sleep(&mut spi_d, &mut delay) {
            Ok(_) => (),
            Err(e) => error!("can't sleep display {}", e),
        }

        Timer::after(Duration::from_secs(30)).await;
        delay.delay_ms(30 * 1000);
    }
}