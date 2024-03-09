#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(incomplete_features)]

use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_net::dns::DnsSocket;
use embassy_net::tcp::client::{TcpClient, TcpClientState};
use embassy_net::{Config, EthernetAddress, HardwareAddress, Stack, StackResources};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH0, PIO0, PIO1};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_time::{Delay, Duration, Timer};
use embedded_graphics::mono_font::ascii::FONT_8X13 as font_size;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::{prelude::*, image::Image, text::Text};
use embedded_hal_bus::spi::ExclusiveDevice;
use epd_waveshare::{epd3in7::*, prelude::*};
use heapless::{String, Vec};
use static_cell::make_static;
use {defmt_rtt as _, panic_probe as _};

use reqwless::request::RequestBuilder;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    PIO1_IRQ_0 => InterruptHandler<PIO1>;
    ADC_IRQ_FIFO => embassy_rp::adc::InterruptHandler;
});

const WIFI_NETWORK: &'static str = core::env!("WIFI_SSID");
const WIFI_PASSWORD: &'static str = core::env!("WIFI_PASSWORD");
const URL: &'static str = core::env!("URL");

#[embassy_executor::task]
async fn wifi_task(
    runner: cyw43::Runner<
            'static,
        Output<'static>,
        PioSpi<'static, PIO0, 0, DMA_CH0>
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
const PRODUCES: &[&str] = &["temp"];

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

fn convert_to_celsius(raw_temp: u16) -> f32 {
    // According to chapter 4.9.5. Temperature Sensor in RP2040 datasheet
    let temp = 27.0 - (raw_temp as f32 * 3.3 / 4096.0 - 0.706) / 0.001721;
    let sign = if temp < 0.0 { -1.0 } else { 1.0 };
    let rounded_temp_x10: i16 = ((temp * 10.0) + 0.5 * sign) as i16;
    (rounded_temp_x10 as f32) / 10.0
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let fw = unsafe { core::slice::from_raw_parts((0x101C0000) as *const u8, 230321) };
    let clm = unsafe { core::slice::from_raw_parts((0x101C0000 + 0x38400) as *const u8, 4752) };

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs_w = Output::new(p.PIN_25, Level::High);

    let mut pio_w = Pio::new(p.PIO0, Irqs);
    let spi_w = PioSpi::new(
        &mut pio_w.common,
        pio_w.sm0,
        pio_w.irq0,
        cs_w,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );

    let mut adc = embassy_rp::adc::Adc::new(p.ADC, Irqs, embassy_rp::adc::Config::default());
    let mut ts = embassy_rp::adc::Channel::new_temp_sensor(p.ADC_TEMP_SENSOR);

    let mut spi_d_cfg = embassy_rp::spi::Config::default();
    spi_d_cfg.frequency = 4000000;
    let spi_d = embassy_rp::spi::Spi::new_blocking_txonly(p.SPI1, p.PIN_10, p.PIN_11, spi_d_cfg);
    let delay = Delay;
    let cs_d = Output::new(p.PIN_9, Level::Low);
    let mut spidev_d = ExclusiveDevice::new(spi_d, cs_d, delay);

    let busy_d = Input::new(p.PIN_13, Pull::None);
    let dc_d = Output::new(p.PIN_8, Level::Low);
    let rst_d = Output::new(p.PIN_12, Level::Low);
    let mut delayns = Delay;
    let mut epd = EPD3in7::new(&mut spidev_d, busy_d, dc_d, rst_d, &mut delayns, None).unwrap();

    let mut display = Display3in7::default();


    let splash_bytes = include_bytes!("../splashnewbw.bmp");
    let splash_bmp = tinybmp::Bmp::<BinaryColor>::from_slice(splash_bytes).unwrap();
    Image::new(&splash_bmp, Point::zero()).draw(&mut display.color_converted()).unwrap_or_else(|_| error!("failed to render splash screen"));

    epd.clear_frame(&mut spidev_d, &mut delayns).unwrap_or_else(|_| error!("can't clear frame"));
    epd.update_and_display_frame(&mut spidev_d, &display.buffer(), &mut delayns)
        .unwrap_or_else(|_| error!("can't update display"));

    info!("displayed...");

    epd.sleep(&mut spidev_d, &mut delayns).unwrap_or_else(|_| error!("can't sleep display"));

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

    info!("connection to: {}", URL);

    info!("registering with server");
    match client.request(reqwless::request::Method::POST, &URL).await {
        Ok(request) => match serde_json_core::to_string::<Device, 128>(&dev) {
            Ok(json) => match request
                .content_type(reqwless::headers::ContentType::ApplicationJson)
                .body(json.as_bytes())
                .send(&mut rx_buffer)
                .await
            {
                Ok(response) => info!("recieved response {}", response),
                Err(_) => error!("sending request"),
            },
            Err(_) => error!("formatting the request failed..."),
        },
        // from my experiementing this happened when the server was offline
        Err(_) => error!("creating request"),
    };

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
        // TODO tune the temp reading, value of ~29 in a cool room...
        info!("temperature reading of {}", convert_to_celsius(adc.read(&mut ts).await.unwrap_or_default()));

        // TODO produce values first
        // request updates on consumed values
        match client.request(reqwless::request::Method::GET, &URL).await {
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
                        match serde_json_core::from_slice::<Vec<ContextResource, 3>>(
                            &response.body().body_buf[0..length],
                        ) {
                            Ok((t,_)) => {
                                info!("{:?}", t);
                                let mut c = 1;
                                for v in t {
                                    match c {
                                        1 => value1 = v.resource.value,
                                        2 => value2 = v.resource.value,
                                        3 => value3 = v.resource.value,
                                        _ => info!("more than expected..."),
                                    }
                                    c += 1;
                                }
                                // v[0]["context"]
                                // v[0]["resource"]["what"]
                                // v[0]["resource"]["value"]
                            },
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

        let mut data: String<64> = String::try_from("Top value: ").unwrap();
        let formatted_data = serde_json_core::to_string::<Value, 32>(&value1).unwrap_or_default();
        let _ = data.push_str(formatted_data.as_str());
        let _ = Text::new(data.as_str(), Point::new(10, 200), text_style).draw(&mut display);

        let mut data: String<64> = String::try_from("Middle value: ").unwrap();
        let _ = data.push_str(&serde_json_core::to_string::<Value, 32>(&value2).unwrap_or_default());
        let _ = Text::new(data.as_str(), Point::new(10, 200 + (1 * text_height * 2) as i32), text_style).draw(&mut display);

        let mut data: String<64> = String::try_from("Bottom value: ").unwrap();
        let _ = data.push_str(&serde_json_core::to_string::<Value, 32>(&value3).unwrap_or_default());
        let _ = Text::new(data.as_str(), Point::new(10, 200 + (2 * text_height * 2) as i32), text_style).draw(&mut display);

        epd.wake_up(&mut spidev_d, &mut delayns).unwrap_or_else(|_| error!("can't wake up display"));

        epd.clear_frame(&mut spidev_d, &mut delayns).unwrap_or_else(|_| error!("can't clear display"));

        epd.update_and_display_frame(&mut spidev_d, &display.buffer(), &mut delayns).unwrap_or_else(|_| error!("can't update and display"));

        epd.sleep(&mut spidev_d, &mut delayns).unwrap_or_else(|_| error!("can't sleep display"));


        Timer::after(Duration::from_secs(30)).await;
    }
}
