use std::net::UdpSocket;
use log::{info, debug};
use mavlink::{self, MavConnection};

// =============================
// Config
// =============================

// MAVLink input (receives forwarded messages from MAVProxy)
const MAVLINK_UDP_PORT: u16 = 14551;

// PWM output destination
const UDP_IP: &str = "192.168.1.14";
const UDP_PORT: u16 = 5050;

// RC channels to use (9 and 10)
const RC_CH_AZIMUTH: u8 = 9;
const RC_CH_ELEVATION: u8 = 10;

// PWM valid range
const PWM_MIN: u16 = 1000;
const PWM_MAX: u16 = 2000;

// =============================
// Main
// =============================

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize logger
    env_logger::Builder::from_default_env()
        .filter_level(log::LevelFilter::Info)
        .init();

    info!("Starting MAVLink PWM Forwarder");
    info!("Listening for MAVLink on UDP port {}", MAVLINK_UDP_PORT);
    info!(
        "Forwarding CH{} and CH{} to {}:{}",
        RC_CH_AZIMUTH, RC_CH_ELEVATION, UDP_IP, UDP_PORT
    );

    // Create MAVLink connection (UDP listen)
    let mavlink_addr = format!("udpin:0.0.0.0:{}", MAVLINK_UDP_PORT);
    let mav_conn = mavlink::connect::<mavlink::ardupilotmega::MavMessage>(&mavlink_addr)?;

    // Create UDP socket for sending PWM values
    let send_sock = UdpSocket::bind("0.0.0.0:0")?;
    let dest_addr = format!("{}:{}", UDP_IP, UDP_PORT);

    info!("Waiting for RC_CHANNELS messages...");

    loop {
        // Receive MAVLink message
        match mav_conn.recv() {
            Ok((_header, msg)) => {
                // Check if it's an RC_CHANNELS message
                if let mavlink::ardupilotmega::MavMessage::RC_CHANNELS(rc_channels) = msg {
                    // Extract PWM values from channels 9 and 10
                    let az_pwm = rc_channels.chan9_raw;
                    let el_pwm = rc_channels.chan10_raw;

                    // Only send if both channels have valid PWM values (1000-2000 range)
                    // Filter out: 0 (no signal), 65535 (UINT16_MAX/unset), or out of range
                    if !is_valid_pwm(az_pwm) || !is_valid_pwm(el_pwm) {
                        debug!("Ignoring invalid PWM: az={}, el={}", az_pwm, el_pwm);
                        continue;
                    }

                    // Pack as big-endian signed integers (i32) and send
                    let mut packet = Vec::with_capacity(8);
                    packet.extend_from_slice(&(az_pwm as i32).to_be_bytes());
                    packet.extend_from_slice(&(el_pwm as i32).to_be_bytes());

                    send_sock.send_to(&packet, &dest_addr)?;

                    info!(
                        "Forwarded | CH{}={}, CH{}={}",
                        RC_CH_AZIMUTH, az_pwm, RC_CH_ELEVATION, el_pwm
                    );
                }
            }
            Err(e) => {
                // Log errors but continue running
                debug!("MAVLink recv error: {}", e);
                continue;
            }
        }
    }
}

/// Check if PWM value is valid (1000-2000 range)
fn is_valid_pwm(pwm: u16) -> bool {
    pwm >= PWM_MIN && pwm <= PWM_MAX
}
