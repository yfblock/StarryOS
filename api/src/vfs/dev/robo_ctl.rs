#![allow(dead_code)]
use core::any::Any;

use axconfig::plat::PHYS_VIRT_OFFSET;
use axerrno::LinuxError;
use axfs_ng_vfs::{NodeFlags, VfsResult};
use axhal::mem::phys_to_virt;
use dw_apb_uart::DW8250;
use memory_addr::pa;
use num_enum::TryFromPrimitive;
use sg200x_bsp::{
    pinmux::{
        FMUX_IIC0_SCL, FMUX_IIC0_SDA, FMUX_JTAG_CPU_TCK, FMUX_JTAG_CPU_TMS, FMUX_UART0_RX,
        FMUX_UART0_TX, Pinmux,
    },
    pwm::{Pwm, PwmChannel, PwmInstance, PwmMode, PwmPolarity},
};
use starry_core::vfs::DeviceOps;
use tock_registers::interfaces::Writeable;

// ─── STS3215 舵机驱动 ───────────────────────────────────────────────────────

const UART2_ADDR: usize = 0x04160000;
const MAX_PACKET_SIZE: usize = 16;
const DEFAULT_PWM_PERIOD: u32 = 10000;
const DEFAULT_PWM_DUTY: u32 = 7000;

fn serial_write(data: &[u8]) {
    let mut uart = DW8250::new(phys_to_virt(pa!(UART2_ADDR)).as_usize());
    data.iter().for_each(|x| uart.putchar(*x));
}

fn delay_ms(ms: u32) {
    axhal::time::busy_wait(core::time::Duration::new(
        ms as u64 / 1000,
        (ms % 1000) * 1_000_000,
    ));
}

fn checksum(data: &[u8]) -> u8 {
    let sum: u32 = data.iter().map(|&b| b as u32).sum();
    (!sum) as u8
}

fn send_cmd(servo_id: u8, instruction: u8, params: &[u8]) {
    let length = (params.len() + 2) as u8;
    let mut pkt = [0u8; MAX_PACKET_SIZE];
    let mut idx = 0;

    pkt[idx] = 0xFF;
    idx += 1;
    pkt[idx] = 0xFF;
    idx += 1;
    pkt[idx] = servo_id;
    idx += 1;
    pkt[idx] = length;
    idx += 1;
    pkt[idx] = instruction;
    idx += 1;

    for &b in params {
        pkt[idx] = b;
        idx += 1;
    }

    let chk = checksum(&pkt[2..idx]);
    pkt[idx] = chk;
    idx += 1;

    serial_write(&pkt[..idx]);
}

fn write_reg(servo_id: u8, addr: u8, data: &[u8]) {
    let mut params = [0u8; MAX_PACKET_SIZE - 6];
    params[0] = addr;
    let n = data.len().min(params.len() - 1);
    params[1..1 + n].copy_from_slice(&data[..n]);
    send_cmd(servo_id, 0x03, &params[..1 + n]);
}

fn move_to_position(servo_id: u8, pos: i32) {
    let pos = pos.clamp(0, 4095) as u16;
    write_reg(servo_id, 0x2A, &pos.to_le_bytes());
}

fn set_speed(servo_id: u8, speed: u16) {
    write_reg(servo_id, 0x2E, &speed.to_le_bytes());
}

fn set_max_torque_limit(servo_id: u8, torque: u16) {
    write_reg(servo_id, 0x10, &torque.to_le_bytes());
}

fn set_protection_current(servo_id: u8, current: u16) {
    write_reg(servo_id, 0x40, &current.to_le_bytes());
}

fn set_overload_torque(servo_id: u8, torque: u8) {
    write_reg(servo_id, 0x24, &[torque]);
}

fn set_operating_mode(servo_id: u8, mode: u8) {
    write_reg(servo_id, 0x21, &[mode]);
}

fn set_p_coefficient(servo_id: u8, value: u8) {
    write_reg(servo_id, 0x15, &[value]);
}

fn set_i_coefficient(servo_id: u8, value: u8) {
    write_reg(servo_id, 0x17, &[value]);
}

fn set_d_coefficient(servo_id: u8, value: u8) {
    write_reg(servo_id, 0x16, &[value]);
}

fn arm_init() {
    let mut uart = DW8250::new(phys_to_virt(pa!(UART2_ADDR)).as_usize());
    uart.init();
    for i in 1..=3u8 {
        set_operating_mode(i, 0);
        set_speed(i, 1500);
        set_p_coefficient(i, 16);
        set_i_coefficient(i, 0);
        set_d_coefficient(i, 32);

        if i == 3 {
            set_max_torque_limit(i, 500);
            set_protection_current(i, 250);
            set_overload_torque(i, 25);
        }
    }
}

fn arm_grab() {
    move_to_position(1, 1800);
    move_to_position(2, 2400);
    move_to_position(3, 4000);
    delay_ms(1000);

    move_to_position(3, 3300);
    delay_ms(1000);

    move_to_position(1, 2600);
    move_to_position(2, 2500);
    move_to_position(3, 3300);
}

fn arm_release() {
    move_to_position(3, 3700);
}

// ─── PWM 电机控制 ───────────────────────────────────────────────────────────
//
// PWM Chip1 (PwmInstance::Pwm1) 对应物理 PWM4~PWM7:
//   Channel0 -> PWM4 (UART0_TX pin)  右电机反转
//   Channel1 -> PWM5 (UART0_RX pin)  右电机正转
//   Channel2 -> PWM6 (JTAG_CPU_TCK)  左电机反转
//   Channel3 -> PWM7 (JTAG_CPU_TMS)  左电机正转

fn pwm_chip() -> Pwm {
    Pwm::new_with_offset(PwmInstance::Pwm1, PHYS_VIRT_OFFSET)
}

fn motor_start_channel(pwm: &mut Pwm, ch: PwmChannel, period: u32, duty: u32) {
    let _ = pwm.configure_channel_raw(ch, period, duty, PwmPolarity::ActiveHigh);
    pwm.set_mode(ch, PwmMode::Continuous);
    pwm.enable_output(ch);
    pwm.start(ch);
}

fn motor_stop_channel(pwm: &mut Pwm, ch: PwmChannel) {
    pwm.stop(ch);
    pwm.disable_output(ch);
}

fn motor_stop_all(pwm: &mut Pwm) {
    for i in 0..4 {
        if let Some(ch) = PwmChannel::from_u8(i) {
            motor_stop_channel(pwm, ch);
        }
    }
}

fn motor_forward(period: u32, duty: u32) {
    let mut pwm = pwm_chip();
    motor_stop_all(&mut pwm);
    motor_start_channel(&mut pwm, PwmChannel::Channel1, period, duty); // 右电机正转
    motor_start_channel(&mut pwm, PwmChannel::Channel3, period, duty); // 左电机正转
}

fn motor_backward(period: u32, duty: u32) {
    let mut pwm = pwm_chip();
    motor_stop_all(&mut pwm);
    motor_start_channel(&mut pwm, PwmChannel::Channel0, period, duty); // 右电机反转
    motor_start_channel(&mut pwm, PwmChannel::Channel2, period, duty); // 左电机反转
}

fn motor_turn_left(period: u32, duty: u32) {
    let mut pwm = pwm_chip();
    motor_stop_all(&mut pwm);
    motor_start_channel(&mut pwm, PwmChannel::Channel2, period, duty); // 左电机反转
    motor_start_channel(&mut pwm, PwmChannel::Channel1, period, duty); // 右电机正转
}

fn motor_turn_right(period: u32, duty: u32) {
    let mut pwm = pwm_chip();
    motor_stop_all(&mut pwm);
    motor_start_channel(&mut pwm, PwmChannel::Channel3, period, duty); // 左电机正转
    motor_start_channel(&mut pwm, PwmChannel::Channel0, period, duty); // 右电机反转
}

fn motor_stop() {
    let mut pwm = pwm_chip();
    motor_stop_all(&mut pwm);
}

// ─── IOCTL 接口定义 ─────────────────────────────────────────────────────────
//
// cmd 编码:
//   1  INIT        初始化机械臂与 PWM 引脚复用       arg: 忽略
//   2  TURN_LEFT   左转（仅右电机转动）               arg: 忽略(使用默认速度) 或 duty值
//   3  TURN_RIGHT  右转（仅左电机转动）               arg: 同上
//   4  FORWARD     前进                              arg: 同上
//   5  BACKWARD    后退                              arg: 同上
//   6  GRAB        机械臂抓取                        arg: 忽略
//   7  RELEASE     机械臂释放                        arg: 忽略
//   8  STOP        停止所有电机                      arg: 忽略
//   9  MOVE_SERVO  移动指定舵机到目标位置             arg: (servo_id << 16) | position
//  10  SET_SPEED   设置舵机速度                      arg: (servo_id << 16) | speed

#[repr(u8)]
#[derive(TryFromPrimitive)]
enum ControlArgs {
    Init      = 1,
    TurnLeft  = 2,
    TurnRight = 3,
    Forward   = 4,
    Backward  = 5,
    Grab      = 6,
    Release   = 7,
    Stop      = 8,
    MoveServo = 9,
    SetSpeed  = 10,
}

pub struct RoboCtl;

impl RoboCtl {
    pub fn new() -> Self {
        let pinmux = Pinmux::new_with_offset(PHYS_VIRT_OFFSET);

        // UART2 pinmux (舵机串口)
        pinmux
            .fmux()
            .iic0_sda
            .write(FMUX_IIC0_SDA::FSEL::UART2_RX);
        pinmux
            .fmux()
            .iic0_scl
            .write(FMUX_IIC0_SCL::FSEL::UART2_TX);

        // PWM pinmux (电机控制)
        pinmux
            .fmux()
            .uart0_tx
            .write(FMUX_UART0_TX::FSEL::PWM_4);
        pinmux
            .fmux()
            .uart0_rx
            .write(FMUX_UART0_RX::FSEL::PWM_5);
        pinmux
            .fmux()
            .jtag_cpu_tck
            .write(FMUX_JTAG_CPU_TCK::FSEL::PWM_6);
        pinmux
            .fmux()
            .jtag_cpu_tms
            .write(FMUX_JTAG_CPU_TMS::FSEL::PWM_7);

        Self
    }
}

impl DeviceOps for RoboCtl {
    fn read_at(&self, _buf: &mut [u8], _offset: u64) -> VfsResult<usize> {
        Ok(0)
    }

    fn write_at(&self, buf: &[u8], _offset: u64) -> VfsResult<usize> {
        Ok(buf.len())
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn flags(&self) -> NodeFlags {
        NodeFlags::NON_CACHEABLE | NodeFlags::STREAM
    }

    fn ioctl(&self, cmd: u32, arg: usize) -> VfsResult<usize> {
        let cmd = ControlArgs::try_from_primitive(cmd as u8).map_err(|_| LinuxError::EINVAL)?;
        let duty = if arg != 0 { arg as u32 } else { DEFAULT_PWM_DUTY };

        match cmd {
            ControlArgs::Init => {
                arm_init();
                move_to_position(1, 2600);
                move_to_position(2, 2500);
                arm_release();
            }
            ControlArgs::Forward => {
                motor_forward(DEFAULT_PWM_PERIOD, duty);
            }
            ControlArgs::Backward => {
                motor_backward(DEFAULT_PWM_PERIOD, duty);
            }
            ControlArgs::TurnLeft => {
                motor_turn_left(DEFAULT_PWM_PERIOD, duty);
            }
            ControlArgs::TurnRight => {
                motor_turn_right(DEFAULT_PWM_PERIOD, duty);
            }
            ControlArgs::Grab => {
                arm_grab();
            }
            ControlArgs::Release => {
                arm_release();
            }
            ControlArgs::Stop => {
                motor_stop();
            }
            ControlArgs::MoveServo => {
                let servo_id = ((arg >> 16) & 0xFF) as u8;
                let position = (arg & 0xFFFF) as i32;
                if servo_id == 0 || servo_id > 3 {
                    return Err(LinuxError::EINVAL.into());
                }
                move_to_position(servo_id, position);
            }
            ControlArgs::SetSpeed => {
                let servo_id = ((arg >> 16) & 0xFF) as u8;
                let speed = (arg & 0xFFFF) as u16;
                if servo_id == 0 || servo_id > 3 {
                    return Err(LinuxError::EINVAL.into());
                }
                set_speed(servo_id, speed);
            }
        }
        Ok(0)
    }
}
