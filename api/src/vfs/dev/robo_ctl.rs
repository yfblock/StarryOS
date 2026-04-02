#![allow(dead_code)]
use core::any::Any;

use axconfig::plat::PHYS_VIRT_OFFSET;
use axerrno::LinuxError;
use axfs_ng_vfs::{NodeFlags, VfsResult};
use axhal::mem::phys_to_virt;
use dw_apb_uart::DW8250;
use memory_addr::pa;
use num_enum::TryFromPrimitive;
use sg200x_bsp::pinmux::{FMUX_IIC0_SCL, FMUX_IIC0_SDA, FMUX_JTAG_CPU_TCK, FMUX_JTAG_CPU_TMS, FMUX_UART0_RX, FMUX_UART0_TX, Pinmux};
use starry_core::vfs::DeviceOps;
use tock_registers::interfaces::Writeable;


/// STS3215 舵机驱动及机械臂控制模块 (no_std)

/// 向串口写入数据（待实现）
pub fn serial_write(data: &[u8]) {
    let mut uart = DW8250::new(phys_to_virt(pa!(0x04160000)).as_usize());
    data.iter().for_each(|x| uart.putchar(*x));
}

/// 延时（毫秒）（待实现）
pub fn delay_ms(ms: u32) {
    // ax_sleep_until(_ms);
    // todo!()
    axhal::time::busy_wait(core::time::Duration::new(
        ms as u64 / 1000,
        (ms % 1000) * 1_000_000,
    ));
}

const MAX_PACKET_SIZE: usize = 16;

fn checksum(data: &[u8]) -> u8 {
    let sum: u32 = data.iter().map(|&b| b as u32).sum();
    (!sum) as u8
}

/// 发送指令包
pub fn send_cmd(servo_id: u8, instruction: u8, params: &[u8]) {
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

/// 写寄存器 (INST_WRITE = 0x03)
pub fn write_reg(servo_id: u8, addr: u8, data: &[u8]) {
    let mut params = [0u8; MAX_PACKET_SIZE - 6];
    params[0] = addr;
    let n = data.len().min(params.len() - 1);
    params[1..1 + n].copy_from_slice(&data[..n]);
    send_cmd(servo_id, 0x03, &params[..1 + n]);
}

/// 移动到绝对位置 (0~4095)
pub fn move_to_position(servo_id: u8, pos: i32) {
    let pos = pos.clamp(0, 4095) as u16;
    write_reg(servo_id, 0x2A, &pos.to_le_bytes());
}

/// 角度控制 (0~360°)
pub fn move_angle(servo_id: u8, angle: u32) {
    let pos = (angle * 4095 / 360) as i32;
    move_to_position(servo_id, pos);
}

/// 设置速度 (寄存器 0x2E)
pub fn set_speed(servo_id: u8, speed: u16) {
    write_reg(servo_id, 0x2E, &speed.to_le_bytes());
}

/// 设置最大扭矩限制 (寄存器 0x10)
pub fn set_max_torque_limit(servo_id: u8, torque: u16) {
    write_reg(servo_id, 0x10, &torque.to_le_bytes());
}

/// 设置保护电流 (寄存器 0x40)
pub fn set_protection_current(servo_id: u8, current: u16) {
    write_reg(servo_id, 0x40, &current.to_le_bytes());
}

/// 设置过载扭矩 (寄存器 0x24, 1字节)
pub fn set_overload_torque(servo_id: u8, torque: u8) {
    write_reg(servo_id, 0x24, &[torque]);
}

/// 设置工作模式 (寄存器 0x21, 1字节)
pub fn set_operating_mode(servo_id: u8, mode: u8) {
    write_reg(servo_id, 0x21, &[mode]);
}

/// 设置 P 系数 (寄存器 0x15, 1字节)
pub fn set_p_coefficient(servo_id: u8, value: u8) {
    write_reg(servo_id, 0x15, &[value]);
}

/// 设置 I 系数 (寄存器 0x17, 1字节)
pub fn set_i_coefficient(servo_id: u8, value: u8) {
    write_reg(servo_id, 0x17, &[value]);
}

/// 设置 D 系数 (寄存器 0x16, 1字节)
pub fn set_d_coefficient(servo_id: u8, value: u8) {
    write_reg(servo_id, 0x16, &[value]);
}

/// 机械臂初始化：配置舵机 1~3 的工作模式、速度和 PID 参数
pub fn arm_init() {
    let mut uart = DW8250::new(phys_to_virt(pa!(0x04160000)).as_usize());
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

/// 机械臂抓取动作
pub fn grab() {
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

/// 机械臂释放动作
pub fn release() {
    move_to_position(3, 3700);
}

pub struct RoboCtl;

#[repr(u8)]
#[derive(num_enum::TryFromPrimitive)]
enum ControlArgs {
    Init      = 1,
    TurnLeft  = 2,
    TurnRight = 3,
    Forward   = 4,
    Backward  = 5,
    Grab      = 6,
    Release   = 7,
}

impl RoboCtl {
    pub fn new() -> Self {
        let pinmux = Pinmux::new_with_offset(PHYS_VIRT_OFFSET);
        pinmux.fmux().iic0_sda.write(FMUX_IIC0_SDA::FSEL::UART2_RX);
        pinmux.fmux().iic0_scl.write(FMUX_IIC0_SCL::FSEL::UART2_TX);

        // Set PWM PinMUX
        pinmux
            .fmux()
            .jtag_cpu_tms
            .write(FMUX_JTAG_CPU_TMS::FSEL::PWM_7);
        pinmux
            .fmux()
            .jtag_cpu_tck
            .write(FMUX_JTAG_CPU_TCK::FSEL::PWM_6);
        pinmux.fmux().uart0_tx.write(FMUX_UART0_TX::FSEL::PWM_4);
        pinmux.fmux().uart0_rx.write(FMUX_UART0_RX::FSEL::PWM_5);
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
        let cmd = ControlArgs::try_from_primitive(cmd as _).map_err(|_| LinuxError::EINVAL)?;
        match cmd {
            ControlArgs::Init => {
                arm_init();
                move_to_position(1, 2600);
                move_to_position(2, 2500);
                release();
            },
            ControlArgs::TurnLeft => todo!(),
            ControlArgs::TurnRight => todo!(),
            ControlArgs::Forward => todo!(),
            ControlArgs::Backward => todo!(),
            ControlArgs::Grab => todo!(),
            ControlArgs::Release => todo!(),
        }
        Ok(0)
    }
}
