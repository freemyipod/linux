//! S5L8730 LCDCON support for gpu/drm/tiny.
use kernel::{
    bit, bindings,
    prelude::*, c_str,
    device, of, drm, module_platform_driver, platform,
    error::{to_result},
    io_mem::IoMem,
    sync::{Arc, ArcBorrow},
};

use core::time::Duration;

module_platform_driver! {
    type: LcdConDriver,
    name: "s5l8730_lcdcon",
    author: "q3k",
    description: "MIPI DBI driver for S5L8730 LCD controller",
    license: "GPL v2",
}

const INFO: drm::drv::DriverInfo = drm::drv::DriverInfo {
    major: 0,
    minor: 1,
    patchlevel: 0,
    name: c_str!("s5l8730"),
    desc: c_str!("S5L8730 Framebuffer"),
    date: c_str!("20230122"),
};


struct LcdConData {
    dev: device::Device,

    conreg: Pin<Box<drm::kms::ConnectorRegistration<ConnectorDriver>>>,
}

struct File {
}

impl drm::file::DriverFile for File {
    type Driver = LcdConDriver;
    fn open(device: &LcdConDevice) -> Result<Box<Self>> {
        pr_info!("DRM device opened\n");
        Ok(Box::try_new(Self {
        })?)
    }
}

struct Resources {
    dev: device::Device,

    con: IoMem<0x1000>,
}

const REG_CON: usize = 0x00;
const REG_CMD: usize = 0x04;
const REG_ACK: usize = 0x10;
const REG_READ: usize = 0x14;
const REG_STATUS: usize = 0x1c;
const REG_WRITE: usize = 0x40;

impl Resources {
    fn new(pdev: &mut platform::Device) -> Result<Self> {
        let con = unsafe {
            pdev.ioremap_resource(0)?
        };
        Ok(Self {
            dev: device::Device::from_dev(pdev),
            con,
        })
    }

    fn fifo_rx_ne(&self) -> bool {
        (self.con.readl_relaxed(REG_STATUS) & bit(0)) != 0
    }

    fn tx_reg_empty(&self) -> bool {
        (self.con.readl_relaxed(REG_STATUS) & bit(4)) == 0
    }

    fn fifo_rx_ne_wait(&self) -> Result<()> {
        kernel::iopoll::read_poll_timeout(|| self.fifo_rx_ne(), Duration::from_micros(100), Duration::from_millis(20))
    }

    fn tx_reg_empty_wait(&self) -> Result<()> {
        kernel::iopoll::read_poll_timeout(|| self.tx_reg_empty(), Duration::from_micros(100), Duration::from_millis(20))
    }

    #[inline(never)]
    fn read_byte(&self) -> Result<u8> {
        self.con.writel_relaxed(0, REG_ACK);
        self.fifo_rx_ne_wait()?;
        Ok((self.con.readl_relaxed(REG_READ) >> 1) as u8)
    }

    fn write_word(&self, b: u32) -> Result<()> {
        self.tx_reg_empty_wait()?;
        self.con.writel_relaxed(b as u32, REG_WRITE);
        Ok(())
    }

    fn reset(&self) -> Result<()> {
        self.tx_reg_empty_wait()?;
        self.con.writel_relaxed(0x1100db8, REG_CON);
        Ok(())
    }

    fn transact_read(&self, cmd: u8, length: usize) -> Result<Vec<u8>> {
        self.tx_reg_empty_wait()?;
        self.con.writel_relaxed(cmd as u32, REG_CMD);

        // Discard first byte result.
        self.read_byte()?;

        let mut res = Vec::try_with_capacity(length)?;
        for i in 0..length {
            res.try_push(self.read_byte()?)?;
        }
        Ok(res)
    }

    fn transact_write<I: IntoIterator<Item = u32>>(&self, cmd: u8, data: I) -> Result<()> {
        self.tx_reg_empty_wait()?;
        self.con.writel_relaxed(cmd as u32, REG_CMD);
        for el in data {
            self.write_word(el)?;
        }
        Ok(())
    }

    // Insert bit between high 23 bits and low 8 bits. Why? How should I know.
    fn mangle(i: u32) -> u32 {
        let h = (i >> 8);
        let l = (i & 0xff);
        (h << 9) | l
    }

    fn pio_write_xr24<I: IntoIterator<Item = u8>>(
        &self,
        x1: i32, y1: i32,
        x2: i32, y2: i32,
        pixels: I
    ) -> Result<()> {
        self.transact_write(0x2a, [
            Self::mangle(x1 as u32),
            Self::mangle(x2 as u32),
        ])?;
        self.transact_write(0x2b, [
            Self::mangle(y1 as u32),
            Self::mangle(y2 as u32),
        ])?;
        let mut txbuf = Vec::new();
        let mut pixels = pixels.into_iter();
        loop {
            let b = match pixels.next() {
                None => break,
                Some(el) => el,
            };
            let g = match pixels.next() {
                None => break,
                Some(el) => el,
            };
            let r = match pixels.next() {
                None => break,
                Some(el) => el,
            };
            pixels.next();
            let r = r as u32 >> 3;
            let g = g as u32 >> 2;
            let b = b as u32 >> 3;
            let rgb = (b << 11) | (g << 5) | r;
            txbuf.try_push(rgb)?;
        }
        self.transact_write(0x2c, txbuf)?;
        Ok(())
    }
    fn pio_write_rg24<I: IntoIterator<Item = u8>>(
        &self,
        x1: i32, y1: i32,
        x2: i32, y2: i32,
        pixels: I
    ) -> Result<()> {
        self.transact_write(0x2a, [
            Self::mangle(x1 as u32),
            Self::mangle(x2 as u32),
        ])?;
        self.transact_write(0x2b, [
            Self::mangle(y1 as u32),
            Self::mangle(y2 as u32),
        ])?;
        let mut txbuf = Vec::new();
        let mut pixels = pixels.into_iter();
        loop {
            let r = match pixels.next() {
                None => break,
                Some(el) => el,
            };
            let g = match pixels.next() {
                None => break,
                Some(el) => el,
            };
            let b = match pixels.next() {
                None => break,
                Some(el) => el,
            };
            let r = r as u32 >> 3;
            let g = g as u32 >> 2;
            let b = b as u32 >> 3;
            let rgb = (b << 11) | (g << 5) | r;
            txbuf.try_push(rgb)?;
        }
        self.transact_write(0x2c, txbuf)?;
        Ok(())
    }
}

struct DriverObject {}

impl drm::gem::BaseDriverObject<Object> for DriverObject {
    fn new(_dev: &LcdConDevice, _size: usize) -> Result<Self> {
        pr_info!("DriverObject::new\n");
        Ok(Self {
        })
    }
}

impl drm::gem::shmem::DriverObject for DriverObject {
    type Driver = LcdConDriver;
}

type Object = drm::gem::shmem::Object<DriverObject>;

struct LcdConDriver;

type DeviceData = device::Data<drm::drv::Registration<LcdConDriver>, Resources, LcdConData>;
type LcdConDevice = drm::device::Device<LcdConDriver>;

impl Drop for LcdConData {
    fn drop(&mut self) {
        pr_info!("LcdConData drop???\n");
    }
}

struct ConnectorDriver;

#[vtable]
impl drm::kms::Connector for ConnectorDriver {
    type Data = Arc<()>;

    fn get_modes(
        data: ArcBorrow<'_, ()>,
        conn: *mut bindings::drm_connector,
    ) -> Result<i32> {
        pr_info!("ConnectorDriver::get_modes\n");
        let mut mode = bindings::drm_display_mode {
            status: 0, type_: bindings::DRM_MODE_TYPE_DRIVER as u8,
            flags: 0, clock: 31500,
            hdisplay: 240, hsync_start: 240, hsync_end: 240, htotal: 240, hskew: 0,
            vdisplay: 376, vsync_start: 376, vsync_end: 376, vtotal: 376, vscan: 0,
            width_mm: 30, height_mm: 47,
            ..Default::default()
        };
        let name = c_str!("230x376");
        unsafe {
            core::ptr::copy_nonoverlapping(
                &name.as_bytes()[0] as *const u8 as *const i8,
                &mut mode.name[0],
                name.len()
            );
        }
        unsafe {
            let mode = kernel::error::from_kernel_err_ptr(bindings::drm_mode_duplicate((*conn).dev, &mode))?;
            bindings::drm_mode_probed_add(conn, mode);
            bindings::drm_set_preferred_mode(conn, 240, 376);
        }
        drop(mode);
        drop(name);
        Ok(1)
    }

    fn atomic_update(
        data: ArcBorrow<'_, ()>,
        raw_plane: *mut bindings::drm_plane,
        raw_state: *mut bindings::drm_atomic_state,
    ) -> Result<()> {
        let plane = unsafe { &mut *raw_plane };
        let state = unsafe { &mut *raw_state };
        let plane_state = unsafe {
            let res = bindings::drm_atomic_get_new_plane_state(state, plane);
            &mut *res
        };
        let old_plane_state = unsafe {
            let res = bindings::drm_atomic_get_old_plane_state(state, plane);
            &mut *res
        };
        let shadow_plane_state = unsafe {
            let res = bindings::to_drm_shadow_plane_state(plane_state);
            &mut *res
        };
        let fb = unsafe { &mut *plane_state.fb };
        let format = unsafe { (*fb.format).format };
        let dev = unsafe { &mut *plane.dev };
        let drmdev = core::mem::ManuallyDrop::new(unsafe { LcdConDevice::from_raw(dev) });

        let data = drmdev.data();
        let res = data.resources().ok_or(ENXIO)?;
        let pitch = fb.pitches[0] as i32;
        let count = pitch * (fb.height as i32);
        let src = unsafe {
            let vaddr = shadow_plane_state.data[0].__bindgen_anon_1.vaddr;
            core::slice::from_raw_parts(vaddr as *const u8, count as usize)
        };
        let bpp = if unsafe { &*fb.format }.format == kernel::fourcc!('X', 'R', '2', '4') {
            4
        } else {
            3
        };

        unsafe {
            to_result(bindings::drm_gem_fb_begin_cpu_access(
                fb, bindings::dma_data_direction_DMA_FROM_DEVICE
            ))?;
        }

        let _end_access = kernel::ScopeGuard::new(|| unsafe {
            bindings::drm_gem_fb_end_cpu_access(
                fb, bindings::dma_data_direction_DMA_FROM_DEVICE
            );
        });

        let mut idx = 0;
        if ! unsafe { bindings::drm_dev_enter(dev, &mut idx) } {
            return Ok(())
        }
        let _end_critical = kernel::ScopeGuard::new(|| unsafe {
            bindings::drm_dev_exit(idx);
        });

        let mut iter = bindings::drm_atomic_helper_damage_iter::default();
        let mut damage = bindings::drm_rect::default();
        unsafe { bindings::drm_atomic_helper_damage_iter_init(&mut iter, old_plane_state, plane_state); }

        loop {
            if ! unsafe { bindings::drm_atomic_helper_damage_iter_next(&mut iter, &mut damage) } {
                break
            }
            let dst_clip = &mut plane_state.dst;
            if ! unsafe { bindings::drm_rect_intersect(dst_clip, &damage)} {
                continue
            }

            for line in damage.y1..damage.y2 {
                let start = line*pitch + damage.x1 * bpp;
                let end = line*pitch + damage.x2 * bpp;
                //pr_info!("start: {}, end: {}, src: {:x}\n", start, end, &src[0] as *const u16 as usize);
                let range = &src[(start as usize)..(end as usize)];
                if bpp == 3 {
                    res.pio_write_rg24(damage.x1, line, 240, 376, range.iter().cloned())?;
                } else {
                    res.pio_write_xr24(damage.x1, line, 240, 376, range.iter().cloned())?;
                }
            }
        }

        Ok(())
    }
}
type ConnectorData = device::Data<drm::kms::ConnectorRegistration<ConnectorDriver>, (), ()>;


#[vtable]
impl drm::drv::Driver for LcdConDriver {
    type Data = Arc<DeviceData>;
    type File = File;
    type Object = Object;

    const INFO: drm::drv::DriverInfo = INFO;
    const FEATURES: u32 = drm::drv::FEAT_GEM | drm::drv::FEAT_MODESET | drm::drv::FEAT_ATOMIC;

    kernel::declare_drm_ioctls! {
    }
}

impl platform::Driver for LcdConDriver {
    type Data = Arc<DeviceData>;

    kernel::define_of_id_table! {(), [
        (of::DeviceId::Compatible(b"samsung,s5l8730-lcdcon"), None)
    ]}

    fn probe(pdev: &mut platform::Device, _id_info: Option<&Self::IdInfo>) -> Result<Self::Data> {
        let dev = device::Device::from_dev(pdev);

        dev_info!(dev, "Probing...\n");

        let res = Resources::new(pdev)?;
        let mut reg = drm::drv::Registration::<LcdConDriver>::new(&dev)?;

        match res.reset() {
            Ok(_) => {},
            Err(err) => {
                dev_err!(dev, "Reset failed: {:?}\n", err);
                return Err(err);
            },
        };

        //let panel_id = match res.transact_read(0x04, 3) {
        //    Ok(pid) => pid,
        //    Err(err) => {
        //        dev_err!(dev, "Panel ID read failed: {:?}\n", err);
        //        return Err(err);
        //    },
        //};
        //dev_info!(dev, "Panel ID: {:02x}{:02x}{:02x}", panel_id[0], panel_id[1], panel_id[2]);

        fixed_init(reg.device_mut());

        let condata = Arc::try_new(())?;
        let conreg = drm::kms::ConnectorRegistration::<ConnectorDriver>::new(condata)?;
        let mut conreg = Pin::from(Box::try_new(conreg)?);
        conreg.as_mut().register(&mut reg)?;

        pr_info!("probe drm_device: {:x}\n", reg.device().raw() as *const bindings::drm_device as usize);
        pr_info!("probe drm::Device: {:x}\n", reg.device() as *const drm::device::Device<_> as usize);
        let data = kernel::new_device_data!(
            reg,
            res,
            LcdConData {
                dev,
                conreg,
            },
            "S5L8730LCDCON::Registrations"
        )?;
        let data = Arc::<DeviceData>::from(data);

        kernel::drm_device_register!(
            data.registrations().ok_or(ENXIO)?.as_pinned_mut(),
            data.clone(),
            0
        )?;

        dev_info!(data.dev, "Probed!\n");
        Ok(data.into())
    }
}

const MODE_CONFIG_FUNCS: bindings::drm_mode_config_funcs = bindings::drm_mode_config_funcs {
    fb_create: Some(bindings::drm_gem_fb_create_with_dirty),
    atomic_check: Some(bindings::drm_atomic_helper_check),
    atomic_commit: Some(bindings::drm_atomic_helper_commit),
    atomic_state_alloc: None,
    atomic_state_clear: None,
    atomic_state_free: None,
    get_format_info: None,
    mode_valid: None,
    output_poll_changed: None,
};

fn fixed_init(dev: &mut LcdConDevice) -> Result<()> {
    // TODO: migrate this to DT nodes (at least a panel node).
    {
        // Safety: scoped reference.
        let dev: &mut bindings::drm_device = unsafe { &mut (*dev.raw_mut()) };
        dev.mode_config.min_width = 0;
        dev.mode_config.min_height = 0;
        dev.mode_config.max_width = 1024;
        dev.mode_config.max_height = 1024;
        dev.mode_config.funcs = &MODE_CONFIG_FUNCS;
        dev.mode_config.preferred_depth = 24;
        //dev.mode_config.prefer_shadow = 0;
        //dev.mode_config.prefer_shadow_fbdev = 1;
    }

    Ok(())
}
