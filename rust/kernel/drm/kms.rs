use crate::{
    prelude::*,
    drm,
    types::PointerWrapper,
    container_of, error::{from_kernel_result, to_result},
};

use core::{
    cell::UnsafeCell,
    marker::{PhantomData, PhantomPinned},
};

#[vtable]
pub trait Connector {
    type Data: PointerWrapper + Sync + Send;

    fn get_modes(
        _data: <Self::Data as PointerWrapper>::Borrowed<'_>,
        _conn: *mut bindings::drm_connector,
    ) -> Result<i32>;

    fn atomic_update(
        _data: <Self::Data as PointerWrapper>::Borrowed<'_>,
        raw_plane: *mut bindings::drm_plane,
        raw_state: *mut bindings::drm_atomic_state,
    ) -> Result<()>;
}

struct ConnectorWithData<T: Connector> {
    data: T::Data,

    plane: bindings::drm_plane,
    crtc: bindings::drm_crtc,
    enc: bindings::drm_encoder,
    conn: bindings::drm_connector,
}

impl<T: Connector> ConnectorWithData<T> {
    fn new(data: T::Data) -> Self {
        Self {
            plane: bindings::drm_plane::default(),
            crtc: bindings::drm_crtc::default(),
            enc: bindings::drm_encoder::default(),
            conn: bindings::drm_connector::default(),
            data,
        }
    }
}

pub struct ConnectorRegistration<T: Connector> {
    c: UnsafeCell<ConnectorWithData<T>>,
    registered: bool,
    vtable_connector: Pin<Box<bindings::drm_connector_funcs>>,
    vtable_helper: Pin<Box<bindings::drm_connector_helper_funcs>>,
    vtable_encoder: Pin<Box<bindings::drm_encoder_funcs>>,
    vtable_plane: Pin<Box<bindings::drm_plane_funcs>>,
    vtable_plane_helper: Pin<Box<bindings::drm_plane_helper_funcs>>,
    vtable_crtc: Pin<Box<bindings::drm_crtc_funcs>>,
    vtable_crtc_helper: Pin<Box<bindings::drm_crtc_helper_funcs>>,
    _p :PhantomData<T>,
    _pin: PhantomPinned,
}

#[macro_export]
macro_rules! fourcc (
    ($a:expr, $b:expr, $c:expr, $d:expr) => (
        (($a as u32) << 0) |
        (($b as u32) << 8) |
        (($c as u32) << 16) |
        (($d as u32) << 24)
    )
);

impl<T: Connector> ConnectorRegistration<T> {
    const CONNECTOR_VTABLE: bindings::drm_connector_funcs = bindings::drm_connector_funcs {
        dpms: None,
        reset: Some(bindings::drm_atomic_helper_connector_reset),
        detect: None,
        force: None,
        fill_modes: Some(bindings::drm_helper_probe_single_connector_modes),
        set_property: None,
        late_register: None,
        early_unregister: None,
        destroy: Some(bindings::drm_connector_cleanup),
        atomic_duplicate_state: Some(bindings::drm_atomic_helper_connector_duplicate_state),
        atomic_destroy_state: Some(bindings::drm_atomic_helper_connector_destroy_state),
        atomic_set_property: None,
        atomic_get_property: None,
        atomic_print_state: None,
        oob_hotplug_event: None,
        debugfs_init: None,
    };
    const CONNECTOR_HELPER_VTABLE: bindings::drm_connector_helper_funcs = bindings::drm_connector_helper_funcs {
        atomic_best_encoder: None,
        atomic_check: None,
        atomic_commit: None,
        best_encoder: None,
        cleanup_writeback_job: None,
        detect_ctx: None,
        get_modes: Some(get_modes_callback::<T>),
        mode_valid: None,
        mode_valid_ctx: None,
        prepare_writeback_job: None,
    };
    const ENCODER_VTABLE: bindings::drm_encoder_funcs = bindings::drm_encoder_funcs {
        destroy: Some(bindings::drm_encoder_cleanup),
        early_unregister: None,
        late_register: None,
        reset: None,
    };
    const PLANE_VTABLE: bindings::drm_plane_funcs = bindings::drm_plane_funcs {
        atomic_destroy_state: Some(bindings::drm_gem_destroy_shadow_plane_state),
        atomic_duplicate_state: Some(bindings::drm_gem_duplicate_shadow_plane_state),
        atomic_get_property: None,
        atomic_print_state: None,
        atomic_set_property: None,
        destroy: Some(bindings::drm_plane_cleanup),
        disable_plane: Some(bindings::drm_atomic_helper_disable_plane),
        early_unregister: None,
        format_mod_supported: None,
        late_register: None,
        reset: Some(bindings::drm_gem_reset_shadow_plane),
        set_property: None,
        update_plane: Some(bindings::drm_atomic_helper_update_plane),
    };
    const PLANE_VTABLE_HELPER: bindings::drm_plane_helper_funcs = bindings::drm_plane_helper_funcs {
        atomic_async_check: None,
        atomic_async_update: None,
        atomic_check: Some(bindings::drm_plane_helper_atomic_check),
        atomic_disable: Some(atomic_disable_callback::<T>),
        atomic_update: Some(atomic_update_callback::<T>),
        begin_fb_access: Some(bindings::drm_gem_begin_shadow_fb_access),
        cleanup_fb: None,
        end_fb_access: Some(bindings::drm_gem_end_shadow_fb_access),
        prepare_fb: None,
    };
    const CRTC_VTABLE: bindings::drm_crtc_funcs = bindings::drm_crtc_funcs {
        atomic_destroy_state: Some(bindings::drm_atomic_helper_crtc_destroy_state),
        atomic_duplicate_state: Some(bindings::drm_atomic_helper_crtc_duplicate_state),
        atomic_get_property: None,
        atomic_print_state: None,
        atomic_set_property: None,
        cursor_move: None,
        cursor_set: None,
        cursor_set2: None,
        destroy: Some(bindings::drm_crtc_cleanup),
        disable_vblank: None,
        early_unregister: None,
        enable_vblank: None,
        gamma_set: None,
        get_crc_sources: None,
        get_vblank_counter: None,
        get_vblank_timestamp: None,
        late_register: None,
        page_flip: Some(bindings::drm_atomic_helper_page_flip),
        page_flip_target: None,
        reset: Some(bindings::drm_atomic_helper_crtc_reset),
        set_config: Some(bindings::drm_atomic_helper_set_config),
        set_crc_source: None,
        set_property: None,
        verify_crc_source: None,
    };
    const CRTC_VTABLE_HELPER: bindings::drm_crtc_helper_funcs = bindings::drm_crtc_helper_funcs {
        atomic_begin: None,
        atomic_check: Some(bindings::drm_crtc_helper_atomic_check),
        atomic_disable: None,
        atomic_enable: None,
        atomic_flush: None,
        commit: None,
        disable: None,
        dpms: None,
        get_scanout_position: None,
        mode_fixup: None,
        mode_set: None,
        mode_set_base: None,
        mode_set_base_atomic: None,
        mode_set_nofb: None,
        mode_valid: Some(mode_valid_callback::<T>),
        prepare: None,
    };

    pub fn new(
        data: T::Data,
    ) -> Result<Self> {
        let vtable_connector = Pin::new(Box::try_new(Self::CONNECTOR_VTABLE)?);
        let vtable_helper = Pin::new(Box::try_new(Self::CONNECTOR_HELPER_VTABLE)?);
        let vtable_encoder = Pin::new(Box::try_new(Self::ENCODER_VTABLE)?);
        let vtable_plane = Pin::new(Box::try_new(Self::PLANE_VTABLE)?);
        let vtable_plane_helper = Pin::new(Box::try_new(Self::PLANE_VTABLE_HELPER)?);
        let vtable_crtc = Pin::new(Box::try_new(Self::CRTC_VTABLE)?);
        let vtable_crtc_helper = Pin::new(Box::try_new(Self::CRTC_VTABLE_HELPER)?);

        Ok(Self {
            c: UnsafeCell::new(ConnectorWithData::<T>::new(data)),
            registered: false,
            vtable_connector,
            vtable_helper,
            vtable_encoder,
            vtable_plane,
            vtable_plane_helper,
            vtable_crtc,
            vtable_crtc_helper,
            _pin: PhantomPinned,
            _p: PhantomData,
        })
    }

    pub fn register<U: drm::drv::Driver>(
        self: Pin<&mut Self>,
        drmreg: &mut drm::drv::Registration<U>,
    ) -> Result<()> {
        if self.registered {
            // Already registered.
            return Err(EINVAL);
        }

        let this = unsafe { self.get_unchecked_mut() };
        let drm = drmreg.device_mut().raw_mut();
        let condata = this.c.get_mut();

        condata.enc.possible_crtcs = 0;

        let fourcc_supported: [u32; 2] = [
            fourcc!('X', 'R', '2', '4'),
            //fourcc!('A', 'R', '2', '4'),
            fourcc!('R', 'G', '2', '4'),
            //fourcc!('R', 'G', '1', '6'),
            //fourcc!('R', 'G', '2', '4'),
            //fourcc!('X', 'R', '3', '0'),
            //fourcc!('A', 'R', '3', '0'),
        ];

        let fourcc_native: [u32; 1] = [
            fourcc!('R', 'G', '2', '4'),
        ];

        let mut fourcc_combined: [u32; 10] = [0; 10];

        let format_modifiers: [u64; 2] = [
            0,
            (1u64 << 56) - 1,
        ];

        unsafe {
            // Primary plane.

            let nformats = bindings::drm_fb_build_fourcc_list(
                drm,
                &fourcc_native[0], 1,
                &fourcc_supported[0], 6,
                &mut fourcc_combined[0], 10,
            );

            to_result(bindings::drm_universal_plane_init(
                drm, &mut condata.plane, 0, &*this.vtable_plane,
                &fourcc_combined[0], nformats as u32,
                &format_modifiers[0],
                bindings::drm_plane_type_DRM_PLANE_TYPE_PRIMARY, core::ptr::null(),
            ))?;

            condata.plane.helper_private = &*this.vtable_plane_helper;
            bindings::drm_plane_enable_fb_damage_clips(
                &mut condata.plane,
            );

            // CRTC.

            to_result(bindings::drm_crtc_init_with_planes(
                drm, &mut condata.crtc,
                &mut condata.plane, core::ptr::null_mut(),
                &*this.vtable_crtc, core::ptr::null(),
            ))?;
            condata.crtc.helper_private = &*this.vtable_crtc_helper;

            // Encoder.            

            to_result(bindings::drm_encoder_init(
                drm,
                &mut condata.enc, 
                &*this.vtable_encoder,
                bindings::DRM_MODE_ENCODER_NONE as i32,
                core::ptr::null(),
            ))?;
            condata.enc.possible_crtcs = bindings::drm_crtc_mask(&condata.crtc);

            // Connector.

            // \equiv drm_connector_helper_add
            condata.conn.helper_private = &*this.vtable_helper;
            to_result(bindings::drm_connector_init(
                drm,
                &mut condata.conn,
                &*this.vtable_connector,
                bindings::DRM_MODE_CONNECTOR_DSI as i32
            ))?;

            to_result(bindings::drm_connector_attach_encoder(
                &mut condata.conn,
                &mut condata.enc, 
            ))?;
        }

        this.registered = true;
        Ok(())
    }
}

unsafe impl<T: Connector> Sync for ConnectorRegistration<T> {}

unsafe impl<T: Connector> Send for ConnectorRegistration<T> {}

impl<T: Connector> Drop for ConnectorRegistration<T> {
    fn drop(&mut self) {
        if !self.registered {
            return
        }
        pr_debug!("ConnectorRegistration::drop\n");

        unsafe { 
            let conn = &mut self.c.get_mut().conn;
            bindings::drm_connector_unregister(conn)
        };
    }
}

unsafe extern "C" fn get_modes_callback<T: Connector>(
    raw_connector: *mut bindings::drm_connector,
) -> core::ffi::c_int {
    from_kernel_result! {
        let raw_connector_ref: &bindings::drm_connector = unsafe { &*raw_connector };
        let condata: *const ConnectorWithData<T> = container_of!(raw_connector_ref, ConnectorWithData<T>, conn);
        let data = unsafe { T::Data::borrow(condata as *const core::ffi::c_void) };
        let v = T::get_modes(data, raw_connector)?;
        pr_info!("get_modes: {}\n", v);
        Ok(v as _)
    }
}

unsafe extern "C" fn atomic_update_callback<T: Connector>(
    raw_plane: *mut bindings::drm_plane,
    raw_state: *mut bindings::drm_atomic_state,
) {
        let raw_plane_ref: &bindings::drm_plane = unsafe { &*raw_plane };
        let condata: *const ConnectorWithData<T> = container_of!(raw_plane_ref, ConnectorWithData<T>, plane);
        let data = unsafe { T::Data::borrow(condata as *const core::ffi::c_void) };
        let v = T::atomic_update(data, raw_plane, raw_state);
        match v {
            Ok(_) => (),
            Err(e) => {
                pr_warn!("atomic update failed: {:?}\n", e);
            }
        }
}

unsafe extern "C" fn atomic_disable_callback<T: Connector>(
    raw_plane: *mut bindings::drm_plane,
    raw_state: *mut bindings::drm_atomic_state,
) {
    pr_info!("atomic disable\n");
}

unsafe extern "C" fn mode_valid_callback<T: Connector>(
    raw_crtc: *mut bindings::drm_crtc,
    raw_mode: *const bindings::drm_display_mode,
) -> bindings::drm_mode_status {
    pr_info!("mode valid\n");
    //return binding::drm_crtc_helper_mode_valid_fixed(crtc, raw_crtc, raw_mode, )
    return bindings::drm_mode_status_MODE_OK;
}
