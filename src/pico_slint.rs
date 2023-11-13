use alloc::rc::Rc;
use rp_pico::hal::Timer;
use slint::platform::{software_renderer, Platform};

pub(crate) struct PicoPlatform {
    pub window: Rc<software_renderer::MinimalSoftwareWindow>,
    pub timer: Timer,
}

impl Platform for PicoPlatform {
    fn create_window_adapter(&self) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        Ok(self.window.clone())
    }
    fn duration_since_start(&self) -> core::time::Duration {
        core::time::Duration::from_micros(self.timer.get_counter().duration_since_epoch().to_micros())
    }
}
