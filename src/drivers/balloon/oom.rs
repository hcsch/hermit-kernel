use core::alloc::Layout;

use talc::{OomHandler, Talc};

use crate::drivers::pci::get_balloon_driver;

pub struct DeflateBalloonOnOom;

impl OomHandler for DeflateBalloonOnOom {
	fn handle_oom(talc: &mut Talc<Self>, layout: Layout) -> Result<(), ()> {
		warn!(
			"Encountered OOM, attempting to deflate VIRTIO traditional memory balloon device to recover..."
		);

		let Some(balloon_driver) = get_balloon_driver() else {
			return Err(());
		};

		let Some(mut ballon_driver_guard) = balloon_driver.try_lock() else {
			error!(
				"VIRTIO traditional memory balloon driver was locked while attempting to allocate more than available. Unable to deflate balloon"
			);
			return Err(());
		};

		ballon_driver_guard.deflate_for_oom(talc, (layout.size() / 4096) as u32)
	}
}
