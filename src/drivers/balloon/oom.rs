use core::alloc::Layout;

use talc::{OomHandler, Talc};

use crate::drivers::pci::get_balloon_driver;

pub struct DeflateBalloonOnOom {
	/// Dummy field to prevent construction of the struct except through [`Self::new`]
	/// which is marked `unsafe`` and documents our requirements for safety.
	#[doc(hidden)]
	_private: (),
}

impl DeflateBalloonOnOom {
	/// Construct a new instance of the balloon deflating [`OomHandler`] for [`Talc`].
	///
	/// # Safety
	/// May only be used with the one instance of [`Talc`] registered as Hermit's
	/// global allocator.
	pub const unsafe fn new() -> Self {
		Self { _private: () }
	}
}

impl OomHandler for DeflateBalloonOnOom {
	fn handle_oom(talc: &mut Talc<Self>, layout: Layout) -> Result<(), ()> {
		warn!("<balloon:oom> Encountered OOM, attempting to deflate balloon to recover...");

		let Some(balloon_driver) = get_balloon_driver() else {
			return Err(());
		};

		let Some(mut ballon_driver_guard) = balloon_driver.try_lock() else {
			error!(
				"<balloon:oom> Driver was locked while attempting to allocate more than available. Unable to deflate balloon"
			);
			return Err(());
		};

		unsafe { ballon_driver_guard.deflate_for_oom(talc, (layout.size() / 4096) as u32) }
	}
}
