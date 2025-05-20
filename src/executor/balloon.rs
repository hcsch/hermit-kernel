use core::future;
use core::task::Poll;

use crate::drivers::pci;
use crate::executor::spawn;

async fn balloon_run() {
	future::poll_fn(|_cx| {
		if let Some(driver) = pci::get_balloon_driver() {
			let mut driver_guard = driver.lock();

			driver_guard.poll_events();

			Poll::Pending
		} else {
			Poll::Ready(())
		}
	})
	.await;
}

pub(crate) fn init() {
	info!("Try to initialize balloon interface!");

	spawn(balloon_run());
}
