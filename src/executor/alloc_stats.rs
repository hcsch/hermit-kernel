use core::task::Poll;
use core::time::Duration;

use crate::executor::spawn;
use crate::mm::ALLOCATOR;
use crate::processor::{get_frequency, get_timestamp};

struct NaiveInterval {
	duration: Duration,
	next_timestamp: u64,
}

impl NaiveInterval {
	pub fn new(duration: Duration) -> Self {
		Self {
			duration,
			next_timestamp: get_timestamp(),
		}
	}

	pub fn tick(&mut self) -> impl Future<Output = ()> {
		core::future::poll_fn(|_cx| {
			debug!("<alloc-stats> got polled");
			if get_timestamp() < self.next_timestamp {
				Poll::Pending
			} else {
				self.next_timestamp +=
					self.duration.as_micros() as u64 * u64::from(get_frequency());
				Poll::Ready(())
			}
		})
	}
}

async fn print_alloc_stats() {
	// let mut interval = NaiveInterval::new(Duration::from_secs(5));
	// loop {
	// 	interval.tick().await;
	core::future::poll_fn::<(), _>(|_cx| {
		// let Some(talc) = ALLOCATOR.inner().try_lock() else {
		// 	continue;
		// };

		let talc = ALLOCATOR.inner().lock();

		debug!("<alloc-stats>\n{}", talc.get_counters());

		Poll::Pending
	})
	.await;
	// }
}

pub(crate) fn init() {
	info!("Spawning allocation stats printing task");
	spawn(print_alloc_stats());
}
