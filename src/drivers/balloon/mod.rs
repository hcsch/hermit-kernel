use alloc::alloc::AllocError;
use alloc::boxed::Box;
use alloc::vec::Vec;
use core::fmt::Debug;
use core::mem::MaybeUninit;

use ahash::RandomState;
use hashbrown::HashMap;
use memory_addresses::VirtAddr;
use memory_addresses::arch::riscv64::BASE_PAGE_SIZE;
use pci_types::InterruptLine;
use smallvec::{SmallVec, smallvec};
use virtio::FeatureBits;
use virtio::balloon::{ConfigVolatileFieldAccess as _, F};
use volatile::VolatileRef;

use super::Driver;
use super::virtio::virtqueue::error::VirtqError;
use super::virtio::virtqueue::split::SplitVq;
use super::virtio::virtqueue::{
	AvailBufferToken, BufferElem, BufferType, VirtQueue, Virtq as _, VqIndex, VqSize,
};
use crate::VIRTIO_MAX_QUEUE_SIZE;
use crate::arch::mm::paging::virtual_to_physical;
#[cfg(not(feature = "pci"))]
use crate::drivers::virtio::transport::mmio::{ComCfg, IsrStatus, NotifCfg};
#[cfg(feature = "pci")]
use crate::drivers::virtio::transport::pci::{ComCfg, IsrStatus, NotifCfg};
use crate::mm::device_alloc::DeviceAlloc;

#[cfg(feature = "pci")]
mod pci;

/// A wrapper struct for the raw configuration structure.
/// Handling the right access to fields, as some are read-only
/// for the driver.
#[derive(Debug)]
struct BalloonDevCfg {
	pub raw: VolatileRef<'static, virtio::balloon::Config>,
	pub dev_id: u16,
	pub features: virtio::balloon::F,
}

impl BalloonDevCfg {
	fn num_pages(&self) -> u32 {
		self.raw.as_ptr().num_pages().read().into()
	}

	fn actual(&mut self) -> u32 {
		self.raw.as_ptr().actual().read().into()
	}

	fn set_actual(&mut self, num_pages: u32) {
		self.raw.as_mut_ptr().actual().write(num_pages.into());
	}
}

/// Virtio traditional memory balloon driver struct.
pub(crate) struct VirtioBalloonDriver {
	dev_cfg: BalloonDevCfg,
	com_cfg: ComCfg,
	isr_stat: IsrStatus,
	notif_cfg: NotifCfg,
	irq: InterruptLine,

	inflateq: BalloonVq,
	deflateq: BalloonVq,

	num_in_balloon: u32,
	num_pending_inflation: u32,
	num_pending_deflation: u32,
	num_targeted: u32,

	balloon_map: BalloonMap,
}

// Backend-independent interface for VIRTIO traditional memory balloon driver
impl VirtioBalloonDriver {
	/// Negotiates a subset of features, understood and wanted by both the OS
	/// and the device.
	fn negotiate_features(
		&mut self,
		driver_features: virtio::balloon::F,
	) -> Result<(), VirtioBalloonError> {
		let device_features = virtio::balloon::F::from(self.com_cfg.dev_features());

		if driver_features.requirements_satisfied() {
			debug!(
				"<balloon> Feature set requested by device driver are in conformance with specification."
			);
		} else {
			return Err(VirtioBalloonError::FeatureRequirementsNotMet { driver_features });
		}

		if device_features.contains(driver_features) {
			// If device supports superset of our driver's current target feature set,
			// write this feature set to common config
			self.com_cfg.set_drv_features(driver_features.into());
			Ok(())
		} else {
			Err(VirtioBalloonError::IncompatibleFeatureSets {
				driver_features,
				device_features,
			})
		}
	}

	/// Initializes the device in adherence to specification.
	///
	/// See Virtio specification v1.2. - 3.1.1
	///                      and v1.2. - 5.5.5
	pub fn init_dev(&mut self) -> Result<(), VirtioBalloonError> {
		// Reset
		self.com_cfg.reset_dev();

		// Indicate device, that OS noticed it
		self.com_cfg.ack_dev();

		// Indicate device, that driver is able to handle it
		self.com_cfg.set_drv();

		// TODO: add support for free page hinting and reporting

		let features = F::VERSION_1;
		self.negotiate_features(features)?;

		// Indicates the device, that the current feature set is final for the driver
		// and will not be changed.
		self.com_cfg.features_ok();

		// Checks if the device has accepted final set. This finishes feature negotiation.
		if self.com_cfg.check_features() {
			info!(
				"<balloon> Features have been negotiated between device {:x} and driver: {features:?}",
				self.dev_cfg.dev_id
			);
			// Set feature set in device config fur future use.
			self.dev_cfg.features = features;
		} else {
			return Err(VirtioBalloonError::FeatureNegotiationFailed {
				device_id: self.dev_cfg.dev_id,
			});
		}

		self.inflateq.init(VirtQueue::Split(
			SplitVq::new(
				&mut self.com_cfg,
				&self.notif_cfg,
				VqSize::from(VIRTIO_MAX_QUEUE_SIZE),
				VqIndex::from(0u16),
				self.dev_cfg.features.into(),
			)
			.expect("Failed to create SplitVq for inflateq due to invalid parameters (bug)"),
		));

		self.deflateq.init(VirtQueue::Split(
			SplitVq::new(
				&mut self.com_cfg,
				&self.notif_cfg,
				VqSize::from(VIRTIO_MAX_QUEUE_SIZE),
				VqIndex::from(1u16),
				self.dev_cfg.features.into(),
			)
			.expect("Failed to create SplitVq for deflateq due to invalid parameters (bug)"),
		));

		// At this point the device is "live"
		self.com_cfg.drv_ok();

		info!("<balloon> Finished initialization");

		self.adjust_balloon_size();

		Ok(())
	}

	fn num_pages_changed(&mut self) -> Option<u32> {
		let new_num_pages = self.dev_cfg.num_pages();

		if new_num_pages == self.num_targeted {
			None
		} else {
			self.num_targeted = new_num_pages;
			Some(new_num_pages)
		}
	}

	pub(crate) fn poll_events(&mut self) {
		trace!("<balloon> Driver is being polled...");
		self.adjust_balloon_size();

		let mut changed = false;

		{
			let num_new_acknowledged_deflated = self.deflateq.discard_new_used();

			if num_new_acknowledged_deflated > 0 {
				info!("<balloon> Deflation acknowledged for {num_new_acknowledged_deflated} pages");

				self.num_pending_deflation -= num_new_acknowledged_deflated as u32;
				self.num_in_balloon -= num_new_acknowledged_deflated as u32;
				changed = true;
			}
		}

		{
			let num_new_acknowledged_inflated = self.inflateq.discard_new_used();

			if num_new_acknowledged_inflated > 0 {
				info!("<balloon> Inflation acknowledged for {num_new_acknowledged_inflated} pages");

				self.num_pending_inflation -= num_new_acknowledged_inflated as u32;
				self.num_in_balloon += num_new_acknowledged_inflated as u32;
				changed = true;
			}
		}

		if changed {
			info!(
				"<balloon> Setting new actual balloon size of {} pages",
				self.num_in_balloon
			);
			self.dev_cfg.set_actual(self.num_in_balloon);
		}
	}

	fn adjust_balloon_size(&mut self) {
		trace!("<balloon> Adjusting balloon size if necessary");
		let Some(new_target_num_pages) = self.num_pages_changed() else {
			trace!("<balloon> No balloon size change requested");
			return;
		};

		if new_target_num_pages < self.num_in_balloon - self.num_pending_deflation {
			let num_to_deflate =
				(self.num_in_balloon - self.num_pending_deflation) - new_target_num_pages;

			info!(
				"<balloon> Size change requested: deflate of {num_to_deflate}, from {} (with {} pending deflation) to {new_target_num_pages}",
				self.num_in_balloon, self.num_pending_deflation
			);

			let mut page_indices = Vec::new_in(DeviceAlloc);

			self.balloon_map
				.get_indices_for_deallocation()
				.take(num_to_deflate as usize)
				.collect_into(&mut page_indices);

			// SAFETY: We ensure with our balloon map that we only deflate pages
			//         that we have previously inflated into the balloon.
			//         Deflating also does not give the host ownership over
			//         additional memory of ours. Merely sending the indices into
			//         the queue does not yet deallocate the pages on our side.
			unsafe {
				self.deflateq
					.send_pages(page_indices.iter().copied(), false)
					.expect("Failed to send pages into the deflateq");
			}

			// SAFETY: For now we don't have [`F::MUST_TELL_HOST`] support, so
			//         we can deallocate all pages immediately once we have sent
			//         them into the deflateq. See VIRTIO v1.2 5.5.6 3.
			unsafe {
				self.balloon_map.deallocate_pages(page_indices.into_iter());
			}

			self.num_pending_deflation += num_to_deflate;
		} else if new_target_num_pages > self.num_in_balloon + self.num_pending_inflation {
			let num_to_inflate =
				new_target_num_pages - (self.num_in_balloon + self.num_pending_inflation);

			info!(
				"<balloon> Size change requested: inflate of {num_to_inflate}, from {} (with {} pending inflation) to {new_target_num_pages}",
				self.num_in_balloon, self.num_pending_inflation
			);

			let page_indices = self
				.balloon_map
				.allocate_pages()
				.take(num_to_inflate as usize);

			// SAFETY: We ensure with our balloon map that we only inflate pages
			//         that we have allocated via the global allocator. Inflating
			//         a page hands ownership over to the host, but we ensure that
			//         the contents of the page are not used until the page has
			//         been deflated again by keeping our allocation in the balloon map.
			unsafe {
				self.inflateq
					.send_pages(page_indices, false)
					.expect("Failed to send pages into the inflateq");
			}

			self.num_pending_inflation += num_to_inflate;
		}
	}
}

impl Driver for VirtioBalloonDriver {
	fn get_interrupt_number(&self) -> InterruptLine {
		self.irq
	}

	fn get_name(&self) -> &'static str {
		"virtio-balloon"
	}
}

struct BalloonVq {
	vq: Option<VirtQueue>,
}

impl BalloonVq {
	pub fn new() -> Self {
		Self { vq: None }
	}

	fn init(&mut self, vq: VirtQueue) {
		self.vq = Some(vq);
	}

	pub fn enable_notifs(&mut self) {
		let Some(vq) = &mut self.vq else {
			debug!("<balloon> BalloonVq::enable_notifs called on uninitialized vq");
			return;
		};

		vq.enable_notifs();
	}

	pub fn disable_notifs(&mut self) {
		let Some(vq) = &mut self.vq else {
			debug!("<balloon> BalloonVq::disable_notifs called on uninitialized vq");
			return;
		};

		vq.disable_notifs();
	}

	fn used_send_buff_to_page_indices(
		used_send_buff: SmallVec<[BufferElem; 2]>,
	) -> impl Iterator<Item = u32> {
		used_send_buff.into_iter().flat_map(|buffer_elem| {
			match buffer_elem {
					BufferElem::Sized(_any) =>
						panic!("Unexpected used `BufferElem::Sized` encountered, BalloonVq should only have sent `BufferElem::Vector`s"),
					BufferElem::Vector(items) => {
						assert!(items.len() % 4 == 0, "Unexpected size of used `BufferElem::Vector`, BalloonVq should only have sent lengths that are multiples of 4");

						items
						.into_iter()
						.array_chunks()
						.map(|bytes: [u8; 4]| u32::from_le_bytes(bytes))
					},
				}
		})
	}

	/// Receive all new page indices marked used by the host.
	/// These are the page indices we have previously sent into the queue in available buffers.
	pub fn recv_new_used(&mut self) -> impl Iterator<Item = u32> {
		let Some(vq) = &mut self.vq else {
			debug!("<balloon> BalloonVq::try_recv_new_used called on uninitialized vq");
			panic!("BalloonVq must be initialized before calling try_recv_new_used");
		};

		let mut current_used_page_indices_iter = None;

		core::iter::from_fn(move || {
			match current_used_page_indices_iter.as_mut() {
				// Must appear in the code before `current_used_page_indices_iter.next()` for an existing iterator (see below).
				// Otherwise Rust is unable to infer the contents of the `Option` (and the type can't be named explicitly).
				// If this inference failure gets fixed, this match can be converted to an `if let Some(iter) = ...`
				None => match vq.try_recv() {
					Ok(new_used) => {
						let mut new_used_page_indices_iter =
							Self::used_send_buff_to_page_indices(new_used.send_buff);

						let used = new_used_page_indices_iter.next()?;

						current_used_page_indices_iter = Some(new_used_page_indices_iter);

						Some(used)
					}

					Err(VirtqError::NoNewUsed) => None,

					Err(error) => {
						panic!(
							"Failed to receive new used virtqueue descriptors with unexpected error: {error:?}"
						)
					}
				},

				Some(current_used_page_indices_iter) => current_used_page_indices_iter.next(),
			}
		})
	}

	/// Discard all new page indices marked used by the host.
	/// These are the page indices we have previously sent into the queue in available buffers.
	pub fn discard_new_used(&mut self) -> usize {
		let Some(vq) = &mut self.vq else {
			debug!("<balloon> BalloonVq::discard_new_used called on uninitialized vq");
			panic!("BalloonVq must be initialized before calling discard_new_used");
		};

		let mut num_discarded = 0;

		loop {
			match vq.try_recv() {
				Ok(new_used) => {
					let num_page_indices =
						Self::used_send_buff_to_page_indices(new_used.send_buff).count();
					trace!(
						"<balloon> Discarded used buffer received from host with {num_page_indices} page indices"
					);
					num_discarded += num_page_indices;
				}

				Err(VirtqError::NoNewUsed) => break,

				Err(error) => {
					panic!(
						"Failed to receive new used virtqueue descriptors with unexpected error: {error:?}"
					)
				}
			}
		}

		num_discarded
	}

	/// Send specified pages into the balloon virtqueue.
	///
	/// To ensure that there is enough space in the queue, call [`Self::recv_new_used`]
	/// or [`Self::discard_new_used`] before sending.
	///
	/// The page indices are of 4096B (4K) pages and are submitted as `u32`s,
	/// i.e. only pages up to (2³² - 1) * 4096 B = 16 TiB in our physical memory
	/// can be submitted here.
	///
	/// # Safety
	/// The caller must ensure that the pages of which the indices are sent into
	/// the inflate queue are not used by the kernel or the application until they
	/// have been deflated again via the deflate queue
	/// (with or without acknowledgement by the host depending on [`F::MUST_TELL_HOST`]).
	pub unsafe fn send_pages(
		&mut self,
		page_indices: impl Iterator<Item = u32>,
		notif: bool,
	) -> Result<(), VirtqError> {
		let Some(vq) = &mut self.vq else {
			error!("<balloon> BalloonVq::send_pages called on uninitialized vq");
			panic!("BalloonVq must be initialized before calling send_pages");
		};

		let mut page_indices_bytes = Vec::new_in(DeviceAlloc);
		page_indices
			// Not specified as little-endian by the spec? Linux does it little-endian for VIRTIO 1.0
			.flat_map(|index| index.to_le_bytes())
			.collect_into(&mut page_indices_bytes);

		let buff_tkn = AvailBufferToken::new(
			smallvec![BufferElem::Vector(page_indices_bytes)],
			smallvec![],
		)
		.expect("We have specified a send_buff so AvailBufferToken::new should succeed");

		vq.dispatch(buff_tkn, notif, BufferType::Direct)?;

		Ok(())
	}
}

/// Errors that can occur during the lifetime and initialization of the [`VirtioBalloonDriver`](`super::VirtioBalloonDriver`)
#[derive(Debug, Copy, Clone)]
pub enum VirtioBalloonError {
	#[cfg(feature = "pci")]
	NoDevCfg { device_id: u16 },
	/// The device did not accept the negotiated features at the last step of negotiation.
	FeatureNegotiationFailed { device_id: u16 },
	/// Set of features requested by driver does not adhere to the requirements of features
	/// indicated by the specification
	FeatureRequirementsNotMet { driver_features: virtio::balloon::F },
	/// The first u64 contains the feature bits wanted by the driver.
	/// but which are incompatible with the device feature set, second u64.
	IncompatibleFeatureSets {
		driver_features: virtio::balloon::F,
		device_features: virtio::balloon::F,
	},
}

#[derive(Debug)]
struct BalloonMap {
	/// A map from balloon page (4K) indices to [`BalloonPage`] objects that
	/// represent the allocation of that page with the global allocator.
	page_map: HashMap<u32, BalloonPage, RandomState>,
}

impl BalloonMap {
	pub fn new() -> Self {
		Self {
			page_map: HashMap::with_hasher(RandomState::new()),
		}
	}

	fn allocate_page(&mut self) -> Result<u32, AllocError> {
		let page = BalloonPage::allocate()?;
		let page_index = page.phys_page_index();

		trace!("<balloon> Allocated ballon page with index {page_index}");

		if let Some(old_balloon_page) = self.page_map.insert(page_index, page) {
			error!(
				"<balloon> Allocated to same physical page twice concurrently: {old_balloon_page:?}, page_index = 0x{page_index:x}"
			);
			panic!("Physical page indices for concurrently allocated balloon pages must be unique");
		}

		Ok(page_index)
	}

	pub fn allocate_pages(&mut self) -> impl Iterator<Item = u32> {
		core::iter::from_fn(|| match self.allocate_page() {
			Ok(page_index) => Some(page_index),
			Err(alloc_error) => {
				warn!(
					"<balloon> Failed to allocate new pages to fill the balloon with, continuing with as many as possible: {alloc_error}"
				);

				None
			}
		})
	}

	pub fn get_indices_for_deallocation(&self) -> impl Iterator<Item = u32> + Clone {
		self.page_map.keys().copied()
	}

	pub unsafe fn deallocate_pages(&mut self, page_indices: impl Iterator<Item = u32>) {
		for page_index in page_indices {
			let Some(_page) = self.page_map.remove(&page_index) else {
				error!(
					"<balloon> Attempted to deallocate balloon page with index for which no page was stored: page_index = 0x{page_index:x}"
				);
				panic!("Deallocation of invalid balloon page");
			};

			trace!("<balloon> Deallocated ballon page with index {page_index}");
		}
	}
}

/// Represents the data inside a page and ensures alignment to 4K page boundaries
/// as 4K pages are what the balloon device works with, exclusively.
#[repr(align(4096))]
struct BalloonPageData(
	#[expect(
		dead_code,
		reason = "contents are owned by the host and should not be read by us"
	)]
	[u8; BASE_PAGE_SIZE],
);

impl Debug for BalloonPageData {
	fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
		f.debug_tuple("BalloonPageData").finish_non_exhaustive()
	}
}

/// Represents a 4K page allocated for the balloon.
///
/// This ensures via encapsulation, that inflated pages, pages released to the host,
/// are not read from / written to while they are in the balloon.
#[derive(Debug)]
struct BalloonPage {
	page_allocation: Box<MaybeUninit<BalloonPageData>>,
}

impl BalloonPage {
	#[must_use = "this returns an object representing the allocation, unless stored, it is deallocated"]
	pub fn allocate() -> Result<Self, AllocError> {
		let page_allocation = Box::try_new_uninit()?;

		Ok(Self { page_allocation })
	}

	/// Compute the physical page index of the allocated page.
	/// TODO: safety docs?
	pub fn phys_page_index(&self) -> u32 {
		let virtual_addr = VirtAddr::from(self.page_allocation.as_ptr().expose_provenance());

		let physical_addr = virtual_to_physical(virtual_addr)
			.expect("We only deal with virtual addresses that are mapped");

		physical_addr
			.as_u64()
			.checked_div(BASE_PAGE_SIZE as u64)
			.expect("Pointer to start of allocated balloon page should be base page size aligned")
			as u32
	}
}
