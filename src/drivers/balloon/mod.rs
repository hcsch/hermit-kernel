use alloc::boxed::Box;
use alloc::collections::vec_deque::VecDeque;
use alloc::vec::Vec;
use core::alloc::Layout;
use core::fmt::Debug;
use core::num::{NonZeroU32, NonZeroUsize};
use core::ptr::NonNull;

use memory_addresses::VirtAddr;
use pci_types::InterruptLine;
use smallvec::{SmallVec, smallvec};
use talc::Talc;
use virtio::FeatureBits;
use virtio::balloon::{ConfigVolatileFieldAccess as _, F};
use volatile::VolatileRef;

use super::Driver;
use super::virtio::virtqueue::error::VirtqError;
use super::virtio::virtqueue::split::SplitVq;
use super::virtio::virtqueue::{AvailBufferToken, BufferElem, BufferType, Virtq, VqIndex, VqSize};
use crate::VIRTIO_MAX_QUEUE_SIZE;
#[cfg(not(feature = "pci"))]
use crate::drivers::virtio::transport::mmio::{ComCfg, IsrStatus, NotifCfg};
#[cfg(feature = "pci")]
use crate::drivers::virtio::transport::pci::{ComCfg, IsrStatus, NotifCfg};
use crate::mm::allocator::HermitOomHandler;
use crate::mm::device_alloc::DeviceAlloc;
use crate::mm::{ALLOCATOR, virtual_to_physical};

#[cfg(feature = "pci")]
pub mod oom;
#[cfg(feature = "pci")]
mod pci;

const BALLOON_PAGE_SIZE: usize = 4096;

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
				"Feature set requested by VIRTIO traditional memory balloon device driver are in conformance with specification."
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
				"Features have been negotiated between VIRTIO traditional memory balloon device {:x} and driver: {features:?}",
				self.dev_cfg.dev_id
			);
			// Set feature set in device config fur future use.
			self.dev_cfg.features = features;
		} else {
			return Err(VirtioBalloonError::FeatureNegotiationFailed {
				device_id: self.dev_cfg.dev_id,
			});
		}

		self.inflateq.init(Box::new(
			SplitVq::new(
				&mut self.com_cfg,
				&self.notif_cfg,
				VqSize::from(VIRTIO_MAX_QUEUE_SIZE),
				VqIndex::from(0u16),
				self.dev_cfg.features.into(),
			)
			.expect("Failed to create SplitVq for inflateq due to invalid parameters (bug)"),
		));

		self.deflateq.init(Box::new(
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

		info!("Finished initialization of VIRTIO traditional memory balloon device");

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
		trace!("Balloon device received poll");
		self.adjust_balloon_size();

		let mut changed = false;

		{
			let num_new_acknowledged_deflated = self.deflateq.discard_new_used();

			if num_new_acknowledged_deflated > 0 {
				info!("Balloon deflation acknowledged for {num_new_acknowledged_deflated} pages");

				self.num_pending_deflation -= num_new_acknowledged_deflated as u32;
				self.num_in_balloon -= num_new_acknowledged_deflated as u32;
				changed = true;
			}
		}

		{
			let num_new_acknowledged_inflated = self.inflateq.discard_new_used();

			if num_new_acknowledged_inflated > 0 {
				info!("Balloon inflation acknowledged for {num_new_acknowledged_inflated} pages");

				self.num_pending_inflation -= num_new_acknowledged_inflated as u32;
				self.num_in_balloon += num_new_acknowledged_inflated as u32;
				changed = true;
			}
		}

		if changed {
			info!(
				"Setting new actual balloon size of {} pages",
				self.num_in_balloon
			);
			self.dev_cfg.set_actual(self.num_in_balloon);
		}
	}

	fn deflate(&mut self, talc: &mut Talc<HermitOomHandler>, num_pages_to_deflate: u32) {
		assert!(
			num_pages_to_deflate <= self.num_in_balloon - self.num_pending_deflation,
			"Can't deflate more pages than there are in the balloon"
		);

		let page_indices = self
			.balloon_map
			.mark_pages_for_deflation(num_pages_to_deflate);

		for chunk_page_indices in &page_indices {
			// SAFETY: We ensure with our balloon map that we only deflate pages
			//         that we have previously inflated into the balloon.
			//         Deflating also does not give the host ownership over
			//         additional memory of ours. Merely sending the indices into
			//         the queue does not yet deallocate the pages on our side.
			unsafe {
				self.deflateq
					.send_pages(chunk_page_indices.iter().copied(), false)
					.expect("Failed to send pages into the deflateq");
			}
		}

		// SAFETY: For now we don't have [`F::MUST_TELL_HOST`] support, so
		//         we can deallocate all pages immediately once we have sent
		//         them into the deflateq. See VIRTIO v1.2 5.5.6 3.
		unsafe {
			self.balloon_map.shrink_chunks(talc, page_indices);
		}

		self.num_pending_deflation += num_pages_to_deflate;
	}

	fn inflate(&mut self, talc: &mut Talc<HermitOomHandler>, num_pages_to_inflate: u32) {
		let page_indices = self.balloon_map.allocate_chunks(talc, num_pages_to_inflate);

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

		self.num_pending_inflation += num_pages_to_inflate;
	}

	fn adjust_balloon_size(&mut self) {
		let Some(new_target_num_pages) = self.num_pages_changed() else {
			// No change requested
			return;
		};

		if new_target_num_pages < self.num_in_balloon - self.num_pending_deflation {
			let num_to_deflate =
				(self.num_in_balloon - self.num_pending_deflation) - new_target_num_pages;

			info!(
				"Balloon size change requested: deflate of {num_to_deflate}, from {} (with {} pending deflation) to {new_target_num_pages}",
				self.num_in_balloon, self.num_pending_deflation
			);

			self.deflate(&mut ALLOCATOR.inner().lock(), num_to_deflate);
		} else if new_target_num_pages > self.num_in_balloon + self.num_pending_inflation {
			let num_to_inflate =
				new_target_num_pages - (self.num_in_balloon + self.num_pending_inflation);

			info!(
				"Balloon size change requested: inflate of {num_to_inflate}, from {} (with {} pending inflation) to {new_target_num_pages}",
				self.num_in_balloon, self.num_pending_inflation
			);

			self.inflate(&mut ALLOCATOR.inner().lock(), num_to_inflate);
		}
	}

	pub fn disable_interrupts(&mut self) {
		self.inflateq.disable_notifs();
		self.deflateq.disable_notifs();
	}

	pub fn enable_interrupts(&mut self) {
		self.inflateq.enable_notifs();
		self.deflateq.enable_notifs();
	}

	pub fn num_deflatable_for_oom(&self) -> u32 {
		self.num_in_balloon
			.saturating_sub(self.dev_cfg.num_pages())
			.saturating_sub(self.num_pending_deflation)
	}

	pub fn deflate_for_oom(
		&mut self,
		talc: &mut Talc<HermitOomHandler>,
		failed_alloc_num_pages: u32,
	) -> Result<(), ()> {
		// We don't really know how much space Talc has left.
		// The allocation might have failed only by a short margin, or by a lot.

		let num_deflatable = self.num_deflatable_for_oom();

		if num_deflatable > 0 {
			// Deflate as many pages as we can up to the amount needed for the allocation.
			// We don't have to wait for host acknowledgement, because for now
			// we don't support [`F::MUST_TELL_HOST`].

			let num_to_deflate = num_deflatable.min(failed_alloc_num_pages);

			info!(
				"Deflating {num_to_deflate} pages in an attempt to recover from an OOM condition"
			);

			self.deflate(talc, num_to_deflate);
			Ok(())
		} else {
			error!("Unable to deflate balloon further");
			// Nothing more we can do
			Err(())
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
	vq: Option<Box<dyn Virtq>>,
}

impl BalloonVq {
	pub fn new() -> Self {
		Self { vq: None }
	}

	fn init(&mut self, vq: Box<dyn Virtq>) {
		self.vq = Some(vq);
	}

	pub fn enable_notifs(&mut self) {
		let Some(vq) = &mut self.vq else {
			debug!("BalloonVq::enable_notifs called on uninitialized vq");
			return;
		};

		vq.enable_notifs();
	}

	pub fn disable_notifs(&mut self) {
		let Some(vq) = &mut self.vq else {
			debug!("BalloonVq::disable_notifs called on uninitialized vq");
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
			debug!("BalloonVq::try_recv_new_used called on uninitialized vq");
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
			debug!("BalloonVq::discard_new_used called on uninitialized vq");
			panic!("BalloonVq must be initialized before calling discard_new_used");
		};

		let mut num_discarded = 0;

		loop {
			match vq.try_recv() {
				Ok(new_used) => {
					let num_page_indices =
						Self::used_send_buff_to_page_indices(new_used.send_buff).count();
					trace!(
						"Discarded used buffer received from host with {num_page_indices} page indices"
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
	pub unsafe fn send_pages<I: IntoIterator<Item = u32>>(
		&mut self,
		page_indices: I,
		notif: bool,
	) -> Result<(), VirtqError> {
		let Some(vq) = &mut self.vq else {
			error!("BalloonVq::send_pages called on uninitialized vq");
			panic!("BalloonVq must be initialized before calling send_pages");
		};

		let mut page_indices_bytes = Vec::new_in(DeviceAlloc);
		page_indices
			.into_iter()
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
	/// A stack of chunks of pages allocated for the balloon.
	/// TODO: Ideally in order of decreasing size?
	chunks: Vec<BalloonAllocation, DeviceAlloc>,
}

impl BalloonMap {
	pub fn new() -> Self {
		Self {
			chunks: Vec::new_in(DeviceAlloc),
		}
	}

	fn allocate_chunk(
		&mut self,
		talc: &mut Talc<HermitOomHandler>,
		num_pages: NonZeroU32,
	) -> Result<impl Iterator<Item = u32>, ()> {
		let page = BalloonAllocation::try_allocate(talc, num_pages)?;

		self.chunks.push(page);

		// Only now get the iterator over physical indices, so it lives as long
		// as chunks, instead of referencing the now moved page variable.
		let mut page_indices = self
			.chunks
			.last()
			.expect("We just pushed one chunk")
			.phys_page_indices()
			.peekable();
		let first_page_index = *page_indices
			.peek()
			.expect("If the allocation didn't fail, we should have at least one page index");

		trace!(
			"Allocated ballon page chunk starting at page index {first_page_index} with {num_pages} pages"
		);

		Ok(page_indices)
	}

	pub fn allocate_chunks(
		&mut self,
		talc: &mut Talc<HermitOomHandler>,
		target_num_pages: u32,
	) -> Vec<u32, DeviceAlloc> {
		let mut page_indices = Vec::new_in(DeviceAlloc);
		let mut current_pow_2 = target_num_pages.ilog2();
		let mut num_remaining = target_num_pages;

		loop {
			let Some(current_pow_2_nz) = NonZeroU32::new(current_pow_2) else {
				if num_remaining > 0 {
					warn!(
						"Failed to allocate new pages to fill the balloon with, continuing with as many as possible"
					);
				}
				break;
			};

			match self.allocate_chunk(talc, current_pow_2_nz) {
				Ok(chunk_page_indices) => {
					num_remaining -= current_pow_2;
					page_indices.extend(chunk_page_indices);
				}
				Err(()) => {
					let old_pow_2 = current_pow_2;
					current_pow_2 = old_pow_2 >> 1;
					trace!(
						"Failed to allocate new chunk of {old_pow_2} pages to fill the balloon with, reducing chunk size to {current_pow_2}"
					);

					continue;
				}
			}
		}

		page_indices
	}

	pub fn mark_pages_for_deflation(
		&mut self,
		target_num_pages: u32,
	) -> Vec<Vec<u32, DeviceAlloc>, DeviceAlloc> {
		let mut num_remaining = target_num_pages;
		let mut per_chunk_page_indices = Vec::new_in(DeviceAlloc);

		// Go through chunks from small/recent to large/old, mark as much as requested if possible.
		// Collect the page indices of marked pages for submission to the deflate queue.

		for chunk in self.chunks.iter_mut().rev() {
			let num_to_mark = chunk.num_available_for_deflation().min(num_remaining);

			let mut page_indices = Vec::new_in(DeviceAlloc);
			chunk
				.mark_queued_for_deflation(num_to_mark)
				.collect_into(&mut page_indices);

			per_chunk_page_indices.push(page_indices);

			num_remaining -= num_to_mark;

			if num_remaining == 0 {
				break;
			}
		}

		if num_remaining > 0 {
			warn!(
				"Attempted to deflate more pages than were in the balloon: no more allocation chunks left to deflate"
			);
		}

		per_chunk_page_indices
	}

	pub unsafe fn shrink_chunks(
		&mut self,
		talc: &mut Talc<HermitOomHandler>,
		acknowledged_deflated_pages: Vec<Vec<u32, DeviceAlloc>, DeviceAlloc>,
	) {
		let mut next_chunk_index = self.chunks.len().checked_sub(1);

		for chunk_deflated_pages in acknowledged_deflated_pages.into_iter() {
			let Some(mut current_chunk_index) = next_chunk_index else {
				error!(
					"Was unable to use all page indices acknowledged for deflation to shrink allocation chunks"
				);
				return;
			};

			loop {
				if self.chunks[current_chunk_index].can_shrink_by_pages(&chunk_deflated_pages) {
					break;
				}

				trace!(
					"Skipped one chunk, because it cannot be shrunk by the current block of deflated pages"
				);

				let Some(new_chunk_index) = current_chunk_index.checked_sub(1) else {
					error!(
						"Was unable to use all page indices acknowledged for deflation to shrink allocation chunks"
					);
					return;
				};

				current_chunk_index = new_chunk_index;
			}

			// SAFETY: TODO
			let shrink_res =
				unsafe { self.chunks[current_chunk_index].shrink(talc, chunk_deflated_pages) };

			match shrink_res {
				ShrinkResult::PagesRemain => (),
				ShrinkResult::Deallocated => {
					self.chunks.remove(current_chunk_index);
				}
			}

			next_chunk_index = current_chunk_index.checked_sub(1);
		}
	}
}

/// Represents a chunk of consecutive 4K pages allocated for the balloon.
///
/// This ensures via encapsulation, that inflated pages, pages released to the host,
/// are not read from / written to while they are in the balloon.
///
/// The allocation represented by this type must be manually deallocated via [`Self:deallocate`].
/// If the type is dropped, the allocation is leaked.
/// This is not unsafe, but undesirable.
#[derive(Debug)]
struct BalloonAllocation {
	allocation_ptr: Option<NonNull<u8>>,
	page_indices: Vec<u32, DeviceAlloc>,
	/// Contains page indices previously marked for deflation with [`Self::mark_queued_for_deflation`].
	/// The order of page indices is the same as it was while the pages were still
	/// in [`Self::page_indices`]. This means a newly marked block of page indices
	/// is pushed to the front of the `VecDeque`, while shrinking pages removes
	/// page indices from the back.
	pages_queued_for_deflation: VecDeque<u32, DeviceAlloc>,
}

// SAFETY: `BalloonAllocation` does not implement `Clone` (or any other cloning mechanism)
// and implies exclusive ownership of an allocation, with the exception of host ineractions.
// Sending it across threads cannot create a situation where we can access
// mutable state across two threads. The host interactions are guarded by
// unsafe functions and in general we don't dereference pointers into our allocation.
unsafe impl Send for BalloonAllocation {}

// SAFETY: We don't allow for any interior mutability as `allocation_ptr` is never
// dereferenced by us and is not exposed outside of our type. Other than that we
// only have plain integer types that are `Sync` themselves.
unsafe impl Sync for BalloonAllocation {}

impl BalloonAllocation {
	/// Get the memory layout for an allocation of `num_pages` 4K pages
	fn layout(num_pages: NonZeroUsize) -> Layout {
		Layout::from_size_align(num_pages.get() * BALLOON_PAGE_SIZE, BALLOON_PAGE_SIZE).expect(
			"Layout of a non-zero amount of 4K pages aligned to 4K page boundaries should be valid",
		)
	}

	/// The current layout of our allocation if we have any pages allocated,
	/// `None` otherwise.
	fn current_layout(&self) -> Option<Layout> {
		self.num_pages_allocated().map(Self::layout)
	}

	/// The total number of pages allocated for this chunk.
	/// This also includes pages marked for deflation that haven't been shrunk away yet.
	fn num_pages_allocated(&self) -> Option<NonZeroUsize> {
		NonZeroUsize::new(self.page_indices.len() + self.pages_queued_for_deflation.len())
	}

	/// The number of pages of this chunk that can be queued for deflation.
	fn num_available_for_deflation(&self) -> u32 {
		self.page_indices.len().try_into().expect(
			"We only deal with 32-bit indexed pages, so our number of pages has to fit in a u32",
		)
	}

	pub fn is_empty(&self) -> bool {
		self.allocation_ptr.is_none()
	}

	pub fn phys_page_indices(&self) -> impl Iterator<Item = u32> {
		self.page_indices.iter().copied()
	}

	#[must_use = "this returns an object representing the allocation, unless stored, it is leaked"]
	pub fn try_allocate(
		talc: &mut Talc<HermitOomHandler>,
		num_pages: NonZeroU32,
	) -> Result<Self, ()> {
		// SAFETY: We require a non-zero number of pages, from which we construct
		//         a non-zero-sized layout of this many 4K pages.
		let allocation_ptr = unsafe {
			talc.try_malloc(Self::layout(num_pages.try_into().expect(
				"We don't support 16-bit or narrower platforms so a u32 should fit into a usize",
			)))
		}?;

		// FIXME: handle Vec allocation failure, deallocating the balloon chunk

		let num_pages = num_pages.get() as usize;

		let mut page_indices = Vec::with_capacity_in(num_pages, DeviceAlloc);
		(0..num_pages)
			.map(|offset| VirtAddr::from_ptr(allocation_ptr.as_ptr()) + offset)
			.map(|virt_addr| {
				virtual_to_physical(virt_addr)
					.expect("We only deal with virtual addresses that are mapped")
			})
			.map(|phys_addr| {
				u32::try_from(phys_addr.as_u64() / BALLOON_PAGE_SIZE as u64)
					.expect("Balloon cannot handle physical pages above 16TiB")
			})
			.collect_into(&mut page_indices);

		Ok(Self {
			allocation_ptr: Some(allocation_ptr),
			page_indices,
			// TODO: can we not reserve this up-front?
			pages_queued_for_deflation: VecDeque::with_capacity_in(num_pages, DeviceAlloc),
		})
	}

	pub fn mark_queued_for_deflation(
		&mut self,
		num_pages_to_mark: u32,
	) -> impl Iterator<Item = u32> {
		assert!(
			num_pages_to_mark as usize <= self.page_indices.len(),
			"Cannot mark mark more pages for deflation than are still contained in the chunk"
		);

		let num_previously_marked = self.pages_queued_for_deflation.len();

		// Move newly marked block of page indices to front of `pages_queued_for_deflation`
		// while maintaining the order of the indices.
		for page_index in self
			.page_indices
			.drain(self.page_indices.len() - num_pages_to_mark as usize..self.page_indices.len())
			.rev()
		{
			self.pages_queued_for_deflation.push_front(page_index);
		}

		self.pages_queued_for_deflation
			.iter()
			.copied()
			.skip(num_previously_marked)
			.take(num_pages_to_mark as usize)
	}

	pub fn can_shrink_by_pages(&self, page_indices: &[u32]) -> bool {
		self.pages_queued_for_deflation
			.iter()
			.rev()
			.zip(page_indices.iter().rev())
			.all(|(marked, deflated)| *marked == *deflated)
	}

	/// Shrinks the allocated chunk by `pages_to_shrink`.
	/// Takes `self` by value and if there are remaining pages in the chunks after
	/// shrinking, returns it via [`ShrinkResult::PagesRemain`]. Otherwise `self`
	/// is consumed with the chunk having been emptied.
	///
	/// `pages_to_shrink` should be a list of page indices previously returned by
	/// [`Self::mark_queued_for_deflation`]. They should be submitted in the order
	/// they were returned by [`Self::mark_queued_for_deflation`] both within such
	/// a list and across multiple calls of this function with different lists.
	/// This ensure we can actually shrink our allocation.
	///
	/// # Safety
	/// Must be called with the same instance of [`Talc`] that was provided to
	/// [`Self::try_allocate`] to create this instance of [`BalloonAllocation`].
	///
	/// Must not be called while the host still has ownership of any of the pages
	/// that are a part of the allocation represented by this struct.
	/// I.e. deallocation may only take place once the host has returned ownership
	/// back to us for all pages of this allocation.
	///
	/// # Panics
	/// If `pages_to_shrink` contains page indices of pages not marked queued for deflation
	#[must_use = "If pages remain after shrinking, remaining BalloonAllocation is returned. Dropping it would leak the allocation"]
	pub unsafe fn shrink(
		&mut self,
		talc: &mut Talc<HermitOomHandler>,
		pages_to_shrink: Vec<u32, DeviceAlloc>,
	) -> ShrinkResult {
		assert!(
			pages_to_shrink.len() <= self.pages_queued_for_deflation.len(),
			"Must mark the amount of the allocation chunk to be shrunk for deflation before shrinking"
		);

		if self.is_empty() {
			warn!("Attempted to shrink already empty balloon allocation chunk");
			return ShrinkResult::Deallocated;
		}

		if pages_to_shrink.is_empty() {
			return ShrinkResult::PagesRemain;
		}

		let old_layout = self
			.current_layout()
			.expect("We checked above that we have at least one page still allocated");

		let Some(first_to_shrink) = self
			.pages_queued_for_deflation
			.iter()
			.find(|page_index| **page_index == pages_to_shrink[0])
			.copied()
		else {
			error!(
				"First page to shrink ({}) was not found inside balloon allocation chunk, can't shrink",
				pages_to_shrink[0]
			);
			panic!("Attempted to shrink balloon allocation chunk by page not inside the chunk")
		};

		if self
			.pages_queued_for_deflation
			.back()
			.is_some_and(|page_index| {
				page_index
					== pages_to_shrink
						.last()
						.expect("We checked for non-emptiness above")
			}) {
			error!(
				"Last page to shrink {} was not found inside balloon allocation chunk, can't shrink",
				pages_to_shrink
					.last()
					.expect("We checked for non-emptiness above")
			);
			panic!(
				"Attempted to shrink balloon allocation chunk by pages not consecutively at the end of the chunk"
			)
		}

		for (page_index_to_shrink, page_index_marked) in pages_to_shrink.into_iter().zip(
			self.pages_queued_for_deflation
				.drain(first_to_shrink as usize..),
		) {
			assert!(
				page_index_to_shrink == page_index_marked,
				"Attempted to shrink balloon allocation chunk by page not inside the chunk"
			);
		}

		let new_num_pages = self.page_indices.len() + self.pages_queued_for_deflation.len();

		if new_num_pages == 0 {
			// SAFETY: TODO
			unsafe {
				talc.free(
					self.allocation_ptr
						.take()
						.expect("We checked above that we still have at least one page allocated"),
					old_layout,
				);
			}

			trace!(
				"Deallocated balloon chunk as all its pages were shrunk away after acknowledged deflation"
			);

			ShrinkResult::Deallocated
		} else {
			// SAFETY: TODO
			unsafe {
				talc.shrink(
					self.allocation_ptr
						.expect("We checked above that we still have at least one page allocated"),
					old_layout,
					new_num_pages,
				);
			}

			trace!(
				"Shrunk balloon chunk with {} pages still remaining and an additional {} pages marked queued for deflation",
				self.page_indices.len(),
				self.pages_queued_for_deflation.len()
			);

			ShrinkResult::PagesRemain
		}
	}
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ShrinkResult {
	PagesRemain,
	Deallocated,
}
