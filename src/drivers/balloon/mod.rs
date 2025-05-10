use alloc::vec::Vec;
use core::fmt::Debug;

use pci_types::InterruptLine;
use smallvec::smallvec;
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

		Ok(())
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

	/// Discard all new page indices marked used by the host.
	/// These are the page indices we have previously sent into the queue in available buffers.
	pub fn discard_new_used(&mut self) {
		let Some(vq) = &mut self.vq else {
			debug!("<balloon> BalloonVq::discard_new_used called on uninitialized vq");
			panic!("BalloonVq must be initialized before calling discard_new_used");
		};

		loop {
			match vq.try_recv() {
				Ok(_new_used) => (),

				Err(VirtqError::NoNewUsed) => break,

				Err(error) => {
					panic!(
						"Failed to receive new used virtqueue descriptors with unexpected error: {error:?}"
					)
				}
			}
		}
	}

	/// Send specified pages into the balloon virtqueue.
	///
	/// To ensure that there is enough space in the queue, call
	/// [`Self::discard_new_used`] before sending.
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
