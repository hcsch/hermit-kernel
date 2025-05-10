use alloc::boxed::Box;
use alloc::vec::Vec;
use core::fmt::Debug;

use pci_types::InterruptLine;
use virtio::FeatureBits;
use virtio::balloon::{ConfigVolatileFieldAccess as _, F};
use volatile::VolatileRef;

use super::Driver;
use super::virtio::virtqueue::Virtq;
#[cfg(not(feature = "pci"))]
use crate::drivers::virtio::transport::mmio::{ComCfg, IsrStatus, NotifCfg};
#[cfg(feature = "pci")]
use crate::drivers::virtio::transport::pci::{ComCfg, IsrStatus, NotifCfg};

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
	vqueues: Vec<Box<dyn Virtq>>,
	irq: InterruptLine,
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

		// For now our feature set is empty (except for the v1 spec compliance feature)
		// so we have nothing more to do.

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
