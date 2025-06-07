use core::fmt::{self, Write};
use core::hint::black_box;

use anstyle::AnsiColor;
use tracing::{Dispatch, Level};
use tracing_core::field::Visit;
use tracing_core::{Field, LevelFilter, Subscriber};
use tracing_subscriber::layer::SubscriberExt;
use tracing_subscriber::util::SubscriberInitExt as _;

use crate::console::CONSOLE;

/// Data structure to filter kernel messages
struct SpanStackSubscriber;

impl Subscriber for SpanStackSubscriber {
	fn enabled(&self, _metadata: &tracing::Metadata<'_>) -> bool {
		true
	}

	fn register_callsite(
		&self,
		metadata: &'static tracing::Metadata<'static>,
	) -> tracing_core::Interest {
		if self.enabled(metadata) {
			tracing_core::Interest::always()
		} else {
			tracing_core::Interest::never()
		}
	}

	fn new_span(&self, span: &tracing_core::span::Attributes<'_>) -> tracing_core::span::Id {
		todo!()
	}

	fn record(&self, span: &tracing_core::span::Id, values: &tracing_core::span::Record<'_>) {
		// nothing for now
	}

	fn record_follows_from(&self, span: &tracing_core::span::Id, follows: &tracing_core::span::Id) {
		// nothing for now
	}

	fn event(&self, event: &tracing::Event<'_>) {
		let mut console_guard = CONSOLE.lock();
		let console = &mut *console_guard;

		let core_id = crate::arch::core_local::core_id();
		let level = ColorLevel(*event.metadata().level());

		let _ = write!(console, "<{level}>[{core_id}]");

		let metadata = event.metadata();
		let target = metadata.target();

		if let Some(line) = metadata.line() {
			let _ = write!(console, "({target}:{line}) ");
		} else {
			let _ = write!(console, "({target})");
		}

		{
			let mut visitor = HermitVisitor::from(&mut *console);
			event.record(&mut visitor);
			let _ = visitor.finish();
		}

		let _ = writeln!(console);
	}

	fn enter(&self, _span: &tracing_core::span::Id) {
		// nothing for now
	}

	fn exit(&self, _span: &tracing_core::span::Id) {
		// nothing for now
	}
}

struct HermitVisitor<'a, W> {
	writer: &'a mut W,
	result: fmt::Result,
	is_empty: bool,
}

impl<'a, W: Write> HermitVisitor<'a, W> {
	fn write(&mut self, debuggable: &impl fmt::Debug) {
		let prefix = if self.is_empty {
			self.is_empty = false;
			""
		} else {
			", "
		};
		self.result = write!(self.writer, "{prefix}{debuggable:?}");
	}

	pub fn finish(self) -> fmt::Result {
		self.result
	}
}

impl<'a, W: Write> From<&'a mut W> for HermitVisitor<'a, W> {
	fn from(writer: &'a mut W) -> Self {
		Self {
			writer,
			result: Ok(()),
			is_empty: true,
		}
	}
}

impl<'a, T: Write> Visit for HermitVisitor<'a, T> {
	fn record_str(&mut self, field: &Field, value: &str) {
		if self.result.is_err() {
			return;
		}

		if field.name() == "message" {
			// Print message without quoting or escaping contents.
			// Debug for fmt::Arguments is the formatted string verbatim
			self.record_debug(field, &format_args!("{value}"));
		} else {
			self.record_debug(field, &value);
		}
	}

	fn record_debug(&mut self, field: &Field, value: &dyn fmt::Debug) {
		if self.result.is_err() {
			return;
		}

		match field.name() {
			"message" => self.write(&value),
			name if name.starts_with("r#") => {
				self.write(&format_args!("{}: {value:?}", &name[2..],));
			}
			name => self.write(&format_args!("{name}: {value:?}")),
		};
	}
}

struct ColorLevel(Level);

impl fmt::Display for ColorLevel {
	fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
		let level = self.0;

		if no_color() {
			write!(f, "{level}")
		} else {
			let color = match level {
				Level::TRACE => AnsiColor::Magenta,
				Level::DEBUG => AnsiColor::Blue,
				Level::INFO => AnsiColor::Green,
				Level::WARN => AnsiColor::Yellow,
				Level::ERROR => AnsiColor::Red,
			};

			let style = anstyle::Style::new().fg_color(Some(color.into()));
			write!(f, "{style}{level}{style:#}")
		}
	}
}

fn no_color() -> bool {
	option_env!("NO_COLOR").is_some_and(|val| !val.is_empty())
}

pub unsafe fn init() {
	// Determines LevelFilter at compile time
	let log_level: Option<&'static str> = option_env!("HERMIT_LOG_LEVEL_FILTER");
	let mut level_filter = LevelFilter::INFO;

	if let Some(log_level) = log_level {
		level_filter = if log_level.eq_ignore_ascii_case("off") {
			LevelFilter::OFF
		} else if log_level.eq_ignore_ascii_case("error") {
			LevelFilter::ERROR
		} else if log_level.eq_ignore_ascii_case("warn") {
			LevelFilter::WARN
		} else if log_level.eq_ignore_ascii_case("info") {
			LevelFilter::INFO
		} else if log_level.eq_ignore_ascii_case("debug") {
			LevelFilter::DEBUG
		} else if log_level.eq_ignore_ascii_case("trace") {
			LevelFilter::TRACE
		} else {
			error!("Could not parse HERMIT_LOG_LEVEL_FILTER, falling back to `info`.");
			LevelFilter::INFO
		};
	}

	Dispatch::new_global(&SpanStackSubscriber);
	SpanStackSubscriber::new()
		.with(level_filter)
		.try_init()
		.expect("Failed to initialized hermit tracing subscriber as global subscriber");
}

#[cfg(any(not(target_arch = "riscv64"), feature = "pci", feature = "tcp"))]
macro_rules! infoheader {
	// This should work on paper, but it's currently not supported :(
	// Refer to https://github.com/rust-lang/rust/issues/46569
	/*($($arg:tt)+) => ({
		info!("");
		info!("{:=^70}", format_args!($($arg)+));
	});*/
	($str:expr) => {{
		::tracing::info!("");
		::tracing::info!("{:=^70}", $str);
	}};
}

#[cfg_attr(target_arch = "riscv64", allow(unused))]
macro_rules! infoentry {
	($str:expr, $rhs:expr) => (infoentry!($str, "{}", $rhs));
	($str:expr, $($arg:tt)+) => (::tracing::info!("{:25}{}", concat!($str, ":"), format_args!($($arg)+)));
}

#[cfg(any(not(target_arch = "riscv64"), feature = "pci", feature = "tcp"))]
macro_rules! infofooter {
	() => {{
		::tracing::info!("{:=^70}", '=');
		::tracing::info!("");
	}};
}
