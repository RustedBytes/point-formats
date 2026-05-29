use crate::format::Format;
use std::fmt;
use std::path::PathBuf;

/// Crate-wide result alias.
pub type Result<T> = std::result::Result<T, Error>;

/// Errors returned by readers, writers, conversion, and format detection.
#[derive(Debug)]
pub enum Error {
    Io(std::io::Error),
    UnknownFormat {
        path: PathBuf,
    },
    UnsupportedFormat {
        format: Format,
        operation: &'static str,
        hint: &'static str,
    },
    Parse {
        format: Format,
        line: Option<usize>,
        message: String,
    },
    InvalidData {
        message: String,
    },
    LossyConversionBlocked {
        from: &'static str,
        to: Format,
        reason: String,
    },
}

impl Error {
    pub(crate) fn parse(
        format: Format,
        line: impl Into<Option<usize>>,
        message: impl Into<String>,
    ) -> Self {
        Self::Parse {
            format,
            line: line.into(),
            message: message.into(),
        }
    }

    pub(crate) fn invalid(message: impl Into<String>) -> Self {
        Self::InvalidData {
            message: message.into(),
        }
    }

    pub(crate) fn unsupported(format: Format, operation: &'static str, hint: &'static str) -> Self {
        Self::UnsupportedFormat {
            format,
            operation,
            hint,
        }
    }
}

impl From<std::io::Error> for Error {
    fn from(error: std::io::Error) -> Self {
        Self::Io(error)
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(error) => write!(f, "I/O error: {error}"),
            Self::UnknownFormat { path } => write!(
                f,
                "could not infer file format from path '{}'",
                path.display()
            ),
            Self::UnsupportedFormat {
                format,
                operation,
                hint,
            } => write!(f, "{operation} is not supported for {format}: {hint}"),
            Self::Parse {
                format,
                line,
                message,
            } => match line {
                Some(line) => write!(f, "failed to parse {format} at line {line}: {message}"),
                None => write!(f, "failed to parse {format}: {message}"),
            },
            Self::InvalidData { message } => write!(f, "invalid data: {message}"),
            Self::LossyConversionBlocked { from, to, reason } => write!(
                f,
                "refusing lossy conversion from {from} to {to}: {reason}; set allow_lossy=true to permit it"
            ),
        }
    }
}

impl std::error::Error for Error {}
