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

#[cfg(feature = "gpkg")]
impl From<rusqlite::Error> for Error {
    fn from(error: rusqlite::Error) -> Self {
        Self::invalid(format!("SQLite error: {error}"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_variants() {
        let err_io = Error::from(std::io::Error::new(
            std::io::ErrorKind::NotFound,
            "file not found",
        ));
        assert!(err_io.to_string().contains("file not found"));

        let err_unknown = Error::UnknownFormat {
            path: PathBuf::from("test.unknown"),
        };
        assert!(err_unknown
            .to_string()
            .contains("could not infer file format"));

        let err_unsupported = Error::unsupported(Format::NetCdf, "read", "use netcdf adapter");
        assert!(err_unsupported
            .to_string()
            .contains("read is not supported for netcdf: use netcdf adapter"));

        let err_parse_line = Error::parse(Format::Ply, 15, "bad keyword");
        assert!(err_parse_line
            .to_string()
            .contains("failed to parse ply at line 15: bad keyword"));

        let err_parse_no_line = Error::parse(Format::Ply, None, "bad header");
        assert!(err_parse_no_line
            .to_string()
            .contains("failed to parse ply: bad header"));

        let err_invalid = Error::invalid("corrupt");
        assert!(err_invalid.to_string().contains("invalid data: corrupt"));

        let err_lossy = Error::LossyConversionBlocked {
            from: "mesh",
            to: Format::Xyz,
            reason: "discarding faces".to_string(),
        };
        assert!(err_lossy
            .to_string()
            .contains("refusing lossy conversion from mesh to xyz: discarding faces"));
    }

    #[cfg(feature = "gpkg")]
    #[test]
    fn test_sqlite_error_conversion() {
        let sql_err = rusqlite::Error::QueryReturnedNoRows;
        let err = Error::from(sql_err);
        assert!(err.to_string().contains("Query returned no rows"));
    }
}
