use reqwest;
use std::{error::Error, fmt, io, num::ParseFloatError};

#[derive(Debug)]
pub enum EOPErrors {
    IoError(std::io::Error),
    ReqwestError(reqwest::Error),
    CsvError(csv::Error),
    ParseFloatError(ParseFloatError),
    InvalidEpoch(hifitime::errors::Errors),
    MissingEOPData,
    DataInterpolationError,
    HttpForbidden,
}

impl fmt::Display for EOPErrors {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            EOPErrors::IoError(e) => write!(f, "I/O error: {}", e),
            EOPErrors::ReqwestError(e) => write!(f, "Request error: {}", e),
            EOPErrors::CsvError(e) => write!(f, "CSV parsing error: {}", e),
            EOPErrors::ParseFloatError(e) => write!(f, "Float parsing error: {}", e),
            EOPErrors::InvalidEpoch(e) => write!(f, "Invalid epoch {}", e),
            EOPErrors::MissingEOPData => write!(f, "EOP data is missing"),
            EOPErrors::DataInterpolationError => write!(f, "Failed to interpolate EOP data"),
            EOPErrors::HttpForbidden => write!(f, "HTTP 403 Forbidden"),
        }
    }
}

impl Error for EOPErrors {}

// Implement `From<T>` conversions for automatic error mapping
impl From<io::Error> for EOPErrors {
    fn from(err: io::Error) -> Self {
        EOPErrors::IoError(err)
    }
}

impl From<reqwest::Error> for EOPErrors {
    fn from(err: reqwest::Error) -> Self {
        EOPErrors::ReqwestError(err)
    }
}

impl From<csv::Error> for EOPErrors {
    fn from(err: csv::Error) -> Self {
        EOPErrors::CsvError(err)
    }
}

impl From<ParseFloatError> for EOPErrors {
    fn from(err: ParseFloatError) -> Self {
        EOPErrors::ParseFloatError(err)
    }
}
