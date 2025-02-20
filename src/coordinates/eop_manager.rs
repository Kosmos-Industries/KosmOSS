use super::eop_errors::EOPErrors;
use crate::coordinates::coordinate_transformation::EOPData;
use chrono::{DateTime, Duration, Utc};
use csv::ReaderBuilder;
use hifitime::Epoch;
use reqwest;
use std::collections::BTreeMap;
use std::fs;
use std::path::PathBuf;

const CACHE_DURATION_HOURS: i64 = 24;
const CACHE_FILE: &str = "eop_cache.csv";
const CELESTRAK_URL: &str = "https://celestrak.org/SpaceData/EOP-All.csv";

pub(super) struct EOPManager {
    cache_path: PathBuf,
    last_update: Option<DateTime<Utc>>,
    eop_data: BTreeMap<i64, EOPData>, // Unix timestamp -> EOPData
}

impl EOPManager {
    /// Creates a new EOPManager. Does not load data as it may fail.
    pub fn new() -> Self {
        let cache_dir = dirs::cache_dir()
            .unwrap_or_else(|| PathBuf::from("."))
            .join("kosmoss");
        fs::create_dir_all(&cache_dir).unwrap_or_default();

        Self {
            cache_path: cache_dir.join(CACHE_FILE),
            last_update: None,
            eop_data: BTreeMap::new(),
        }
    }

    /// Explicitly loads cached data. Returns an error if loading fails.
    pub fn initialize(&mut self) -> Result<(), EOPErrors> {
        self.load_embedded_data()
            .or_else(|_| self.update_cache_if_needed())
    }

    /// Allows users to refresh the EOP data manually at runtime.
    pub fn refresh_data(&mut self) -> Result<(), EOPErrors> {
        println!("Refreshing EOP data...");

        match self.download_eop_data() {
            Ok(_) => {
                self.parse_eop_data()?; // Parse new data only if download succeeded
                self.last_update = Some(Utc::now());
                println!("EOP data refreshed successfully.");
                Ok(())
            }
            Err(e) => Err(e),
        }
    }

    /// Returns the EOP data for a given epoch. If the data is not available and if `refresh` is `true`
    /// it will attempt to refresh the cache.
    pub fn get_eop_data(&mut self, epoch: Epoch, refresh: bool) -> Result<EOPData, EOPErrors> {
        match self.update_cache_if_needed() {
            Ok(_) => self.interpolate_eop_data(epoch),
            Err(e) => {
                eprintln!("Warning: Failed to load cached EOP data: {}", e);
                if refresh {
                    println!("Refreshing EOP data...");
                    self.refresh_data()?;
                    self.interpolate_eop_data(epoch)
                } else {
                    Err(EOPErrors::MissingEOPData)
                }
            }
        }
    }

    /// Downloads the latest EOP data.
    fn download_eop_data(&self) -> Result<(), EOPErrors> {
        let client = reqwest::blocking::Client::new();
        let response = client.get(CELESTRAK_URL).send()?;
        let status = response.status();

        if status == reqwest::StatusCode::FORBIDDEN {
            return Err(EOPErrors::HttpForbidden);
        }

        if !response.status().is_success() {
            return Err(EOPErrors::ReqwestError(
                response.error_for_status().unwrap_err(),
            ));
        }

        fs::write(&self.cache_path, response.bytes()?)?;
        Ok(())
    }

    /// Loads the EOP data that was downloaded at compile time.
    fn load_embedded_data(&mut self) -> Result<(), EOPErrors> {
        let data = include_bytes!(concat!(env!("OUT_DIR"), "/eop_cache.csv")); // Use compile-time cached data
        self.parse_eop_data_from_bytes(data)?;
        Ok(())
    }

    fn update_cache_if_needed(&mut self) -> Result<(), EOPErrors> {
        let should_update = match self.last_update {
            None => true,
            Some(last_update) => Utc::now() - last_update > Duration::hours(CACHE_DURATION_HOURS),
        };

        if should_update {
            println!("Updating EOP data cache...");
            self.download_eop_data()?;
            self.parse_eop_data()?;
            self.last_update = Some(Utc::now());
        }

        Ok(())
    }

    /// Parses EOP data from file.
    fn parse_eop_data(&mut self) -> Result<(), EOPErrors> {
        let data = fs::read(&self.cache_path)?;
        self.parse_eop_data_from_bytes(&data)
    }

    /// Parses in-memory CSV data.
    fn parse_eop_data_from_bytes(&mut self, data: &[u8]) -> Result<(), EOPErrors> {
        let mut rdr = ReaderBuilder::new().has_headers(true).from_reader(data);
        self.eop_data.clear();

        for result in rdr.records() {
            let record = result?;
            if record.len() < 7 {
                continue;
            }

            let timestamp = Epoch::from_gregorian_str(&record[0])
                .map_err(EOPErrors::InvalidEpoch)?
                .to_unix_seconds() as i64;
            let eop = crate::coordinates::coordinate_transformation::EOPData {
                x_pole: record[1].parse::<f64>()?,
                y_pole: record[2].parse::<f64>()?,
                ut1_utc: record[3].parse::<f64>()?,
                lod: record[4].parse::<f64>()?,
                ddpsi: record[5].parse::<f64>()?,
                ddeps: record[6].parse::<f64>()?,
            };

            self.eop_data.insert(timestamp, eop);
        }

        if self.eop_data.is_empty() {
            return Err(EOPErrors::MissingEOPData);
        }

        Ok(())
    }

    /// Interpolates EOP data for a given epoch.
    fn interpolate_eop_data(&self, epoch: Epoch) -> Result<EOPData, EOPErrors> {
        if self.eop_data.is_empty() {
            return Err(EOPErrors::MissingEOPData);
        }

        let target_time = epoch.to_unix_seconds() as i64;
        let mut iter = self.eop_data.range(..=target_time);
        let after = iter.next_back();
        let before = iter.next_back();

        match (before, after) {
            (Some((&t1, eop1)), Some((&t2, eop2))) => {
                let fraction = (target_time - t1) as f64 / (t2 - t1) as f64;
                Ok(EOPData::interpolate(eop1, eop2, fraction))
            }
            (Some((_, eop)), None) | (None, Some((_, eop))) => {
                println!("Warning: Using nearest EOP value without interpolation");
                Ok(eop.clone())
            }
            (None, None) => Err(EOPErrors::DataInterpolationError),
        }
    }
}
