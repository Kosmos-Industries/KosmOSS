use crate::coordinates::coordinate_transformation::EOPData;
use chrono::NaiveDateTime;
use chrono::{DateTime, Duration, Utc};
use csv::ReaderBuilder;
use hifitime::Epoch;
use reqwest;
use std::collections::BTreeMap;
use std::error::Error;
use std::fs::{self, File};
use std::io::Write;
use std::path::PathBuf;

const CACHE_DURATION_HOURS: i64 = 24;
const CACHE_FILE: &str = "eop_cache.csv";
const CELESTRAK_URL: &str = "https://celestrak.org/SpaceData/EOP-All.csv";

pub struct EOPManager {
    cache_path: PathBuf,
    last_update: Option<DateTime<Utc>>,
    eop_data: BTreeMap<i64, EOPData>, // Unix timestamp -> EOPData
}

impl EOPManager {
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

    pub fn get_eop_data(&mut self, epoch: Epoch) -> Result<EOPData, Box<dyn Error>> {
        self.update_cache_if_needed()?;
        self.interpolate_eop_data(epoch)
    }

    fn update_cache_if_needed(&mut self) -> Result<(), Box<dyn Error>> {
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

    fn download_eop_data(&self) -> Result<(), Box<dyn Error>> {
        let client = reqwest::blocking::Client::new();
        let response = client.get(CELESTRAK_URL).send()?;

        if !response.status().is_success() {
            return Err("Failed to download EOP data".into());
        }

        let mut file = File::create(&self.cache_path)?;
        file.write_all(&response.bytes()?)?;
        Ok(())
    }

    fn parse_eop_data(&mut self) -> Result<(), Box<dyn Error>> {
        let file = File::open(&self.cache_path)?;
        let mut rdr = ReaderBuilder::new().has_headers(true).from_reader(file);

        self.eop_data.clear();

        for result in rdr.records() {
            let record = result?;
            if record.len() < 7 {
                continue;
            }

            // Parse date (format: YYYY-MM-DD)
            let date = NaiveDateTime::parse_from_str(
                &format!("{} 00:00:00", &record[0]),
                "%Y-%m-%d %H:%M:%S",
            )?;
            let timestamp = date.and_utc().timestamp();

            // Parse EOP values
            let eop = EOPData {
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
            return Err("No valid EOP data found in cache file".into());
        }

        Ok(())
    }

    fn interpolate_eop_data(&self, epoch: Epoch) -> Result<EOPData, Box<dyn Error>> {
        if self.eop_data.is_empty() {
            return Err("No EOP data available".into());
        }

        // Convert Epoch to Unix timestamp
        let target_time = epoch.to_unix_seconds() as i64;

        // Find the two closest data points
        let mut iter = self.eop_data.range(..=target_time);
        let after = iter.next_back();
        let before = iter.next_back();

        match (before, after) {
            (Some((&t1, eop1)), Some((&t2, eop2))) => {
                // Calculate interpolation fraction
                let fraction = (target_time - t1) as f64 / (t2 - t1) as f64;
                Ok(EOPData::interpolate(eop1, eop2, fraction))
            }
            (Some((_, eop)), None) | (None, Some((_, eop))) => {
                println!("Warning: Using nearest EOP value without interpolation");
                Ok(eop.clone())
            }
            (None, None) => {
                println!("Warning: No valid EOP data found, using defaults");
                Ok(EOPData {
                    x_pole: 0.161556,
                    y_pole: 0.247219,
                    ut1_utc: -0.0890529,
                    lod: 0.0017,
                    ddpsi: -0.052,
                    ddeps: -0.003,
                })
            }
        }
    }
}
