use reqwest::blocking::Client;
use std::env;
use std::error::Error;
use std::fs::{self, File};
use std::io::Write;
use std::path::PathBuf;
use std::time::{Duration, SystemTime};

const CELESTRAK_URL: &str = "https://celestrak.org/SpaceData/EOP-All.csv";
const CACHE_FILE: &str = "eop_cache.csv";
const CACHE_EXPIRATION_HOURS: u64 = 6; // CelesTrak updates every 6 hours

fn main() {
    // Get Cargo's OUT_DIR (temporary build directory)
    let out_dir = env::var("OUT_DIR").expect("Cargo should set OUT_DIR");
    let cache_path = PathBuf::from(out_dir).join(CACHE_FILE);

    // Download and store the EOP data
    match fetch_eop_data(&cache_path) {
        Ok(_) => println!("EOP data fetched successfully!"),
        Err(e) => panic!("Failed to fetch EOP data: {}", e),
    }
}

fn fetch_eop_data(cache_path: &PathBuf) -> Result<(), Box<dyn Error>> {
    // Check last modified time of cache
    if let Ok(metadata) = fs::metadata(cache_path) {
        if let Ok(modified) = metadata.modified() {
            let now = SystemTime::now();
            let age = now.duration_since(modified).unwrap_or(Duration::ZERO);

            // Skip download if the cache is still fresh
            if age < Duration::from_secs(CACHE_EXPIRATION_HOURS * 3600) {
                eprintln!(
                    "Skipping download: Cached EOP data is still fresh ({} minutes old).",
                    age.as_secs() / 60
                );
                return Ok(());
            }
        }
    }

    eprintln!("Fetching new EOP data from: {}", CELESTRAK_URL);

    let client = Client::new();
    let response = client.get(CELESTRAK_URL).send()?;
    let status = response.status();

    if !status.is_success() {
        let response_body = response
            .text()
            .unwrap_or_else(|_| "Failed to read response body".to_string());
        return Err(format!(
            "HTTP request failed: {} - Response: {}",
            status, response_body
        )
        .into());
    }

    let bytes = response.bytes()?;
    eprintln!("Downloaded {} bytes of EOP data.", bytes.len());

    let mut file = File::create(cache_path)?;
    file.write_all(&bytes)?;

    eprintln!("EOP data successfully written to {:?}", cache_path);
    Ok(())
}
