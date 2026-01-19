use tracing_subscriber::EnvFilter;

#[tokio::main]
async fn main() {
    // Setup logging (set RUST_LOG=info or debug)
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env().add_directive("info".parse().unwrap()))
        .init(); // installs the subscriber globally

    if let Err(e) = lekiwi_zenoh_runtime::runtime::run().await {
        eprintln!("Runtime error: {}", e);
        std::process::exit(1);
    }
}