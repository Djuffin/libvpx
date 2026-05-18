//! Build-time discovery of the VP8 test-data directory.
//!
//! Three lookup phases, first hit wins:
//!   1. `$LIBVPX_TEST_DATA_PATH`, if it contains the canonical sentinel
//!      file `vp80-00-comprehensive-001.ivf`.
//!   2. A handful of conventional locations relative to the crate.
//!   3. Download the missing pieces from the WebM project bucket into
//!      `$OUT_DIR/vp8_test_data/`, verified against `test/test-data.sha1`.
//!
//! Whatever path wins is exported to the test binaries as
//! `VP8_TEST_DATA_DIR`. If every phase fails (no local data, no
//! `curl`/`sha1sum`, no network) the env var is emitted empty and tests
//! detect that and skip themselves — the build never fails on its own.

use std::env;
use std::path::{Path, PathBuf};
use std::process::Command;

/// Sentinel file used to confirm a candidate directory actually holds
/// the VP8 conformance data.
const SENTINEL: &str = "vp80-00-comprehensive-001.ivf";

/// The five VP8-only invalid fixtures (`.ivf` + `.res` companions).
const INVALID_FIXTURES: &[&str] = &[
    "invalid-bug-1443.ivf",
    "invalid-bug-148271109.ivf",
    "invalid-token-partition.ivf",
    "invalid-vp80-00-comprehensive-s17661_r01-05_b6-.ivf",
    "invalid-vp80-00-comprehensive-018.ivf.2kf_0x6.ivf",
];

const DOWNLOAD_BASE: &str =
    "https://storage.googleapis.com/downloads.webmproject.org/test_data/libvpx/";

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let sha1_path = manifest_dir.join("..").join("test").join("test-data.sha1");

    println!("cargo:rerun-if-env-changed=LIBVPX_TEST_DATA_PATH");
    println!("cargo:rerun-if-changed={}", sha1_path.display());
    println!("cargo:rerun-if-changed=build.rs");

    let dir = discover(&manifest_dir, &sha1_path).unwrap_or_default();
    println!("cargo:rustc-env=VP8_TEST_DATA_DIR={dir}");
}

fn discover(manifest_dir: &Path, sha1_path: &Path) -> Option<String> {
    // Phase 1: explicit env var.
    if let Ok(p) = env::var("LIBVPX_TEST_DATA_PATH") {
        let candidate = PathBuf::from(&p);
        if candidate.join(SENTINEL).is_file() {
            println!("cargo:warning=VP8 test data: using LIBVPX_TEST_DATA_PATH={p}");
            return Some(p);
        } else {
            println!(
                "cargo:warning=LIBVPX_TEST_DATA_PATH={p} does not contain {SENTINEL}; falling back"
            );
        }
    }

    // Phase 2: conventional locations.
    let probes = [manifest_dir.join("test_data")];
    for p in &probes {
        if p.join(SENTINEL).is_file() {
            let s = p.to_string_lossy().into_owned();
            println!("cargo:warning=VP8 test data: using local probe {s}");
            return Some(s);
        }
    }

    // Phase 3: download.
    download_path(manifest_dir, sha1_path)
}

/// Build the cache dir at `$OUT_DIR/vp8_test_data` and ensure every
/// required file is present (downloading + sha1-verifying anything that
/// is missing). Returns `Some(dir)` on full success, `None` otherwise.
fn download_path(_manifest_dir: &Path, sha1_path: &Path) -> Option<String> {
    let manifest = match std::fs::read_to_string(sha1_path) {
        Ok(s) => s,
        Err(e) => {
            println!(
                "cargo:warning=VP8 test data: cannot read sha1 manifest {}: {e}",
                sha1_path.display()
            );
            return None;
        }
    };

    let needed = required_files(&manifest);
    if !needed.iter().any(|(name, _)| name == SENTINEL) {
        println!("cargo:warning=VP8 test data: sentinel {SENTINEL} missing from sha1 manifest");
        return None;
    }

    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap()).join("vp8_test_data");
    if let Err(e) = std::fs::create_dir_all(&out_dir) {
        println!(
            "cargo:warning=VP8 test data: cannot create cache dir {}: {e}",
            out_dir.display()
        );
        return None;
    }

    // Bail early if curl / sha1sum aren't on PATH.
    if !tool_available("curl") {
        println!("cargo:warning=VP8 test data: curl not on PATH; cannot download");
        return None;
    }
    if !tool_available("sha1sum") {
        println!("cargo:warning=VP8 test data: sha1sum not on PATH; cannot verify downloads");
        return None;
    }

    for (name, sha1) in &needed {
        let dest = out_dir.join(name);
        if dest.is_file() && verify_sha1(&dest, sha1) {
            continue;
        }
        if dest.exists() {
            let _ = std::fs::remove_file(&dest);
        }
        let url = format!("{DOWNLOAD_BASE}{name}");
        println!("cargo:warning=VP8 test data: downloading {name}");
        let status = Command::new("curl")
            .args(["-fsSL", "-o"])
            .arg(&dest)
            .arg(&url)
            .status();
        match status {
            Ok(s) if s.success() => {}
            Ok(s) => {
                println!("cargo:warning=VP8 test data: curl exited {s} for {name}");
                let _ = std::fs::remove_file(&dest);
                return None;
            }
            Err(e) => {
                println!("cargo:warning=VP8 test data: curl failed for {name}: {e}");
                return None;
            }
        }
        if !verify_sha1(&dest, sha1) {
            println!("cargo:warning=VP8 test data: sha1 mismatch on {name}, removing");
            let _ = std::fs::remove_file(&dest);
            return None;
        }
    }

    Some(out_dir.to_string_lossy().into_owned())
}

/// Walk the manifest and pull out every line whose filename we need.
/// Manifest line format (BSD sha1sum): `<40hex> *<filename>`.
fn required_files(manifest: &str) -> Vec<(String, String)> {
    let mut out = Vec::new();
    for line in manifest.lines() {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let (sha1, rest) = match line.split_once(' ') {
            Some(p) => p,
            None => continue,
        };
        // Strip the `*` (binary mode) marker if present.
        let name = rest.trim_start_matches('*').trim();
        if is_required(name) {
            out.push((name.to_owned(), sha1.to_owned()));
        }
    }
    out
}

fn is_required(name: &str) -> bool {
    // Every vp80-* fixture (the 62 conformance ivfs plus their .md5
    // companions all share that prefix in the manifest).
    if name.starts_with("vp80-") {
        return true;
    }
    // The hand-picked invalid fixtures and their .res files.
    for f in INVALID_FIXTURES {
        if name == *f || name == format!("{f}.res").as_str() {
            return true;
        }
    }
    false
}

fn tool_available(tool: &str) -> bool {
    Command::new(tool)
        .arg("--version")
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

fn verify_sha1(path: &Path, expected: &str) -> bool {
    let out = match Command::new("sha1sum").arg(path).output() {
        Ok(o) if o.status.success() => o,
        _ => return false,
    };
    let stdout = String::from_utf8_lossy(&out.stdout);
    match stdout.split_whitespace().next() {
        Some(got) => got.eq_ignore_ascii_case(expected),
        None => false,
    }
}
