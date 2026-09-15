use std::{env, path::PathBuf};

fn main() {
    println!("cargo:rerun-if-env-changed=MUJOCO_DOWNLOAD_DIR");

    let download_dir =
        PathBuf::from(env::var("MUJOCO_DOWNLOAD_DIR").expect("MUJOCO_DOWNLOAD_DIR is not set"));

    let lib_dir = download_dir.join("mujoco-3.12.0").join("lib");

    println!("cargo:rustc-link-arg=-Wl,-rpath,{}", lib_dir.display());
}
