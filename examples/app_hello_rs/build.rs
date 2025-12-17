use std::env;
use std::path::PathBuf;

fn main() {
    println!("cargo:rerun-if-changed=wrapper.h");
    println!("cargo:rerun-if-changed=build.rs");

    // Generate bindings into the crate source tree (so it is visible under `src/`).
    // Note: this will create/overwrite `src/bindings.rs` during build.
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let bindings_path = manifest_dir.join("src").join("bindings.rs");

    bindgen::Builder::default()
        .header("wrapper.h")
        // 在 no_std 环境下用 core 而不是 std
        .use_core()
        // 使用 cty 作为 C 类型前缀，比如 cty::c_float
        .ctypes_prefix("cty")
        // 不生成 layout 测试（对嵌入式没意义，还可能编不过）
        .layout_tests(false)
        .generate()
        .expect("Unable to generate bindings")
        .write_to_file(bindings_path)
        .expect("Couldn't write bindings!");
}
