use std::path::PathBuf;

fn main() {
    let manifest_dir = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let jolt_root = manifest_dir.join("vendor/JoltPhysics");
    assert!(
        jolt_root.join("Jolt/Jolt.h").exists(),
        "vendored Jolt not found at {}",
        jolt_root.display()
    );

    // Build the Jolt static library from the local checkout.
    // Distribution config: no debug renderer, no profiler, no FP exceptions,
    // so the C++ vtables/layouts match a plain Release-with-defaults build.
    // SIMD stays on (AVX2/SSE4 defaults) for full Jolt performance.
    let mut config = cmake::Config::new(jolt_root.join("Build"));
    config
        .profile("Distribution")
        .build_target("Jolt")
        .define("TARGET_UNIT_TESTS", "OFF")
        .define("TARGET_HELLO_WORLD", "OFF")
        .define("TARGET_PERFORMANCE_TEST", "OFF")
        .define("TARGET_SAMPLES", "OFF")
        .define("TARGET_VIEWER", "OFF")
        .define("ENABLE_INSTALL", "OFF")
        .define("INTERPROCEDURAL_OPTIMIZATION", "OFF")
        .define("USE_STATIC_MSVC_RUNTIME_LIBRARY", "OFF")
        .define("JPH_USE_DX12", "OFF")
        .define("JPH_USE_VK", "OFF")
        .define("JPH_USE_MTL", "OFF");
    if cfg!(target_os = "windows") {
        config.cxxflag("/EHsc");
    }
    let dst = config.build();

    // Visual Studio is a multi-config generator: the lib lands under
    // <out>/build/Distribution/. Single-config generators (Ninja/Make) put
    // it directly under <out>/build/. Search every plausible location.
    for candidate in [
        "build/Distribution",
        "build/Release",
        "build",
        "build/lib",
        "lib",
        "lib64",
    ] {
        println!(
            "cargo:rustc-link-search=native={}",
            dst.join(candidate).display()
        );
    }
    println!("cargo:rustc-link-lib=static=Jolt");

    // The shim MUST see the same feature defines as the library: Jolt
    // selects intrinsics, vectorcall paths, and virtuals (debug renderer,
    // object stream) from these. A mismatch corrupts the stack at the FFI
    // boundary. These mirror Jolt's CMake defaults (see Build/CMakeLists.txt).
    // NDEBUG keeps asserts off in the shim, matching the Distribution library
    // build (USE_ASSERTS=OFF): without it, a debug cargo build defines
    // JPH_DEBUG, auto-enables JPH_ENABLE_ASSERTS in the shim, and the link
    // fails on the missing AssertFailed symbol.
    let mut shim = cc::Build::new();
    shim.cpp(true)
        .std("c++17")
        .file("wrapper/wrapper.cpp")
        .include(&jolt_root)
        .define("NDEBUG", None)
        .define("JPH_OBJECT_STREAM", None)
        .define("JPH_USE_SSE4_1", None)
        .define("JPH_USE_SSE4_2", None)
        .define("JPH_USE_AVX", None)
        .define("JPH_USE_AVX2", None)
        .define("JPH_USE_LZCNT", None)
        .define("JPH_USE_TZCNT", None)
        .define("JPH_USE_F16C", None)
        .define("JPH_USE_FMADD", None)
        .warnings(false);
    if cfg!(target_os = "windows") {
        shim.flag("/EHsc").flag("/arch:AVX2");
    } else {
        shim.flag("-mavx2")
            .flag("-mbmi")
            .flag("-mpopcnt")
            .flag("-mlzcnt")
            .flag("-mf16c");
    }
    shim.compile("jolt_shim");
    println!("cargo:rerun-if-changed=wrapper/wrapper.cpp");
}
