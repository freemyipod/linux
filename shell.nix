let
  nixpkgsCommit = "15b85dedcbaf9997bea11832106adb2195486443";
  nixpkgsSrc = fetchTarball {
    url = "https://github.com/NixOS/nixpkgs/archive/${nixpkgsCommit}.tar.gz";
    sha256 = "sha256:1s2ih6ch5rhz1m3s9vkbxrdvvmxvxkmyck9zyc3q87gg3hsn10jb";
  };
  pkgs = import nixpkgsSrc {};
in

pkgs.mkShell {
  packages = with pkgs; [
    pkg-config ncurses
    gcc-arm-embedded
    bison flex bc
    openssl
    gmp libmpc mpfr
    ubootTools
    clang lld libllvm
    dfu-util
    rustc rust-bindgen
    elfutils
  ];

  shellHook = ''
    export RUST_LIB_SRC=${pkgs.rustPlatform.rustLibSrc}
    export ARCH=arm
    export LLVM=1
    # Workaround for https://github.com/NixOS/nixpkgs/issues/201254
    if [ "$(uname -m)" == "aarch64" ]; then
        export HOSTRUSTFLAGS=-Clink-args=-lgcc
    fi
  '';
}
