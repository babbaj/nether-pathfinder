{ pkgs ? import <nixpkgs> {} }:

with pkgs;
let
    #pkgsWin = pkgs.pkgsCross.mingwW64;
    pkgsWin = import <nixpkgs> {
        crossSystem = {
          config = "x86_64-w64-mingw32";
          libc = "msvcrt"; # This distinguishes the mingw (non posix) toolchain
        };
      };
in
#llvmPackages_13.stdenv.mkDerivation {
pkgsWin.stdenv.mkDerivation {
    pname = "nether-pathfinder";
    version = "master";

    src = ./.;


    nativeBuildInputs = [ cmake jdk ];

    LIBRARY_PATH = "${pkgsWin.windows.pthreads}";
    #cmakeFlags = "-DSHARED_LIBRARY=1";

    installPhase = ''
        mkdir -p $out/bin
        #cp libnether_pathfinder.so $out/bin
        #cp libnether_pathfinder.dll $out/bin
        cp nether_pathfinder.exe $out/bin
        #cp ${pkgsWin.windows.mcfgthreads}/bin/mcfgthread-12.dll $out/bin
    '';
}
