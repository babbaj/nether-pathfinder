{ pkgs ? import <nixpkgs> {} }:

with pkgs;
stdenv.mkDerivation {
    pname = "nether-pathfinder";
    version = "master";

    src = ./.;

    nativeBuildInputs = [ cmake jdk ];

    cmakeFlags = "-DSHARED_LIBRARY=1";

    installPhase = ''
        mkdir -p $out/bin
        cp libnether_pathfinder.so $out/bin
    '';
}
