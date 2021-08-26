{ pkgs ? import <nixpkgs> {} }:

with pkgs;
stdenv.mkDerivation {
    pname = "nether-pathfinder";
    version = "master";

    src = ./.;

    nativeBuildInputs = [ cmake ];

    buildInputs = [ jdk ];

    cmakeFlags = "-DSHARED_LIBRARY=1";

    installPhase = ''
        cp libnether_pathfinder.so $out/bin
    '';
}
