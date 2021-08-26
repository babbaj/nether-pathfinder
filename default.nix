{ pkgs ? import <nixpkgs> {} }:

with pkgs;
stdenv.mkDerivation {
    pname = "nether-pathfinder";

    src = ./.;

    nativeBuilInputs = [ cmake ];

    buildInputs = [ jdk ];

    installPhase = ''
        cp libnetherpathfinder.so $out/bin
    '';
}
