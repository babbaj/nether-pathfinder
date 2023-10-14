{
  description = "A very basic flake";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixpkgs-unstable";
    #flake-utils.url = "github:numtide/flake-utils/master";
  };

  outputs = { self, nixpkgs }: let
    system = "x86_64-linux";
    pkgs = import nixpkgs { inherit system; };
  in {

    devShells.${system}.default = pkgs.mkShell {
      nativeBuildInputs = with pkgs; [ clang_14 pkg-config ];
      buildInputs = with pkgs; [
        gbenchmark zlib
      ];

      shellHook = ''
        export NIX_CFLAGS_COMPILE="-march=native"
        export CC=${pkgs.clang_14}/bin/clang
        export CXX=${pkgs.clang_14}/bin/clang++
      '';
    };
  };
}
