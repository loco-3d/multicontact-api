{
  description = "Library for creating smooth cubic splines";

  inputs = {
    ndcurves.url = "github:loco-3d/ndcurves";
    flake-parts.follows = "ndcurves/flake-parts";
    nixpkgs.follows = "ndcurves/nixpkgs";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } {
      systems = inputs.nixpkgs.lib.systems.flakeExposed;
      perSystem =
        { pkgs, self', system, ... }:
        {
          apps.default = {
            type = "app";
            program = pkgs.python3.withPackages (_: [ self'.packages.default ]);
          };
          devShells.default = pkgs.mkShell { inputsFrom = [ self'.packages.default ]; };
          packages = {
            default = self'.packages.multicontact-api;
            multicontact-api = pkgs.callPackage ./. { inherit (inputs.ndcurves.packages.${system}) ndcurves; };
          };
        };
    };
}
