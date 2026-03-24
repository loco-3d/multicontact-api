{
  description = "API to define and store Contact phases and Contact Sequences.";

  inputs = {
    gepetto.url = "github:gepetto/nix";
    flakoboros.follows = "gepetto/flakoboros";
    gazebros2nix.follows = "gepetto/gazebros2nix";
    flake-parts.follows = "gepetto/flake-parts";
    nixpkgs.follows = "gepetto/nixpkgs";
    nix-ros-overlay.follows = "gepetto/nix-ros-overlay";
    systems.follows = "gepetto/systems";
    treefmt-nix.follows = "gepetto/treefmt-nix";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { lib, self, ... }:
      {
        systems = import inputs.systems;
        imports = [
          inputs.gepetto.flakeModule
          {
            flakoboros = {
              pyOverrideAttrs.multicontact-api = _: _: { };
              overrideAttrs.multicontact-api = _: {
                src = lib.fileset.toSource {
                  root = ./.;
                  fileset = lib.fileset.unions [
                    ./bindings
                    ./include
                    ./notebooks
                    ./unittest
                    ./CMakeLists.txt
                    ./package.xml
                  ];
                };
              };
            };
          }
        ];
      }
    );
}
