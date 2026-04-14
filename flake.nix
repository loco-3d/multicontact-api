{
  description = "API to define and store Contact phases and Contact Sequences.";

  inputs.gepetto.url = "github:gepetto/nix";

  outputs =
    inputs:
    inputs.gepetto.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        extraDevPyPackages = [ "multicontact-api" ];
        overrideAttrs.multicontact-api = {
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
      }
    );
}
