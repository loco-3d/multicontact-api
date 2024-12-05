{
  lib,
  stdenv,
  fetchFromGitHub,
  cmake,
  ndcurves,
  python3Packages,
  pythonSupport ? false,
}:

stdenv.mkDerivation (finalAttrs: {
  pname = "multicontact-api";
  version = "3.0.3";

  /*
  src = fetchFromGitHub {
    owner = "loco-3d";
    repo = "multicontact-api";
    rev = "refs/tags/v${finalAttrs.version}";
    hash = "sha256-OPhsdALQxT4ByxRztH9olxz8e7Q8Hxv+ejxKKjF3OMU=";
  };
  */
  src = lib.fileset.toSource {
    root = ./.;
    fileset = lib.fileset.unions [
      ./bindings
      ./CMakeLists.txt
      ./include
      ./package.xml
      ./unittest
    ];
  };

  nativeBuildInputs = [
    cmake
  ];

  propagatedBuildInputs = lib.optional pythonSupport python3Packages.ndcurves
  ++ lib.optional (!pythonSupport) ndcurves;

  meta = {
    description = "define, store and use ContactSequence objects";
    homepage = "https://github.com/loco-3d/multicontact-api";
    license = lib.licenses.bsd2;
    maintainers = with lib.maintainers; [ nim65s ];
    mainProgram = "multicontact-api";
    platforms = lib.platforms.unix;
  };
})
