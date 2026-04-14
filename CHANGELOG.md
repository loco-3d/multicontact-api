# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [4.2.0] - 2026-04-14

- Fix use of pinocchio 3 to get compatibility with pinocchio 4
- CMake: drop git submodule
- Nix: switch to flakoboros

## [4.1.1] - 2026-02-12

- Add CHANGELOG.md

## [4.1.0] - 2025-10-23

Changes in v4.0.1:
- Update package.xml with:
    - an underscore in the package name
    - the description updated
    - addition of the jrl_cmakemodules
- Format
- Nix packaging updated with gepetto/nix
- Fix python install
- Bump minimum CMake version to 3.22.1
- Update nix CI

## [4.0.0] - 2024-12-11

Changes in v4.0.0:

    updates for ndcurves v2
    require CMake >= 3.10
    setup nix & CI
    setup mergify
    flake8 isort black -> ruff


## [3.0.3] - 2023-05-14

Changes in v3.0.3:

    fix for eigenpy v3
    CMake: fetch submodule, setup default build type, bump standard
    setup isort
    symc submodule
    pre-commit autoupdate

## [3.0.2] - 2022-09-02

Changes in v3.0.2:

    update to eigenpy 2.7.12
    reformat
    update tooling


## [3.0.1] - 2022-01-07

Changes in v3.0.1:

    added package.xml
    fixed format

## [3.0.0] - 2022-01-07

Changes in v3.0.0:
- ⚠️ Removed serialization storage feature
- added notebooks
- updated to ndcurves

## [2.1.0] - 2020-07-27

Changes in v2.1.0:

- fix curve serialization version
- Backward compatibility for serialization
- fix contactSequence check methods
- fix concatenate effector
- Improve ContactModel
- Add script to load examples in gepetto-gui

## [2.0.0] - 2020-02-27

Changes since v1.1.2:

- ⚠️ breaks backward compatibility ⚠️
- trajectories are represented by objects from the Curves package and
- not with discrete set of point
- additions and changes to several fields to the contact-phase structure
- to store data about the whole body motion
- Contact-phase now contains the active contact patchs in a `map<effector name ; contact patch >`


## [1.1.2] - 2020-01-27

Changes since v1.1.1:
- fixed .pc
- updated CMake

## [1.1.1] - 2019-09-13

Changes since v1.1.0:
- Maintenance release
- Fix compatibility with recent pinocchio versions
- Reformat project
- improve CMake

## [1.1.0] - 2019-04-16

Changes since v1.0.1:
- store centroidal quantities (com, vcom and force) in contact-phase
- packaging details


## [1.0.1] - 2019-03-29

Changes since v1.0.0:
- setup CI
- fix python module creation, installation and test
- fix imports for python 3

## [1.0.0] - 2019-03-29

- Initial release

[Unreleased]: https://github.com/loco-3d/multicontact-api/compare/v4.2.0...HEAD
[4.2.0]: https://github.com/loco-3d/multicontact-api/compare/v4.1.1...v4.2.0
[4.1.1]: https://github.com/loco-3d/multicontact-api/compare/v4.1.0...v4.1.1
[4.1.0]: https://github.com/loco-3d/multicontact-api/compare/v4.0.0...v4.1.0
[4.0.0]: https://github.com/loco-3d/multicontact-api/compare/v3.0.3...v4.0.0
[3.0.3]: https://github.com/loco-3d/multicontact-api/compare/v3.0.2...v3.0.3
[3.0.2]: https://github.com/loco-3d/multicontact-api/compare/v3.0.1...v3.0.2
[3.0.1]: https://github.com/loco-3d/multicontact-api/compare/v3.0.0...v3.0.1
[3.0.0]: https://github.com/loco-3d/multicontact-api/compare/v2.1.0...v3.0.0
[2.1.0]: https://github.com/loco-3d/multicontact-api/compare/v2.0.0...v2.1.0
[2.0.0]: https://github.com/loco-3d/multicontact-api/compare/v1.1.2...v2.0.0
[1.1.2]: https://github.com/loco-3d/multicontact-api/compare/v1.1.1...v1.1.2
[1.1.1]: https://github.com/loco-3d/multicontact-api/compare/v1.1.0...v1.1.1
[1.1.0]: https://github.com/loco-3d/multicontact-api/compare/v1.0.1...v1.1.0
[1.0.1]: https://github.com/loco-3d/multicontact-api/compare/v1.0.0...v1.0.1
[1.0.0]: https://github.com/loco-3d/multicontact-api/releases/tag/v1.0.0
