# Repository Guidelines

## Project Structure & Module Organization
This repository is a Qt 6 C++ desktop application (`SLCalibration`) built with qmake.

- Root: application sources (`main.cpp`, `mainwindow.*`, controller/reconstruction modules), UI forms (`*.ui`), and project file (`SLCalibration.pro`).
- `Projector/`: projector SDK integration and low-level USB/HID sources.
- `MvCamera/`: camera SDK headers and related interfaces.
- Runtime/config artifacts: `config.ini`, `debug.log` (local-only), plus generated build output in `debug/`, `release/`, `build/`, `.vs/`, `x64/`.

Keep new feature code near related modules (for example, camera logic in `CameraController.*`-adjacent files).

## Build, Test, and Development Commands
Use Windows + MSVC 2022 + Qt 6.7.x.

```bat
build_debug.bat
```
Sets MSVC/Qt/OpenCV environment, runs `qmake`, then `nmake` for a debug build.

```bat
build_temp.bat
```
Same build flow, but writes full logs to `build_output.log` for troubleshooting.

```bat
qmake SLCalibration.pro -spec win32-msvc "CONFIG+=debug" && nmake
```
Manual equivalent when iterating on build flags.

## Coding Style & Naming Conventions
- Language: C++ (Qt Widgets), headers/sources paired as `ModuleName.h/.cpp`.
- Indentation: 4 spaces; keep brace style and spacing consistent with existing files.
- Naming: classes/types use PascalCase (for example, `ProjectorController`, `CameraInfo`); member variables commonly use `m_` prefix.
- UI files should stay descriptive and aligned to widget purpose (`singlecalibrationwidget.ui`, etc.).
- No repository-wide formatter config is committed; match local style in touched files.

## Testing Guidelines
No automated test framework is currently committed. Validate changes with:

1. Successful debug build (`build_debug.bat` or manual `qmake` + `nmake`).
2. UI smoke checks for affected tabs/widgets.
3. Hardware-path checks when relevant (camera/projector detection, capture, reconstruction).

If you add automated tests, place them under a new `tests/` directory and document execution steps in your PR.

## Commit & Pull Request Guidelines
History is currently minimal and uses short, direct subjects (for example, `20260305添加子界面联动切换`, `Initial commit`).

- Keep commit titles concise, one logical change per commit.
- Prefer format: `<area>: <summary>` (or concise Chinese equivalent), e.g., `recon: fix point cloud refresh`.
- PRs should include: purpose, key files changed, local build result, and screenshots for UI changes.
- Link related issues/tasks and note environment assumptions (SDK/Qt/OpenCV paths) when relevant.
