Keil -> CMake migration notes

What was done:
- Confirmed CMake project in `CMakeLists.txt` and `cmake/stm32cubemx` includes required sources (startup, system, drivers).
- Created `scripts/cleanup_keil_artifacts.ps1` to remove Keil build outputs safely.
- Updated `.gitignore` to ignore common Keil artifacts under `MDK-ARM/CTRL`.
- Removed Keil build artifacts from `MDK-ARM/CTRL` directory (user requested deletion).

Next steps for verification:
- Run `cmake -S . -B build` and `cmake --build build` with `arm-none-eabi` toolchain to produce ELF.
- Verify `build/` contains firmware ELF and update `.vscode/launch.json` `executable` path if different.
- Run on hardware via ST-Link after confirming connections and debug configuration.

If anything was removed by mistake, recover from backups or source control.
