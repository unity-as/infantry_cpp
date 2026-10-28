"""Compile real module sources against simulated UART/CAN and F4/H7 clocks."""
import argparse
import pathlib
import shutil
import subprocess

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", default=shutil.which("g++"))
    parser.add_argument("--source-root", type=pathlib.Path, default=ROOT)
    parser.add_argument("--platform", choices=("f4", "h7"), default="f4")
    parser.add_argument("--cases", nargs="+", choices=[
        "tim", "daemon", "serial", "serial_1ms", "serial_startup", "vt13", "dt7", "dji", "dm"])
    args = parser.parse_args()
    if not args.cxx:
        parser.error("g++ is required; pass --cxx PATH")
    cases = args.cases or ["tim", "daemon", "serial", "serial_1ms", "serial_startup"]
    if args.cases is None and args.platform == "f4":
        cases += ["vt13", "dt7", "dji", "dm"]
    source_root = args.source_root.resolve()
    build = source_root / "build" / "leg-sync-tests"
    stubs = build / "stubs"
    stubs.mkdir(parents=True, exist_ok=True)
    # Only hardware-facing dependencies are substituted. TIM, Daemon, Serial,
    # CRC, receivers, PID and both motor implementations are compiled unchanged.
    for name in ("tim.h", "bsp_usart.h", "bsp_can.h", "bsp_fdcan.h", "bsp_dwt.h",
                 "bsp_log.h", "bsp_log_config.h"):
        (stubs / name).write_text(
            '#include "' + (HERE / "hal_stubs.h").as_posix() + '"\n',
            encoding="utf-8")
    modules = ["bsp/bsp_tim/bsp_tim", "modules/daemon/daemon",
               "modules/serial/serial", "modules/crc/crc",
               "modules/vt13/vt13", "modules/dt7/dt7",
               "modules/alg/pid/pid", "modules/motor/dji_motor/dji_motor",
               "modules/motor/dm_motor/dm_motor"]
    sources = [source_root / "project" / (name + ".cpp") for name in modules]
    includes = [stubs] + [source.parent for source in sources]
    exe = build / "regression.exe"
    command = [args.cxx, "-std=c++20", "-O0", "-g", "-Wall", "-Wextra",
               "-mno-ms-bitfields"]  # Match GCC ARM packed bitfield layout.
    if args.platform == "h7":
        command += ["-DTEST_H7"]
    command += ["-I" + str(path) for path in includes]
    command += [str(HERE / "regression.cpp")]
    command += [str(source) for source in sources] + ["-o", str(exe)]
    subprocess.run(command, check=True, cwd=source_root)
    for case in cases:
        subprocess.run([str(exe), case], check=True, cwd=source_root)


if __name__ == "__main__":
    main()
