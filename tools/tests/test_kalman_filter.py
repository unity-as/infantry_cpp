"""Run EKF numerical regression with the project's actual CMSIS-DSP sources."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


REPO = Path(__file__).resolve().parents[2]


class KalmanFilterTest(unittest.TestCase):
    def run_command(self, command):
        result = subprocess.run(command, capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        return result

    def test_native_ekf_regression(self):
        cc, cxx = shutil.which("gcc"), shutil.which("g++")
        if not cc or not cxx:
            self.skipTest("native gcc and g++ required")
        flags = ["-D__GNUC_PYTHON__", "-DDISABLEFLOAT16", "-DARM_MATH_MATRIX_CHECK",
                 "-I", str(REPO / "Drivers/CMSIS/DSP/Include"),
                 "-I", str(REPO / "project/modules/utils"),
                 "-I", str(REPO / "project/modules/alg/kalman")]
        with tempfile.TemporaryDirectory(prefix="kalman_test_") as directory:
            folder = Path(directory)
            objects = []
            for name in ("init", "mult", "add", "sub", "scale", "trans", "inverse"):
                source = REPO / f"Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_{name}_f32.c"
                obj = folder / f"{name}.o"
                self.run_command([cc, "-std=c11", "-O0", *flags, "-c", str(source), "-o", str(obj)])
                objects.append(str(obj))
            for optimization in ("-O0", "-O2"):
                with self.subTest(optimization=optimization):
                    exe = folder / "kalman_test.exe"
                    self.run_command([cxx, "-std=c++20", optimization, "-Wall", "-Wextra", "-Werror",
                                    "-fno-exceptions", "-fno-rtti", *flags,
                                    str(REPO / "tools/tests/kalman_filter_test.cpp"),
                                    *objects, "-o", str(exe)])
                    result = self.run_command([str(exe)])
                    print(optimization, result.stdout.strip(), flush=True)


if __name__ == "__main__":
    unittest.main()
