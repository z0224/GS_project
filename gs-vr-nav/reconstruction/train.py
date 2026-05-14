"""使用官方仓库进行 3D Gaussian Splatting 训练的封装。"""

from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path


class GaussianSplattingTrainer:
    def __init__(
        self,
        gs_repo_path: str | Path,
        colmap_workspace: str | Path,
        output_dir: str | Path,
        config: dict | None = None,
    ):
        """
        gs_repo_path: 已克隆的 https://github.com/graphdeco-inria/gaussian-splatting 仓库路径
        colmap_workspace: 包含 COLMAP 稀疏重建结果的目录（cameras.bin、images.bin、points3D.bin）
        output_dir: 训练模型保存目录
        config: 覆盖默认训练参数（iterations、sh_degree 等）
        """
        self.gs_repo_path = Path(gs_repo_path)
        self.colmap_workspace = Path(colmap_workspace)
        self.output_dir = Path(output_dir)
        self.config = dict(config or {})
        self._last_iterations: int | None = None

    def run_colmap(self, image_dir: str | Path) -> Path:
        """Run COLMAP feature extraction, matching, and mapping."""
        colmap_executable = shutil.which("colmap")
        if colmap_executable is None:
            raise RuntimeError("COLMAP is not installed or not available on PATH.")

        image_dir = Path(image_dir)
        if not image_dir.exists():
            raise FileNotFoundError(f"Image directory does not exist: {image_dir}")

        self.colmap_workspace.mkdir(parents=True, exist_ok=True)
        database_path = self.colmap_workspace / "database.db"
        sparse_path = self.colmap_workspace / "sparse"
        sparse_path.mkdir(parents=True, exist_ok=True)

        # CN: COLMAP 三步：提特征、做匹配、根据匹配结果恢复稀疏相机/点云结构。
        # EN: COLMAP runs in three steps: feature extraction, matching, and sparse mapping.
        commands = [
            [
                colmap_executable,
                "feature_extractor",
                "--database_path",
                str(database_path),
                "--image_path",
                str(image_dir),
            ],
            [
                colmap_executable,
                "exhaustive_matcher",
                "--database_path",
                str(database_path),
            ],
            [
                colmap_executable,
                "mapper",
                "--database_path",
                str(database_path),
                "--image_path",
                str(image_dir),
                "--output_path",
                str(sparse_path),
            ],
        ]
        for command in commands:
            _run_command(command, cwd=self.colmap_workspace)

        sparse_zero = sparse_path / "0"
        if not sparse_zero.exists():
            raise FileNotFoundError(f"COLMAP completed but did not create expected sparse model: {sparse_zero}")
        self.colmap_workspace = sparse_zero
        return sparse_zero

    def train(self, iterations: int = 30000, sh_degree: int = 3) -> Path:
        """Run the official 3DGS training script and return the expected final PLY path."""
        iterations = int(self.config.get("iterations", iterations))
        sh_degree = int(self.config.get("sh_degree", sh_degree))

        train_script = self.gs_repo_path / "train.py"
        if not train_script.exists():
            raise FileNotFoundError(f"3DGS train.py was not found: {train_script}")
        if not self.colmap_workspace.exists():
            raise FileNotFoundError(f"COLMAP workspace does not exist: {self.colmap_workspace}")

        self.output_dir.mkdir(parents=True, exist_ok=True)
        # CN: 这里委托官方 3DGS 仓库训练，当前项目只负责组织输入输出和参数。
        # EN: Training is delegated to the official 3DGS repo; this project manages paths and parameters.
        command = [
            sys.executable,
            str(train_script),
            "-s",
            str(self.colmap_workspace),
            "-m",
            str(self.output_dir),
            "--iterations",
            str(iterations),
            "--sh_degree",
            str(sh_degree),
        ]
        _run_command(command, cwd=self.gs_repo_path)
        self._last_iterations = iterations

        output_ply = self.output_dir / "point_cloud" / f"iteration_{iterations}" / "point_cloud.ply"
        if not output_ply.exists():
            raise FileNotFoundError(f"3DGS training completed but expected PLY was not found: {output_ply}")
        return output_ply

    def get_output_ply(self) -> Path:
        """Return the final trained PLY path from the latest training run."""
        if self._last_iterations is None:
            self._last_iterations = int(self.config.get("iterations", 30000))

        output_ply = self.output_dir / "point_cloud" / f"iteration_{self._last_iterations}" / "point_cloud.ply"
        if not output_ply.exists():
            raise FileNotFoundError(f"Trained 3DGS PLY does not exist: {output_ply}")
        return output_ply


def _run_command(command: list[str], cwd: Path) -> None:
    try:
        subprocess.run(command, cwd=cwd, check=True, text=True, capture_output=True)
    except FileNotFoundError as exc:
        raise RuntimeError(f"Executable not found while running command: {' '.join(command)}") from exc
    except subprocess.CalledProcessError as exc:
        stdout = exc.stdout.strip() if exc.stdout else ""
        stderr = exc.stderr.strip() if exc.stderr else ""
        details = "\n".join(part for part in [stdout, stderr] if part)
        message = f"Command failed with exit code {exc.returncode}: {' '.join(command)}"
        if details:
            message = f"{message}\n{details}"
        raise RuntimeError(message) from exc
