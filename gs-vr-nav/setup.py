"""Package configuration for the GS-VR-Nav research scaffold."""

from setuptools import find_packages, setup


if __name__ == "__main__":
    setup(
        name="gs-vr-nav",
        version="0.1.0",
        description="Geographic-aligned Gaussian Splatting for continuous VR navigation.",
        packages=find_packages(),
        python_requires=">=3.10",
        install_requires=[
            "numpy>=1.24",
            "scipy>=1.10",
            "Pillow>=9.5",
            "piexif>=1.1.3",
            "pyproj>=3.5",
            "matplotlib>=3.7",
            "open3d>=0.17",
            "plyfile>=1.0",
            "PyYAML>=6.0",
        ],
    )
