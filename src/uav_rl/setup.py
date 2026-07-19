from setuptools import find_packages, setup

package_name = "uav_rl"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="lyj",
    maintainer_email="leeyj950322@gmail.com",
    description="RL environments and training utilities for UAV GNC.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "check_uav_rl_env = uav_rl.scripts.check_env:main",
            "train_uav_ppo = uav_rl.scripts.train_ppo:main",
            "rollout_uav_policy = uav_rl.scripts.rollout_policy:main",
            "plot_rl_rollout = uav_rl.scripts.plot_rollout:main",
        ],
    },
)
