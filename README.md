# Habitat Data Collector

**Habitat Data Collector** is a standalone simulation application built on top of [Habitat-Sim](https://github.com/facebookresearch/habitat-sim) and [Habitat-Lab](https://github.com/facebookresearch/habitat-lab). It supports scene loading, object insertion/removal, ROS2 output, and data recording. This tool is designed for collecting and testing datasets for robot perception, navigation, and mapping tasks.

---

## 📦 Environment Setup

### 1. Clone the repository with submodules

```bash
git clone --recurse-submodules https://github.com/Eku127/habitat-data-collector.git
cd habitat-data-collector
```

### 2. Create the Conda environment

```bash
conda env create -f environment.yml
conda activate habitat_data_collector
```

### 3. Build and install habitat-sim

```bash
cd 3rdparty/habitat-sim

# Build and install in editable mode with CMake policy version fix
CMAKE_ARGS="-DCMAKE_POLICY_VERSION_MINIMUM=3.5" \
pip install . -v --config-settings editable=true
```

### 4. Install habitat-lab

```bash
cd ../habitat-lab
pip install -e .
# back to root dir
cd ../..
```

### 5. [Optional] Source the ROS2 environment (Humble)

```bash
source /opt/ros/humble/setup.bash  # or setup.zsh
```

---

## 🚀 Run the Collector

Run the main simulation launcher in the root directory:

```bash
python -m habitat_data_collector.main
```

By default, it uses the config file at `config/habitat_data_collector.yaml` to initialize the environment.

---

## 📁 Project Structure

```
habitat-data-collector/
├── habitat_data_collector/   # Main application code
│   ├── main.py
│   └── utils/
├── config/                   # YAML configuration files
├── 3rdparty/                 # Git submodules: habitat-sim & habitat-lab
├── environment.yml           # Conda environment spec
└── README.md
```

---

## ⚠️ Notes

- ROS2 Humble must be pre-installed and sourced before running.
- Habitat-Sim and Habitat-Lab are included as Git submodules.
- habitat-sim is compiled and installed using pip with editable mode.
- habitat-lab is installed using `pip install -e .`.
- Configuration is managed using [OmegaConf](https://omegaconf.readthedocs.io/) and [Hydra](https://hydra.cc/).

---

## 🛠️ Developer Info

To customize the scene, output, or ROS settings, modify `config/habitat_data_collector.yaml`. To extend functionality, explore the utilities in `habitat_data_collector/utils`.

---

## 📜 License

MIT License

