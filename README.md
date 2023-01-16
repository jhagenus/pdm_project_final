# Final project of Planning & Decision Making (RO47005)


**Creators:**
- L. Dalenberg      - 4933451
- J. Dolfin         - 5560047
- J. Hagenus        - 5099528
- D.J. Scherrenburg - 5175151

### Table of Contents
- [Setup using Poetry](https://github.com/jhagenus/pdm_project_final#setup-using-poetry)
  - [Setup of the gym environment](https://github.com/jhagenus/pdm_project_final/blob/main/README.md#setup-of-the-gym-environment)
  - [Setup of the project](https://github.com/jhagenus/pdm_project_final/blob/main/README.md#setup-of-the-project)
  - [Running the simulation](https://github.com/jhagenus/pdm_project_final/blob/main/README.md#running-the-simulation)
- [Setup using anaconda](https://github.com/jhagenus/pdm_project_final/blob/main/README.md#Setup-using-Conda)
  - [Setup of the environment](https://github.com/jhagenus/pdm_project_final/blob/main/README.md#setup-of-the-environment)
  - [Running the simulation](https://github.com/jhagenus/pdm_project_final/blob/main/README.md#running-the-simulation)


## Setup using Poetry
The following sectors will describe the steps needed for the setup of the environment using Poetry. If this does not work for you, [please try using Conda]

### Setup of the gym environment
The environment that is used for the project is a gym environment. This environment is created by Max Spahn.

The gym environment can be installed by running the following command in the root directory of the project:
```
cd
git clone https://github.com/maxspahn/gym_envs_urdf.git
cd ~/gym_envs_urdf
pip3 install .
poetry install
poetry shell
```

### Setup of the project
The project can be installed by running the following command in the root directory of the project:
```
cd ~/gym_envs_urdf
pip install motion-planning-scenes
git clone git@github.com:jhagenus/pdm_project_final.git
cd ~/gym_envs_urdf/pdm_project_final
sudo apt install python3.8 -y    # Installation to make sure that the right version of python is installed.
python3.8 -m pip install -r requirements.txt    # Installation to make sure that the right version of the needed modules is installed.
```

### Running the simulation
To execute the simulation, run the following commands in the poetry shell:
```
cd ~/gym_envs_urdf/pdm_project_final/
python3.8 ./static/vehicle_simulation.py
```

## Setup using Conda
The following sectors will describe the steps needed for the setup of the environment using Conda. If this does not work for you, [please try using Poetry](https://github.com/jhagenus/pdm_project_final#setup-using-poetry)


### Setup of the environment
```
git clone https://github.com/maxspahn/gym_envs_urdf.git
cd gym_envs_urdf
git clone git@github.com:jhagenus/pdm_project_final.git
cd pdm_project_final
conda env create -f pdm.yml
conda activate PDM3.8
cd ..
pip3 install .
```

### Running the simulation
To execute the simulation, run the following commands in the poetry shell:
```
conda activate PDM3.8
cd ~/gym_envs_urdf/pdm_project_final/
python3 ./static/vehicle_simulation.py

```
