# DRL Highway (MSc thesis)
This project has been created as part of my MSc thesis. This simulator has been created in Unity using their Machine Learning Agents Toolkit [ML Agents](https://github.com/Unity-Technologies/ml-agents), an open-source Unity plugin that enables games and simulations to serve as environments for training intelligent agents. The agents in this project have been trained using Deep Reinforcement Learning (DRL). For further reading, see (thesis link). 

## Getting started
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites
Python and the needed Python packages were installed through [Anaconda](https://www.anaconda.com/distribution/). The ML Agents Toolkit can be found on their [GitHub](https://github.com/Unity-Technologies/ml-agents), where it is **highly recommended to go through the installation instructions in the documentation**. You should clone their repository since you'll need to download it anyway because of the Unity package. Unity was installed using Unity Hub. 

### Installation
The following software was used to create this project:
* Python version 3.6.10
* Tensorflow version 1.7.1
* Unity version 2019.3.5f1
* [ml-agents](https://github.com/Unity-Technologies/ml-agents) version 0.15.0
	* `mlagents` Python package
	* `gym_unity` Python package
	* `com.unity.ml-agents` Unity package

## Used Unity assets
All assets used in this project are free and as follows:

| Name | Developer | License
| --- | --- | ---
| [RoadArchitect v1.7](https://github.com/MicroGSD/RoadArchitect) | MicroGSD | MIT
| [Audi RS5 2019](https://sketchfab.com/3d-models/audi-rs5-2019-fae1a4186d464a6aae351ce9e9ff2401) | Indians | CC Attribution
| [Low-poly Civilian vehicle #5](https://assetstore.unity.com/packages/3d/vehicles/land/low-poly-civilian-vehicle-5-124987) | Pro 3D models | Unity asset
| [Low-poly Sports car #20](https://assetstore.unity.com/packages/3d/vehicles/land/low-poly-sports-car-20-144253) | Pro 3D models | Unity asset
| [Turbine](https://sketchfab.com/3d-models/turbine-76a5c63001e14041be291a5b1a3d924b) | Peter Primini | CC Attribution
| [Terrain Tools Sample Asset Pack](https://assetstore.unity.com/packages/2d/textures-materials/terrain-tools-sample-asset-pack-145808) | Unity Technologies | Unity asset

## Acknowledgements
Big thanks to Bazilinskyy et al. (2020) for providing me with their project [couped-sim](https://github.com/bazilinskyy/coupled-sim) as a reference.