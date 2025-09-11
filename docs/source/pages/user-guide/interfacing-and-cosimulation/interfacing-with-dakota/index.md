# Interfacing with Dakota


![Dakota header](dakota.png)

* The main script is called through DAKOTA from the root of the case using `dakota input.in`, where `input.in` is your DAKOTA input file.
* DAKOTA calls a Python entrypoint script which defines the entire recipe, from sampling data pre-processing, to simulator run, to simulation data post-processing.
* The OS command signature for the entrypoint script is defined under `interface/fork/analysis_drivers`.
* The data for each run is stored under each a run subfolder stored under the case root at `./run.#`. This includes:
	* A copy of the entrypoint script `entrypoint.sh`
	* A copy of the simulation options template `template.json`
	* A populated copy of the simulation options `parameters.inp`
	* The results of the DAKOTA run `results.out`
	* The raw data from the simulator `output/output.json`

The entrypoint script uses higher-level wrappers to perform the required OS function calls depending on the chosen simulation framework (pure DYNO, DYNO/ROS, etc.).

* When using ROS, the procedure is entirely analogous, but the pre-processing, simulator run and post-processing steps differ in their commands and underlying framework.
