# Combined Structural and Dimensional Synthesis of a Parallel Robot for Cryogenic Handling Tasks

Authors:

Moritz Schappler, Philipp Jahn, Annika Raatz, and Tobias Ortmaier

This code is supplemental material to the following submission to the MHI Colloquium 2021:

```bib
@InProceedings{SchapplerJahRaaOrt2022,
  author    = {Schappler, Moritz and Jahn, Philipp and Raatz, Annika and Ortmaier, Tobias},
  editor    = {Schüppstuhl, Thorsten and Tracht, Kirsten and Henrich, Dominik},
  booktitle = {Annals of Scientific Society for Assembly, Handling and Industrial Robotics 2021},
  title     = {Combined Structural and Dimensional Synthesis of a Parallel Robot for Cryogenic Handling Tasks},
  year      = {2022},
  address   = {Cham},
  publisher = {Springer International Publishing},
  isbn      = {978-3-030-74032-0},
  doi       = {10.1007/978-3-030-74032-0_6},
}
```


## Contents and Useage

This repository contains Matlab scripts and Latex code to reproduce all results of the paper. Beforehand, the steps from the [prerequesites section](#prerequisites) have to be performed.

### Replicate Results for Multi-Objective Optimization

1. To reproduce the results, first the dimensional synthesis has to be performed by running the Matlab script `dimsynth/config_pareto.m`.
    * If existing results shall be evaluated, their location has to be set with `calculations/mhi_dimsynth_data_dir.m`.
    * Modifications on the trajectory and task can be performed in `dimsynth/environment_cryotask.m` and `dimsynth/traj_cryotask.m`.
2. The results have to be post-processed by the scripts from `calculations`:
    * `eval_figures_pareto.m`: Assemble all Pareto fronts for the individual robots from the single repetitions
    * `robot_names.m`: The names of the robots have to be generated (e.g. for the legend); only once.
    * `eval_figures_pareto_groups.m`: Group the robots to a smaller set (by combining their platform coupling alignments; as done in the paper). This also creates **Fig. 5 of the paper**.
    * `select_eval_robot_examples.m`: Select the specific robot structures for the detailed view in Fig. 4, Fig. 6 and Table 1.
3. Figures and Tables of the paper can be reproduced with the Matlab scripts
    * `paper/figures/robot_images.m`: Creates one image file for each robot. These are taken in `robots2.svg` to create **Fig. 4 of the paper** and in `robots3.svg` to create **Fig. 6 of the paper**.
    * `paper/tables/results_tables_latex.m` creates **Table 1 of the paper** as latex code in `tab_kinpar.tex`.

The steps 2 to 3 can be performed after each iteration of the dimensional synthesis using `run_evaluation.m`.

### Performing a Single-Objective Optimization

The single-objective optimization did not prove sufficient as only one objective (e.g. joint range) did not cover the desired space of the solutions. The steps for performing the optimization are:

1. Run the dimensional synthesis: `dimsynth/config_cryotask.m`
2. Combine results into one table: `results_stack_tables.m`
3. Create evaluation figures: `eval_figures.m`
4. Detailed look at the results with scripts from the [debug section](#debug).

### Structural Synthesis

Matlab scripts to run the structural synthesis for the 3T0R handling task again are stored in `structsynth`.
The structures are **discussed in Sec. 4** of the paper.
The numbers given there are obtained by `number_possible_structures.m`.  
The structural synthesis does not have to be performed again, as the the strctures are already included in the parallel robots database. The synthesis can be run again with `structsynth_3T0R_PRRRR_novariants.m` for structures containing only prismatic and revolute joints and `structsynth_3T0R_PRRRR_variants.m` for structures containing universal joints.

### Setting Up Cluster Computation

The structural and dimensional synthesis both work well on standard desktop computers and the results can be reproduced there. However, the approach benefits from massive parallel computation on a computing cluster. If a cluster is not available, the corresponding options in the dimensional synthesis have to be deactivated with `Set.general.computing_cluster = false`. If the cluster is available, the tool `matlab_pbs_transfer` has to be used. This requires a cluster running [PBS](https://en.wikipedia.org/wiki/Portable_Batch_System), like the [cluster of the Leibniz University Hannover](https://www.luis.uni-hannover.de/en/services/computing/scientific-computing/).

### Debug Properties of the Results <a name="debug">

* `compare_G1G4.m`: Check how the vertical alignment (G1) and conical alignment (G4) can be transferred
* `eval_results_G4_parameters.m`: Generate a list of all inclination angles of the conical base robots. This can be used to debug anormalities.
* `eval_results_mdh_theta1.m`: Check the theta-parameters of the prismatic joints
* `compare_results_kinematics.m`: Show the different DH parameters of the results
* `postprocess_results.m`: Create a table with additional columns for evaluation


## Prerequisites <a name="prerequisites">

For the optimization and the figure generation to work, the following steps have to be performed:

1. Install Matlab with global optimization toolbox (tested with R2020a)
2. Set up the Matlab mex compiler, if not already done
3. Set up the path to the results for this paper by copying `calculations/mhi_dimsynth_data_dir.m.template` without the template suffix to the same location and by changing the content of the file to point to the stored results of the synthesis (not content of this git repository due to large size).
4. Download Matlab dependencies for the toolchain of dimensional synthesis:
    * https://github.com/SchapplM/robotics-toolbox (tested with rev. 43d0393; 2021-01-26)
    * https://github.com/SchapplM/robsynth-serroblib (tested with rev. 3193e8ab6; 2020-12-01)
    * https://github.com/SchapplM/robsynth-parroblib (tested with rev. e10c4a9; 2021-01-26)
    * https://github.com/SchapplM/robsynth-structdimsynth (tested with rev. b10cb4f; 2021-02-04)
    * https://github.com/SchapplM/robsynth-modelgen (tested with rev. 33b345a; 2021-02-02)
    * https://github.com/SchapplM/matlab_toolbox (tested with rev. 2934a77; 2020-12-18)
    * https://github.com/SchapplM/robotics-dep-geometry (tested with rev. 53aa6d12; 2020-09-13)
    * https://github.com/SchapplM/robotics-dep-traj (tested with rev. 41ceefd4; 2020-02-15)
    * https://github.com/SchapplM/robotics-dep-ext (tested with rev. d85bdea;2021-01-24)
    * https://github.com/SchapplM/matlab_pbs_transfer (tested with rev. 635ecd8; 2021-01-26)
5. Set up the toolboxes to work and modify the following files according to the instructions
    * robsynth-serroblib/maplerepo_path.m.template (has to link to robsynth-modelgen)
    * robsynth-modelgen/robotics_repo_path.template (has to link to robsynth-robotics_toolbox)
    * robsynth-structdimsynt/computingcluster_repo_path.m.template (has to link to matlab_pbs_transfer)
6. Run the path initialization scripts (`..._path_init.m`) of all downloaded software repos in Matlab. Put the run commands in the startup.m file of Matlab for easier use.

