% Führe die Auswertung der Ergebnisse einmal vollständig durch.

% Moritz Schappler, schappler@imes.uni-hannover.de, 2021-01
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc

this_path = fileparts( mfilename('fullpath') );
addpath(this_path);
run(fullfile(fileparts(which('run_evaluation.m')), 'calculations', ...
  'eval_figures_pareto.m')); close all;
run(fullfile(fileparts(which('run_evaluation.m')), 'calculations', ...
  'robot_names.m')); close all;
run(fullfile(fileparts(which('run_evaluation.m')), 'calculations', ...
  'eval_figures_pareto_groups.m')); close all;
run(fullfile(fileparts(which('run_evaluation.m')), 'calculations', ...
  'select_eval_robot_examples.m')); close all;
run(fullfile(fileparts(which('run_evaluation.m')), 'paper', 'tables', ...
  'results_tables_latex.m')); close all;
run(fullfile(fileparts(which('run_evaluation.m')), 'paper', 'figures', ...
  'robot_images.m')); close all;
