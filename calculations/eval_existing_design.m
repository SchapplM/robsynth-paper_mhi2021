% Auswertung des aktuellen Entwurfs zum Vergleich mit der Maßsynthese

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear


%% Lade Roboterdefinition (aus anderem Versuch, zur Vereinfachung)
if isempty(which('mhi_dimsynth_data_dir'))
  error(['You have to create a file mhi_dimsynth_data_dir pointing to the ', ...
    'directory containing the results of the dimensional synthesis']);
end
resdirtotal = mhi_dimsynth_data_dir();
OptName = 'cryopkm_20210117_bisheuteabend2_ps11';
LfdNr = 12;
RobName = 'P3PRRRR8V1G1P2A1';
setfile = dir(fullfile(resdirtotal, OptName, '*settings.mat'));
d1 = load(fullfile(resdirtotal, OptName, setfile(1).name));
Set_i = cds_settings_update(d1.Set);
resfile = fullfile(resdirtotal, OptName, sprintf('Rob%d_%s_Endergebnis.mat', ...
  LfdNr, RobName));
tmp = load(resfile);
RobotOptRes_i = tmp.RobotOptRes;
Structure = tmp.RobotOptRes.Structure;

% Entferne Kollisionskörper für Zylinder-Rand. Führt zu Kollision bei 240mm
% Gestell, da Sicherheitsbereich in kominierter Synthese größer ist.
% Set_i.task.obstacles = struct( 'type', [], 'params', [] );
parroblib_addtopath({RobName});
[R, Structure] = cds_dimsynth_robot(Set_i, d1.Traj, Structure, true);

%% Ersetze die Parameter
vn = Structure.varnames;
pval = NaN(length(vn),1);
pval(strcmp(vn, 'scale')) = 1;
pval(strcmp(vn, 'pkin 2: a4')) = 334.6e-3;
pval(strcmp(vn, 'pkin 5: d4')) = 0;
pval(strcmp(vn, 'base z')) = 0.68 + 0.11;
% Aus Besprechung mit Philipp vom 25.09.2020. Reduziere den Gestellradius
% von dort wegen des höheren Sicherheitsbereichs in der kombinierten Synth.
pval(strcmp(vn, 'base radius')) = 240e-3 - 10.001e-3;
pval(strcmp(vn, 'platform radius')) = 80e-3;
assert(all(~isnan(pval)), 'p darf nicht NaN sein');

%% Berechne die Zielfunktion. Dadurch Detail-Kennzahlen zur Kinematik
Set = Set_i;
Set.general.plot_details_in_fitness = 1e11;
Set.general.plot_robot_in_fitness = 1e11;
Set.general.save_robot_details_plot_fitness_file_extensions = {};
cds_log(0, '', 'init', Set);
clear cds_save_particle_details cds_fitness
[fval, physval, Q] = cds_fitness(R, Set,d1.Traj, Structure, pval);
PSO_Detail_Data_tmp = cds_save_particle_details(Set, R, 0, 0, NaN, NaN, NaN, NaN, 'output');

%% Validiere die Ergebnisse

fprintf('Daten des Roboters:\n');
fprintf('Gestell-Radius: %1.3fmm\n', 1e3*R.DesPar.base_par(1));
fprintf('Plattform-Radius: %1.3fmm\n', 1e3*R.DesPar.platform_par(1));

%% Berechne zusätzliche Daten und speichere die Ergebnisse
% Siehe select_eval_robot_examples.m (dort das gleiche und kommentiert)
[~,~,~,jointrange] = cds_obj_jointrange(R, Set, Structure, Q);
Q_with_ref = [Q; repmat(R.Leg(1).DesPar.joint_stiffness_qref, 3, 1)'];
[~,~,~,jointrange_with_springrest] = cds_obj_jointrange(R, Set, Structure, Q_with_ref);
condJ = PSO_Detail_Data_tmp.constraint_obj_val(1, 4, 1);  
Traj_0 = cds_transform_traj(R, d1.Traj);
X = Traj_0.X;
datadir = fullfile(fileparts(which('eval_existing_design.m')),'..','data');
objective_names = Set_i.optimization.objective;
save(fullfile(datadir, sprintf('detail_result_engineering_solution.mat')), ...
  'R', 'pval', 'fval', 'physval', 'condJ', 'Structure', 'Q', 'X', ... % Daten zum Ergebnis
  'jointrange', 'jointrange_with_springrest', 'objective_names');
