% Werte die Simulationsergebnisse anhand ihrer MDH-Parameter aus

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

importdir = mhi_dimsynth_data_dir();
%% Definitionen
outputdir = fileparts(which('compare_results_kinematics.m'));

%% Öffnen der Ergebnis-Tabelle
% (Wird in results_stack_tables.m erstellt)
tablepath = fullfile(outputdir, 'results_all_reps.csv');
ResTab = readtable(tablepath, 'ReadVariableNames', true);

%% MDH-Parameter laden
% Initialisierung der neu zu erstellenden Tabelle
head_row = cell(1,5);
head_row(1:3) = {'OptName', 'LfdNr', 'Name'};
% Füge alle möglichen PKM-Kinematikparameter hinzu
head_row(4:5) = {'r_base', 'r_plf'};
for i = 1:5
  mdh_types = {'a', 'd'};
  for j = 1:2
    head_row(end+1) = {sprintf('%s%d', mdh_types{j}, i)}; %#ok<SAGROW>
  end
end

ResTab_MDH = cell2table(cell(0,length(head_row)), 'VariableNames', head_row);
for i = 1:size(ResTab,1)
  if ResTab.Fval_Opt(i) > 1e3
    continue % Ungültige Lösung nicht weiter betrachten
  end
  OptName = ResTab.OptName{i}; % aktuell nur ein Durchlauf betrachtet.
  LfdNr = ResTab.LfdNr(i);
  RobName = ResTab.Name{i};
  resfile = fullfile(importdir, OptName, sprintf('Rob%d_%s_Endergebnis.mat', LfdNr, RobName));
  tmp = load(resfile);

  % Bestimme die Kinematik-Parameter der PKM
  R = tmp.RobotOptRes.R;

  d_mdh_all =  R.Leg(1).MDH.d;
  a_mdh_all =  R.Leg(1).MDH.a;

  % In Tabelle eintragen
  Row_i = {OptName, LfdNr, RobName, R.DesPar.base_par(1), R.DesPar.platform_par(1)};
  for kk = 1:5
    Row_i = [Row_i, a_mdh_all(kk), d_mdh_all(kk)]; %#ok<AGROW>
  end
  % Datenzeile anhängen
  ResTab_MDH = [ResTab_MDH; Row_i]; %#ok<AGROW>
end
%% Auswertungstabelle speichern
exporttabpath = fullfile(outputdir, sprintf('kinematics_eval.csv'));
writetable(ResTab_MDH, exporttabpath, 'Delimiter', ';');
fprintf('Auswertung zu Kinematik-Parametern nach %s geschrieben.\n', exporttabpath);
