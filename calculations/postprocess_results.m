% Nachverarbeitung der Maßsynthese-Ergebnisse für gezieltere Auswertung

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-07
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

%% Definitionen
outputdir = fileparts(which('results_stack_tables.m'));
if isempty(which('mhi_dimsynth_data_dir'))
  error(['You have to create a file mhi_dimsynth_data_dir pointing to the ', ...
    'directory containing the results of the dimensional synthesis']);
end
resdirtotal = mhi_dimsynth_data_dir();

%% Öffnen der Ergebnis-Tabelle
% (Wird in results_stack_tables.m erstellt)
tablepath = fullfile(outputdir, 'results_all_reps.csv');
ResTab = readtable(tablepath, 'ReadVariableNames', true);
% Initialisierung der neu zu erstellenden Tabelle
NLEG = 3;
NLEGJmax = 5;
head_row = cell(1,5+NLEG*NLEGJmax);
head_row(1:5) = {'OptName', 'LfdNr', 'Name', 'Erfolg', 'qr_max_revolute'};
for i = 1:NLEG
  for j = 1:NLEGJmax
    if j == 1, unit = 'mm'; else, unit = 'deg'; end
    head_row{5+(i-1)*NLEGJmax+j} = sprintf('qr_L%d_J%d_in_%s', i, j, unit);
  end
end
ResTab_JR = cell2table(cell(0,length(head_row)), 'VariableNames', head_row);

%% Auslesen der Detail-Ergebnisse
for i = 1:size(ResTab,1)
  OptName = ResTab.OptName{i};
  LfdNr = ResTab.LfdNr(i);
  RobName = ResTab.Name{i};
  resfile = fullfile(resdirtotal, OptName, sprintf('Rob%d_%s_Endergebnis.mat', LfdNr, RobName));
  tmp = load(resfile);
  % Bestimme maximale Gelenkauslenkung für alle Gelenke
  Q = tmp.RobotOptRes.Traj_Q;
  R = tmp.RobotOptRes.R;
  q_range = NaN(R.NJ, 1);
  q_range(R.MDH.sigma==1) = diff(minmax2(Q(:,R.MDH.sigma==1)')');
  q_range(R.MDH.sigma==0) = angle_range( Q(:,R.MDH.sigma==0));
  % Abgleich mit Kennzahl aus Gelenkbereich
  Itab = strcmp(ResTab.Name, RobName);
  if abs(max(q_range) - ResTab.Gelenkbereich_phys(Itab)) > 1e-6
    warning('Maximaler Gelenkbereich in Tabelle nicht konsistent mit Ergebnisdatei');
    continue
  end
  if any(isnan(q_range))
    % warning('Gelenkwinkelbereich konnte nicht bestimmt werden (vermutlich Traj. falsch)');
    continue
  end
  % Umrechnung in Winkelgrad
  q_range_eng = q_range ./ cat(1, R.Leg(:).qunitmult_eng_sci);
  
  success = true;
  if ResTab.Fval_Opt(Itab) > 1e3
    % Maßsynthese war nicht erfolgreich
    success = false;
  end
  % In Tabelle eintragen
  Row_i = {OptName, LfdNr, RobName, success, max(q_range_eng(R.MDH.sigma==0))};
  for k = 1:NLEG
    for j = 1:NLEGJmax
      Row_i = [Row_i, q_range_eng(R.I1J_LEG(k)+j-1)]; %#ok<AGROW>
    end
  end
  % Datenzeile anhängen
  ResTab_JR = [ResTab_JR; Row_i]; %#ok<AGROW>
end
% Tabelle neu sortieren: Bester oben. Gültige Lösungen zuerst
ResTab_JR = sortrows(ResTab_JR, [-4 5]);
% Detailauswertung speichern
exporttabpath = fullfile(outputdir, 'jointrange_eval.csv');
writetable(ResTab_JR, exporttabpath, 'Delimiter', ';');
fprintf('Auswertung zu Gelenkbereich nach %s geschrieben.\n', exporttabpath);
