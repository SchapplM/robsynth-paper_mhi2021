% Werte die Simulationsergebnisse anhand ihrer G4-Parameter aus
% Untersuche, ob die Kegelsteigung des Gestells richtig umgesetzt wurde
% 
% Vorher ausführen:
% * Aggregation der Daten mit eval_figures_pareto_groups.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

if isempty(which('mhi_dimsynth_data_dir'))
  error(['You have to create a file mhi_dimsynth_data_dir pointing to the ', ...
    'directory containing the results of the dimensional synthesis']);
end
importdir = mhi_dimsynth_data_dir();
%% Definitionen
outputdir = fileparts(which('eval_results_G4_parameters.m'));
datadir = fullfile(outputdir,'..','data');

%% Zusammenfassungen der bisherige Versuche laden
% (Wird in eval_figures_pareto.m erstellt)
tmp = load(fullfile(datadir, 'results_all_reps_pareto.mat'), 'ResTab_ges');
ResTab = tmp.ResTab_ges;
I_G4 = contains(ResTab.Name, 'G4');
ResTab = ResTab(I_G4, :);
I_iO = ResTab.Fval_Opt < 1e3; % nehme nur i.O. Versuche
ResTab = ResTab(I_iO, :);
Robots = unique(ResTab.Name);
%% MDH-Parameter laden
% Initialisierung der neu zu erstellenden Tabelle
head_row = cell(1,2);
head_row(1:2) = {'RobName', 'G4_alpha'};
ResTab_Details = cell2table(cell(0,length(head_row)), 'VariableNames', head_row);
for i = 1:length(Robots)
  RobName = Robots{i};
  % Pareto-Front für Roboter laden
  robtablepath = fullfile(datadir, sprintf('%s_paretofront.csv',RobName));
  if ~exist(robtablepath, 'file')
    continue
  end
  RobParetoTable = readtable(robtablepath, 'ReadVariableNames', true, 'Delimiter', ';');

  % Gehe alle Simulationsläufe durch und lade die jeweiligen Daten
  OptNames_Robi = unique(RobParetoTable.OptName);
  Row_i = {};
  for j = 1:length(OptNames_Robi)
    OptName = OptNames_Robi{j};

    % Lade die Daten
    LfdNr = ResTab.LfdNr(strcmp(ResTab.Name,RobName) & strcmp(ResTab.OptName,OptName));
    resfile = fullfile(importdir, OptName, sprintf('Rob%d_%s_Endergebnis.mat', LfdNr, RobName));
    tmp = load(resfile);
    % Bestimme maximale Gelenkauslenkung für alle Gelenke
    Ip_G4elev = strcmp(tmp.RobotOptRes.Structure.varnames, 'base_morph_coneelev');
    if sum(Ip_G4elev) == 0
      error('Geladene Parameter passen nicht. Filter sind fehlerhaft.');
    end
    % Wähle nur die Pareto-optimalen Punkte aus
    Ipar = RobParetoTable.ParetoIndNr(strcmp(RobParetoTable.OptName, OptName));
    alpha = tmp.RobotOptRes.p_val_pareto(Ipar,Ip_G4elev)*180/pi;
    % In Tabelle eintragen
    for k = 1:length(alpha)
      Row_i = [Row_i; {RobName, alpha(k)}]; %#ok<AGROW>
    end
    % Debug: Wie sieht die Verteilung des alpha-Parameters während der
    % Optimierung aus? Daten liegen noch nicht vor.
    % tmp.PSO_Detail_Data...
  end
  % Datenzeile anhängen
  ResTab_Details = [ResTab_Details; Row_i]; %#ok<AGROW>
end
%% Auswertungstabelle speichern
exporttabpath = fullfile(outputdir, sprintf('cone_alpha_eval.csv'));
writetable(ResTab_Details, exporttabpath, 'Delimiter', ';');
fprintf('Auswertung zu Kegelsteigungs-Parameter nach %s geschrieben.\n', exporttabpath);
