% Lade alle Ergebnistabellen der einzelnen Wiederholungen der Maßsynthese
% und speichere sie in eine Tabelle

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc

%% Definitionen
outputdir = fileparts(which('results_stack_tables.m'));
if isempty(which('mhi_dimsynth_data_dir'))
  error(['You have to create a file mhi_dimsynth_data_dir pointing to the ', ...
    'directory containing the results of the dimensional synthesis']);
end
resdirtotal = mhi_dimsynth_data_dir();

%% Zusammenfassungen der bisherige Versuche laden
resdirs = {'cryopkm_20200903_jr_rep1', 'cryopkm_20200903_jr_rep2'};

for i = 1:length(resdirs)
  tablepath = fullfile(resdirtotal, resdirs{i}, sprintf('%s_results_table.csv', resdirs{i}));
  ResTab_i = readtable(tablepath, 'HeaderLines', 2);
  ResTab_i_headers = readtable(tablepath, 'ReadVariableNames', true);
  ResTab_i.Properties.VariableNames = ResTab_i_headers.Properties.VariableNames;
  ResTab_i = addvars(ResTab_i, repmat(resdirs(i),size(ResTab_i,1),1), 'Before', 1);
  ResTab_i.Properties.VariableNames(1) = {'OptName'};
  if i == 1
    ResTab_ges = ResTab_i;
  else
    ResTab_ges = [ResTab_ges; ResTab_i]; %#ok<AGROW>
  end
end
% Sortiere nach Fitness-Wert
ResTab_ges = sortrows(ResTab_ges, find(strcmp(ResTab_i.Properties.VariableNames,'Fval_Opt')));
writetable(ResTab_ges, fullfile(outputdir, 'results_all_reps.csv'), 'Delimiter', ';');

%% Bestes Ergebnis für jede Struktur in separate Tabelle
% Liste der Roboter: Fasse mehrere Wiederholungen für jeden Roboter zusammen
Robots = unique(ResTab_ges.Name);
I_ges = true(size(ResTab_ges,1));
ncols = length(ResTab_ges.Properties.VariableNames);
ResTab_Best = cell2table(cell(0,ncols), 'VariableNames', ResTab_ges.Properties.VariableNames);
for i = 1:length(Robots)
  RobName = Robots{i};
  % Finde das beste Ergebnis für den Roboter heraus
  II_Robi = find(strcmp(ResTab_ges.Name, RobName));
  [~,ii_Robi_best] = min(ResTab_ges.Fval_Opt(II_Robi));
  
  % An neue Tabelle anhhängen
  ResTab_Best = [ResTab_Best; ResTab_ges(II_Robi(ii_Robi_best),:)]; %#ok<AGROW>
end
% Sortieren: Bester oben
ResTab_Best = sortrows(ResTab_Best, 8);
% Speichern
exporttabpath = fullfile(outputdir, sprintf('results_best_robots.csv'));
writetable(ResTab_Best, exporttabpath, 'Delimiter', ';');