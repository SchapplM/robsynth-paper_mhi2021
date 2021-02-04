% Prüfe, wie viele Strukturen prinzipiell möglich sind. Diese Zahlen stehen
% im Text am Anfang von Kap. 4

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-09
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

DoF = [1 1 1 0 0 0];
Set = cds_settings_defaults(struct('DoF', DoF));
% Set.general.verbosity = 0;
Set.structures.use_serial = false;
Structures1 = cds_gen_robot_list(Set);

Set.structures.max_index_active = 1;
Structures2 = cds_gen_robot_list(Set);

Set.structures.joint_filter = 'P****';
Structures3 = cds_gen_robot_list(Set);

Set.structures.parrob_basejointfilter = [1 4];
Structures4 = cds_gen_robot_list(Set);

% Zähle Anzahl der Beinketten
LegChain_Count = NaN(4,1);
for i = 1:4
  Structures = eval(sprintf('Structures%d',i));
  LegChains = cell(1, length(Structures));
  for j = 1:length(Structures)
    S = Structures{j};
    % Name der Beinkette
    expression = 'P(\d)([RP\dV]+)[G]?(\d*)[P]?(\d*)A(\d+)'; % Format "P3RRR1G1P1A1" oder "P3RRR1V1G1P1A1"
    [tokens, ~] = regexp(S.Name,expression,'tokens','match');
    if isempty(tokens)
      error('Eingegebener Name %s entspricht nicht dem Namensschema', S.Name);
    end
    res = tokens{1};
    LegChains{j} = res{2}; % Name der Beinkette (ohne Sx)
  end
  LegChain_Count(i) = length(unique(LegChains));
end

fprintf('Ohne weitere Einschränkungen: %d PKM; %d Beinketten.\n', ...
  length(Structures1), LegChain_Count(1));
if length(Structures1) ~= 328 || LegChain_Count(1) ~= 51
  warning('Die Menge der Strukturen hat sich gegenüber der Paper-Version geändert!');
end
fprintf('Nur angetriebenes erstes Gelenk: %d PKM; %d Beinketten.\n', ...
  length(Structures2), LegChain_Count(2));
if length(Structures2) ~= 109 || LegChain_Count(2) ~= 20
  warning('Die Menge der Strukturen hat sich gegenüber der Paper-Version geändert!');
end
fprintf('Erstes Gelenk Schubgelenk: %d PKM; %d Beinketten.\n', ...
  length(Structures3), LegChain_Count(3));
if length(Structures3) ~= 75 || LegChain_Count(3) ~= 9
  warning('Die Menge der Strukturen hat sich gegenüber der Paper-Version geändert!');
end
fprintf('Nur senkrechte/schräge Gestellgelenke: %d PKM; %d Beinketten.\n', ...
  length(Structures4), LegChain_Count(4));
if length(Structures4) ~= 33 || LegChain_Count(4) ~= 9
  warning('Die Menge der Strukturen hat sich gegenüber der Paper-Version geändert!');
end

% Wie viele PKM werde aussortiert?
repopath = fullfile(fileparts(which('number_possible_structures.m')), '..');
tablepath = fullfile(repopath, 'data', 'results_all_reps_pareto.csv');
ResTab = readtable(tablepath, 'ReadVariableNames', true);
I_iO = ResTab.Fval_Opt < 1e3;
Structures5_Names = unique(ResTab.Name(I_iO))';
fprintf('Erfüllung der Nebenbedingungen (Kollision, Bauraum, Kondition): %d PKM\n', length(Structures5_Names));
