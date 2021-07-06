% Werte Pareto-Fronten verschiedener Roboter aus
% Funktioniert nur, wenn wenige Kriterien gewählt werden. Ansonsten werden
% es eher Streudiagramme
% 
% Quellen:
% [CoelloPulLec2004] Handling Multiple Objectives With Particle Swarm Optimization


% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-09
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

%% Initialisierung
if isempty(which('mhi_dimsynth_data_dir'))
  error(['You have to create a file mhi_dimsynth_data_dir pointing to the ', ...
    'directory containing the results of the dimensional synthesis']);
end
resdirtotal = mhi_dimsynth_data_dir();

outputdir = fileparts(which('eval_figures_pareto.m'));

ps = 11; % Auch andere Kombinationen von Kriterien möglich
% Mögliche Zielkriterien: condition, actforce, jointrange, energy
if ps ==  1, pareto_settings = {'condition', 'actforce'}; end
if ps ==  2, pareto_settings = {'actforce', 'jointrange'}; end
if ps ==  3, pareto_settings = {'condition', 'jointrange'}; end
if ps == 10, pareto_settings = {'positionerror', 'jointrange'}; end
if ps == 11, pareto_settings = {'positionerror', 'actforce'}; end

%% Auswahl der Ergebnisse und Ergebnisse durchgehen
% Namensschema: Siehe dimsynth/config_pareto.m
% Ergebnis-Ordner müssen im Datenverzeichnis liegen (entweder Offline-Aus-
% wertung oder neue Maßsynthese).
resdirs = {};
for i = 11
  for j = 1:5
    resdirs = {resdirs{:}, sprintf('cryopkm_20210122_newcoll_nobasexx_ps%d_rep%d', i, j)}; %#ok<CCAT>
  end
  % GA-Ergebnisse ändern nicht prinzipiell den MOPSO-Ansatz (der hat
  % wesentlich bessere Exploration). Nehme zusätzlich, da Pareto-Front
  % damit dichter ist.
  for j = 1:2
    resdirs = {resdirs{:}, sprintf('cryopkm_20210122_newcoll_nobase_ga_ps%d_rep%d', i, j)}; %#ok<CCAT>
  end
  resdirs = {resdirs{:}, 'cryopkm_20210125_P3PRRRR3V1G4P2A1_refine2_mopso_ps11'}; %#ok<CCAT>
  for j = 1:2
    resdirs = {resdirs{:}, sprintf('cryopkm_20210126_refine3_ga_ps%d_rep%d', i, j)}; %#ok<CCAT>
  end
  for j = 1:2
    resdirs = {resdirs{:}, sprintf('cryopkm_20210126_refine3_ps%d_rep%d', i, j)}; %#ok<CCAT>
  end
end

for i = 1:length(resdirs)
  tablepath = fullfile(resdirtotal, resdirs{i}, sprintf('%s_results_table.csv', resdirs{i}));
  % Finde die Zielfunktionen heraus (Auslesen der ersten Einstellungsdatei
  % reicht)
  setfile = dir(fullfile(resdirtotal, resdirs{i}, '*settings.mat'));
  if isempty(setfile)
    warning('Einstellungsdatei und damit Ergebnis %s existiert nicht', resdirs{i});
    continue
  end
  d1 = load(fullfile(resdirtotal, resdirs{i}, setfile(1).name));
  Set_i = d1.Set;
  objstr = '';
  for j = 1:length(Set_i.optimization.objective)
    objstr = [objstr, ',', Set_i.optimization.objective{j}]; %#ok<AGROW>
  end
  objstr = objstr(2:end); % Erstes Komma entfernen
  ResTab_i = readtable(tablepath, 'HeaderLines', 1);
  ResTab_i_headers = readtable(tablepath, 'ReadVariableNames', true);
  ResTab_i.Properties.VariableNames = ResTab_i_headers.Properties.VariableNames;
  ResTab_i = addvars(ResTab_i, repmat(resdirs(i),size(ResTab_i,1),1), 'Before', 1);
  ResTab_i.Properties.VariableNames(1) = {'OptName'};
  ResTab_i = addvars(ResTab_i, repmat({objstr},size(ResTab_i,1),1), 'Before', 8);
  ResTab_i.Properties.VariableNames(8) = {'Zielfunktion'};
  if i == 1
    ResTab_ges = ResTab_i;
  else
    ResTab_ges = [ResTab_ges; ResTab_i]; %#ok<AGROW>
  end
end
writetable(ResTab_ges, fullfile(outputdir, '..', 'data', 'results_all_reps_pareto.csv'), ...
  'Delimiter', ';');
save(fullfile(outputdir, '..', 'data', 'results_all_reps_pareto.mat'), 'ResTab_ges');
%% Alle Roboter einzeln durchgehen
markerlist = {'x', 's', 'v', '^', '*', 'o'};
colorlist =  {'r', 'g', 'b', 'c', 'm', 'k'};
Robots = unique(ResTab_ges.Name);
% Filter bestimmte Robotertypen heraus (wurden teilweise versehentlich gestartet)
Robots = Robots(~contains(Robots, 'G2'));
Robots = Robots(~contains(Robots, 'G3'));

I_robleg = false(length(Robots), 1);
% Betrachte nur Optimierungsläufe mit i.O.-Ergebnis
I_iO = ResTab_ges.Fval_Opt < 1e3;
% Betrachte nur Optimierungsläufe mit passender Zielfunktion
I_objmatch = contains(ResTab_ges.Zielfunktion,pareto_settings{1}) & ...
             contains(ResTab_ges.Zielfunktion,pareto_settings{2});
figure(10*ps+1);clf;
figure(10*ps+2);clf;
leghdl = [];
legstr = {};
countrob = 0;
countmarker = 0;
for i = 1:length(Robots)
  RobName = Robots{i};
  I_robmatch = strcmp(ResTab_ges.Name, RobName);
  % Generiere die Marker-Symbole bereits vor der Prüfung, ob die richtigen
  % Zielfunktionen gewählt sind. Dadurch wird sichergestellt, dass die
  % Marker in allen Pareto-Diagrammen gleich sind.
  if any(I_iO&I_robmatch)
    countmarker = countmarker + 1; % Hochzählen für die Marker und Farben
    ic = mod((countmarker-1),6)+1; % Index für Farben und Marker generieren
    im = ceil(countmarker/6);
    marker = [markerlist{im}, colorlist{ic}];
  end
  II_Robi = find(I_iO&I_objmatch&I_robmatch);
  if isempty(II_Robi)
    continue
  end
  fprintf('Rob %d (%s): Lade Daten (%d i.O.-Wiederholungen)\n', i, RobName, length(II_Robi));
  
  countrob = countrob + 1; % Für Legende: Nur erfolgreiche PKM zählen
  I_robleg(i) = true;
  hdl = NaN(2,1);
  numrep_i = 0;
  pt_i = cell2table(cell(0,2), 'VariableNames', {'OptName', 'ParetoIndNr'});
  %% Stelle Pareto-Front aus verschiedenen Durchläufen zusammen
  pf_data = []; % Pareto-Front mit physikalischen Daten. Spalten bezogen auf pareto_settings
  pf_data_fval = []; % Funktionswerte der Pareto-Front (zur einfacheren Zuordnung zu den Optimierungsergebnissen)
  for j = 1:length(II_Robi) % Verschiedene Gut-Durchläufe durchgehen
    OptName = ResTab_ges.OptName{II_Robi(j)};
    LfdNr = ResTab_ges.LfdNr(II_Robi(j));
    resfile = fullfile(resdirtotal, OptName, sprintf('Rob%d_%s_Endergebnis.mat', LfdNr, RobName));
    tmp = load(resfile);
    RobotOptRes_j = tmp.RobotOptRes;
    kk1 = find(strcmp(Set_i.optimization.objective, pareto_settings{1}));
    kk2 = find(strcmp(Set_i.optimization.objective, pareto_settings{2}));
    % Wähle nur Durchläufe, bei denen nur die gewählten Kriterien für
    % die Pareto-Front benutzt wurden. Sonst eher Streudiagramm.
    if length(Set_i.optimization.objective)>2 || isempty(kk1) || isempty(kk2)
      continue % Entweder zu viele oder die falschen Zielkriterien
    end
    
    pf_data = [pf_data; RobotOptRes_j.physval_pareto(:,[kk1,kk2])]; %#ok<AGROW>
    pf_data_fval = [pf_data_fval; RobotOptRes_j.fval_pareto(:,[kk1,kk2])]; %#ok<AGROW>
    row_i = cell(size(RobotOptRes_j.physval_pareto,1),2);
    row_i(:,1) = repmat({OptName},size(RobotOptRes_j.physval_pareto,1),1);
    for k = 1:size(RobotOptRes_j.physval_pareto,1)
      row_i{k,2} = k;
    end
    pt_i = [pt_i; row_i]; %#ok<AGROW>
    numrep_i = numrep_i + 1;
  end
  [~, Ikk] = sort(pf_data_fval(:,1)); % Sortiere nach erstem Kriterium
  pt_i = pt_i(Ikk,:);
  pf_data = pf_data(Ikk,:);
  pf_data_fval = pf_data_fval(Ikk,:);
  % Erstelle eine einzige Pareto-Front. Die Fronten mehrere Durchläufe sind
  % durch die heuristische Optimierung gegenseitig dominant.
  % Definition Pareto-Front: Siehe CoelloPulLec2004 Gl. (1)-(6)
  Idom_ges = pareto_dominance(pf_data);
  fprintf(['Durch mehrfache Durchführung der Optimierung müssen %d/%d Partikel ', ...
    'von der Pareto-Front entfernt werden.\n'], sum(Idom_ges), length(Idom_ges));
  pf_data = pf_data(~Idom_ges,:);
  pf_data_fval = pf_data_fval(~Idom_ges,:);
  pt_i = pt_i(~Idom_ges,:);
  % Speichere die Ergebnisse der Daten für diesen Roboter
  writetable(pt_i, fullfile(outputdir, '..', 'data', ...
    sprintf('%s_paretofront.csv', RobName)), 'Delimiter', ';');
  %% Zeichne die Ergebnisse in das Bild ein
  % Bild mit Zielfunktionswerten
  [obj_units, objscale] = cds_objective_plotdetails(Set_i);
  change_current_figure(10*ps+1); hold on;

  hdl(1)=plot(pf_data_fval(:,1), pf_data_fval(:,2), marker);
  xlabel(sprintf('%s (normalized)', pareto_settings{1}));
  ylabel(sprintf('%s (normalized)', pareto_settings{2}));
  grid on;
  % Bild mit physikalischen Werten
  change_current_figure(10*ps+2); hold on;
  hdl(2)=plot(objscale(1)*pf_data(:,1), ...
              objscale(2)*pf_data(:,2), marker);
  xlabel(sprintf('%s in %s', pareto_settings{1}, obj_units{1}));
  ylabel(sprintf('%s in %s', pareto_settings{2}, obj_units{2}));
  grid on;
  if isnan(hdl(1))
    warning('Keine Ergebnisse für Roboter %d (%s) gefunden.', i, Robots{i});
  else
    leghdl(countrob,:) = hdl; %#ok<SAGROW>
    legstr{countrob} = sprintf('%d/%d (%s); %d Wdh.', i, length(Robots), Robots{i}, numrep_i); %#ok<SAGROW>
  end
end
for jj = 1:2
  change_current_figure(10*ps+jj);
  set(10*ps+jj, 'numbertitle', 'off');
  if jj == 1
    title(sprintf('Pareto-Front %s vs %s (Fitness-Werte normalisiert)', pareto_settings{1}, pareto_settings{2}));
    legend(leghdl(:,1), legstr);
    set(10*ps+jj, 'name', sprintf('fval_%s_%s', pareto_settings{1}(1:4), pareto_settings{2}(1:4)));
  else
    title(sprintf('Pareto-Front %s vs %s (Fitness-Werte physikalisch)', pareto_settings{1}, pareto_settings{2}));
    legend(leghdl(:,2), legstr);
    set(10*ps+jj, 'name', sprintf('phys_%s_%s', pareto_settings{1}(1:4), pareto_settings{2}(1:4)));
  end
end
saveas(10*ps+2, fullfile(outputdir, sprintf('figure_pareto_%s_%s.fig', pareto_settings{1}, pareto_settings{2})));
export_fig(10*ps+2, fullfile(outputdir, sprintf('figure_pareto_%s_%s.pdf', pareto_settings{1}, pareto_settings{2})));

