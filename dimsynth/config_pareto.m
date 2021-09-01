% Einstellungen für komb. Struktur und Maßsynthese für Kryo-PKM
% Mehrkriterielle Optimierung

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

% Schalter zum Nachverarbeiten der vom Cluster heruntergeladenen Ergebnisse
% (Zusammenfassen einer aufgeteilten Berechnung)
merge_results_only = false;
% Aufgaben-FG
DoF = [1 1 1 0 0 0];
% Einstellungs-Struktur initialisieren
Set = cds_settings_defaults(struct('DoF', DoF));
%% Einstellungen aus Aufgabengeometrie
Set = environment_cryotask(Set);

%% Trajektorie laden
Set.task.Ts = 5e-2;
Set.task.Tv = 1e-1;
Set.task.amax = 2;
Traj = traj_cryotask(Set.task);
% Debug: Visualisierung der Aufgabe
% cds_show_task(Traj, Set);
%% Lade PKM mit erfolgreicher einkriterieller Optimierung
% Wenn bisher keine erfolgreiche Konfiguration unter Einhaltung aller Neben- 
% bedingungen gefunden wurde, ergibt die Pareto-Optimierung keinen Sinn.
repopath = fullfile(fileparts(which('config_pareto.m')), '..');
tablepath = fullfile(repopath, 'data', 'results_all_reps_pareto.csv');
tmp = load(fullfile(repopath, 'data','results_all_reps_pareto.mat'));
ResTab = tmp.ResTab_ges;
I_iO = ResTab.Fval_Opt < 1e3; % Funktionierende PKM (Nebenbedingungen können erneut geprüft werden)
I_valid = ~isnan(ResTab.Masse_fval); % durch einen Fehler werden einige als Erfolgreich trotz nicht berechneter Traj. gekennzeichnet.
%% Sonstige Optimierungseinstellungen
optname = 'cryopkm_20210126_refine3';
num_repetitions = 2; % Anzahl der Wiederholungen der Optimierung
Set.optimization.NumIndividuals = 100;
Set.optimization.MaxIter = 100;
Set.general.plot_details_in_fitness = 0e7; % Debug-Plots für Kollisionen/Bauraum
% Set.general.plot_details_in_fitness = 5e3; % Debug-Plots für Gelenksprünge
Set.general.plot_robot_in_fitness = 0e3;
Set.general.verbosity = 3;
Set.general.matfile_verbosity = 3;
Set.general.save_robot_details_plot_fitness_file_extensions = {'fig'};
Set.general.animation_styles = {'3D'};
Set.general.save_animation_file_extensions = {'mp4'};
Set.general.maxduration_animation = 10;  % Länge begrenzen
% Auswahl der Strukturen.
% Mit Positiv-Liste (händische Auswahl aus den bisherigen i.O.-Ergebnissen)
Set.structures.whitelist = {'P3PRRRR3G4P2A1', 'P3PRRRR3V1G4P2A1', ...
  'P3PRRRR6G1P2A1', 'P3PRRRR6G1P3A1', ...
  'P3PRRRR6G4P2A1', 'P3PRRRR6V1G1P2A1', 'P3PRRRR6V1G1P3A1', ...
  'P3PRRRR6V1G4P2A1', 'P3PRRRR6V1G4P3A1', 'P3PRRRR8G1P2A1', ...
  'P3PRRRR8G1P3A1', 'P3PRRRR8G4P2A1', 'P3PRRRR8V1G1P2A1', ...
  'P3PRRRR8V1G1P3A1', 'P3PRRRR8V1G4P2A1', 'P3PRRRR8V2G1P2A1', ...
  'P3PRRRR8V2G1P3A1', 'P3PRRRR8V2G4P2A1'};
% Debug: Eine Struktur mehrfach optimieren
% Set.structures.whitelist = {'P3PRRRR3V1G4P2A1'};
% Set.structures.repeatlist = {{'P3PRRRR3V1G4P2A1', 6}};
% Automatische Auswahl aus den bisherigen Positiv-Ergebnissen
% Set.structures.whitelist = unique(ResTab.Name(I_iO&I_valid))';
% Für den Fall, dass keine Positiv-Liste genommen wird:
Set.structures.joint_filter = 'P****';
Set.structures.max_index_active = 1;
Set.structures.use_serial = false;
Set.structures.parrob_basejointfilter = [1 4];
% Die Steigung von konisch angeordneten Schubgelenken als Gestell soll auch
% optimiert werden
Set.optimization.base_morphology = true;
% Konditionszahl darf nicht total schlecht sein. Dann werden
% Antriebskräfte und innere Kräfte zu hoch (Erfahrungswerte)
Set.optimization.constraint_obj(4) = 200; % max. Wert für Konditionszahl
% Die Antriebskraft sollte nicht zu groß werden.
Set.optimization.constraint_obj(3) = 100; % max. Wert in N
% Die Materialspannung (innere Kräfte) sollten nicht zu groß werden
Set.optimization.constraint_obj(6) = 0.5; % 50% von Grenze für Abbruch
% Debug: Nur mit statischen Kräften rechnen. (zur Vereinfachung)
% Set.optimization.static_force_only = true;

% Optimiere die Nullstellung der Gelenkelastizitäten als Entwurfsopt.
Set.optimization.desopt_vars = {'joint_stiffness_qref'};

% Debug: Benutze GA
% Set.optimization.algorithm = 'gamultiobj';

% Vereinfachung: Nur Masse auf der Plattform (3kg) betrachten
% Set.optimization.nolinkmass = true;
% Set.optimization.noplatformmass = true;
% Debug: Abbruchkriterien so definieren, dass nur eine einzige gültige
% Lösung direkt genommen wird
% Set.optimization.obj_limit = [1e3;1e3]; % Unter 1e3 ist gültig.
% Vorlagen neu erzeugen, falls veraltete Dateien vorliegen
Set.general.create_template_functions = false;
Set.general.parcomp_struct = true; % Maßsynthese parallel durchführen
Set.general.parcomp_plot = true; % Bilder parallel erzeugen
Set.general.computing_cluster = true; % Auf Cluster rechnen
Set.general.cluster_maxrobotspernode = 24; % Aufteilen, wenn mehr.
% Debug: Bei vorzeitigem Abbruch Ergebnisse rekonstruieren
Set.general.only_finish_aborted = false;
% Set.general.nosummary = true; % Nur Testen des Durchlaufs
% Zusätzlich die Dynamik-Komponenten auswerten
Set.general.eval_figures = [Set.general.eval_figures,'dynamics'];
% Keine alten Ergebnisse laden (Zeigen der Reproduzierbarkeit)
Set.optimization.InitPopRatioOldResults = 0;
% Fast nur alte Ergebnisse laden und mit diesen weiterrechnen
% Wird benutzt, um bestehende Pareto-Front von Gut-Ergebnissen zu verbessern.
Set.optimization.InitPopRatioOldResults = 0.95;
%% Starten
for ps = 11 %[1 2 11 12 13] % Pareto-Kombinationen
  if ps == 1
    Set.optimization.objective = {'condition', 'actforce'};
  elseif ps == 2
    Set.optimization.objective = {'jointrange', 'actforce'};
  elseif ps == 3
    Set.optimization.objective = {'jointrange', 'energy'};
  elseif ps == 4
    Set.optimization.objective = {'condition', 'energy'};
  elseif ps == 5
    % Nur kinematische Optimierung (für Ersteinreichung)
    Set.optimization.objective = {'condition', 'jointrange'};
  elseif ps == 6
    % Zum Herausfinden der Korrelation mit actforce
    Set.optimization.objective = {'manipulability', 'actforce'};
  elseif ps == 7
    Set.optimization.objective = {'minjacsingval', 'actforce'};
  elseif ps == 8
    % Kinematische Indizes
    Set.optimization.objective = {'manipulability', 'jointrange'};
  elseif ps == 9
    % Kinematische Indizes
    Set.optimization.objective = {'minjacsingval', 'jointrange'};
  elseif ps == 10
    % Kinematische Indizes
    Set.optimization.objective = {'positionerror', 'jointrange'};
  elseif ps == 11
    Set.optimization.objective = {'positionerror', 'actforce'};
  elseif ps == 12
    Set.optimization.objective = {'materialstress', 'actforce'};
  elseif ps == 13
    Set.optimization.objective = {'materialstress', 'jointrange'};
  elseif ps == 14
    Set.optimization.objective = {'chainlength', 'actforce'};
  end
  for k = 1:num_repetitions
    if num_repetitions == 1 % Ohne Wiederholung
      Set.optimization.optname = sprintf('%s_ps%d', optname, ps);
    else % Mehrfache Durchführung
      Set.optimization.optname = sprintf('%s_ps%d_rep%d', optname, ps, k);
    end
    if ~merge_results_only
      cds_start(Set, Traj);
      pause(30); % Damit nicht alle exakt zeitgleich starten; paralleler Start des parpools nicht möglich
    else
      cds_merge_results( Set.optimization.optname, 'copy', true, true );
    end
  end
end

%% Debug: Nur Ergebnis-Bilder neu erzeugen
return
% Blanko-Einstellungen reichen. Werden neu geladen.
Set = cds_settings_defaults(struct('DoF', [1 1 1 0 0 0]));
Traj = [];
% Debug: Nur Tabelle neu generieren:
Set.general.regenerate_summary_only = true;
Set.general.eval_figures = {'pareto_all_phys'};
Set.general.animation_styles = {};
Set.general.parcomp_plot = false;
Set.optimization.optname = 'cryopkm_20210108_ps11_p2'; % Beispiel zum Neu-Erstellen
cds_start(Set, Traj);
