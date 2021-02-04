% Einstellungen für komb. Struktur und Maßsynthese für Kryo-PKM (einkriteriell)
% Szenario: Griff von oben in eine Tonne. Vermeide Kollisionen mit Umgebung

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-07
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
% cds_show_task(Traj, Set)

%% Sonstige Optimierungseinstellungen
Set.optimization.objective = 'actforce';
optname = 'cryopkm_20210117_so2';
num_repetitions = 1; % Anzahl der Wiederholungen der Optimierung
Set.optimization.NumIndividuals = 100;
Set.optimization.MaxIter = 100;
Set.general.plot_details_in_fitness = 0*1e4; % Debug-Plots für Kollisionen/Bauraum
% Set.general.plot_details_in_fitness = 0*5e3; % Debug-Plots für Gelenksprünge
Set.general.plot_robot_in_fitness = 0*1e4;
Set.general.verbosity = 4;
Set.general.matfile_verbosity = 3;
Set.general.save_robot_details_plot_fitness_file_extensions = {'fig'};
Set.general.animation_styles = {'3D'};
Set.general.save_animation_file_extensions = {'mp4'};
Set.general.maxduration_animation = 10;  % Länge begrenzen
% Nur Parallele Roboter (wegen passiver Gelenke im Kaltbereich)
Set.structures.use_serial = false;
% Nur erstes Gelenk darf aktiv sein (wegen Kryo-Umgebung in Behälter)
Set.structures.max_index_active = 1;
% Erstes Gelenk soll Schubgelenk sein (Einschätzung des Entwicklers:
% Drehgelenke führen zu einer zu ausladenden Struktur)
Set.structures.joint_filter = 'P****';
% Die Steigung von konisch angeordneten Schubgelenken als Gestell soll auch
% optimiert werden
Set.optimization.base_morphology = true;
% Nehme nur Koppelgelenk-Modi, bei denen das gestellfeste Gelenk nach oben
% zeigt (oder schräg) (in der Ebene gibt sowieso Bauraumverletzungen)
Set.structures.parrob_basejointfilter = [1 4];
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
% Debug: Lokale Ergebnisordner für zusätzliche Anfangswerte
% Set.optimization.result_dirs_for_init_pop = { ...
%   '/mnt/FP500/IMES/CLUSTER/REPO/structgeomsynth/results', ...
%   '/mnt/FP500/IMES/PRJ/imes-projekt-dfg_robotersynthese/03_Entwicklung/match_Kryo_PKM'};

% Vereinfachung: Nur Masse auf der Plattform (3kg) betrachten
% Set.optimization.nolinkmass = true;
% Set.optimization.noplatformmass = true;
% Debug: Abbruchkriterien so definieren, dass nur eine einzige gültige
% Lösung direkt genommen wird
% Set.optimization.obj_limit = 1e3; % Unter 1e3 ist gültig.
% Vorlagen neu erzeugen, falls veraltete Dateien vorliegen
Set.general.create_template_functions = false;
% Mögliche Kandidaten, die halbwegs gut für obigen Aufgabe funktionieren:
% P3RRRRR6G2P2A1, P3RRRRR7G3P3A1, P3RRRRR10G2P2A2, P3PRRRR4G1P2A1
% Set.structures.whitelist = {'P3PRRRR8V1G4P2A1'}; % P3PRRRR6V1A1
Set.general.parcomp_struct = true; % Maßsynthese parallel durchführen
Set.general.parcomp_plot = true; % Bilder parallel erzeugen
Set.general.computing_cluster = true; % Auf Cluster rechnen
% Set.general.cluster_maxrobotspernode = 12;
% Debug: Bei vorzeitigem Abbruch Ergebnisse rekonstruieren
Set.general.only_finish_aborted = false;
% Set.general.nosummary = true; % Nur Testen des Durchlaufs
% Zusätzlich die Dynamik-Komponenten auswerten
Set.general.eval_figures = [Set.general.eval_figures,'dynamics'];
% Keine alten Ergebnisse laden (Zeigen der Reproduzierbarkeit)
% Set.optimization.InitPopRatioOldResults = 0;
%% Starten
for k = 1:num_repetitions
  if num_repetitions == 1 % Ohne Wiederholung
    Set.optimization.optname = optname;
  else % Mehrfache Durchführung
    Set.optimization.optname = sprintf('%s_rep%d', optname, k);
  end
  if ~merge_results_only
    cds_start
    pause(30); % Damit nicht alle exakt zeitgleich starten; paralleler Start des parpools nicht möglich
  else
    cds_merge_results( Set.optimization.optname, 'copy', true, true );
  end
end
