% Erzeuge ein Bild für die Aufgabe des Roboters

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

%% Führe die Initialisierung der Aufgabe aus
this_path = fileparts(which('robot_task.m'));
addpath(fullfile(this_path, '..', '..', 'dimsynth'));
Set = cds_settings_defaults(struct('DoF', [1 1 1 0 0 0]));
Set = environment_cryotask(Set);
Traj = traj_cryotask(Set.task);
%% Speichere das Bild
% Bild zeichnen
cds_show_task(Traj, Set)

figure_format_publication(gca);
set(gca, 'Box', 'off');
sgtitle(''); title('');
set_size_plot_subplot(3, ...
  12,8,gca,...
  0,0,0,0,0,0)
drawnow();
% Bild speichern
outputdir = fileparts(which('robot_task.m')); % Ordner dieser Datei
  export_fig(['robot_task', '_r864.png'], '-r864')