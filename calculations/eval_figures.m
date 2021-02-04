% Bild für Vergleich unterschiedlicher Ergebnisse der Maßsynthese

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc
close all

%% Definitionen
outputdir = fileparts(which('eval_figures.m'));

%% Öffnen der Ergebnis-Tabelle
% (Wird in results_stack_tables.m erstellt)
tablepath = fullfile(outputdir, 'results_all_reps.csv');
ResTab = readtable(tablepath, 'ReadVariableNames', true);
% Liste der Roboter: Fasse mehrere Wiederholungen für jeden Roboter zusammen
Robots = unique(ResTab.Name);
% Zum Plotten werden später nur funktionierende Ergebnisse genommen.
I_iO = ResTab.Fval_Opt < 1e3;
%% Bild zeichnen: Konditionszahl vs Gelenkwinkelbereich
figure(1);clf;hold on;
for i = 1:length(Robots)
  RobName = Robots{i};
  I_Robi = strcmp(ResTab.Name, RobName);
  I = I_Robi & I_iO;
  plot(ResTab.Kondition_phys(I), 180/pi*ResTab.Gelenkbereich_phys(I), 'x');
end
grid on;
xlabel('cond(J) in mm/mm'); % Einheit nur für 3T0R mit Schubgelenk so.
ylabel('max(q)-min(q) in deg'); % Nur, wenn nur Drehgelenke in Kennzahl.
set(gca, 'XScale', 'log');
export_fig(1, fullfile(outputdir, sprintf('figure_condition_jointrange.pdf')));
%% Bild zeichnen: Konditionszahl vs Materialspannung
figure(2);clf;hold on;
for i = 1:length(Robots)
  RobName = Robots{i};
  I_Robi = strcmp(ResTab.Name, RobName);
  I = I_Robi & I_iO;
  plot(ResTab.Kondition_phys(I), 100*ResTab.Materialspannung(I), 'x');
end
grid on;
xlabel('cond(J) in mm/mm'); % Einheit nur für 3T0R mit Schubgelenk so.
ylabel('Materialspannung in %');
set(gca, 'XScale', 'log');
export_fig(2, fullfile(outputdir, sprintf('figure_condition_matstress.pdf')));
%% Bild zeichnen: Konditionszahl vs Antriebskraft
figure(3);clf;hold on;
for i = 1:length(Robots)
  RobName = Robots{i};
  I_Robi = strcmp(ResTab.Name, RobName);
  I = I_Robi & I_iO;
  plot(ResTab.Kondition_phys(I), ResTab.Antriebskraft_phys(I), 'x');
end
grid on;
xlabel('cond(J) in mm/mm'); % Einheit nur für 3T0R mit Schubgelenk so.
ylabel('Antriebskraft in N');
set(gca, 'XScale', 'log');
export_fig(3, fullfile(outputdir, sprintf('figure_condition_vs_actuatorforce.pdf')));