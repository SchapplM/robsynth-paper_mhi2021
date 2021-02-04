% Werte die Simulationsergebnisse anhand ihres theta1-Parameter aus

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

dimsynthpath = fileparts(which('structgeomsynth_path_init.m'));
importdir = fullfile(dimsynthpath, 'results');
%% Definitionen
outputdir = fileparts(which('eval_figures.m'));

%% Zusammenfassungen der bisherige Versuche laden
% (Wird in results_stack_tables.m erstellt)
tablepath = fullfile(outputdir, 'results_all_reps.csv');
ResTab = readtable(tablepath, 'ReadVariableNames', true);

%% MDH-Parameter laden
% Initialisierung der neu zu erstellenden Tabelle
head_row = cell(1,5);
head_row(1:5) = {'OptName', 'LfdNr', 'Name', 'Erfolg_IK', 'theta1'};
ResTab_MDH = cell2table(cell(0,length(head_row)), 'VariableNames', head_row);
for i = 1:size(ResTab,1)
  OptName = ResTab.OptName{i}; % aktuell nur ein Durchlauf betrachtet.
  LfdNr = ResTab.LfdNr(i);
  RobName = ResTab.Name{i};
  resfile = fullfile(importdir, OptName, sprintf('Rob%d_%s_Endergebnis.mat', LfdNr, RobName));
  tmp = load(resfile);
  % Bestimme den MDH-Parameter theta1
  R = tmp.RobotOptRes.R;
  theta1 = R.Leg(1).MDH.theta(1)*180/pi;
  success = true;
  if contains(ResTab.Fval_Text(i), 'IK Fehler')
    % IK Maßsynthese war nicht erfolgreich
    success = false;
  end
  % In Tabelle eintragen
  Row_i = {OptName, LfdNr, RobName, success, theta1};
  % Datenzeile anhängen
  ResTab_MDH = [ResTab_MDH; Row_i]; %#ok<AGROW>
end
%% Auswertungstabelle speichern
exporttabpath = fullfile(outputdir, sprintf('theta1_eval.csv'));
writetable(ResTab_MDH, exporttabpath, 'Delimiter', ';');
fprintf('Auswertung zu MDH-Parametern nach %s geschrieben.\n', exporttabpath);

%% Auswertungsbild erstellen
figure(1);clf;hold on;
Robots = unique(ResTab_MDH.Name);
I_IKsucc = ResTab_MDH.Erfolg_IK==1;
for i = 1:length(Robots)
  RobName = Robots{i};
  I_Robi = strcmp(ResTab.Name, RobName);
  if any(I_Robi&I_IKsucc)
    hdl1=plot(i, ResTab_MDH.theta1(I_Robi&I_IKsucc), 'go');
  end
  if any(I_Robi&~I_IKsucc)
    plot(i, ResTab_MDH.theta1(I_Robi&~I_IKsucc), 'rx');
  end
end
hdl2=plot(NaN,NaN, 'rx'); % Für Legende
grid on;
legend([hdl1(1);hdl2(1)], {'IK i.O.', 'IK n.i.O.'}, ...
  'location', 'northoutside', 'orientation', 'horizontal');
xlabel('Rob lfd Nr.');
ylabel('theta1 [deg]');
sgtitle('Auswertung IK-Erfolg vs theta1');

%% Ursachen für IK-Erfolg klassifizieren
head_row = {'Name', 'G', 'P', 'Erfolg_0', 'Erfolg_90', 'Erfolg_dazwischen', ...
  'Erfolg_nur0', 'Erfolg_nur90', 'Erfolg_nur090', 'Erfolg_nurnicht0', 'Erfolg_nurnicht90', 'Erfolg_immer'};
ResTab_RobotDetail = cell2table(cell(0,length(head_row)), 'VariableNames', head_row);
expression = 'P(\d)([RP]+)(\d+)[V]?(\d*)[G]?(\d*)[P]?(\d*)';
for i = 1:length(Robots)
  RobName = Robots{i};
  [tokens, ~] = regexp(RobName,expression,'tokens','match');
  res = tokens{1};
  Coupling = [str2double(res{5}), str2double(res{6})];
  I_Robi = strcmp(ResTab.Name, RobName);
  % TODO: Besser -1,0,1 als Skala, wobei -1=kein Erfolg, 0=nicht geprüft, 1=Erfolg
  I_theta_0  = I_Robi&(abs(ResTab_MDH.theta1) < 1e-9);
  I_theta_90 = I_Robi&(abs(abs(ResTab_MDH.theta1)-90) < 1e-9);
  I_theta_x  = I_Robi&(~I_theta_0 & ~I_theta_90);
  % Annahme: Alle Kombinationen müssen probiert worden sein.
  if sum(I_theta_0) == 0,         s_theta_0 = 0;
  elseif any(I_theta_0&I_IKsucc), s_theta_0 = 1;
  else,                           s_theta_0 = -1;
  end
  if sum(I_theta_90) == 0,         s_theta_90 = 0;
  elseif any(I_theta_90&I_IKsucc), s_theta_90 = 1;
  else,                            s_theta_90 = -1;
  end
  if sum(I_theta_x) == 0,         s_theta_x = 0;
  elseif any(I_theta_x&I_IKsucc), s_theta_x = 1;
  else,                           s_theta_x = -1;
  end
  s_theta_only0 =      s_theta_0==1              &(s_theta_90==-1&s_theta_x==-1);
  s_theta_only90 =    s_theta_90==1              &(s_theta_0==-1 &s_theta_x==-1);
  s_theta_only090 =  (s_theta_90==1&s_theta_0==1)&s_theta_x==-1;
  s_theta_onlynot0 = (s_theta_90==1|s_theta_x==1)&s_theta_0==-1;
  s_theta_onlynot90 = (s_theta_0==1|s_theta_x==1)&s_theta_90==-1;
  Row_i = {RobName, Coupling(1), Coupling(2), ...
    s_theta_0, s_theta_90, s_theta_x, ...
    s_theta_only0,s_theta_only90,s_theta_only090,s_theta_onlynot0,s_theta_onlynot90, ...
    s_theta_0==1&s_theta_90==1&s_theta_x==1};
  ResTab_RobotDetail = [ResTab_RobotDetail; Row_i]; %#ok<AGROW>
  if s_theta_only0&&s_theta_only90 || s_theta_onlynot0&&s_theta_onlynot90
    warning('Logik-Fehler: Zu viele Einträge gesetzt.');
  end
end
% Sortieren nach G-P-Nummer als wahrscheinlichstem Kriterium
ResTab_RobotDetail = sortrows(ResTab_RobotDetail, [2 3]);
exporttabpath_IK = fullfile(outputdir, sprintf('theta1_IK_robot_eval.csv'));
writetable(ResTab_RobotDetail, exporttabpath_IK, 'Delimiter', ';');
fprintf('Auswertung zu IK-Erfolg für theta1 nach %s geschrieben.\n', exporttabpath_IK);