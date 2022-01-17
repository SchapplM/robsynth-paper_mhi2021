% Wandle die Namen der Roboter in ein lesbares Format um
% Berechne dazu die Fitness-Funktion einmalig um Gelenkwinkel für die
% Referenzpunkte zu bestimmen. Dadurch wird die Parallelität der Gelenke
% geprüft.
% 
% Vorher ausführen:
% * eval_figures_pareto.m
% 
% Erzeugt Datei:
% * robot_names_latex.csv
% 
% Quelle:
% [KongGos2007] Kong, X., Gosselin, C.M.: Type synthesis of parallel
% mechanisms. Springer Berlin Heidelberg (2007)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

%% Definitionen
outputdir = fileparts(which('robot_names.m'));
datadir = fullfile(outputdir,'..','data');
if isempty(which('mhi_dimsynth_data_dir'))
  error(['You have to create a file mhi_dimsynth_data_dir pointing to the ', ...
    'directory containing the results of the dimensional synthesis']);
end
resdirtotal = mhi_dimsynth_data_dir();
serroblibpath=fileparts(which('serroblib_path_init.m'));
%% Öffnen der Ergebnis-Tabelle
% (Wird in results_stack_tables.m erstellt)
tablepath = fullfile(datadir, 'results_all_reps_pareto.csv');
ResTab = readtable(tablepath, 'ReadVariableNames', true);

%% Generiere die Zeichenfolge für die Gelenkkette
Robots = unique(ResTab.Name);
ResTab_NameTrans = cell2table(cell(0,6), 'VariableNames', {'PKM_Name', ...
  'Gnum', 'Pnum', 'Chain_Name', 'ChainStructure', 'Chain_ShortName'});
for i = 1:length(Robots)
  RobName = Robots{i};
  fprintf('Bestimme Bezeichnung für Rob %d (%s)\n', i, RobName);
  parroblib_addtopath({RobName});
  II_Robi = find(strcmp(ResTab.Name, RobName));
  j = II_Robi(1); % Lade das erste (geht nur um den Roboter selbst)

  %% Lade Ergebnis und Roboter
  OptName = ResTab.OptName{j};
  LfdNr = ResTab.LfdNr(j);
  setfile = dir(fullfile(resdirtotal, OptName, '*settings.mat'));
  d1 = load(fullfile(resdirtotal, OptName, setfile(1).name));
  Set = cds_settings_update(d1.Set);
  resfile = fullfile(resdirtotal, OptName, ...
    sprintf('Rob%d_%s_Endergebnis.mat', LfdNr, RobName));
  tmp = load(resfile);
  resfile_details = fullfile(resdirtotal, OptName, ...
    sprintf('Rob%d_%s_Details.mat', LfdNr, RobName));
  tmp2 = load(resfile_details);
  if isfield(tmp2.RobotOptDetails, 'q0') % altes Format
    q0 = tmp2.RobotOptDetails.q0;
  else % neues Format (nach Paper-Einreichung)
    q0 = tmp.RobotOptRes.q0;
  end
%   R = tmp.RobotOptRes.R;
  PName = tmp.RobotOptRes.Structure.Name;
  [~,LEG_Names] = parroblib_load_robot(PName);
  % Debug:
%   serroblib_create_template_functions(LEG_Names(1),false);
%   parroblib_create_template_functions({PName},false);
  parroblib_update_template_functions({PName});
%   matlabfcn2mex({[PName(1:end-6),'_invkin']}); % einige Dateien werden hiermit doppelt kompiliert
%   Chain_Name = tmp.RobotOptRes.R.Leg(1).mdlname_var;
  Chain_Name = LEG_Names{1};
  NLegJ = str2double(Chain_Name(2));
%   tmp.RobotOptRes.R.Leg(1).NJ;
  mdllistfile_Ndof = fullfile(serroblibpath, sprintf('mdl_%ddof', NLegJ), sprintf('S%d_list.mat',NLegJ));
  l = load(mdllistfile_Ndof, 'Names_Ndof', 'BitArrays_Origin', 'AdditionalInfo', 'BitArrays_Ndof');
  ilc = strcmp(l.Names_Ndof, Chain_Name);
  % Technische Gelenke bestimmen
  SName_TechJoint = fliplr(regexprep(num2str(l.AdditionalInfo(ilc,7)), ...
    {'1','2','3','4','5'}, {'R','P','C','U','S'}));
  %% Roboter-Klasse initialisieren
  [R, Structure] = cds_dimsynth_robot(Set, d1.Traj, d1.Structures{LfdNr}, true);
  % Parameter des Ergebnisses eintragen (für fkine-Berechnung unten)
  cds_update_robot_parameters(R, Set, Structure, tmp.RobotOptRes.p_val);
  % Gelenkwinkel des Startwerts für IK eintragen
  for kk = 1:R.NLEG
    R.Leg(kk).qref = q0(R.I1J_LEG(kk):R.I2J_LEG(kk));
  end
  % Fitness-Funktion nachrechnen um Gelenk-Trajektorie zu bestimmen. Ändere
  % die Einstellungen, so dass keine Dynamik berechnet wird (geht schneller).
  clear cds_save_particle_details cds_fitness
  Structure_tmp = Structure;
  Structure_tmp.calc_cut = false;
  Structure_tmp.calc_dyn_act = false;
  Structure_tmp.calc_spring_act = false;
  Structure_tmp.calc_spring_reg = false;
  Structure_tmp.calc_dyn_reg = false;
  % Erzwinge Prüfung dieses Anfangswerts für Trajektorie (falls IK anderes
  % Ergebnis hat). Diese Option sollte nicht notwendig sein. Wird zur
  % Sicherheit trotzdem gemacht.
  Structure_tmp.q0_traj = q0;
  Set.optimization.objective = {'condition'};
  Set.optimization.constraint_obj(:) = 0;
  Set.optimization.desopt_vars = {}; % keine Entwurfsoptimierung, hier nur Kinematik.
  Set.optimization.joint_stiffness_passive_revolute = 0;
  % Debug: Bei Verletzung von Zielfunktionen Bilder zeichnen
  % Set.general.plot_details_in_fitness = -1e3;
  % Keine Eingabe von Ergebnissen von Entwufsoptimierung.
  % Schubgelenk-Offsets hier neu berechnen (falls Konfiguration umklappt)
  [fval_i_test, ~, Q] = cds_fitness(R, Set,d1.Traj, ...
    Structure_tmp, tmp.RobotOptRes.p_val);
  if any(fval_i_test > 1e3)
    % Eigentlich darf dieser Fall nicht vorkommen. Ist aber aus numerischen
    % Gründen leider doch manchmal möglich.
    warning('Die Nebenbedingungen wurden bei erneuter Prüfung verletzt');
    if any(fval_i_test > 1e11) || ... % siehe cds_constraints.
        any(fval_i_test < 1e9) && any(fval_i_test > 1e4) % siehe cds_constraints_traj.
      % Versuche nochmal neu, die Fitness-Funktion zu berechnen
      error(['Keine gültige Gelenkwinkel berechnet. Parallelität der ', ...
        'Gelenke und damit Name nicht bestimmbar.'])
    end
  end
  %% Parallelität der Gelenke anzeigen (anhand der Trajektorie).
  % Direkte Kinematik für alle Zeitschritte berechnen
  Zges = NaN(size(Q,1), 3*NLegJ); % Alle z-Achsen speichern zum späteren Vergleich
  pgroups_all = zeros(size(Q,1), NLegJ);
  sigma_leg = R.Leg(1).MDH.sigma;
  for k = 1:size(Q,1)
    Tc = R.Leg(1).fkine(Q(k,1:NLegJ)');
    for kk = 1:NLegJ
      Zges(k,(kk-1)*3+1:kk*3) = Tc(1:3,3,1+kk);
    end
    % Werte die Parallelität der Achsen aus
    for jj = 1:NLegJ
      if sigma_leg(jj) == 1
        % Schubgelenk. Gruppe nicht zählen.
        continue
      elseif jj == 1
        pgroups_all(k, jj) = 1;
        continue
      end
      for kk = 1:jj-1
        if sigma_leg(kk) == 1
          % Parallelität zum Schubgelenk wird nicht betrachtet
          % TODO: Mit [KongGos2007] abgleichen, ob es dafür noch ein Symbol
          % gibt.
          continue
        end
        % Prüfe welches die erste z-Achse ist, die identisch mit der
        % aktuellen ist
        z_jj = Zges(k,(jj-1)*3+1:jj*3);
        z_kk = Zges(k,(kk-1)*3+1:kk*3);
        if all(abs(z_jj-z_kk) < 1e-10)
          pgroups_all(k, jj) = pgroups_all(k, kk); % Gelenk jj ist parallel zu Gelenk kk
          break
        else % Debug
          % deltaphi = acos(dot(z_jj,z_kk));
          % fprintf('Beingelenk %d vs %d: %1.1f deg Verdreht\n', jj, kk, 180/pi*deltaphi);
        end
      end
      if pgroups_all(k, jj) == 0
        pgroups_all(k, jj) = pgroups_all(k, jj-1) + 1; % Neue Gruppe
      end
    end
  end
  if any(any(diff(pgroups_all)))
    error('Die Parallelität ändert sich im Zeitverlauf. Darf nicht sein.');
  end
  pgroups = pgroups_all(1,:);
  %% Parallelität der Gelenke anzeigen (anhand der DH-Parameter).
  % Ist für PKM nicht vollständig, da durch die Zwangsbedingungen zusätzliche
  % Parallelitäten entstehen.
%   alpha_leg = tmp.RobotOptRes.R.Leg(1).MDH.alpha;
%   sigma_leg = tmp.RobotOptRes.R.Leg(1).MDH.sigma;
%   pgroups = zeros(NLegJ,1);
%   for j = 2:NLegJ
%     if sigma_leg(j) == 1
%       % Schubgelenk. Gruppe nicht zählen.
%       continue
%     elseif j == 1
%       pgroups(j) = 1;
%     end
%     if all(pgroups(1:j)==0)
%       pgroups(j) = 1; % ist das erste Gelenk
%       continue
%     end
%     if alpha_leg(j) == 0
%       pgroups(j) = pgroups(j-1); % parallel zu vorherigem Drehgelenk
%     else
%       pgroups(j) = pgroups(j-1)+1; % Zähle neue Gruppe hoch
%     end
%   end

  %% Zeichenkette generieren. Siehe Kong/Gosselin 2007, S.10
  Chain_StructName = '';
  for j = 1:NLegJ
    if Chain_Name(2+j)=='P'
      Chain_StructName = [Chain_StructName, 'P']; %#ok<AGROW>
    elseif pgroups(j) == 1
      Chain_StructName = [Chain_StructName, '\`R']; %#ok<AGROW>
    elseif pgroups(j) == 2
      Chain_StructName = [Chain_StructName, '\''R']; %#ok<AGROW>
    elseif pgroups(j) == 3
      Chain_StructName = [Chain_StructName, '\=R']; %#ok<AGROW>
    else
      error('mehr als drei Achsrichtungen nicht vorgesehen');
    end
  end
  % In Tabelle speichern
  Gnum = d1.Structures{LfdNr}.Coupling(1);
  Pnum = d1.Structures{LfdNr}.Coupling(2);
  Row_i = {RobName, Gnum, Pnum, Chain_Name, Chain_StructName, SName_TechJoint};
  ResTab_NameTrans = [ResTab_NameTrans; Row_i]; %#ok<AGROW>
end

%% Speichere das wieder ab
namestablepath = fullfile(datadir, 'robot_names_latex.csv');
writetable(ResTab_NameTrans, namestablepath, 'Delimiter', ';');
fprintf('Tabelle %s geschrieben\n', namestablepath);
