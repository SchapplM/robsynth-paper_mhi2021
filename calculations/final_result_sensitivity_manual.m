% Prüfe Sensitivität des Endergebnisses bei Änderung der Parameter manuell
% Manuelle Anpassung der Parameter ausgehend vom Optimierungsergebnis.
% 
% Vorlage für dieses Skript: select_eval_robot_examples.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-01
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

%% Benutzereingaben
posacc_sel = 40e-6;
regenerate_templates = false; %#ok<*UNRCH> % nur bei erstem Aufruf notwendig.
%% Sonstige Initialisierung
if isempty(which('mhi_dimsynth_data_dir'))
  error(['You have to create a file mhi_dimsynth_data_dir pointing to the ', ...
    'directory containing the results of the dimensional synthesis']);
end
importdir = mhi_dimsynth_data_dir();
datadir = fullfile(fileparts(which('select_eval_robot_examples.m')),'..','data');
tmp = load(fullfile(datadir, 'results_all_reps_pareto.mat'));
ResTab = tmp.ResTab_ges;
tmp = load(fullfile(datadir, 'robot_groups.mat'));
RobotGroups = tmp.RobotGroups;

%% Partikel nachrechnen
for i = 5%:size(RobotGroups,1)
  %% Laden des Ergebnisses (siehe select_eval_robot_examples.m)
  GroupName = RobotGroups{i,1};
  if RobotGroups{i,3} == 0, continue; end % keine Ergebnisse vorliegend
  fprintf('Lade Daten für PKM-Gruppe %d/%d (%s)\n', i, size(RobotGroups,1), GroupName);
  data_i = load(fullfile(datadir, sprintf('group_%s_paretofront.mat', GroupName)));
  % Manuelle Anpassung der zu findenden Roboter
  II = (1:size(data_i.pt_i.PosAcc,1))';
  % niedrigen Antriebskraft.
  [~,iinearest] = min(abs(data_i.pt_i.PosAcc(II) - posacc_sel));
  inearest = II(iinearest);
  
  % Lade Daten für diesen Roboter aus den Ergebnissen
  Ipar = data_i.pt_i.ParetoIndNr(inearest);
  OptName = data_i.pt_i.OptName{inearest};
  RobName = data_i.pt_i.RobName{inearest};
  LfdNr = ResTab.LfdNr(strcmp(ResTab.Name,RobName) & strcmp(ResTab.OptName,OptName));
  fprintf('Wähle Opt. %s, Rob. %d, %s, Partikel %d\n', OptName, LfdNr, RobName, Ipar);
  setfile = dir(fullfile(importdir, OptName, '*settings.mat'));
  d1 = load(fullfile(importdir, OptName, setfile(1).name));
  Set_i = cds_settings_update(d1.Set);
  resfile = fullfile(importdir, OptName, sprintf('Rob%d_%s_Endergebnis.mat', LfdNr, RobName));
  tmp = load(resfile);
  RobotOptRes_i = tmp.RobotOptRes;
  resfile2 = fullfile(importdir, OptName, sprintf('Rob%d_%s_Details.mat', LfdNr, RobName));

  tmp = load(resfile2);
  PSO_Detail_Data_i = tmp.PSO_Detail_Data;
  RobotOptDetails_i = tmp.RobotOptDetails;
  kk1 = find(strcmp(Set_i.optimization.objective, 'positionerror'));
  % Suche in den Detail-Daten
  PosErrMatrix = reshape(PSO_Detail_Data_i.physval(:,kk1,:), ...
    size(PSO_Detail_Data_i.physval,1), size(PSO_Detail_Data_i.physval,3));
  k = find(abs(PosErrMatrix(:)-data_i.pt_i.PosAcc(inearest))<1e-10, 1, 'first');
  [k_ind,k_gen] = ind2sub(fliplr(size(PSO_Detail_Data_i.comptime)),k);
  physval = PSO_Detail_Data_i.physval(k_ind,:,k_gen);
  if abs(physval(kk1)-data_i.pt_i.PosAcc(inearest))>1e-10
    error('Gesuchter Wert konnte nicht gefunden werden. Logik-Fehler');
  end
  fval = PSO_Detail_Data_i.fval(k_ind,:,k_gen)';
  pval_orig = PSO_Detail_Data_i.pval(k_ind,:,k_gen)';
  pval_desopt = PSO_Detail_Data_i.desopt_pval(k_ind,:,k_gen)';
  
  %% Vorbereitung
  parroblib_addtopath({RobName}); % Für Ausführung der Fitness-Fcn
  if regenerate_templates
    parroblib_create_template_functions({RobName}, false); % Für Erstellung fehlender Dateien
    R_test = parroblib_create_robot_class(RobName, 1, 1);
    R_test.fill_fcn_handles(true, true); % Zur Kompilierung fehlender Funktionen zum Nachrechnen der Fitness-Funktion
  end
  
  [R, Structure] = cds_dimsynth_robot(Set_i, d1.Traj, d1.Structures{LfdNr}, true);
  %% Partikel nachbearbeiten 
  pval = pval_orig;
  disp('Optimierungsparameter:');
  disp(Structure.varnames);
  if i == 5
    % Hier Parameter manuell anpassen und dann Auswertungsbilder anschauen.
    % Beispiel-Werte zum Debuggen:
    pval(strcmp(Structure.varnames,'base_morph_coneelev')) = 3*pi/180;
    pval(strcmp(Structure.varnames,'scale')) = 1;
    pval(strcmp(Structure.varnames,'pkin 5: d3')) = 0.05;
    pval(strcmp(Structure.varnames,'pkin 2: a3')) = -0.25;
    pval(strcmp(Structure.varnames,'pkin 3: a5')) = 0.4;
  end
  disp('Parametergrenzen und Parameter aus Optimierung (alt, neu):');
  disp([Structure.varlim, pval_orig, pval]);
  if any(pval ~= pval_orig)
    pval_desopt(:) = NaN; % Entwurfsoptimierung muss neu durchgeführt werden
  end
  %% Fitness-Funktion berechnen
  % Fitness-Funktion neu definieren (mit weniger Log-Ausgaben)
  Set = Set_i;
  kk1 = strcmp(Set.optimization.objective,'actforce');
  Set.general.plot_robot_in_fitness = 1e11; % immer Bild zeichnen
  Set.general.plot_details_in_fitness = 1e11;
  Set.general.save_robot_details_plot_fitness_file_extensions = {};
  Set.general.verbosity = 4;
  cds_log(0, '', 'init', Set);
  Structure_tmp = Structure;
  Structure_tmp.calc_dyn_act = Structure.calc_dyn_act | Structure.calc_dyn_reg;
  Structure_tmp.calc_spring_act = Structure.calc_spring_act | Structure.calc_spring_reg;
  Structure_tmp.calc_spring_reg = false;
  Structure_tmp.calc_dyn_reg = false;
  clear cds_save_particle_details cds_fitness
  [fval_i_test, physval_i_test, Q] = cds_fitness(R, Set,d1.Traj, ...
    Structure_tmp, pval, pval_desopt);
  PSO_Detail_Data_tmp = cds_save_particle_details(Set, R, 0, 0, NaN, NaN, NaN, NaN, 'output');
  if any(fval_i_test > 1e3)
    warning('Die Nebenbedingungen wurden bei erneuter Prüfung verletzt');
    continue
  end
end