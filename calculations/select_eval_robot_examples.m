% Wähle Roboter aus den Ergebnissen aus für die Detail-Untersuchung im Paper
% (Detail-Bilder und Tabelle).
% Beeinflusst robot_images.m und results_tables_latex.m
% Rechne die Fitness-Funktion nach und speichere alle relevanten Daten ab.
% Der Roboter wird aus der Pareto-Front genommen, entsprechend der
% Einstellung.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-09
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
%% Benutzereingaben
recalc_fitnessfcn = true; % Neuberechnung der Fitness-Funktion (optional)
% Wähle das Partikel, dessen Positionsfehler am nächsten am gewünschten
% Wert ist. Wähle nicht Roboter mit möglichst kleiner Antriebskraft, da
% dann die Beinketten sehr lang werden und die Roboter komisch aussehen.
posacc_sel = 40e-6;
regenerate_templates = false; %#ok<*UNRCH> % nur bei erstem Aufruf notwendig.
%% Sonstige Initialisierung
importdir = mhi_dimsynth_data_dir();
datadir = fullfile(fileparts(which('select_eval_robot_examples.m')),'..','data');
tmp = load(fullfile(datadir, 'results_all_reps_pareto.mat'));
ResTab = tmp.ResTab_ges;
tmp = load(fullfile(datadir, 'robot_groups.mat'));
RobotGroups = tmp.RobotGroups;

%% Alle Gruppen durchgehen
for i = 1:size(RobotGroups,1)
  GroupName = RobotGroups{i,1};
  if RobotGroups{i,3} == 0, continue; end % keine Ergebnisse vorliegend
  fprintf('Lade Daten für PKM-Gruppe %d/%d (%s)\n', i, size(RobotGroups,1), GroupName);
  data_i = load(fullfile(datadir, sprintf('group_%s_paretofront.mat', GroupName)));
  % Manuelle Anpassung der zu findenden Roboter
  II = (1:size(data_i.pt_i.PosAcc,1))';
%   if i == 1
%     % Suche den Eintrag, dessen Neigungswinkel passend ist (ist teilweise
%     % noch zu waagerecht für ein schönes Bild). Mindestens 30° von der
%     % waagerechten. Als Zahlen-Indizes.
%     % TODO: Fast senkrechte Anordnungen sollten kein Opt.-Ergebnis sein.
%     II = find(abs(data_i.pt_i.BaseJointElevation-pi/2) > 30*pi/180);
%   end
  % Suche den Eintrag der Pareto-Front, der am nächsten dran ist an der
  % niedrigen Antriebskraft.
  [~,iinearest] = min(abs(data_i.pt_i.PosAcc(II) - posacc_sel));
  inearest = II(iinearest);
  
  % Lade Daten für diesen Roboter aus den Ergebnissen
  Ipar = data_i.pt_i.ParetoIndNr(inearest);
  OptName = data_i.pt_i.OptName{inearest};
  RobName = data_i.pt_i.RobName{inearest};
  LfdNr = data_i.pt_i.LfdNr(inearest);
  fprintf('Wähle Opt. %s, Rob. %d, %s, Partikel %d\n', OptName, LfdNr, RobName, Ipar);
  setfile = dir(fullfile(importdir, OptName, '*settings.mat'));
  d1 = load(fullfile(importdir, OptName, setfile(1).name));
  Set_i = d1.Set;
  resfile = fullfile(importdir, OptName, sprintf('Rob%d_%s_Endergebnis.mat', LfdNr, RobName));
  tmp = load(resfile);
  RobotOptRes_i = tmp.RobotOptRes;
  resfile2 = fullfile(importdir, OptName, sprintf('Rob%d_%s_Details.mat', LfdNr, RobName));
  if exist(resfile2,'file')
    % Lade auch die Detail-Ergebnisse. Geht nicht ohne, wenn die
    % Detail-Ergebnisse genutzt wurden, um die Pareto-Front anzupassen.
    % (z.B. durch nachträgliche Filterung nach zusätzlichen Kriterien
    tmp = load(resfile2);
    PSO_Detail_Data_i = tmp.PSO_Detail_Data;
    RobotOptDetails_i = tmp.RobotOptDetails;
  else
    PSO_Detail_Data_i = [];
    RobotOptDetails_i = [];
  end
  % Nummer der Zielfunktion "Positionsfehler"
  kk1 = find(strcmp(Set_i.optimization.objective, 'positionerror'));
  kk2 = find(strcmp(Set_i.optimization.objective, 'actforce'));
  % Prüfe, ob Werte zueinander passen. Kann nicht mehr direkt die
  % Pareto-Fronten nehmen, da die Pareto-Fronten neu gebildet werden. Es
  % muss immer das passende Partikel in den Zwischenergebnissen gesucht
  % werden.
  if false && abs(RobotOptRes_i.physval_pareto(Ipar,kk1) - data_i.pt_i.PosAcc(inearest)) < 1e-10
    % Der Gesuchte Wert liegt ganz normal auf der Pareto-Front aus dem
    % Optimierungsergebnis
    % Lade weitere Daten aus der Ergebnis-Datei (aus Endergebnis-Variable)
    pval = RobotOptRes_i.p_val_pareto(Ipar,:)';
    physval = RobotOptRes_i.physval_pareto(Ipar,:)';
    fval = RobotOptRes_i.fval_pareto(Ipar,:)';
    pval_desopt = RobotOptRes_i.desopt_pval_pareto(Ipar,:)';
  else
    if isempty(PSO_Detail_Data_i)
      error(['Ergebnis-Partikel liegt nicht in der finalen Pareto-Front ', ...
        'der Optimierung. Detail-Auswertung notwendig. Datei aber nicht da: %s'], ...
        resfile2);
    end
    % Suche in den Detail-Daten. Nehme beide Kriterien, damit die Daten
    % eindeutig sind. Ansonsten können mehrere Parametersätze teilweise die
    % gleichen Zielkriterien erzeugen
    PosErrMatrix = reshape(PSO_Detail_Data_i.physval(:,kk1,:), ...
      size(PSO_Detail_Data_i.physval,1), size(PSO_Detail_Data_i.physval,3));
    ActForceMatrix = reshape(PSO_Detail_Data_i.physval(:,kk2,:), ...
      size(PSO_Detail_Data_i.physval,1), size(PSO_Detail_Data_i.physval,3));
    k = find( ...
      abs(PosErrMatrix(:)  -data_i.pt_i.PosAcc(inearest))<1e-10 & ...
      abs(ActForceMatrix(:)-data_i.pt_i.ActForce(inearest))<1e-10);
    if length(k) > 1 % Es gibt mehrere Partikel mit genau diesen Werten für die Zielkriterien
      % Prüfe, ob wenigstens die Parameter unterschiedlich sind.
      pval_all = NaN(length(k), size(PSO_Detail_Data_i.pval,2));
      for ii = 1:length(k)
        [ii_ind,ii_gen] = ind2sub(fliplr(size(PSO_Detail_Data_i.comptime)),k(ii));
        pval_all(ii,:) = PSO_Detail_Data_i.pval(ii_ind,:,ii_gen);
      end
      if size(unique(pval_all,'rows'), 1) == 1
        k = k(1); % sind unterschiedlich. Also identische Roboter.
      else % nicht unterschiedlich. Prinzipielles Problem (redundante Parameter)
        error('Suchkriterium in Daten ist nicht eindeutig');
      end
    end
    [k_ind,k_gen] = ind2sub(fliplr(size(PSO_Detail_Data_i.comptime)),k);
    physval = PSO_Detail_Data_i.physval(k_ind,:,k_gen);
    if abs(physval(kk1)-data_i.pt_i.PosAcc(inearest))>1e-10
      error('Gesuchter Wert konnte nicht gefunden werden. Logik-Fehler');
    end
    fval = PSO_Detail_Data_i.fval(k_ind,:,k_gen)';
    pval = PSO_Detail_Data_i.pval(k_ind,:,k_gen)';
    pval_desopt = PSO_Detail_Data_i.desopt_pval(k_ind,:,k_gen)';
  end

  if ~isempty(PSO_Detail_Data_i)
    % Lese die IK-Anfangswerte aus den Ergebnissen aus (sind indirekt in ge-
    % speicherten Zwischenwerten enthalten)
    [k_gen, k_ind] = cds_load_particle_details(PSO_Detail_Data_i, fval);
    pval_test = PSO_Detail_Data_i.pval(k_ind,:,k_gen)' - pval;
    if any(abs(pval_test))
      error('Indizies für Generation/Individuum stimmen nicht (Parameter nicht gleich)');
    end
    q0 = PSO_Detail_Data_i.q0_ik(k_ind,:,k_gen)';
  end
  %% Nachrechnen der Fitness-Funktion
  % um die Gelenkwinkel aus der IK zu erhalten. Annahme: Die gespeicherten 
  % Detail-Informationen stehen aus Speicherplatzgründen nicht zur Verfügung.
  parroblib_addtopath({RobName}); % Für Ausführung der Fitness-Fcn
  if regenerate_templates
    parroblib_create_template_functions({RobName}, false); % Für Erstellung fehlender Dateien
    R_test = parroblib_create_robot_class(RobName, 1, 1);
    R_test.fill_fcn_handles(true, true); % Zur Kompilierung fehlender Funktionen zum Nachrechnen der Fitness-Funktion
  end
  [R, Structure] = cds_dimsynth_robot(Set_i, d1.Traj, d1.Structures{LfdNr}, true);
  % Fitness-Funktion neu definieren (mit weniger Log-Ausgaben)
  Set = Set_i;
  kk1 = strcmp(Set.optimization.objective,'actforce');
  Set.general.plot_details_in_fitness = 0; % debug: 1e10
  Set.general.save_robot_details_plot_fitness_file_extensions = {};
  Set.general.verbosity = 4; % debug: 3
  cds_log(0, '', 'init', Set);
  % IK-Anfangswerte für dieses Partikel setzen
  if ~isempty(PSO_Detail_Data_i) % geht nur, wenn Detail-Daten vorliegen.
    for iLeg = 1:R.NLEG
      R.Leg(iLeg).qref = q0(R.I1J_LEG(iLeg):R.I2J_LEG(iLeg));
    end
  end
  % Funktionsaufruf siehe cds_check_results_reproducability.m
  Structure_tmp = RobotOptRes_i.Structure;
  Structure_tmp.calc_dyn_act = Structure.calc_dyn_act | Structure.calc_dyn_reg;
  Structure_tmp.calc_spring_act = Structure.calc_spring_act | Structure.calc_spring_reg;
  Structure_tmp.calc_spring_reg = false;
  Structure_tmp.calc_dyn_reg = false;
  clear cds_save_particle_details cds_fitness
  [fval_i_test, physval_i_test, Q] = cds_fitness(R, Set,d1.Traj, ...
    Structure_tmp, pval, pval_desopt);
  PSO_Detail_Data_tmp = cds_save_particle_details(Set, R, 0, 0, NaN, NaN, NaN, NaN, 'output');
  condJ = PSO_Detail_Data_tmp.constraint_obj_val(1, 4, 1);  
  if any(fval_i_test > 1e3)
    warning('Die Nebenbedingungen wurden bei erneuter Prüfung verletzt');
    continue
  end
  q0_neu = PSO_Detail_Data_tmp.q0_ik(1,:,1)';
  test_fval = fval - fval_i_test;
  test_physval = physval - physval_i_test;
  if abs(test_fval(kk1)) > 1e-10 % Durch geänderte Implementierung möglich
    warning(['Die Antriebskraft hat einen anderen Wert beim neu nachrechnen ', ...
      '(%1.1f vs %1.1f). Physikalischer Wert: %1.4f vs %1.4f. IK hat anderes Ergebnis!'], ...
      fval_i_test(kk1), fval(kk1), physval_i_test(kk1), physval(kk1));
    if abs(test_physval(kk1)) > 5 % 5N wird noch für das Bild toleriert (Annahme: Auf Cluster war es richtig)
      error('Der Fehler ist zu groß. Das lässt sich nicht mehr mit Zufallszahlen-Toleranz erklären');
    end
  end
  if any(abs(test_fval(~kk1)) > 1e-5)
    warning(['Andere Zielfunktionen haben einen anderen Wert beim neu nachrechnen: ', ...
      '[%s] vs [%s]. Physikalisch: [%s] vs [%s]'], ...
      disp_array(fval_i_test(~kk1),'%1.6e'), disp_array(fval(~kk1),'%1.6f'), ...
      disp_array(physval_i_test(~kk1),'%1.3e'), disp_array(physval(~kk1),'%1.3e'));
  end
  %% Verändere die Basis-Position, damit die Plots einheitlich sind
  % Dadurch, dass die Basis-Position ein freier Parameter ist, sind die
  % Roboter manchmal zu weit oben. Die Korrektur ist hier am besten möglich
  % Dadurch stimmen die Daten in der Tabelle und in den später gezeichneten
  % Roboter-Bildern überein (besonders die Offset-Länge).
  
  % Ändere weiter unten den Parameter-Vektor der Optimierung
  pval_neu = pval;
  
  % Extrahiere die wesentlichen Daten der ursprünglichen Konfiguration
  R_backup = copy(R);
  Traj_0 = cds_transform_traj(R, d1.Traj);
  q_alt = Q(1,:)';
  x0_alt = Traj_0.X(1,:)';
  qoff_alt = R.Leg(1).DesPar.joint_offset(1);
  [~,~,~,mass_alt] = cds_obj_mass(R);
  phi_alt = R.constr1(Q(1,:)', x0_alt);
  assert(all(abs(phi_alt) < 1e-6), 'IK stimmt schon bei erstem Ergebnis nicht');
  
  % Debug: Zeichne vorherige Version des Roboters
  change_current_figure(100);clf;
  view(3); axis auto; hold on; grid on;
  xlabel('x in m');ylabel('y in m');zlabel('z in m');
  s_plot = struct( 'ks_legs', [], 'ks_platform', [], ...
    'straight', 1, 'mode', 1);
  title(sprintf('Gruppe %d, %s: q aus Fitness-Funktion', i, GroupName));
  R.plot( q_alt, Traj_0.X(1,:)', s_plot);

  % Ändere die Ausrichtung des Basis-Gelenks, wenn es nach oben zeigt (im
  % Welt-KS). Dadurch wird der Schubgelenk-Offset negativ
  if qoff_alt < 0 && Structure.Coupling(1) == 4 && false
    % Drehe das Basis-Gelenk um 180° und Ändere die Richtung der Koordinate
    % TODO: Das funktioniert so noch nicht. Die Drehung hat auch Einfluss
    % auf die Drehgelenke. Dann muss aber alles neu berechnet werden.
    % Passe nur die Darstellung in der Tabelle an.
    % Außerdem geht die Information über die Neigungsrichtung verloren.
    R.align_base_coupling(Structure.Coupling(1), ...
        [R.DesPar.base_par(1), pi-R.DesPar.base_par(2)]);
    pval_neu(Structure_tmp.vartypes==8) = R.DesPar.base_par(2);
    Q(:,R.MDH.sigma==1) = -Q(:,R.MDH.sigma==1);
  end
  % Einstellung der neuen Basis-Position. Daraus berechnung aller weiterer
  % Größen. Ist 2D-Geometrie-Problem (Dreieck), da nur die Basis-Höhe
  % verändert wird.
  h_Zylinder = 0.68;
  r_W_0_alt = R.T_W_0(1:3,4);
  % Setze die neue Basis 11cm über den Zylinder-Rand. Dadurch ist der Plot
  % etwas übersichtlicher. Ist genau der Wert, der für die erste PKM noch
  % geht. Bei sehr flachem Winkel des Schubgelenks, wird das Gestell sehr
  % groß, wenn die Basis-Position hochgesetzt wird.
  r_W_0_neu = [0;0;h_Zylinder+0.11];
  % Winkel der Schubachse gegen das Basis-KS
  alpha = acos(R.Leg(1).T_W_0(1:3,3)'*[0;0;1]);
  delta_base_W = r_W_0_neu - r_W_0_alt;
  delta_base_0 = R.T_W_0(1:3,1:3)'*delta_base_W;
  % Änderung des Gestellradius (Gegenkathete) und Schubgelenk-Länge (Hypo.)
  delta_r = -delta_base_0(3)*tan(alpha);
  delta_q = -delta_base_0(3)/cos(alpha);
  % Berechnung des neuen Gestellradius und der Gelenkkoordinaten
  r_platform_alt = R.DesPar.platform_par(1);
  r_base_alt = R.DesPar.base_par(1);
  r_base_neu = r_base_alt + delta_r;
  Q(:,R.MDH.sigma==1) = Q(:,R.MDH.sigma==1) + delta_q;
  q_neu = Q(1,:)';
  % Neuen Gestell-Radius eintragen und Trajektorie im neuen Basis-KS
  R.align_base_coupling(Structure.Coupling(1), ...
      [r_base_neu, R.DesPar.base_par(2:end)]);
  R.update_base(r_W_0_neu);
  Traj_0_neu = cds_transform_traj(R, d1.Traj);
  x0_neu = Traj_0_neu.X(1,:)';
  phi_neu= R.constr1(q_neu, x0_neu);

  % Debug: Zeichne Roboter mit der neuen Basis-Position
  change_current_figure(101);clf;
  view(3); axis auto; hold on; grid on;
  xlabel('x in m');ylabel('y in m');zlabel('z in m');
  s_plot = struct( 'ks_legs', [], 'ks_platform', [], ...
    'straight', 1, 'mode', 1);
  title(sprintf('Gruppe %d, %s: Nach Basis-Korrektur', i, GroupName));
  R.plot( q_neu, x0_neu, s_plot);
  drawnow();
  assert(all(abs(phi_neu) < 1e-6), 'IK stimmt nicht nach Basis-Verschiebung');

  % Setze den Anfangswert der vorherigen Gelenk-Trajektorie ein, damit
  % keine Konfiguration umklappen kann und garantiert die gleiche IK-Lösung
  % gewählt wird. Ist wohl doch nicht notwendig.
  % for iLeg = 1:R.NLEG
  %   R.Leg(iLeg).qref(1:end) = Q(1,R.I1J_LEG(iLeg):R.I2J_LEG(iLeg));
  % end
  
  % Neu in Roboter-Klasse eintragen
  for iLeg = 1:R.NLEG
    R.Leg(iLeg).qlim(1,:) = R.Leg(iLeg).qlim(1,:)+delta_q;
    R.Leg(iLeg).qref(1) =   R.Leg(iLeg).qref(1)  +delta_q;
  end
  % Fitness-Funktion neu ausrechnen. Dadurch wird die Roboter-Klasse auto-
  % matisch aktualisiert (insbesondere Schubgelenk-Offset)
  % Dafür neue Parameter bestimmen
  baseratio = r_base_neu/r_base_alt;
  % Neue Basis-Höhe eintragen
  pval_neu(Structure_tmp.vartypes==2)=r_W_0_neu(3); % direkter Wert
  % Radius der Basis anpassen (Verhältniswert weil relativ zu Skalierung)
  pval_neu(Structure_tmp.vartypes==6)=pval(Structure_tmp.vartypes==6)*baseratio;
  % Plattform nicht anpassen (Parameter ist absolut und nicht wie sonst
  % relativ zur Basis. Liegt daran, dass absolute Grenzen gegeben sind.
  % pval_neu(Structure_tmp.vartypes==7)=pval(Structure_tmp.vartypes==7)*baseratio;
  % Entferne den vorgegebenen Wert für die Schubgelenk-Offsets und erzwinge
  % damit die erneuge Optimierung. Muss anders sein, da die Basis jetzt
  % anders liegt.
  pval_desopt_tmp = pval_desopt;
  pval_desopt_tmp(Structure_tmp.desopt_ptypes==1) = NaN;
  % pval_desopt_tmp(:) = NaN; % Debug: EO neu;
  % Fitness neu aufrufen
  [fval_i_neu, physval_i_neu, Q_neu] = cds_fitness(R, Set,d1.Traj, ...
    Structure_tmp, pval_neu, pval_desopt_tmp); % Debug: RobotOptRes_i.Structure
  % Prüfe, ob die Parameter richtig aktualisiert wurden
  r_platform_neu = R.DesPar.platform_par(1);
  assert(norm(r_platform_neu-r_platform_alt)<1e-8, sprintf(['Plattform-Radius ', ...
    'hat sich geändert. Darf nicht sein. Vorher: %1.1fmm, nachher: %1.1fmm'], ...
    1e3*r_platform_alt, 1e3*r_platform_neu));
  r_base_neu2 = R.DesPar.base_par(1);
  assert(abs(r_base_neu-r_base_neu2)<1e-8, sprintf(['Gestell-Radius ', ...
    'wurde nicht wie gewünscht eingestellt. Soll: %1.1fmm, ist: %1.1fmm'], ...
    1e3*r_base_neu, 1e3*r_base_neu2));
  % Prüfe, ob die neuen Parameter übernommen werden können
  use_old_parameters = false;
  if fval_i_neu < 1e3 % weiterhin i.O.
    qoff_neu = R.Leg(1).DesPar.joint_offset(1);
    [~,~,~,mass_neu] = cds_obj_mass(R);
    % Prüfe, wie groß der entstandene Fehler ist
    delta_acc = physval_i_neu(1) - physval_i_test(1);
    assert(abs(delta_acc) < 0.01e-6, ...
      'Positionsgenauigkeit darf sich durch Basis-Verschiebung nicht ändern');
    delta_Fa_abs = physval_i_neu(2) - physval_i_test(2);
    delta_Fa_rel = delta_Fa_abs/physval_i_test(2);
    fprintf(['Durch die Anpassung der Basis-Position um %1.1fmm ist die Antriebskraft ', ...
      'um %1.3fN gestiegen (durch längere Schubgelenk-Offsets von %1.1fmm). ', ...
      'Die Masse wurde von %1.2fkg auf %1.2fkg geändert.\n'], ...
      1e3*delta_base_W(3), delta_Fa_abs, 1e3*(qoff_neu-qoff_alt), mass_alt, mass_neu);
    % Erlaube eine Abweichung von 3N, was eigentlich relativ viel ist, aber
    % die Ergebnisse nicht qualitativ ändert (Unterschied war größer)
    if delta_Fa_rel > 0.10 || ... % Erlaube 10% Unterschied. Annahme, das macht die Ergebnisse nicht schlechter
       delta_Fa_abs > 3 || ... % 3N entspricht noch dem Abstand der Ergebnisse
        abs(mass_alt-mass_neu) > 0.5 % Anderer Schubgelenk-Offset darf die Masse etwas ändern.
      warning(['Durch die Anpassung der Basis hat sich eine Zielfunktion zu ', ...
        'stark verändert. Erneute Korrektur notwendig']);
      use_old_parameters = true;
    end
  else % nach Basis-Verschiebung nicht mehr i.O.
    use_old_parameters = true; 
    warning('Die Basis-Verschiebung führte zur Verletzung von Nebenbedingungen');
  end
  
  if use_old_parameters % Zurücksetzen auf die alten Parameter
    R = copy(R_backup);
    % Prüfe die Gelenkwinkel vorher/nachher
    figure(200);clf; RP = ['R', 'P'];
    for jj = 1:R.NJ
      legnum = find(jj>=R.I1J_LEG, 1, 'last');
      legjointnum = jj-(R.I1J_LEG(legnum)-1);
      subplot(ceil(sqrt(R.NJ)), ceil(R.NJ/ceil(sqrt(R.NJ))), jj);
      hold on; grid on;
      plot(Traj_0.t, Q(:,jj));
      plot(Traj_0.t, Q_neu(:,jj));
      title(sprintf('q %d (%s), L%d,J%d', i, RP(R.MDH.sigma(jj)+1), legnum, legjointnum));
    end
    linkxaxes
  else
    Q = Q_neu;
  end

  %% Abschließende Berechnungen und Abspeichern
  % Speichere die Trajektorie in der Variable X (für späteres Plotten)
  Traj_0 = cds_transform_traj(R, d1.Traj);
  X = Traj_0.X;
  
  % Berechne Gelenkwinkelspannweite
  [~,~,~,jointrange] = cds_obj_jointrange(R, Set, Structure, Q);
  if jointrange > Set.optimization.max_range_passive_revolute
    warning('Das zu zeigende Ergebnis verletzt die Gelenkspannweiten-Nebenbedingung');
  end
  
  % Berechne die Spannweite unter Einbeziehung der Ruhelage der Gelenke.
  % Die Grenze für die Spannweite basiert auf den Anforderungen der Festkörpergelenke
  Q_with_ref = [Q; repmat(R.Leg(1).DesPar.joint_stiffness_qref, 3, 1)'];
  [~,~,~,jointrange_with_springrest] = cds_obj_jointrange(R, Set, Structure, Q_with_ref);
  fprintf(['Gelenkwinkel-Wertebereich aus Bewegung: %1.1f°. Mit Gelenkfeder-', ...
    'Ruhelage: %1.1f°. Erlaubt: %1.1f°\n'], 180/pi*jointrange, ...
    180/pi*jointrange_with_springrest, 180/pi*Set.optimization.max_range_passive_revolute);
  if jointrange_with_springrest > Set.optimization.max_range_passive_revolute
    warning(['Die Gelenknullstellung und die Gelenktrajektorie verletzen ', ...
      'zusammen die Gelenkspannweiten-Nebenbedingung. %1.1f° > %1.1f°'], ...
      jointrange_with_springrest*180/pi, Set.optimization.max_range_passive_revolute*180/pi);
  end

  % Berechnung der tatsächlichen Gestellgröße (siehe cds_constraints.m)
  Structure = RobotOptRes_i.Structure;
  [Structure.collbodies_robot, Structure.installspace_collbodies] = ...
      cds_update_collbodies(R, Set_i, Structure, Q);
  I_guidance = Structure.collbodies_robot.type==13;
  pts = [Structure.collbodies_robot.params(I_guidance,1:3); ...
         Structure.collbodies_robot.params(I_guidance,4:6)];
  r_base_eff = max((pts(:,1).^2 + pts(:,2).^2).^0.5);
  % Ergebnis-Variable abspeichern
  objective_names = Set_i.optimization.objective;
  save(fullfile(datadir, sprintf('detail_result_group_%s.mat', GroupName)), ...
    'R', 'pval', 'fval', 'physval', 'condJ', 'Structure', 'Q', 'X', ... % Daten zum Ergebnis
    'jointrange', 'jointrange_with_springrest', ...
    'objective_names', 'r_base_eff', 'OptName', 'RobName', 'LfdNr', 'Ipar'); % Herkunft des Ergebnisses
end
fprintf('Je ein Ergebnis aus %d verschiedenen Gruppen ausgewählt und gespeichert\n', size(RobotGroups,1));
