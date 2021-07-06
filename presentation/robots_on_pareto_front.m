% Vergleiche Parametrierungen eines Roboters auf seiner Pareto-Front
% Erzeuge beispielhafte Bilder von Robotern auf der Front (für Vortrag)
% 
% Vorher ausführen:
% * eval_figures_pareto_groups.m
% 
% Siehe: select_eval_robot_examples.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-04
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
close all

outputdir = fileparts(which('robots_on_pareto_front.m')); % Ordner dieser Datei
importdir = mhi_dimsynth_data_dir();
datadir = fullfile(outputdir, '..', 'data');

tmp = load(fullfile(datadir, 'robot_groups.mat'));
RobotGroups = tmp.RobotGroups;

%% Alle Gruppen durchgehen
% countrob = 0; % Nummern aus Legende
for i = 1:size(RobotGroups,1)
  GroupName = RobotGroups{i,1};
  if RobotGroups{i,3} == 0
    warning('Für Gruppe %d (%s) liegen keine Ergebnisse vor.', i, GroupName);
    continue;
  end
  
  data_i = load(fullfile(datadir, sprintf('group_%s_paretofront.mat', GroupName)), 'pt_i');
  pt_i = data_i.pt_i;
  fprintf('Zeichne Bilder für PKM-Gruppe %d/%d (%s)\n', i, size(RobotGroups,1), GroupName);
  
  erg = load(fullfile(datadir, sprintf('detail_result_group_%s.mat', GroupName)));
  % countrob = countrob + 1; % nur gültige PKM-Gruppen zählen
  countrob = i; % Geht nur, wenn alle Gruppen gültig sind.

  %% Pareto-Front durchgehen
  [~,I_pf] = sort(pt_i.PosAcc);
  pt_i = pt_i(I_pf,:);
  % Nehme äquidistante Stützstellen über das erste Zielkriterium
  % (waagerechte Achse der Pareto-Front)
  pa_x = linspace(pt_i.PosAcc(1),pt_i.PosAcc(end),10)';
  I_pa_x = unique(knnsearch( pt_i.PosAcc , pa_x ));
  for ii = I_pa_x(:)' % Stützstellen durchgehen
    %% Parameter aktualisieren
    Ipar = pt_i.ParetoIndNr(ii);
    OptName = pt_i.OptName{ii};
    RobName = pt_i.RobName{ii};
    LfdNr = pt_i.LfdNr(ii);
    
    fprintf(['Wähle Punkt %d/%d auf der Gesamt-Pareto-Front. Opt. %s, ', ...
      'Rob. %d, %s, Partikel %d\n'], ii, size(pt_i,1), OptName, LfdNr, RobName, Ipar);
    setfile = dir(fullfile(importdir, OptName, '*settings.mat'));

    d1 = load(fullfile(importdir, OptName, setfile(1).name));
    Set_i = cds_settings_update(d1.Set);
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
    %% Parameter aus Ergebnisse laden
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
      abs(PosErrMatrix(:)  -pt_i.PosAcc(ii))<1e-10 & ...
      abs(ActForceMatrix(:)-pt_i.ActForce(ii))<1e-10);
    if length(k) > 1 % Es gibt mehrere Partikel mit genau diesen Werten für die Zielkriterien
      % Prüfe, ob wenigstens die Parameter unterschiedlich sind.
      pval_all = NaN(length(k), size(PSO_Detail_Data_i.pval,2));
      for jj = 1:length(k)
        [ii_ind,ii_gen] = ind2sub(fliplr(size(PSO_Detail_Data_i.comptime)),k(jj));
        pval_all(jj,:) = PSO_Detail_Data_i.pval(ii_ind,:,ii_gen);
      end
      pval_all_unique = unique(pval_all,'rows');
      if size(pval_all_unique, 1) == 1
        k = k(1); % sind unterschiedlich. Also identische Roboter.
      else % nicht unterschiedlich. Prinzipielles Problem (redundante Parameter)        
        warning(['Suchkriterium in Daten ist nicht eindeutig. %d verschie', ...
          'dene Parameter haben exakt gleiche Zielfunktion.'], size(pval_all_unique, 1));
        ParamTable = array2table(pval_all_unique);
        ParamTable.Properties.VariableNames = RobotOptRes_i.Structure.varnames;
        disp(ParamTable);
        % Nehme den ersten Parameter. Es gibt dann zwei verschieden
        % aussehende Roboter, die aber eigentlich exakt gleich sind.
        k = k(1);
      end
    end
    [k_ind,k_gen] = ind2sub(fliplr(size(PSO_Detail_Data_i.comptime)),k);
    physval = PSO_Detail_Data_i.physval(k_ind,:,k_gen);
    if abs(physval(kk1)-pt_i.PosAcc(ii))>1e-10
      error('Gesuchter Wert konnte nicht gefunden werden. Logik-Fehler');
    end
    fval = PSO_Detail_Data_i.fval(k_ind,:,k_gen)';
    pval = PSO_Detail_Data_i.pval(k_ind,:,k_gen)';
    pval_desopt = PSO_Detail_Data_i.desopt_pval(k_ind,:,k_gen)';
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
    
    %% Roboter initialisieren
    [R, Structure] = cds_dimsynth_robot(Set_i, d1.Traj, d1.Structures{LfdNr}, true);

    %% Funktionsaufruf siehe cds_check_results_reproducability.m
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
    Structure_tmp = Structure; % Alternativ: RobotOptRes_i.Structure (evtl veraltete Version)
    Structure_tmp.calc_dyn_act = Structure.calc_dyn_act | Structure.calc_dyn_reg;
    Structure_tmp.calc_spring_act = Structure.calc_spring_act | Structure.calc_spring_reg;
    Structure_tmp.calc_spring_reg = false;
    Structure_tmp.calc_dyn_reg = false;
    clear cds_save_particle_details cds_fitness
    [fval_i_test, physval_i_test, Q] = cds_fitness(R, Set,d1.Traj, ...
      Structure_tmp, pval, pval_desopt);
    
    %% Anpassung der Visualisierung
    % Dicke der Segmente reduzieren
    for kk = 1:R.NLEG
      R.Leg(kk).DesPar.seg_par(:,1) = 1e-3;
      R.Leg(kk).DesPar.seg_par(:,2) = 1e-2;
    end
    parroblib_addtopath({R.mdlname});

    Traj_0 = cds_transform_traj(R, d1.Traj);
    X = Traj_0.X;
    Q_plot = Q;
    X_plot = X;

    %% Bild zeichnen und Formatieren
    change_current_figure(1); clf; hold on;
    s_plot = struct('ks_legs', [], 'ks_platform', [], 'mode', 4);
    % Plotte in mittlerer Stellung des Roboters
    x_mean = mean(minmax2(X_plot(:,1:3)'),2);
    % Finde den Punkt der Trajektorie, der der Mitte am nächsten ist.
    [~,I_mean] = min(sum((repmat(x_mean',size(X_plot,1),1)-X_plot(:,1:3)).^2,2));
    R.plot(Q_plot(I_mean,:)', X_plot(I_mean,:)', s_plot);
    view(3);
    title('');xlabel('');ylabel('');zlabel('');
    LegColors = [ [0 1 0]; [0 0 0]; [1 0 1] ]; % Grün, Schwarz, Violett
    % Automatisch generierten Plot nachverarbeiten
    ch = get(gca, 'Children');
    for jj = 1:length(ch)
      % KS entfernen
      if strcmp(ch(jj).Type, 'hgtransform')
        delete(ch(jj)); continue
      end
      % Dreieck entfernen, das die Basis anzeigt. Kann man besser in InkScape
      % neu zeichnen
      if strcmp(ch(jj).Type, 'line')
        delete(ch(jj)); continue
      end
      % Weise den Beinketten neue Farben zu
      for kk = 1:3
        if contains(get(ch(jj), 'DisplayName'), sprintf('Leg_%d', kk))
          set(ch(jj), 'EdgeColor', LegColors(kk,:));
        end
      end 
    end
    % Zusätzlich einen Kreis für den oberen Rand des Behälters einzeichnen.
    % Kein niedrigeres Zeichnen der Grenze möglich, da dann die Drehgelenke
    % manchmal drüber sind (Lösung ist oft exakt auf Grenze)
    h_Zylinder = 0.68; % Höhe
    r_Zylinder = 0.28; % Radius <-- etwas aufweiten. Sieht optisch sonst komisch aus, wenn Segment genau auf Grenze liegt.
    drawCircle3d([0,0,h_Zylinder,r_Zylinder], 'LineWidth', 2, 'Color', 'c')

    % Nehme überall die gleiche Perspektive für Vergleichbarkeit der Bilder
    view(42,5); % Standard-Perspektive
    % Nur Roboter zeichnen, sonst nichts.
    set(gca,'XTICKLABEL',{});set(gca,'YTICKLABEL', {});set(gca,'ZTICKLABEL',{});
    set(gca,'xtick',[],'ytick',[],'ztick',[]);
    set(get(gca, 'XAxis'), 'visible', 'off');
    set(get(gca, 'YAxis'), 'visible', 'off');
    set(get(gca, 'ZAxis'), 'visible', 'off');
    figure_format_publication(gca);
    set(gca, 'Box', 'off');
    set(1, 'windowstyle', 'normal');
    set_size_plot_subplot(1, ...
      8,8,gca,...
      0,0,0,0,0,0)
    drawnow();
    % Bild speichern
    name = sprintf('RobGroup%d_%s_PlotNr%d_ParetoNr%d', countrob, GroupName, ...
      find(I_pa_x==ii), ii);
    cd(outputdir);
    export_fig([name, '_r864.png'], '-r864');
    % Datei mit Zusatz-Infos (nicht notwendig, wenn CSV-Tabelle gespeichert
    % wird)
%     fid = fopen(fullfile(outputdir, [name, '_info.txt']), 'w');
%     fprintf(fid, 'OptName: %s\nLfdNr: %d\nRobName: %s\nParetoIndNr: %d\n', ...
%       OptName, LfdNr, RobName, Ipar);
%     fprintf(fid, 'PosAcc: %1.3fµm\nActForce: %1.3fN\\n', ...
%       1e6*pt_i.PosAcc(ii), pt_i.ActForce(ii));
%     fclose(fid);
  end
  pt_x = pt_i(I_pa_x,:);
  writetable(pt_x, fullfile(outputdir, sprintf( ...
    'RobGroup%d_%s_paretofront_selection.csv', countrob, GroupName)), ...
    'Delimiter', ';');
  fprintf('Bilder und Tabelle für %d Roboter auf Pareto-Front gespeichert: %s\n', ...
    length(I_pa_x), outputdir);
end
