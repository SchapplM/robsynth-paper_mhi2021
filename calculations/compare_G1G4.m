% Vergleiche PKM mit Gestell-Anordnung G1 (senkrecht) und G4 (schräg)
% Nehme die gleiche Kinematik-Grundstruktur und überführe von G4 nach G1
% 
% Vorher ausführen:
% * Aggregation der Daten mit eval_figures_pareto_groups.m
% 
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-09
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

% Benutzereingabe
RobName_Kin = 'P3PRRRR8V1';
G1_Appendix = 'G1P2A1';
G4_Appendix = 'G4P2A1';
pareto_settings = {'positionerror','jointrange'};

obj_units = {'µm', 'deg'};
objscale = [1e6, 180/pi];

importdir = mhi_dimsynth_data_dir();
outputdir = fileparts(which('compare_G1G4.m'));
datadir = fullfile(outputdir,'..','data');
tmp = load(fullfile(datadir, 'results_all_reps_pareto.mat'), 'ResTab_ges');
ResTab = tmp.ResTab_ges;
% Gehe Pareto-Front für beide PKM durch
% Zeichne die jeweilige Pareto-Front
% Variiere für jedes Partikel auf der Pareto-Front den Winkel alpha
% Zeichne eine Linie für das Ergebnis in die Pareto-Front ein

Robots = {[RobName_Kin,G1_Appendix], [RobName_Kin,G4_Appendix]};
Markers = {'rs', 'bs'}; % Wähle konsistent zu anderen Bildern
figure(1); clf; hold on; grid on;
xlabel(sprintf('%s in %s', pareto_settings{1}, obj_units{1}));
ylabel(sprintf('%s in %s', pareto_settings{2}, obj_units{2}));
title('Pareto-Front Steigungswinkel Gestellgelenk');
figure(2); clf; hold on; grid on;
xlabel('Nummer der Roboterparameter');
ylabel('Wert der Roboterparameter');
title('Roboterparameter Steigungswinkel Gestellgelenk');
for i = length(Robots):-1:1 % Fange mit G4 an, dann erst G1
  RobName = Robots{i};
  parroblib_addtopath({RobName}); % Für Ausführung der Fitness-Fcn weiter unten
  robtablepath = fullfile(datadir, sprintf('%s_paretofront.csv',RobName));
  RobParetoTable = readtable(robtablepath, 'ReadVariableNames', true, 'Delimiter', ';');

  % Gehe alle Simulationsläufe durch und lade die jeweiligen Daten
  OptNames_Robi = unique(RobParetoTable.OptName);
  fprintf('Untersuche Rob %s. %d Verschiedene Optimierungsläufe.\n', RobName, length(OptNames_Robi));
  Row_i = {};
  for j = 1:length(OptNames_Robi)

    OptName = OptNames_Robi{j};
    % Lade die Daten
    LfdNr = ResTab.LfdNr(strcmp(ResTab.Name,RobName) & strcmp(ResTab.OptName,OptName));
    resfile = fullfile(importdir, OptName, sprintf('Rob%d_%s_Endergebnis.mat', LfdNr, RobName));
    Res = load(resfile);
    if i==2
      ResG4 = Res;
    else
      ResG1 = Res;
    end
    % Wähle nur die Pareto-optimalen Partikel aller Optimierungsläufe aus
    Ipar = RobParetoTable.ParetoIndNr(strcmp(RobParetoTable.OptName, OptName));
    % Indizes der Optimierungsparameter
    kk1 = find(strcmp(Res.Set.optimization.objective, pareto_settings{1}));
    kk2 = find(strcmp(Res.Set.optimization.objective, pareto_settings{2}));
    % Plotte die Partikel
    change_current_figure(1);
    plot(objscale(1)*Res.RobotOptRes.physval_pareto(Ipar,kk1), ...
         objscale(2)*Res.RobotOptRes.physval_pareto(Ipar,kk2), Markers{i});
    % Plotte die Parameter
    change_current_figure(2);
    plot((1:size(Res.RobotOptRes.p_val_pareto,2))', Res.RobotOptRes.p_val_pareto, [Markers{i},'-']);
    %% Gehe G1-Partikel durch und überführe nach G4
    if i==1 % G1-Struktur (senkrechte Anordnung)
      for k = randi(length(Ipar)) % Teste ein einziges zufälliges Partikel; 1:length(Ipar)
        fprintf('Opt %d/%d; Partikel %d/%d: Umrechnung von G1 auf G4 und Prüfung.\n', ...
          j, length(OptNames_Robi), k, length(Ipar));
        p_G1 = Res.RobotOptRes.p_val_pareto(Ipar(k),:)';
        fval_k = Res.RobotOptRes.fval_pareto(Ipar(k),:)';
        % Setze Parameter von G1 in G4 ein
        p_G4 = NaN(size(ResG4.RobotOptRes.p_val));
        for jj = 1:length(p_G4)
          I_G1G4j = strcmp(Res.RobotOptRes.Structure.varnames, ResG4.RobotOptRes.Structure.varnames{jj});
          if sum(I_G1G4j) ~= 1, continue; end
          p_G4(jj) = p_G1(I_G1G4j);
        end
        Ip_G4elev = strcmp(ResG4.RobotOptRes.Structure.varnames, 'base_morph_coneelev');
        if sum(Ip_G4elev) == 0, error('Steigungsparameter kommt nicht vor.'); end
        p_G4(Ip_G4elev) = 0; % Winkel 0 heißt senkrecht nach oben, wie G1.
        % Berechne die Fitness-Funktionen der gleichwertigen Parameter
        clear cds_save_particle_details cds_fitness cds_log
        Set = Res.Set; Set.general.verbosity = 0;
        cds_log(0, '', 'init', Set);
        fitnessfcnG1 = @(p)cds_fitness(Res.RobotOptRes.R,Set,Res.Traj,Res.RobotOptRes.Structure,p(:));
        fval_G1_test = fitnessfcnG1(p_G1);
        clear cds_save_particle_details cds_fitness cds_log
        Set = ResG4.Set; Set.general.verbosity = 0;
        cds_log(0, '', 'init', Set);
        fitnessfcnG4 = @(p)cds_fitness(ResG4.RobotOptRes.R,Set,ResG4.Traj,ResG4.RobotOptRes.Structure,p(:));
        fval_G4_test = fitnessfcnG4(p_G4);
        % Vergleich der Ergebnisse
        test_G1G4 = fval_G1_test - fval_G4_test;
        if any(abs(test_G1G4) > 1e-8)
          error('PKM mit G4-Modell stimmt nicht mit G1-Modell überein (%s vs %s). Fehler.', ...
            Robots{2}, Robots{1});
        end
      end
    end
    %% Gehe die G4-Partikel durch und mache Parameterstudie
    % Gehe jedes Partikel durch und variiere die Eigenschaften
    if i == 2% Folgendes funktioniert nur für G4-PKM 
      for k = randi(length(Ipar),1,3) % Parameterstudie für zufälliges Partikel; 1:length(Ipar)
        pval_k = Res.RobotOptRes.p_val_pareto(Ipar(k),:)';
        fval_k = Res.RobotOptRes.fval_pareto(Ipar(k),:)';
        % Testweise nachrechnen der Fitness-Funktion
        clear cds_save_particle_details cds_fitness cds_log
        % Fitness-Funktion neu definieren (mit weniger Log-Ausgaben)
        Set = Res.Set;
        Set.general.verbosity = 0;
        cds_log(0, '', 'init', Set);
        % Lese die IK-Anfangswerte aus den Ergebnissen aus (sind indirekt in ge-
        % speicherten Zwischenwerten enthalten)
        [k_gen,k_ind] = cds_load_particle_details(Res.PSO_Detail_Data, fval_k);
        q0 = Res.PSO_Detail_Data.q0_ik(k_ind,:,k_gen)';
        R = Res.RobotOptRes.R;
        for iLeg = 1:R.NLEG
          R.Leg(iLeg).qref = q0(R.I1J_LEG(iLeg):R.I2J_LEG(iLeg));
        end
        fitnessfcn = @(p)cds_fitness(R,Set,Res.Traj,Res.RobotOptRes.Structure,p(:));
        fval_k_test = fitnessfcn(pval_k);
        test_fval = fval_k - fval_k_test;
        if any(abs(test_fval) > 1e-6)
          warning(['Fitness-Wert %d/%d konnte nicht reproduziert werden Vorher ', ...
            '[%s], jetzt [%s]. Änderung der Implementierung?'], k, length(Ipar), ...
            disp_array(fval_k','%1.1f'), disp_array(fval_k_test','%1.1f'));
          continue
        end
        % Index des Neigungswinkels des Gestells
        Ip_G4elev = strcmp(Res.RobotOptRes.Structure.varnames, 'base_morph_coneelev');
        if sum(Ip_G4elev) == 0
          error('Geladene Parameter passen nicht. Filter sind fehlerhaft.');
        end
        fprintf('Opt %d/%d; Partikel %d/%d: Steigungsparameter %1.3f°\n', ...
          j, length(OptNames_Robi), k, length(Ipar), 180/pi*pval_k(Ip_G4elev));
        % Stelle Werte für alpha zusammen. Fange mit 180° an (senkrecht).
        if abs(pval_k(Ip_G4elev)-pi) < pi/2 % ist näher an pi dran
          alpha_values = pi/180*(180:-2.5:165);
        else % näher an Null dran
          alpha_values = pi/180*(0:2.5:15);
        end
        alpha_var = fliplr(unique([pval_k(Ip_G4elev), alpha_values]));
        pval_var = repmat(pval_k',length(alpha_var),1);
        pval_var(:,Ip_G4elev) = alpha_var;
        fval_var = NaN(size(pval_var,1),length(fval_k));
        physval_var = fval_var;
        fprintf(['Führe Parameterstudie mit %d Werten für Neigungswinkel durch ', ...
          '(für Pareto-Partikel %d/%d aus Opt. %d/%d)\n'], ...
          length(pval_var), k, length(Ipar), j, length(OptNames_Robi))
        clear cds_save_particle_details cds_fitness
        for l = 1:length(pval_var)
          fval_var(l,:) = fitnessfcn(pval_var(l,:)');
        end
        % Physikalische Werte extrahieren
        PSO_Detail_Data = cds_save_particle_details([], [], 0, 0, NaN, NaN, NaN, NaN, 'output');
        fval_var_test = physval_var; % NaN-Init.
        for oc = 1:length(fval_k) % Optimierungskriterien durchgehen
          physval_oc = squeeze(PSO_Detail_Data.physval(:,oc,:)); % Dim1:Ind.; Dim2:Gen.
          physval_var(:,oc) = physval_oc(1:length(pval_var))';
          % Teste auf Logik-Fehler bei Speicherung und Abruf. Übereinstimmung
          % mit vorheriger Ausgabe der Fitness-Funktion.
          fval_oc = squeeze(PSO_Detail_Data.fval(:,oc,:)); % Test-Variable
          test_fvaloc = fval_var(:,oc) - fval_oc(1:length(pval_var))';
          if any(abs(test_fvaloc) ~= 0)
            error('Abruf der physikalischen Werte zu den Fitness-Werten funktioniert nicht');
          end
        end
        % Filtern
        fval_var(fval_var>1e3) = NaN; % NB-Verletzung ignorieren
        [~,II] = sort(fval_var(:,1));
        fval_var = fval_var(II,:);
        physval_var = physval_var(II,:);
        pval_var = pval_var(II,:);
        % Ergebnis der Studie kurz zusammenfassen.
        Idom = pareto_dominance([physval_var;Res.RobotOptRes.physval_pareto]);
        Idom_var = Idom(1:size(physval_var,1));
        Idom_var(isnan(fval_var(:,1))) = true;
        fprintf('Neigungswinkel [%s] dominieren die Pareto-Front:.\n', ...
          disp_array(180/pi*alpha_var(~Idom_var),'%1.1f'));
        disp([objscale(1)*physval_var(~Idom_var,kk1),objscale(2)*physval_var(~Idom_var,kk2)])
        fprintf('Neigungswinkel [%s] werden dominiert:.\n', ...
          disp_array(180/pi*alpha_var(Idom_var),'%1.1f'));
        disp([objscale(1)*physval_var( Idom_var,kk1),objscale(2)*physval_var( Idom_var,kk2)]);
        % Zeichnen
        change_current_figure(1);
        plot(objscale(1)*physval_var(:,kk1), objscale(2)*physval_var(:,kk2), [Markers{i}(1),'x-']);
        drawnow();
      end
    end
    break % nur eine einzelne Optimierung anschauen, keine Wiederholungen
  end
end
