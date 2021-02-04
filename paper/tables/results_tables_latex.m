% Erzeuge Latex-Tabellen mit den Ergebnissen
% 
% Vorher ausführen:
% * Aggregation der Daten mit eval_figures_pareto_groups.m
% * Zusammenstellen der Details mit select_eval_robot_examples.m
% 
% Erzeugt tab_kinpar.tex. Der Inhalt wird dann ins Paper kopiert.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

outputdir = fileparts(which('results_tables_latex.m')); % Ordner dieser Datei
datadir = fullfile(outputdir,'..','..','data');
tmp = load(fullfile(datadir, 'robot_groups.mat'));
RobotGroups = tmp.RobotGroups;
namestablepath = fullfile(datadir, 'robot_names_latex.csv');
RobNamesTab = readtable(namestablepath, 'Delimiter', ';', 'ReadVariableNames', true);

%% Alle Gruppen durchgehen
countrob = 0; % Nummern aus Legende
% Schreibe Latex-Daten in Textdatei
resuts_latex_file = fullfile(outputdir, 'tab_kinpar.tex');
fid = fopen(resuts_latex_file, 'w');
Name_LegChain_Gen_vorher = '';
for i = 1:size(RobotGroups,1)+1
  if i <= size(RobotGroups,1)
    GroupName = RobotGroups{i,1};
    if RobotGroups{i,3} == 0, continue; end % keine Ergebnisse vorliegend
    fprintf('Schreibe Tabellenzeile für PKM-Gruppe %d/%d (%s)\n', i, size(RobotGroups,1), GroupName);
    erg = load(fullfile(datadir, sprintf('detail_result_group_%s.mat', GroupName)));
  else
    erg = load(fullfile(datadir, 'detail_result_engineering_solution.mat'));
    erg.RobName = erg.R.mdlname;
  end
  countrob = countrob + 1;
  
  % Daten der Ergebnisse laden
  fprintf('Anzahl Variablen: %d\n', length(erg.Structure.vartypes));
  disp(strjoin(erg.Structure.varnames));
  fprintf('Parametertypen: [%s]\n', disp_array(erg.Structure.vartypes', '%1.0f'));
  
  % Nummer der Zielfunktion bestimmen
  kk1 = find(strcmp(erg.objective_names, 'positionerror'));
  kk2 = find(strcmp(erg.objective_names, 'actforce'));
  
  % Kinematik-Parameter zusammenstellen
  R = erg.R;
  d_mdh_all =  R.Leg(1).MDH.d;
  a_mdh_all =  R.Leg(1).MDH.a;
  r_base = R.DesPar.base_par(1);
  r_plf = R.DesPar.platform_par(1);
  qoff = R.Leg(1).DesPar.joint_offset(1);
  Ip_G4elev = strcmp(erg.Structure.varnames, 'base_morph_coneelev');
  if ~any(Ip_G4elev)
    phi_base = 0;
  else
    phi_base = erg.pval(Ip_G4elev);
  end
  % Trage Parameter, die nicht optimiert werden und Null sind, als NaN ein
  for kk = 1:5
    if ~any(contains(erg.Structure.varnames, sprintf('d%d', kk))) && d_mdh_all(kk)==0
      d_mdh_all(kk) = NaN;
    end
    if ~any(contains(erg.Structure.varnames, sprintf('a%d', kk))) && a_mdh_all(kk)==0
      a_mdh_all(kk) = NaN;
    end
  end
  
  % Erzeuge den Vektor der in der Tabelle einzutragenden Parameter. Nehme
  % den Betrag, da das negative Vorzeichen bei den DH-Parametern ohne die
  % genauen Werte für alpha und theta nicht aussagekräftig ist und eher
  % verwirrt.
  kinparvec = [1e3*r_base, 180/pi*phi_base, 1e3*r_plf, abs(1e3*qoff), ...
    abs(1e3*a_mdh_all(3)), abs(1e3*d_mdh_all(3)), ...
    abs(1e3*a_mdh_all(4)), abs(1e3*d_mdh_all(4)), abs(1e3*a_mdh_all(5))];
  % Masse bestimmen (ziehe 3kg Zusatz-Last wieder ab). Muss konsistent zu
  % Einstellungen der Maßsynthese sein.
  m_sum = R.NLEG*sum(R.Leg(1).DynPar.mges)+(R.DynPar.mges(end)-3);
  
  if i <= size(RobotGroups,1)
    % Laufende Nummer (aus Legende) eintragen
    latex_row = sprintf('%d', countrob);
    % Symbol des Markers eintragen (zum schnelleren Blick in Legende)
    latex_row = [latex_row, sprintf([' & \\vcenteredinclude', ...
      '{tables/group%d_marker.pdf}'], i)]; %#ok<AGROW>
  else % Doppelspalte für "Eng."
    latex_row = '\multicolumn{2}{|c|}{Eng.}';
  end
  % Namen des Roboters erzeugen (nicht mehr in Tabelle einfügen, da bereits
  % in Legende des Bildes).
  % jj_rn = strcmp(RobNamesTab.PKM_Name, erg.RobName);
  % latex_row = [latex_row, ' & ', sprintf('%s', RobNamesTab.ChainStructure{jj_rn})]; %#ok<AGROW>
  % if length(RobNamesTab.Chain_ShortName{jj_rn}) < 5
  %   latex_row = [latex_row, sprintf(' (%s)', RobNamesTab.Chain_ShortName{jj_rn})]; %#ok<AGROW>
  % end
  % Positionsfehler eintragen
  latex_row = [latex_row, sprintf(' & %1.0f', erg.physval(kk1)*1e6)]; %#ok<AGROW>
  % Antriebskraft eintragen
  latex_row = [latex_row, sprintf(' & %1.0f', erg.physval(kk2))]; %#ok<AGROW>
  % Konditionszahl eintragen
  latex_row = [latex_row, sprintf(' & %1.1f', erg.condJ)]; %#ok<AGROW>
  % Winkelspannweite eintragen
  latex_row = [latex_row, sprintf(' & %1.1f', erg.jointrange*180/pi')]; %#ok<AGROW>
  % Winkelspannweite mit Berücksichtigung der Feder-Ruhelage eintragen?
  % Nicht machen. Trägt gar keine Informationen.
  % latex_row = [latex_row, sprintf(' & %1.1f', erg.jointrange_with_springrest*180/pi')]; %#ok<AGROW>
  % Masse eintragen
  latex_row = [latex_row, sprintf(' & %1.1f', m_sum)]; %#ok<AGROW>
  % Anzahl der Optimierungsparameter
  latex_row = [latex_row, sprintf(' & %d',  length(erg.Structure.vartypes))]; %#ok<AGROW>
  % Kinematikparameter eintragen
  for i_kinpar = 1:length(kinparvec)
    if isnan(kinparvec(i_kinpar)) % Trage einen Strich ein
      latex_row = [latex_row, sprintf(' & \\multicolumn{1}{c|}{---}')]; %#ok<AGROW>
    else % Trage Parameter als Zahlenwert ein
      latex_row = [latex_row, sprintf(' & $%1.0f$', kinparvec(i_kinpar))]; %#ok<AGROW>
    end
  end
  
  % Strich ziehen, wenn die grundlegende Kinematik gewechselt hat.
  % Kriterium dafür die die Beinketten-Kinematik
  [~,LEG_Names]=parroblib_load_robot(erg.RobName);
  LegName = LEG_Names{1};
  serroblibpath=fileparts(which('serroblib_path_init.m'));
  NLJ = str2double(LegName(2));
  mdllistfile_Ndof = fullfile(serroblibpath, sprintf('mdl_%ddof', ...
    NLJ), sprintf('S%d_list.mat',NLJ));
  l = load(mdllistfile_Ndof, 'Names_Ndof', 'AdditionalInfo');
  Ir_db = find(strcmp(l.Names_Ndof, LegName));
  I_genmdl = l.AdditionalInfo(Ir_db,3);
  Name_LegChain_Gen = l.Names_Ndof{I_genmdl};
  if i > 1 && ~strcmp(Name_LegChain_Gen, Name_LegChain_Gen_vorher)
    fprintf(fid, '\\hline\n');
  end
  Name_LegChain_Gen_vorher = Name_LegChain_Gen;
  
  % Eigentliche Datenzeile schreiben
  if i <= size(RobotGroups,1)
    fprintf(fid, '%s \\\\ %% %s/Rob%d_%s (Pareto-Index %d)\n', latex_row, erg.OptName, erg.LfdNr, ...
      erg.RobName, erg.Ipar);
  else
    fprintf(fid, '%s \\\\ %% Engineering Solution; see eval_existing_design.m\n', latex_row);
  end
end
fclose(fid);
fprintf('Tabelle nach %s geschrieben\n', resuts_latex_file);

%% Zeichne die Marker in jeweils ein eigenes Bild zum Einfügen in Tabelle
return % muss nur einmal gemacht werden
for i = 1:length(RobotGroups) %#ok<UNRCH>
  change_current_figure(1000);clf;
  plot(1,1,RobotGroups{i,4});
  figure_format_publication(gca);
  set(gcf, 'color', 'none'); % transparent. Funktioniert nicht in pdf.
  set(gca, 'Box', 'off');
  set(gca, 'XTICK', [], 'YTICK', []);
  set(get(gca, 'XAXIS'), 'visible', 'off');
  set(get(gca, 'YAXIS'), 'visible', 'off');
  set_size_plot_subplot(1000, ...
    1,1,gca,0,0,0,0,0,0)
  export_fig(fullfile(outputdir, sprintf('group%d_marker.pdf', i)));
  export_fig(fullfile(outputdir, sprintf('group%d_marker.pdf', i)));
  cd(outputdir);
  export_fig(sprintf('group%d_marker.png', i), '-r864');
end

fprintf('Marker nach %s exportiert\n', outputdir);
