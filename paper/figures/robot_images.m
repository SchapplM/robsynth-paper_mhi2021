% Erzeuge hochaufgelöste Bilder der einzelnen Roboter aus den
% Optimierungsergebnissen.
% 
% Vorher ausführen:
% * Aggregation der Daten mit eval_figures_pareto_groups.m
% * Zusammenstellen der Details mit select_eval_robot_examples.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
close all

warning('off', 'Coder:MATLAB:rankDeficientMatrix'); % für invkin2
outputdir = fileparts(which('robot_images.m')); % Ordner dieser Datei
datadir = fullfile(outputdir,'..','..','data');
tmp = load(fullfile(datadir, 'robot_groups.mat'));
RobotGroups = tmp.RobotGroups;
namestablepath = fullfile(datadir, 'robot_names_latex.csv');
ResTab_NameTrans = readtable(namestablepath, 'Delimiter', ';');

%% Alle Gruppen durchgehen
countrob = 0; % Nummern aus Legende
for i = 1:size(RobotGroups,1)
  GroupName = RobotGroups{i,1};
  if RobotGroups{i,3} == 0, continue; end % keine Ergebnisse vorliegend
  fprintf('Zeichne Bild für PKM-Gruppe %d/%d (%s)\n', i, size(RobotGroups,1), GroupName);
  erg = load(fullfile(datadir, sprintf('detail_result_group_%s.mat', GroupName)));
  % countrob = countrob + 1; % nur gültige PKM-Gruppen zählen
  countrob = i; % Geht nur, wenn alle Gruppen gültig sind.
  % Daten der Ergebnisse laden
  fprintf('Anzahl Variablen: %d\n', length(erg.Structure.vartypes));
  disp(strjoin(erg.Structure.varnames));
  fprintf('Parametertypen: [%s]\n', disp_array(erg.Structure.vartypes', '%1.0f'));
  close all
  %% Roboter initialisieren
  R = erg.R;
  % Dicke der Segmente reduzieren
  for kk = 1:R.NLEG
    R.Leg(kk).DesPar.seg_par(:,1) = 1e-3;
    R.Leg(kk).DesPar.seg_par(:,2) = 1e-2;
  end
  parroblib_addtopath({R.mdlname});
  %% Anpassung der Visualisierung
  Q = erg.Q;
  X = erg.X;
  Q_plot = Q;
  X_plot = X;
  % Dieser Schritt muss nicht mehr gemacht werden, da die Basis-Position
  % und Schubgelenk-Offsets mittlerweile in select_eval_robot_examples.m
  % angepasst wurden.
  if false
  % Bestimme die KS und den Neigungswinkel der Beinketten-Basis
  [Tc_Lges_0,Tc_Lges_W] = R.fkine_legs(Q(1,:)');
  alpha = acos(R.Leg(1).T_W_0(1:3,3)'*[0;0;1]);
  % Offset für die Schubgelenke setzen. Dadurch wird das Schubgelenk weiter
  % oben gezeichnet (roter Quader) und nur nach unten zum nächsten Gelenk
  % verlängert. Wird zwar mittlerweile in der Maßsynthese berücksichtigt,
  % dort wird aber auch die Basis-Höhe optimiert. Wenn die Basis hoch
  % steht, entsteht ein langer Offset, den man ohne den folgenden Code
  % nicht mehr wegbekommt.
%   if false
%   q1_min = min(min(Q(:,R.MDH.sigma==1)));
%   q1_max = max(max(Q(:,R.MDH.sigma==1)));
  % Siehe Aufzeichnungen Schappler vom 24.09.2020.
  % Position des Schubgelenks (erste Beinkette)
  r_0_S1 = R.Leg(1).T_W_0*[0;0;Q(1,1);1];
  % Höchste Position der Drehgelenke in der Beinkette (bezogen auf Welt-KS)
  % TODO: Aktuell sind hier die Schubgelenke mit enthalten!
  min_z0_pos_legjoints = max(jointssqueeze(Tc_Lges_W(3,4,:)));
  % Kleinster Abstand zwischen Schub- und Drehgelenken
  d_min = min_z0_pos_legjoints - r_0_S1(3);
  h_safe = 0.1; % um so viel höher als das höchste Drehgelenk soll das Schubgelenk sein
  off_min = 0.15; % Minimaler Offset, der gewählt werden soll
  % Differenz im Basis-KS
  r_0_deltao = max(-d_min + h_safe);
  % Umrechnen auf Beinketten-Basis-KS
  r_1_deltao = R.Leg(1).T_W_0(1:3,1:3)' * [0;0;r_0_deltao];
  % Offset aus der z-Komponente (entspricht Schubachse)
  offset = max(abs(r_1_deltao(3)), off_min)*sign(r_1_deltao(3));
  % Eintragen in Roboter-Klasse
  for kk = 1:R.NLEG
    R.Leg(kk).DesPar.joint_offset = [offset;NaN(4,1)];
  end
  return
%   end
  % Noch manuelle Anpassungen, damit die Plots schöner aussehen.
  % Kinematisch ist alles gleichwertig.
  delta_base_0 = 0;
  if abs(cos(alpha)) > 0.9
    % Bestimme Position der ersten Schubachse
    Q_off = Q;
    offset = R.Leg(1).DesPar.joint_offset(1); % aus Optimierung
    Q_off(:,R.MDH.sigma==1) = Q_off(:,R.MDH.sigma==1) - offset;
    [Tc_Lges_0] = R.fkine_legs(Q_off(1,:)');
    
    % Basis verschieben (so, dass Schubgelenk direkt an der Basis anfängt)
    % Durch den Offset wird das Schubgelenk aber verschoben.
    % TODO: Das wäre eigentlich eleganter als die Lösung mit der direkten
    % Kinematik.
%     h_deltaB_min = 0.1; % immer mindestens um diese Höhe anheben
%     if abs(q1_min) < abs(q1_max) % hängt von Zeichnung der seriellen Kette ab
%       h_deltaB = (q1_min)*cos(alpha);
%     else
%       h_deltaB = (q1_max)*cos(alpha);
%     end
    % delta_base_0 = max(abs(h_deltaB),h_deltaB_min)*sign(h_deltaB); % bezogen auf Basis-KS
%     delta_base_0 = max(h_deltaB,h_deltaB_min); % bezogen auf Basis-KS
    % Das Schubgelenk mit Berücksichtigung des Offsets soll bei 0.1 liegen.
    delta_base_0 = -Tc_Lges_0(3,4,2) + 0.1;
    % Neue Basis-Position eintragen
    r_W_0_alt = R.T_W_0(1:3,4);
    r_W_0_neu = r_W_0_alt + R.T_W_0(1:3,1:3)*[0;0;delta_base_0];
    R.update_base(r_W_0_neu);
    % Die Größe des Gestells ändert sich dadurch auch (wenn die
    % Schubgelenke nach oben zusammenlaufen, wird bei positivem
    % delta_base_0 der Gestell-Radius kleiner.
    r_base_neu = R.DesPar.base_par(1) - delta_base_0*sin(alpha);
    R.align_base_coupling(erg.Structure.Coupling(1), ...
      [r_base_neu, R.DesPar.base_par(2:end)]);
    % Plattform im Basis-KS auch verschieben
    X_plot(:,3) = X_plot(:,3) + delta_base_0;
    % Schubgelenk-Koordinaten entsprechend ändern? Geht nur bei senkrechter Basis.
%     delta_q = delta_base_0/cos(alpha);
    % Inverse Kinematik muss neu berechnet werden
    % Dafür müssen die Grenzen zurückgesetzt werden (des Schubgelenks)
    for kk = 1:R.NLEG
      R.Leg(kk).qlim(1,:) = R.Leg(kk).qlim(1,:) + [-delta_base_0, delta_base_0];
    end
    % IK für komplette Trajektorie, damit die Grenzen daraus neu berechnet
    % werden.
    for kk = 1:size(Q,1)
      % Schätzung der neuen Koordinaten (hauptsächlich Änderung der
      % Schubgelenk-Koordinate)
      if kk == 1 % Anfangswerte der IK setzen
        q_neu = Q(kk,:)';
        q_neu(R.MDH.sigma==1) = q_neu(R.MDH.sigma==1) + delta_base_0*cos(alpha);
        phi_alt = R.constr1(Q(kk,:)', X_plot(kk,:)');
        phi_neu = R.constr1(q_neu, X_plot(kk,:)');
        if max(abs(phi_neu)) > max(abs(phi_neu))
          error('Eigene Berechnung des IK-Anfangswerts war schlecht');
        end
      else
        q_neu = Q_plot(kk-1,:)'; % stetige Trajektorie; vorheriger Wert daneben.
      end
      [Q_plot(kk,:),Phi] = R.invkin2(X_plot(kk,:)', q_neu);
      if any(abs(Phi)>1e-4)
        error('Durch manuelle Änderungen für den Plot stimmt die IK nicht mehr');
      end
    end
    % Prüfe auf Plausibilität des IK-Ergebnisses
    if cos(alpha)>(1-1e-6) % senkrechte Anordnung
      test_Q_plot = Q_plot - Q;
      test_Q_plot(abs(abs(test_Q_plot)-2*pi)<0.1) = NaN; % 2pi ignorieren
      test_Q_plot(:,R.MDH.sigma==1) = test_Q_plot(:,R.MDH.sigma==1)-delta_base_0; % Verschiebung der PKM in Gelenken nachholen
      if max(abs(test_Q_plot(:))) > 1e-4
        error('Neue Gelenkwinkel, obwohl Plattform und Gestell gleichermaßen verschoben wurden und Anordnung senkrecht ist.');
      end
    end
    % Neue Gelenkwinkelgrenzen eintragen für Plot (Führungsschiene)
    for kk = 1:R.NLEG
      Q_kk = Q_plot(:,R.I1J_LEG(kk):R.I2J_LEG(kk));
      R.Leg(kk).qlim(R.Leg(kk).MDH.sigma==1,:) = minmax2(Q_kk(:,R.Leg(kk).MDH.sigma==1)');
    end
  end
  end
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
  % automatisch generierten Plot nachverarbeiten
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
  
  % Dummy-Plot für Legende mit Roboter-Marker
  hdl=plot3(0,0,NaN, RobotGroups{i,4});
  I_latexname = strcmp(ResTab_NameTrans.PKM_Name, RobotGroups{i,2}{1});
  legstr_i = [sprintf('%d: ', countrob), '3',ResTab_NameTrans.ChainStructure{I_latexname}];
  legstr_i = strrep(legstr_i, 'P', '\underline{P}');
  % abgespeicherten Marker-Typ einzeichnen (zur Wiedererkennung beim
  % Zusammenstellen der Bilder)
  % lh = legend(hdl, legstr_i, 'interpreter', 'latex'); 
  % set(lh, 'location', 'northeastoutside');
  
  % Nehme überall die gleiche Perspektive für Vergleichbarkeit der Bilder?
  % Nein. Dann sind die Verdeckungen zu hoch.
  view(42,5); % Standard-Perspektive
  % Manuelle Anpassungen für einzelne Bilder möglichst wenig Verdeckungen)
  if i == 3
    view(14,9);
  elseif i == 4
    view(24,11);
  elseif i == 5
    view(-24,9);
  elseif i == 6
    view(-89,12);
  elseif i == 7
    view(18,12);
  elseif i == 8
    view(39,19);
  end
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
  name = sprintf('RobotFig_PlotNum%d_RobGroup%d_%s', countrob, i, GroupName);
  cd(outputdir);
  export_fig([name, '_r864.png'], '-r864');
  % export_fig([name, '.pdf']);
end
