% Geometrie der Kryo-PKM-Aufgabe definieren: Bauraum und Kollisionskörper
% 
% Eingabe:
% Set
%   Einstellungsvariable aus cds_settings_defaults
% 
% Ausgabe:
% Set
%   wie Eingang, mit mehr Informationen (durchgeschleift)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-07
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function Set = environment_cryotask(Set)

%% Bauraum / Aufgabengeometrie
% Der Raum außerhalb der vorgegebenen Objekte ist nicht erlaubt für den
% Roboter.
% Kryobehälter hat Innendurchmesser 600mm, Höhe 680mm. Reduziere die Maße
% etwas für Sicherheitsabstand. Boden des Zylinders ist Ursprung des Welt-KS
% Parameter des Zylinders (1)
h_Zylinder = 0.68; % Höhe
r_Zylinder = 0.25; % Radius
p_Zylinder = [[0,0,0], ... % Erster Punkt (unten)
  [0,0,h_Zylinder], ... % Zweiter Punkt (oben)
  r_Zylinder, NaN(1,3)];
% Der Quader (2) soll einen größeren Bauraum des Roboters oberhalb des
% Behältnisses erlauben
b_Quader = 1.0; % Seitenlänge des Quaders
h_Quader = 0.8; % Höhe des Quaders
p_Quader = [[-b_Quader/2, -b_Quader/2, h_Zylinder], ... % Aufpunkt (unten links); direkt am oberen Zylinderrand
  [b_Quader,0,0], [0,b_Quader,0], ... % Zwei Kantenvektoren
  h_Quader]; % Länge des letzten Kantenvektors
Set.task.installspace = struct( ...
  'type', uint8([2; 1]), ... % Nummern der Geometrie-Typen, siehe Implementierung.
  'params', [p_Zylinder; ... % (1) Zylinder
             p_Quader], ...  % (2) Quader
  'links', {{1:6; 0}}); % bewegte Gelenke müssen im Zylinder sein und bleiben. Basis und Schubgelenk-Führungsschienen müssen oberhalb im Quader sein.
% Hindernisse definieren: Objekte für Übergang zwischen Zylinder und Quader
% (damit der Roboter nicht schräg vom Quader zum Zylinder durchgreift)
% Erzeuge quadratische Sperre aus Kapseln (nichts anderes implementiert)
coord_caps = r_Zylinder+0.1; % Eckpunkt-Koordinate der Kapseln (Symmetrisch um Ursprung)
Set.task.obstacles = struct( ...
  'type', repmat(uint8(3),4,1), ... % Kapseln
  'params', [[[-coord_caps -coord_caps h_Zylinder-0.1], [-coord_caps  coord_caps h_Zylinder-0.1], 0.10]; ...
             [[ coord_caps -coord_caps h_Zylinder-0.1], [ coord_caps  coord_caps h_Zylinder-0.1], 0.10]; ...
             [[-coord_caps -coord_caps h_Zylinder-0.1], [ coord_caps -coord_caps h_Zylinder-0.1], 0.10]; ...
             [[-coord_caps  coord_caps h_Zylinder-0.1], [ coord_caps  coord_caps h_Zylinder-0.1], 0.10]]);
% Zusätzlich noch einfache Geraden eintragen. Sonst kann schräg über der
% Rundung der Kapsel trotzdem noch durchgegriffen werden. Dadurch haben
% schräge Gestellgelenke mehr Freiheit, als sie haben dürften.
for ii = 1:2 % drei übereinander (oben anfangen)
  for jj = 1:8 % vier um den Zylinder herum
    cap_z = h_Zylinder - (ii-1)*0.05;
    % Vektor zum Mittelpunkt der Kapsel
    cap_c = rotz(pi/4*jj)*[r_Zylinder;0;cap_z];
    % Richtungsvektor der Kapsel
    cap_v = rotz(pi/4*jj)*[0;1;0];
    % Beide Endpunkte der Kapsel
    cap1 = cap_c+cap_v*coord_caps;
    cap2 = cap_c-cap_v*coord_caps;
    Set.task.obstacles.type = [Set.task.obstacles.type; uint8(3)];
    Set.task.obstacles.params = [Set.task.obstacles.params; ...
      [cap1', cap2', 5e-3]];
  end
end
%% Eigenschaften des Roboters, von Aufgabengeometrie beeinflusst
% Das Gestell sollte nicht größer als die Bauraumgrenze (Quader) sein.
% In Anforderungen auf Durchmesser des Behälters festgelegt. Setze die
% maximale Größe etwas größer (hier: Radius).
Set.optimization.base_size_limits = [0.100, 0.400];
% Die Führungsschienen der Schubgelenke dürfen über die Grenzen des
% Gestells (schräg nach oben) hinausstehen. Dadurch oben 800mm Radius möglich
Set.optimization.base_tolerance_prismatic_guidance = 2.0;
% Die Plattform sollte nicht größer als die Bauraumgrenze (Zylinder) sein.
% Min.-Radius 80mm vorgegeben aus Gelenkgröße. Max.-Radius 250mm aus Behälter
Set.optimization.platform_size_limits = [0.080, 0.250];
% Die Basis muss innerhalb der oberen Bauraumgrenze (Quader) sein.
% Setze die xy-Position auf die Mitte, da Aufgabe bekannterweise
% symmetrisch ist für PKM. Genauso keine Optimierung der z-Komponente. Ist
% redundant zu anderen Optimierungskriterien. Die Führungsschienen werden
% automatisch weiter nach hinten gesetzt, falls kinematisch notwendig.
Set.optimization.basepos_limits = [[0, 0]; [0, 0]; [h_Zylinder+0.1 h_Zylinder+0.1]];
% Roboter soll hängend von oben nach unten arbeiten.
% (Bei Schubgelenken als erste Achse eigentlich egal, aber für Wahl der
% positiven Koordinatenrichtung entscheidend).
Set.structures.mounting_parallel = 'ceiling';
%% Sonstige Eigenschaften des Roboters
% Modelliere die Festkörpergelenke als Drehfeder in passiven Gelenken.
% Annahme: Gegenmoment von 1.4Nm bei Auslenkung von 27.5°. Entspricht
% favorisiertem Prototypen für das Festkörpergelenk.
Set.optimization.joint_stiffness_passive_revolute = 1.4/(pi/180*27.5);
% Die Festkörpergelenke begrenzen die möglichen Drehwinkel
Set.optimization.max_range_passive_revolute = 55*pi/180;
% Kollisionsprüfung ist notwendig.
Set.optimization.constraint_collisions = true;
%% Zusätzliche Masse am Endeffektor
% Repräsentiert aufgenommenes Objekt und den Greifer.
% Hat etwas höheren Wert als in späterer Anwendung, damit Reserve in
% Auslegung da ist und damit Ungenauigkeit der Dynamik-Parametrierung der
% Roboterstruktur ausgeglichen wird (sonst wird der relative Vergleich der
% Strukturen dadurch stark beeinträchtigt)
% Enthält auch Greifer. 3kg daher in Ordnung.
Set.task.payload = struct('m', 3, 'rS', zeros(3,1), 'Ic', zeros(6,1));
Set.task.payload.Ic(1:3) =  2/5 * Set.task.payload.m * (60e-3)^2; % Kugel Radius 60mm
