% Definition der Trajektorie für die Kryo-PKM
% 
% Eingabe:
% trajset
%   Einstellungen für Trajektorie
%   Siehe cds_settings_defaults.m
% 
% Ausgabe:
% Traj
%   Struktur mit Trajektorie des Endeffektors (Zeit, Position, Geschw.)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-07
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function Traj = traj_cryotask(trajset)
x0 = [0.5, 0.5, 0, 0, 0, 0]';

%% Punkte definieren
% Punkte für Probenröhrchen
% Siehe Bild in "Rahmenparameter Kryo KPM.docx"
% Die Ecke bei A1 entspricht (0/0)
% x zählt von links nach rechts (1,2,3,4,...)
% y zählt von oben nach unten (A,B,C,D,...), daher negativ
x_val = 1e-3*((9:9:12*9) -9 +  (127.8 -11*9)/2);
y_val = -1e-3*((9:9:8*9) -9 + ( 85.5-7*9)/2);
[Rack_x,Rack_y] = ndgrid(x_val,y_val);
% Nehme die Eckpunkte und einen Punkt aus der Mitte
II_sel = [[1,1]; [1,length(y_val)]; [length(x_val),1]; ...
  [length(x_val),length(y_val)]; round([length(x_val),length(y_val)]/2)];
% Positionen der Racks (obere linke Ecke) bzgl des Welt-KS (in der Mitte des
% Behälters)
T_W_R1 = transl([-100; 100;0]*1e-3);
T_W_R2 = transl([-100;-100+85.5;0]*1e-3);
% Position des Scanners (Mitte des verbliebenen Platzes rechts)
r_W_S = 1e-3*[-100+127.8+(200-127.8)/2; 0; 110];
% Mittelstellung im Behälter auf Arbeitshöhe
r_W_M = 1e-3*[0;0;110];
% Arbeitshöhe vor dem Greifen
h_pregrasp = 1e-3*110;
% Arbeitshöhe beim Greifen
h_grasp = 1e-3*(110-45.2);

%% Trajektorie zusammenstellen
k=1; XE = [r_W_M', [0,0,0]];
% Fahrt zum Startpunkt
% k=k+1; XE(k,:) = [r_W_M', [0,0,0]];
% Anfahrt aller Testpunkte und Scannen
for i = 1:5
  % Fahrt zur Position auf Rack 1
  r_R_Pi = [Rack_x(II_sel(i,1),II_sel(i,2)); ...
             Rack_y(II_sel(i,1),II_sel(i,2));0];
  % Umrechnung in Welt-KS
  r_W_Pi = eye(3,4)*T_W_R1*[r_R_Pi;1];
  % Greifposition über Probenplatz
  k=k+1; XE(k,:) = [[r_W_Pi(1:2)',h_pregrasp], [0,0,0]];
  % Greifen (herunterfahren)
  k=k+1; XE(k,:) = XE(k-1,:); XE(k,3) = h_grasp;
  % Herausholen
  k=k+1; XE(k,:) = XE(k-1,:); XE(k,3) = h_pregrasp;
  % Fahrt zum Scanner
  k=k+1; XE(k,:) = [r_W_S', [0,0,0]];
  % Fahrt zur Position auf Rack 2
  % Umrechnung in Welt-KS
  r_W_Pi = eye(3,4)*T_W_R2*[r_R_Pi;1];
  k=k+1; XE(k,:) = [[r_W_Pi(1:2)',h_pregrasp], [0,0,0]];
  % Herunterfahren
  k=k+1; XE(k,:) = XE(k-1,:); XE(k,3) = h_grasp;
  % Wieder hochfahren
  k=k+1; XE(k,:) = XE(k-1,:); XE(k,3) = h_pregrasp;
end
% Fahre in Mittelstellung zurück
k=k+1; XE(k,:) = [r_W_M', [0,0,0]];
%% Trajektorie generieren
if trajset.profile == 1
  [X_ges,XD_ges,XDD_ges,T_ges] = traj_trapez2_multipoint(XE, ...
    trajset.vmax, trajset.vmax/trajset.amax, trajset.Tv, trajset.Ts, 0);
elseif trajset.profile == 0 % Nur Eckpunkte
  X_ges = XE;
  XD_ges = XE*0;
  XDD_ges = XE*0;
  T_ges = (1:size(XE,1))'; % Muss Spaltenvektor sein
else
  error('Profil nicht definiert');
end

%% Ausgabe
Traj = struct('X', X_ges, 'XD', XD_ges, 'XDD', XDD_ges, 't', T_ges, 'XE', XE);
return
