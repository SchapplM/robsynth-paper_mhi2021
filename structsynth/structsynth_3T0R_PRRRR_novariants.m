% Erzeuge PKM mit EE-FG 3T0R, deren Beinkette aus allgemeinen Modellen bestehen
% 
% Die unten stehenden Einstellungen können noch manuell angepasst werden:
% * Nur Auswahl bestimmter Koppelgelenk-Anordnungen
% * Wechsel zwischen Ausführung der Berechnung auf Cluster und lokaler
%   Auswertung und Eintragen ind die Roboterbibliothek.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-07
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
addpath(fullfile(fileparts(which('structgeomsynth_path_init.m')),'struktsynth_par'));
settings = struct( ...
  'EE_FG_Nr', 2, ... % 3T0R
  'dryrun', false, ...
  'check_existing', true, ...
  'selectgeneral', true, ... % nur zur Erzeugung ...
  'selectvariants', false, ... % ... des Namens der Optimierung
  'whitelist_SerialKin', {{'S5PRRRR2', 'S5PRRRR3', 'S5PRRRR4', 'S5PRRRR5', ...
  'S5PRRRR6', 'S5PRRRR7', 'S5PRRRR8', 'S5PRRRR9', 'S5PRRRR10'}}, ...
  'ignore_check_leg_dof', true, ... % Funktion zur Schnell-Überprüfung evtl falsch
  'check_resstatus', 0:8, ... % Alle testen
  'comp_cluster', true, ...
  ...'base_couplings', 4, ... % Debug: nur einzelne Kombinationen
  ...'plf_couplings', [3 8], ... % Debug: nur einzelne Kombinationen
  'parcomp_structsynth', 1, ...
  'parcomp_mexcompile', 0, ...
  'max_actuation_idx', 1);
% Zur Auswertung nach Cluster-Berechnung:
settings.comp_cluster = false;
settings.offline = true;

parroblib_add_robots_symact