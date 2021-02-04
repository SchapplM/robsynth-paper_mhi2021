% PKM erneut prüfen, die bei der Maßsynthese kinematisch nicht funktionierten
% 
% Folgende gingen nicht in Maßsynthese vom 06.08.2020:
% P3PRRRR8G1P1A1
% P3PRRRR8G1P2A1
% P3PRRRR8V1G1P3A1
% P3PRRRR8V2G1P3A1

% Folgende gingen nicht in Maßsynthese vom 07.08.2020 (mit theta1=0):
% P3PRRRR6G1P1A1
% P3PRRRR6G1P2A1
% P3PRRRR6V1G1P1A1
% P3PRRRR6V1G1P3A1
% P3PRRRR8V1G1P2A1
% P3PRRRR8V1G4P2A1
% P3PRRRR8V2G1P2A1
% P3PRRRR8V2G4P2A1

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
addpath(fullfile(fileparts(which('structgeomsynth_path_init.m')),'struktsynth_par'));
%% Allgemeine Einstellungen
settings = struct( ...
  'EE_FG_Nr', 2, ... % 3T0R
  'check_existing', true, ...
  'dryrun', false, ...
  'check_resstatus', 0:8, ... % immer neu prüfen
  'parcomp_structsynth', 0, ...
  'parcomp_mexcompile', 0, ...
  'max_actuation_idx', 1);
%% Einstellung für P3PRRRR8...
settings.whitelist_SerialKin = {'S5PRRRR8'};
settings.onlygeneral = true;
settings.base_couplings = 1;
settings.plf_couplings = 1:2;
parroblib_add_robots_symact

%% Einstellung für P3PRRRR8V1/P3PRRRR8V2...
settings.whitelist_SerialKin = {'S5PRRRR8V1', 'S5PRRRR8V2'};
settings.onlygeneral = false;
settings.base_couplings = 1;
settings.plf_couplings = 3;
parroblib_add_robots_symact

%% Prüfe fehlerhafte PKM vom 10.08.2020
% Folgende gingen nicht in Maßsynthese vom 10.08.2020:
% P3PRRRR6G1P2A1
% P3PRRRR6V1G1P2A1
% P3PRRRR6V1G4P2A1
% P3PRRRR8G1P2A1
settings.selectgeneral = true;
settings.selectvariants = true;
settings.whitelist_SerialKin = {'S5PRRRR6', 'S5PRRRR6V1', 'S5PRRRR8'};
settings.base_couplings = [1,4];
settings.plf_couplings = 2;
parroblib_add_robots_symact


%% Einstellung für P3PRRRR6G1P1A1...
settings.whitelist_SerialKin = {'S5PRRRR6'};
settings.onlygeneral = true;
settings.base_couplings = 1;
settings.plf_couplings = 1;
parroblib_add_robots_symact

