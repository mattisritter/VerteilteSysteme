function [c,te] = cd_refpoly_vmax(vmax, x0, xs)
% Berechnet die Koeffizienten für des Führungssignal uVp1 der Vorsteuerung 
% Parameter Typ         Beschreibung
% vmax      1x1 double  Maximale Soll-Geschwindigkeit v* [m/s]
% x0        1x1 double  Startposition [m]
% xs        1x1 double  Soll-Bogenlänge x* [m]
% Rückgabe  Typ         Beschreibung
% c         6x1 double  Koeffizienten des Führungssignal wp
% te        1x1 double  Endzeit des Manövers [s]
te = 15 * xs / (8 * vmax);
c = [65536*vmax*vmax*vmax*vmax*vmax /(xs*xs*xs*xs*253125);...
     -4096*vmax*vmax*vmax*vmax / (xs*xs*xs*3375);...
     1024*vmax*vmax*vmax / (xs*xs*675);...
     0;...
     0;...
     x0];
end

