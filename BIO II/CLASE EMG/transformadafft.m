function [G,g,t,Fs,dc] = transformadafft(nombrearchivo,columna,fmaxima,intervalo,dc)
%% Función TRANSFORMADAFFT
% La función TRANSFORMADAFFT realiza la carga de un archivo csv y
% posteriormente retorna la FFT de la señal multiplicada por el
% factor 2*pi/L (con L igual a la longitud del vector analizado).
%
% Ejemplos:
%
% Considerando que se dispone de un archivo de nombre ECG1kHz.csv, con
% frecuencia de muestreo de 1MHz y que la señal se almacena en la segunda
% columna del archivo CSV; la función TRANSFORMADAFFT se puede invocar así:
%
% TRANSFORMADAFFT('ECG1kHz.csv',2);
%
% Si se desea limitar el intervalo de frecuencias a 80kHz, se debe
% ejecutar:
%
% TRANSFORMADAFFT('ECG1kHz.csv',2,80000);
%
% Si se desea analizar solo un periodo de la señal, y considerando que cada
% periodo es de 1001 puntos, se podría especificar el intervalo de puntos a
% analizar:
%
% TRANSFORMADAFFT('ECG1kHz.csv',2,80000,1100:2100);
%
% Si se desea incluir la componente DC en la gráfica del espectro positivo,
% se debe agregar un 0 como parámetro adicional:
%
% TRANSFORMADAFFT('ECG1kHz.csv',2,80000,1100:2100,0);
%
% Función TRANSFORMADAFFT, elaborado por Hans López. hanslop@gmail.com
% Versión 2.0. 12 de febrero de 2019.
% - Se eliminó la necesidad de especificar la frecuencia de muestreo.
% - Se corrigió la amplitud del espectro de magnitud.
% - Se ajustó el orden de los argumentos (porque se eliminó Fs).
% - Se agregó la variable DC, para incluir la componente DC en el espectro
% positivo.
% - Se agregaron: g, t, Fs y el nivel DC de la señal, como argumentos de
% salida.
% Versión 1.0. 19 de noviembre de 2019.
    switch nargin
        case 0
            columna = 2;
            a = csvread('ECG1kHz.csv',columna,0);
            y = a(:,2)';
            L = numel(y);
            t = a(:,1)';
            duracion = t(end) - t(1);
            Fs = L/duracion;
            fmaxima = Fs/2;
            intervalo = 1:numel(y);
            dc = mean(y);
        case 1
            columna = 2; 
            a = csvread(nombrearchivo,columna,0);
            y = a(:,2)';
            L = numel(y);
            t = a(:,1)';
            duracion = t(end) - t(1);
            Fs = L/duracion;
            fmaxima = Fs/2;
            intervalo = 1:numel(y);
            dc = mean(y);
        case 2
            a = csvread(nombrearchivo,columna,0);
            y = a(:,2)';
            L = numel(y);
            t = a(:,1)';
            duracion = t(end) - t(1);
            Fs = L/duracion;
            fmaxima = Fs/2;
            intervalo = 1:numel(y);
            dc = mean(y);
        case 3
            a = csvread(nombrearchivo,columna,0);
            y = a(:,2)';
            L = numel(y);
            t = a(:,1)';
            duracion = t(end) - t(1);
            Fs = L/duracion;
            intervalo = 1:numel(y);
            dc = mean(y);
        case 4
            a = nombrearchivo(:,1);
            y = nombrearchivo(:,2);
            L = numel(y);
            t = a(:,1)';
            duracion = t(end) - t(1);
            Fs = L/duracion;
            y = a(intervalo,columna)';
            dc = mean(y);
        case 5
            a = csvread(nombrearchivo,columna,0);
            y = a(:,2)';
            L = numel(y);
            t = a(:,1)';
            duracion = t(end) - t(1);
            Fs = L/duracion;
    end
    
    y = a(intervalo,columna)';
    g = y - dc;

    
    T = numel(g)/Fs;

    t = 0:1/Fs:T-1/Fs;
    f = (0:fmaxima*T)/T;

    G = 2*pi*fft(g)/numel(g);
    magnitud = abs(G);
    dc = mean(y);
end