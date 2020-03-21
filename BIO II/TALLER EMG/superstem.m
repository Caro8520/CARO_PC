function [] = superstem(x,y,titulo,ejex,ejey,colordelta)
%% Función SUPERSTEM
%   Utiliza la función stem para realizar la gráfica de f[n]. SUPERSTEM
%   cambia el fondo de la figura a color blanco y el tamaño de las fuentes.
%   Adicionalmente ajusta de forma automática los límites de las gráficas.
%
%   La función permite agregar el título de la gráfica, y el nombre de los
%   ejes como parámetros de entrada.
%
%   Ejemplo:
%
%   n = -15:15;
%   SUPERSTEM(n,sinc(n/5),'Gráfica de sinc[n/5]','n','sinc[n/5]');
%
%   Función SUPERSTEM, Versión 4.0, 10 de octubre de 2019
%   - se realizaron ajustes gráficos para concordar con las proporciones de
%   la función SUPERPLOT.
%   Versión 3.0, 29 de septiembre de 2017
%   - se modificó el interprete de los ejes a LaTeX. El interprete del
%   título no se modificó.
%   Versión 2.0, 28 de noviembre de 2016
%   - Se corrige un error que no permitia graficar secuencias constantes.
%   - Se modifican los límites de la variable independiente, para que
%   queden a media unidad de cada extremo.
%   Versión 1.0, 26 de noviembre de 2016
%   Elaborada por Hans López, hanslop@gmail.com
%    
    switch nargin
        case 0
            disp('Faltan argumentos de entrada');
            return;
        case 1
            stem(x,'.','Linewidth',2,'MarkerSize',16);
            grid on;
            set(gca,'FontSize',14);
            set(gcf,'Color','white');
          
            xminimo = 0.5;
            xmaximo = length(x)+0.5;
            yminimo = min(x)-0.1*(max(x)-min(x));
            ymaximo = max(x)+0.1*(max(x)-min(x));

            if yminimo ~= ymaximo
                axis([xminimo, xmaximo, yminimo, ymaximo]);
            elseif ymaximo>0
                axis([xminimo, xmaximo, -0.1*ymaximo, 1.1*ymaximo]);
            elseif ymaximo<0
                axis([xminimo, xmaximo, 1.1*ymaximo, -0.1*ymaximo]);
            else
                axis([xminimo, xmaximo, -1, 1]);
            end
            
            return;
        case 2
            stem(x,y,'.','Linewidth',2,'MarkerSize',16);
        case 3
            stem(x,y,'.','Linewidth',2,'MarkerSize',16);
            title(titulo,'FontSize',16);
        case 4
            stem(x,y,'.','Linewidth',2,'MarkerSize',16);
            title(titulo,'FontSize',16);
            xlabel(ejex,'FontSize',16,'Interpreter','latex');
        case 5
            stem(x,y,'.','Linewidth',2,'MarkerSize',16);      
            title(titulo,'FontSize',16);
            xlabel(ejex,'FontSize',16,'Interpreter','latex');
            ylabel(ejey,'FontSize',16,'Interpreter','latex');
        case 6
            stem(x(y>0),y(y>0),'^','color',colordelta,'Linewidth',2,'MarkerSize',16);
            hold on;
            stem(x(y<0),y(y<0),'v','color',colordelta,'Linewidth',2,'MarkerSize',16);
            hold off;
            title(titulo,'FontSize',16);
            xlabel(ejex,'FontSize',16,'Interpreter','latex');
            ylabel(ejey,'FontSize',16,'Interpreter','latex');    
    end
    grid on;
    set(gca,'FontSize',14);
    set(gca,'TickLabelInterpreter','latex');
    set(gcf,'Color','white');
    xminimo = min(x)-0.5;
    xmaximo = max(x)+0.5;
    yminimo = min(y)-0.1*(max(y)-min(y));
    ymaximo = max(y)+0.1*(max(y)-min(y));
    
    if yminimo ~= ymaximo
        axis([xminimo, xmaximo, yminimo, ymaximo]);
    elseif ymaximo>0
        axis([xminimo, xmaximo, -0.1*ymaximo, 1.1*ymaximo]);
    elseif ymaximo<0
        axis([xminimo, xmaximo, 1.1*ymaximo, -0.1*ymaximo]);
    else
        axis([xminimo, xmaximo, -1, 1]);
    end
  
end