function [] = superplot(x,y,titulo,ejex,ejey,grosor,colorlinea)
%   Utiliza la función plot para realizar la gráfica de f(x). SUPERPLOT
%   cambia el fondo de la figura a color blanco y el tamaño de las fuentes.
%   Adicionalmente ajusta de forma automática los límites de las gráficas.
%
%   La función permite agregar el título de la gráfica, y el nombre de los
%   ejes como parámetros de entrada.
%
%   Ejemplo:
%
%   x = linspace(-5,5,1000);
%   SUPERPLOT(x,sinc(x),'Función Seno Cardinal','$x$','sinc($x$)');
%
%   Función SUPERPLOT, Versión 5.0, 28 de agosto de 2019
%   - Se cambió de gráfica de puntos a línea contínua de grosor 2.
%   Versión 4.0, 23 de marzo de 2019
%   - se agregó un séptimo parámetro para modificar el color de la línea.
%   Versión 3.0, 18 de mayo de 2017
%   - se modificó el interprete de los ejes a LaTeX. El interprete del
%   título no se modificó.
%   Versión 2.0, 2 de diciembre de 2016:
%   - Se modificó el tamaño del punto a 10, para mejorar la visualización.
%   - El tamaño del punto se puede modificar indicándolo en el sexto
%   parámetro, como muestra el ejemplo.
%   Versión 1.0, 29 de septiembre de 2015
%   Elaborada por Hans López, hanslop@gmail.com
%    
switch nargin
    case 0
        disp('Faltan argumentos de entrada');
        return;
    case 1
        plot(x,'LineWidth',2);
        grid on;
        set(gca,'FontSize',14);
        set(gcf,'Color','white');
        axis([1 length(x) min(x)-0.1*(max(x)-min(x)) max(x)+0.1*(max(x)-min(x))]);
        return;
    case 2
        plot(x,y,'LineWidth',2);
    case 3
        plot(x,y,'LineWidth',2);
        title(titulo,'FontSize',16);
    case 4
        plot(x,y,'LineWidth',2);
        title(titulo,'FontSize',16);
        xlabel(ejex,'FontSize',16,'Interpreter','latex');
    case 5
        plot(x,y,'LineWidth',2);       
        title(titulo,'FontSize',16);
        xlabel(ejex,'FontSize',16,'Interpreter','latex');
        ylabel(ejey,'FontSize',16,'Interpreter','latex');
    case 6        
        plot(x,y,'LineWidth',grosor);       
        title(titulo,'FontSize',16);
        xlabel(ejex,'FontSize',16,'Interpreter','latex');
        ylabel(ejey,'FontSize',16,'Interpreter','latex');
    case 7        
        plot(x,y,'Color',colorlinea,'LineWidth',grosor);       
        title(titulo,'FontSize',16);
        xlabel(ejex,'FontSize',16,'Interpreter','latex');
        ylabel(ejey,'FontSize',16,'Interpreter','latex');
end
grid on;
set(gca,'FontSize',14);
set(gca,'TickLabelInterpreter','latex');
set(gcf,'Color','white');
axis([min(x) max(x) min(y)-0.1*(max(y)-min(y)) max(y)+0.1*(max(y)-min(y))]);
end