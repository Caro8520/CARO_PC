function [] = pdfcdfcontinua(datos,minimo,maximo)
    switch nargin
        case 1
            rango = max(datos(:)')-min(datos(:)');
            minimo = min(datos(:)')-0.2*rango;
            maximo = max(datos(:)')+0.2*rango;
    end            
    datos = datos(:)';
    total = numel(datos);
    particiones = floor(sqrt(total));
    x = linspace(minimo,maximo,particiones);
    
    fr = histc(datos,x)/total;
    cdf = cumsum(fr);
    subplot(2,1,2);
    superplot(x,cdf,...
    'Estimación de la Distribución Acumulativa',...
    '$x$','$F_X(x)$');
    subplot(2,1,1);
    
    pdf = fr/(x(2)-x(1));
    
    superplot(x,pdf,...
    'Estimación de la función de densidad',...
    '$x$','$f_X(x)$');    
end