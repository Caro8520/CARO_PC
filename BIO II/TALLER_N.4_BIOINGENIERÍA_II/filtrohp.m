function H = filtrohp(frecuencias,fc,orden)
    n = frecuencias;
    coeficientes = butterworthcoef(orden);
    
    H = 1;
    if orden > 0    
        for m = 1:numel(coeficientes)
            H = H.*(-4*pi*pi.*n.*n)./...
                ((-4*pi*pi.*n.*n)+ (4*pi*pi*n*fc.*coeficientes(m)).*i + 4*pi*pi*fc*fc);
        end
        
        if mod(orden,2) ~= 0 %si el orden del filtro es impar
            H = H.*(2*pi*n*i)./(2*pi*n*i+2*pi*fc);
        end
    end
end

function y = butterworthcoef(n)
    if (n/2)==(floor(n/2))
        maximo = n/2;
    else
        maximo = (n-1)/2;    
    end
        y = zeros(1,maximo);        
    for k = 1:maximo
        y(k) = -2*cos((2*k+n-1)*pi/(2*n));
    end
end