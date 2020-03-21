function [] = superfuncion(datos,nombre)
switch nargin
    case 0
        disp('Falta el vector de datos');
        return;
    case 1
        nombre = 'aleatorio';
end

p = linspace(0,1,100000);
datos = 1.000000000000001*datos(:)';
numero = quantile(datos,p);

encabezado = ['function y = ',nombre,'(m,n)'];
cuerpo = 'numero = [';
final = '];y = numero(randi(numel(numero),m,n));end';

fid = fopen([nombre,'.m'],'w');

%fprintf(fid,'%s\n%s\n%10.8f\n%s\n',encabezado,cuerpo,numero,final);
fprintf(fid,'%s\n%s\n%s\n%s\n',encabezado,cuerpo,numero,final);
fclose(fid);
disp([nombre,'.m codificado. Revisa tu carpeta de MATLAB, por favor.']);
disp(['Luego puedes ejecutar: ',nombre,'(m,n).']);
end