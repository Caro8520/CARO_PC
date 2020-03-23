close all,clear all,clc
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%POSICIÓN INICIAL DEL VEHÍCULO
x = zeros(3,1);
y = zeros(3,1);
theta = zeros(3,1);
u = zeros(3,1);
r = zeros(3,1);
%POSICIÓN DESEADA DEL VEHÍCULO
xd=-2;
yd=-2;
thetad=(180/pi)*atan2(yd-y(1),xd-x(1));
%PARAMETROS FIJOS DEL VEHÍCULO
Tm = 0.01;      %PERIODO DE MUESTREO DE LOS MOTORES
Tv = 0.1;       %PERIODO DE MUESTREO DEL VEHICULO
vC=0.005;       %VELOCIDAD LINEAL DESEADA
%PARAMETROS MODELO DEL CARRO
m = 4;          %Masa del vehiculo en (Kg)
L = 0.235;      %Distancia entre las ruedas del vehiculo en (m)
rr = 0.035;     %Radio de las ruedas del vehiculo en (m)
R = 0.2;        %Radio de la base del vehiculo en (m)
Iz = 1/4*m*R^2; %Inercia al rededor del eje Z en (Kg-m^2)
%PARAMETROS DINÁMICA DEL VEHICULO
Kt = 1.5;
Xf = 13;      
Xr = 1.64;  
xG = 0;    
Nf = 0.233;  
Nr = 0.008;
%PARAMETROS MOTOR DERECHO DISCRETIZADO
c1d=0.06869;
c2d=0.04179;
d1d=-1.065;
d2d=0.2231;
%PARAMETROS CONTROL PI MOTOR DERECHO
kd=1.2198;
ad=0.7761;
%PARAMETROS MOTOR IZQUIERDO DISCRETIZADO
c1i=0.05992;
c2i=0.03708;
d1i=-1.105;
d2i=0.2346;
%PARAMETROS CONTROL PI MOTOR IZQUIERDO
ki=1.3574;
ai=0.8144;
%VALORES INICIALES MOTOR DERECHO
wR = zeros(3,1);
uR = zeros(3,1);
ewR = zeros(3,1);
%VALORES INICIALES MOTOR IZQUIERDO
wL = zeros(3,1);
uL = zeros(3,1);
ewL = zeros(3,1);
%VALORES INCIALES VARIABLES DEL CONTROLADOR DE ORIENTACIÓN PID
wC=zeros(3,1);
vR=zeros(3,1);
vL=zeros(3,1);
e_theta=zeros(3,1);
%PARAMETROS DEL CONTROL DE ORIENTACIÓN PID
Kp=0.0003;
Kd=0.000005;
Ki=0.000002;
b0=((Kp*Tv)+(Ki*Tv^2)+(Kd))/(Tv);
b1=-((Kp*Tv)+(2*Kd))/(Tv);
b2=Kd/Tv;
%CALCULO DE LA DISTANCIA AL PUNTO DESEADO
i=3;        %CONTADOR VEHICULO
actualizar=1;
dd(i,1)=sqrt((xd-x(i))^2+(yd-y(i))^2);
dd(i-1,1)=dd(i,1)+1;
while dd(i,1)>0.5
        e_theta(i)=thetad-theta(i);                                     %ERROR ANGULAR
        wC(i)=wC(i-1)+b0*e_theta(i)+b1*e_theta(i-1)+b2*e_theta(i-2);    %VELOCIDAD ANGULAR
        vR(i)=vC+(1/2)*R*wC(i);                                         %CINEMATICA INVERSA LLANTA DERECHA
        vL(i)=vC-(1/2)*R*wC(i);                                         %CINEMATICA INVERSA LLANTA IZQUIERDA
        wR(i)=vR(i)/rr;                                                 %VELOCIDAD ANGULAR LLANTA DERECHA
        wL(i)=vL(i)/rr;                                                 %VELOCIDAD ANGULAR LLANTA IZQUIERDA
        %DINÁMICA
        u(i+1) = u(i) + Tv*1/m*(Kt*wR(i)/rr + Kt*wL(i)/rr - Xf*u(i) - Xr*u(i)^2 + xG*r(i)^2);
        if u(i+1)>0.7
            u(i+1)=0.7;
        elseif u(i+1)<-0.7
            u(i+1)=-0.7;
        end
        r(i+1) = r(i) + Tv*1/Iz*(0.5*Kt*L*wR(i)/rr - 0.5*Kt*L*wL(i)/rr - Nf*r(i) - Nr*r(i)^2 - m*xG*u(i)*r(i));
        if r(i+1)>5
            r(i+1)=5;
        elseif r(i+1)<-5
            r(i+1)=-5;
        end
        %POSICIÓN DEL VEHÍCULO
        theta(i+1) = theta(i) + 180/pi*Tv*r(i+1);
        x(i+1) = x(i) + Tv*u(i+1)*cosd(theta(i+1));
        y(i+1) = y(i) + Tv*u(i+1)*sind(theta(i+1));
        dd(i+1,1)=sqrt((xd-x(i+1))^2+(yd-y(i+1))^2);
        i=i+1;
end
%GRÁFICA DE LA TRAYECTORIA EN EL PLANO (x,y)
nn=0.05;
axis([0 2 0 2])
clf
for i= 1:length(x)
    figure(1),plot([x(i)  x(i)+nn*cosd(theta(i))],[y(i) y(i)+nn*sind(theta(i))],'LineWidth',2), hold on
    %pause(0.2)
end
plot(xd,yd,'rx','MarkerSize',20,'lineWidth',5)
axis equal,grid on
figure,plot(0:0.1:0.1*(length(u)-1),u,'lineWidth',2),grid on,title("VELOCIDAD DE AVANCE u(t)")
figure,plot(0:0.1:0.1*(length(u)-1),r,'r','lineWidth',2),grid on,title("VELOCIDAD ANGULAR r(t)")
figure,plot(0:0.1:0.1*(length(e_theta)-1),e_theta,'g','lineWidth',2),grid on,title("ERROR ORIENTACIÓN e_{\theta}(t)")