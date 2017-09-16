close all;
clear;
clc;
 
%parameter global 
ft = 16.5; %waktu total
dt = 0.33; %waktu sampling
r = 2.75; %jari-jari roda
l = 6; %lebar robot
 
%parameter leader virtual
xv = 30; %posisi awal x 
yv = 40; %posisi awal y 
vv = 3; %kecepatan awal
av = 0; %percepatan
thv = 0; %posisi sudut awal
wv = 0; %kecepatan sudut awal
 
%parameter agen
%agen1
xa1 = 20; %posisi awal x 
ya1 = 30; %posisi awal y 
tha1 = 0; %posisi sudut awal
%agen2
xa2 = 20; %posisi awal x 
ya2 = 60; %posisi awal y 
tha2 = 270; %posisi sudut awal
%agen3
xa3 = 5; %posisi awal x 
ya3 = 5; %posisi awal y 
tha3 = 270; %posisi sudut awal
 
%parameter flocking
%agen1
KPx1 = 0.21; %konstanta proporsional x
KPy1 = 0.21; %konstanta proporsional y
Tix1 = 105; %konstanta integral x
Tiy1 = 105; %konstanta integral y
Tdx1 = 0; %konstanta derivative x
Tdy1 = 0; %konstanta derivative y
%agen2
KPx2 = 0.21; %konstanta proporsional x
KPy2 = 0.21; %konstanta proporsional y
Tix2 = 105; %konstanta integral x
Tiy2 = 105; %konstanta integral y
Tdx2 = 0; %konstanta derivative x
Tdy2 = 0; %konstanta derivative y
%agen3
KPx3 = 0.21; %konstanta proporsional x
KPy3 = 0.21; %konstanta proporsional y
Tix3 = 105; %konstanta integral x
Tiy3 = 105; %konstanta integral y
Tdx3 = 0; %konstanta derivative x
Tdy3 = 0; %konstanta derivative y
 
%konstanta refference
jrk1 = 8;
jrk2 = 8;
jrk3 = 8;
 
%-----------------------------------------------------------------
%-----------------------------------------------------------------
% PROGRAM FLOCKING
%-----------------------------------------------------------------
%-----------------------------------------------------------------
gam1 = tha1*pi/180;
gam2 = tha2*pi/180;
gam3 = tha3*pi/180;
t = 0;
pin = (-pi);
 
for i = 1: ((ft/dt)+1)
    %-------------------------------------------------------------
    % INISIASI
    %-------------------------------------------------------------
    %definisi awal
    psi1 = 0; %sudut flocking
    psi2 = 45; %sudut flocking
    psi3 = 135; %sudut flocking
    b1 = 0; %jarak flocking
    b2 = 25; %jarak flocking
    b3 = 25; %jarak flocking
    
    %leader virtual
    vv(i+1) = vv(i) + av*dt; %kecepatan
    thv(i+1) = thv(i) + wv*dt; %sudut
    dxv(i) = vv(i)*cos(thv(i)*pi/180); %kecepatan sumbu x 
    dyv(i) = vv(i)*sin(thv(i)*pi/180); %kecepatan sumbu y 
    xv(i+1) = xv(i)+(dxv(i)*dt); %posisi x 
    yv(i+1) = yv(i)+(dyv(i)*dt); %posisi y
      
    %trajectory
    %agen1
    bx1(i) = b1*cos((thv(i)+psi1-90)*pi/180);
    by1(i) = b1*sin((thv(i)+psi1-90)*pi/180);
    xr1(i) = xv(i) - bx1(i);
    yr1(i) = yv(i) - by1(i);
    if i == 1
        dxr1(i) = 0;
        dyr1(i) = 0;
    else
        dxr1(i) = (xr1(i)-xr1(i-1))/dt;
        dyr1(i) = (yr1(i)-yr1(i-1))/dt;
    end
    %agen2
    bx2(i) = b2*cos((thv(i)+psi2-90)*pi/180);
    by2(i) = b2*sin((thv(i)+psi2-90)*pi/180);
    xr2(i) = xv(i) - bx2(i);
    yr2(i) = yv(i) - by2(i);
    if i == 1
        dxr2(i) = 0;
        dyr2(i) = 0;
    else
        dxr2(i) = (xr2(i)-xr2(i-1))/dt;
        dyr2(i) = (yr2(i)-yr2(i-1))/dt;
    end
    %agen3
    bx3(i) = b3*cos((thv(i)+psi3-90)*pi/180);
    by3(i) = b3*sin((thv(i)+psi3-90)*pi/180);
    xr3(i) = xv(i) - bx3(i);
    yr3(i) = yv(i) - by3(i);
    if i == 1
        dxr3(i) = 0;
        dyr3(i) = 0;
    else
        dxr3(i) = (xr3(i)-xr3(i-1))/dt;
        dyr3(i) = (yr3(i)-yr3(i-1))/dt;
    end
    
    %-------------------------------------------------------------
    % FLOCKING
    %-------------------------------------------------------------
    %refference
    %agen1
    xf1(i)= xa1(i)+(jrk1*cos(gam1(i)));
    yf1(i)= ya1(i)+(jrk1*sin(gam1(i)));
    %agen2
    xf2(i)= xa2(i)+(jrk2*cos(gam2(i)));
    yf2(i)= ya2(i)+(jrk2*sin(gam2(i)));
    %agen3
    xf3(i)= xa3(i)+(jrk3*cos(gam3(i)));
    yf3(i)= ya3(i)+(jrk3*sin(gam3(i)));

    % Pemilihan leader
    ea(i) = sqrt(((xr1(i)-xf1(i))^2) + ((yr1(i)-yf1(i))^2));
    eb(i) = sqrt(((xr2(i)-xf1(i))^2) + ((yr2(i)-yf1(i))^2));
    ec(i) = sqrt(((xr3(i)-xf1(i))^2) + ((yr3(i)-yf1(i))^2));
    ed(i) = sqrt(((xr1(i)-xf2(i))^2) + ((yr1(i)-yf2(i))^2));
    ee(i) = sqrt(((xr2(i)-xf2(i))^2) + ((yr2(i)-yf2(i))^2));
    ef(i) = sqrt(((xr3(i)-xf2(i))^2) + ((yr3(i)-yf2(i))^2));
    eg(i) = sqrt(((xr1(i)-xf3(i))^2) + ((yr1(i)-yf3(i))^2));
    eh(i) = sqrt(((xr2(i)-xf3(i))^2) + ((yr2(i)-yf3(i))^2));
    ei(i) = sqrt(((xr3(i)-xf3(i))^2) + ((yr3(i)-yf3(i))^2));
    
    % Error
    if ((ea(i) <= ed(i)) && (ea(i) <= eg(i)))
        % Agen1 sebagai leader
        ex1(i) = xr1(i)-xf1(i);
        exL1(i+1) = ex1(i);
        ey1(i) = yr1(i)-yf1(i);
        eyL1(i+1) = ey1(i);
        if (ee(i) <= eh(i))
            % Agen2 sebagai follower 1
            ex2(i) = xr2(i)-xf2(i);
            exL2(i+1) = ex2(i);
            ey2(i) = yr2(i)-yf2(i);
            eyL2(i+1) = ey2(i);
            % Agen3 sebagai follower 2
            ex3(i) = xr3(i)-xf3(i);
            exL3(i+1) = ex3(i);
            ey3(i) = yr3(i)-yf3(i);
            eyL3(i+1) = ey3(i);
        else
            % Agen3 sebagai follower 1
            ex3(i) = xr2(i)-xf3(i);
            exL3(i+1) = ex3(i);
            ey3(i) = yr2(i)-yf3(i);
            eyL3(i+1) = ey3(i);
            % Agen2 sebagai follower 2
            ex2(i) = xr3(i)-xf2(i);
            exL2(i+1) = ex2(i);
            ey2(i) = yr3(i)-yf2(i);
            eyL2(i+1) = ey2(i);
        end
    elseif ((ed(i) < ea(i)) && (ed(i) <= eg(i)))
        % Agen2 sebagai leader
        ex2(i) = xr1(i)-xf2(i);
        exL2(i+1) = ex2(i);
        ey2(i) = yr1(i)-yf2(i);
        eyL2(i+1) = ey2(i);
        if (eb(i) <= eh(i))
            % Agen1 sebagai follower 1
            ex1(i) = xr2(i)-xf1(i);
            exL1(i+1) = ex1(i);
            ey1(i) = yr2(i)-yf1(i);
            eyL1(i+1) = ey1(i);
            % Agen3 sebagai follower 2
            ex3(i) = xr3(i)-xf3(i);
            exL3(i+1) = ex3(i);
            ey3(i) = yr3(i)-yf3(i);
            eyL3(i+1) = ey3(i);
        else
            % Agen3 sebagai follower 1
            ex3(i) = xr2(i)-xf3(i);
            exL3(i+1) = ex3(i);
            ey3(i) = yr2(i)-yf3(i);
            eyL3(i+1) = ey3(i);
            % Agen1 sebagai follower 2
            ex1(i) = xr3(i)-xf1(i);
            exL1(i+1) = ex1(i);
            ey1(i) = yr3(i)-yf1(i);
            eyL1(i+1) = ey1(i);
        end
    else 
        % Agen3 sebagai leader
        ex3(i) = xr1(i)-xf3(i);
        exL3(i) = ex3(i);
        ey3(i) = yr1(i)-yf3(i);
        eyL3(i) = ey3(i);
        if (eb(i) <= ee(i))
            % Agen1 sebagai follower 1
            ex1(i) = xr2(i)-xf1(i);
            exL1(i) = ex1(i);
            ey1(i) = yr2(i)-yf1(i);
            eyL1(i) = ey1(i);
            % Agen2 sebagai follower 2
            ex2(i) = xr3(i)-xf2(i);
            exL2(i) = ex2(i);
            ey2(i) = yr3(i)-yf2(i);
            eyL2(i) = ey2(i);
        else
            % Agen2 sebagai follower 1
            ex2(i) = xr2(i)-xf2(i);
            exL2(i) = ex2(i);
            ey2(i) = yr2(i)-yf2(i);
            eyL2(i) = ey2(i);
            % Agen1 sebagai follower 2
            ex1(i) = xr3(i)-xf1(i);
            exL1(i) = ex1(i);
            ey1(i) = yr3(i)-yf1(i);
            eyL1(i) = ey1(i);
        end
    end
    
    % Penggantian Error
    if (i <= 1)
        exL1(i) = ex1(i);
        eyL1(i) = ey1(i);
        exL2(i) = ex2(i);
        eyL2(i) = ey2(i);
        exL3(i) = ex3(i);
        eyL3(i) = ey3(i);
    else
        exL1(i+1) = ex1(i);
        eyL1(i+1) = ey1(i);
        exL2(i+1) = ex2(i);
        eyL2(i+1) = ey2(i);
        exL3(i+1) = ex3(i);
        eyL3(i+1) = ey3(i);
    end

    %proporsional
    %agen1
    Px1(i) = KPx1*ex1(i);
    Py1(i) = KPy1*ey1(i);
    %agen2
    Px2(i) = KPx2*ex2(i);
    Py2(i) = KPy2*ey2(i);
    %agen3
    Px3(i) = KPx3*ex3(i);
    Py3(i) = KPy3*ey3(i);
    
    %integral
    %agen1
    if i == 1
        Iex1(i) = (exL1(i) + ex1(i))*dt/2;
        Iey1(i) = (eyL1(i) + ey1(i))*dt/2;
    else
        Iex1(i) = Ix1(i-1) + ((exL1(i) + ex1(i))*dt/2);
        Iey1(i) = Iy1(i-1) + ((eyL1(i) + ey1(i))*dt/2);
    end
    Ix1(i) = KPx1*Iex1(i)/Tix1;
    Iy1(i) = KPy1*Iey1(i)/Tiy1;
    %agen2
    if i == 1
        Iex2(i) = (exL2(i) + ex2(i))*dt/2;
        Iey2(i) = (eyL2(i) + ey2(i))*dt/2;
    else
        Iex2(i) = Ix2(i-1) + ((exL2(i) + ex2(i))*dt/2);
        Iey2(i) = Iy2(i-1) + ((eyL2(i) + ey2(i))*dt/2);
    end
    Ix2(i) = KPx2*Iex2(i)/Tix2;
    Iy2(i) = KPy2*Iey2(i)/Tiy2;
    %agen3
    if i == 1
        Iex3(i) = (exL3(i) + ex3(i))*dt/2;
        Iey3(i) = (eyL3(i) + ey3(i))*dt/2;
    else
        Iex3(i) = Ix3(i-1) + ((exL3(i) + ex3(i))*dt/2);
        Iey3(i) = Iy3(i-1) + ((eyL3(i) + ey3(i))*dt/2);
    end
    Ix3(i) = KPx3*Iex3(i)/Tix3;
    Iy3(i) = KPy3*Iey3(i)/Tiy3;
    
    %derivative
    %agen1
    Dex1(i) = (ex1(i)-exL1(i))/dt;
    Dey1(i) = (ey1(i)-eyL1(i))/dt;
    Dx1(i) = KPx1*Dex1(i)*Tdx1;
    Dy1(i) = KPy1*Dey1(i)*Tdy1;
    %agen2
    Dex2(i) = (ex2(i)-exL2(i))/dt;
    Dey2(i) = (ey2(i)-eyL2(i))/dt;
    Dx2(i) = KPx2*Dex2(i)*Tdx2;
    Dy2(i) = KPy2*Dey2(i)*Tdy2;
    %agen3
    Dex3(i) = (ex3(i)-exL3(i))/dt;
    Dey3(i) = (ey3(i)-eyL3(i))/dt;
    Dx3(i) = KPx3*Dex3(i)*Tdx3;
    Dy3(i) = KPy3*Dey3(i)*Tdy3;
    
    %kecepatan X dan Y UGV setelah dikontrol
    %agen1
    dxa1(i)= dxr1(i)+Px1(i)+Ix1(i)+Dx1(i);
    dya1(i)= dyr1(i)+Py1(i)+Iy1(i)+Dy1(i);
    %agen2
    dxa2(i)= dxr2(i)+Px2(i)+Ix2(i)+Dx2(i);
    dya2(i)= dyr2(i)+Py2(i)+Iy2(i)+Dy2(i);
    %agen3
    dxa3(i)= dxr3(i)+Px3(i)+Ix3(i)+Dx3(i);
    dya3(i)= dyr3(i)+Py3(i)+Iy3(i)+Dy3(i);
    
    %kecepatan translasi dan rotasi
    %agen1
    kec1(i) = (dxa1(i)*cos(gam1(i)))+(dya1(i)*sin(gam1(i)));
    omega1(i) = ((-dxa1(i)*sin(gam1(i)))/jrk1)+((dya1(i)*cos(gam1(i))/jrk1));
    %agen2
    kec2(i) = (dxa2(i)*cos(gam2(i)))+(dya2(i)*sin(gam2(i)));
    omega2(i) = ((-dxa2(i)*sin(gam2(i)))/jrk2)+((dya2(i)*cos(gam2(i))/jrk2));
    %agen3
    kec3(i) = (dxa3(i)*cos(gam3(i)))+(dya3(i)*sin(gam3(i)));
    omega3(i) = ((-dxa3(i)*sin(gam3(i)))/jrk3)+((dya3(i)*cos(gam3(i))/jrk3));
 
    %output
    %agen1
    xa1(i+1) = xa1(i) + kec1(i)*cos(gam1(i))*dt;
    ya1(i+1) = ya1(i) + kec1(i)*sin(gam1(i))*dt;
    gam1(i+1) = gam1(i) + omega1(i)*dt;
    %agen2
    xa2(i+1) = xa2(i) + kec2(i)*cos(gam2(i))*dt;
    ya2(i+1) = ya2(i) + kec2(i)*sin(gam2(i))*dt;
    gam2(i+1) = gam2(i) + omega2(i)*dt;
    %agen3
    xa3(i+1) = xa3(i) + kec3(i)*cos(gam3(i))*dt;
    ya3(i+1) = ya3(i) + kec3(i)*sin(gam3(i))*dt;
    gam3(i+1) = gam3(i) + omega3(i)*dt;
    
    % Definisi waktu
    t(i+1) = t(i) + dt;
    t2(i) = i*dt;
    
    %manipulated variable
    % Agen1
    wa1(i) = omega1(i)*180/pi;
    if omega1(i)==0
        va1(i)=kec1(i);
    else
        va1(i) = (sqrt(((kec1(i)*dt)^2)/(2*(1-cos((abs(omega1(i))*dt)))))*2*pi*(abs(omega1(i))*dt/(2*pi)))/dt;
    end
    % Agen2
    wa2(i) = omega2(i)*180/pi;
    if omega1(i)==0
        va2(i)=kec2(i);
    else
        va2(i) = (sqrt(((kec2(i)*dt)^2)/(2*(1-cos((abs(omega2(i))*dt)))))*2*pi*(abs(omega2(i))*dt/(2*pi)))/dt;
    end
    % Agen3
    wa3(i) = omega3(i)*180/pi;
    if omega1(i)==0
        va3(i)=kec3(i);
    else
        va3(i) = (sqrt(((kec3(i)*dt)^2)/(2*(1-cos((abs(omega3(i))*dt)))))*2*pi*(abs(omega3(i))*dt/(2*pi)))/dt;
    end
    
    % RPM
    RPMr1(i) = (30/(pi*r))*(va1(i)+(wa1(i)*pi*l/180));
    RPMl1(i) = (30/(pi*r))*(va1(i)-(wa1(i)*pi*l/180));
    RPMr2(i) = (30/(pi*r))*(va2(i)+(wa2(i)*pi*l/180));
    RPMl2(i) = (30/(pi*r))*(va2(i)-(wa2(i)*pi*l/180));
    RPMr3(i) = (30/(pi*r))*(va3(i)+(wa3(i)*pi*l/180));
    RPMl3(i) = (30/(pi*r))*(va3(i)-(wa3(i)*pi*l/180));
    
    
    % Power motor
    PWR1A(i) = ((0.64*(RPMr1(i))));
    PWR1B(i) = ((0.64*(RPMl1(i))));
    PWR2A(i) = ((0.64*(RPMr2(i))));
    PWR2B(i) = ((0.64*(RPMl2(i))));
    PWR3A(i) = ((0.64*(RPMr3(i))));
    PWR3B(i) = ((0.64*(RPMl3(i))));

%-----------------------------------------------------------------
%-----------------------------------------------------------------
% PROGRAM PLOT SIMULASI
%-----------------------------------------------------------------
%-----------------------------------------------------------------
%-----------------------------------------------------------------
% INISIASI
%-----------------------------------------------------------------xa1 = xa1'; ya1 = ya1'; gam1 = gam1'; t = t';
xa2 = xa2'; ya2 = ya2'; gam2 = gam2'; t = t';
xa3 = xa3'; ya3 = ya3'; gam3 = gam3'; t = t';
 
figure(1);
plot(xv,yv,'k-',xr1,yr1,'r-',xr2,yr2,'g-',xr3,yr3,'b-',xa1,ya1,'r.-',xa2,ya2,'g.-',xa3,ya3,'b.-'); axis equal; grid; axis([0 160 0 120]);
legend ('leader virtual','trajectory 1','trajectory 2','trajectory 3','UGV 1','UGV 2','UGV 3');
hold on

%-----------------------------------------------------------------
% PLOT ROBOT
%-----------------------------------------------------------------%agen1
Xc1 = xa1(1);
Yc1 = ya1(1);
%agen2
Xc2 = xa2(1);
Yc2 = ya2(1);
%agen3
Xc3 = xa3(1);
Yc3 = ya3(1);
 
hold on
axis equal
axis([0 160 0 120])
grid on
Xr_c  = 5;
Yr_c  = 0;
Xr_c1 = 5/1.4142;
Yr_c1 = 0;
gama  = pi/4;
 
path = size(t);
Na   = path(1,1);    
    
for j = 1:((ft/dt)+1)
    R1   = 5/1.414;
    R2   = 5/1.414;
    robot_width = l;
    %agen1
    Xc1 = xa1(j);
    Yc1 = ya1(j);
    fai1 = gam1(j);
    X1r   = Xc1+Xr_c*cos(fai1)-Yr_c*sin(fai1);
    Y1r   = Yc1+Xr_c*sin(fai1)+Yr_c*cos(fai1);
    X1r1  = Xc1+Xr_c1*cos(fai1+gama)-Yr_c1*sin(fai1+gama);
    Y1r1  = Yc1+Xr_c1*sin(fai1+gama)+Yr_c1*cos(fai1+gama);
    sinfai1(j) = sin(fai1);
    cosfai1(j) = cos(fai1);
    faire1(j) = fai1;
    %agen2
    Xc2 = xa2(j);
    Yc2 = ya2(j);
    fai2 = gam2(j);
    X2r   = Xc2+Xr_c*cos(fai2)-Yr_c*sin(fai2);
    Y2r   = Yc2+Xr_c*sin(fai2)+Yr_c*cos(fai2);
    X2r1  = Xc2+Xr_c1*cos(fai2+gama)-Yr_c1*sin(fai2+gama);
    Y2r1  = Yc2+Xr_c1*sin(fai2+gama)+Yr_c1*cos(fai2+gama);
    sinfai2(j) = sin(fai2);
    cosfai2(j) = cos(fai2);
    faire2(j) = fai2;
    %agen3
    Xc3 = xa3(j);
    Yc3 = ya3(j);
    fai3 = gam3(j);
    X3r   = Xc3+Xr_c*cos(fai3)-Yr_c*sin(fai3);
    Y3r   = Yc3+Xr_c*sin(fai3)+Yr_c*cos(fai3);
    X3r1  = Xc3+Xr_c1*cos(fai3+gama)-Yr_c1*sin(fai3+gama);
    Y3r1  = Yc3+Xr_c1*sin(fai3+gama)+Yr_c1*cos(fai3+gama);
    sinfai3(j) = sin(fai3);
    cosfai3(j) = cos(fai3);
    faire3(j) = fai3;
 
    [Robot1] = Robotplot(Xc1, Yc1, fai1, robot_width);
    [Robot2] = Robotplot(Xc2, Yc2, fai2, robot_width);
    [Robot3] = Robotplot(Xc3, Yc3, fai3, robot_width);
 
    %-------------------------------------------------------------
    % MEMBUAT ROBOT
    %-------------------------------------------------------------
    %agen1
    Robot1x1 = [Robot1(1,1) Robot1(1,3)];
    Robot1y1 = [Robot1(1,2) Robot1(1,4)];
    Robot2x1 = [Robot1(1,3) Robot1(1,5)];
    Robot2y1 = [Robot1(1,4) Robot1(1,6)];
    Robot3x1 = [Robot1(1,5) Robot1(1,7)];
    Robot3y1 = [Robot1(1,6) Robot1(1,8)];
    Robot4x1 = [Robot1(1,7) Robot1(1,1)];
    Robot4y1 = [Robot1(1,8) Robot1(1,2)];
    %agen2
    Robot1x2 = [Robot2(1,1) Robot2(1,3)];
    Robot1y2 = [Robot2(1,2) Robot2(1,4)];
    Robot2x2 = [Robot2(1,3) Robot2(1,5)];
    Robot2y2 = [Robot2(1,4) Robot2(1,6)];
    Robot3x2 = [Robot2(1,5) Robot2(1,7)];
    Robot3y2 = [Robot2(1,6) Robot2(1,8)];
    Robot4x2 = [Robot2(1,7) Robot2(1,1)];
    Robot4y2 = [Robot2(1,8) Robot2(1,2)];
    %agen3
    Robot1x3 = [Robot3(1,1) Robot3(1,3)];
    Robot1y3 = [Robot3(1,2) Robot3(1,4)];
    Robot2x3 = [Robot3(1,3) Robot3(1,5)];
    Robot2y3 = [Robot3(1,4) Robot3(1,6)];
    Robot3x3 = [Robot3(1,5) Robot3(1,7)];
    Robot3y3 = [Robot3(1,6) Robot3(1,8)];
    Robot4x3 = [Robot3(1,7) Robot3(1,1)];
    Robot4y3 = [Robot3(1,8) Robot3(1,2)];
 
    %agen1
    Robotlink1x  = [X1r Xc1];
    Robotlink1y  = [Y1r Yc1];
    Robotlink1x1 = [X1r1 Xc1];
    Robotlink1y1 = [Y1r1 Yc1];
    Robotlink1x2 = [X1r1 X1r];
    Robotlink1y2 = [Y1r1 Y1r];
    %agen2
    Robotlink2x  = [X2r Xc2];
    Robotlink2y  = [Y2r Yc2];
    Robotlink2x1 = [X2r1 Xc2];
    Robotlink2y1 = [Y2r1 Yc2];
    Robotlink2x2 = [X2r1 X2r];
    Robotlink2y2 = [Y2r1 Y2r];
    %agen3
    Robotlink3x  = [X3r Xc3];
    Robotlink3y  = [Y3r Yc3];
    Robotlink3x1 = [X3r1 Xc3];
    Robotlink3y1 = [Y3r1 Yc3];
    Robotlink3x2 = [X3r1 X3r];
    Robotlink3y2 = [Y3r1 Y3r];
 
    %agen1
    RobotBox1(1,:) = [Robot1x1 Robot2x1 Robot3x1 Robot4x1];
    RobotBox1(2,:) = [Robot1y1 Robot2y1 Robot3y1 Robot4y1];
    %agen2
    RobotBox2(1,:) = [Robot1x2 Robot2x2 Robot3x2 Robot4x2];
    RobotBox2(2,:) = [Robot1y2 Robot2y2 Robot3y2 Robot4y2];
    %agen3
    RobotBox3(1,:) = [Robot1x3 Robot2x3 Robot3x3 Robot4x3];
    RobotBox3(2,:) = [Robot1y3 Robot2y3 Robot3y3 Robot4y3];
 
%     plot (Xc1,Yc1,'r-o',Xc2,Yc2,'g-o',Xc3,Yc3,'b-o','LineWidth',1);
 
    %agen1
    X1rY1r         = [X1r Y1r];
    Xc1Yc1         = [Xc1 Yc1];
    X1r1Y1r1       = [X1r1 Y1r1];
    X1rY1rtoXc1Yc1   = distance(X1rY1r,Xc1Yc1);
    X1r1Y1r1toXc1Yc1 = distance(X1r1Y1r1,Xc1Yc1);
    X1rY1rtoX1r1Y1r1 = distance(X1rY1r,X1r1Y1r1);
    %agen2
    X2rY2r         = [X2r Y2r];
    Xc2Yc2         = [Xc2 Yc2];
    X2r1Y2r1       = [X2r1 Y2r1];
    X2rY2rtoXc2Yc2   = distance(X2rY2r,Xc2Yc2);
    X2r1Y2r1toXc2Yc2 = distance(X2r1Y2r1,Xc2Yc2);
    X2rY2rtoX2r1Y2r1 = distance(X2rY2r,X2r1Y2r1);
    %agen3
    X3rY3r         = [X3r Y3r];
    Xc3Yc3         = [Xc3 Yc3];
    X3r1Y3r1       = [X3r1 Y3r1];
    X3rY3rtoXc3Yc3   = distance(X3rY3r,Xc3Yc3);
    X3r1Y3r1toXc3Yc3 = distance(X3r1Y3r1,Xc3Yc3);
    X3rY3rtoX3r1Y3r1 = distance(X3rY3r,X3r1Y3r1);
 
    %agen1
    MMcos1 = (-(X1rY1rtoXc1Yc1*X1rY1rtoXc1Yc1-X1r1Y1r1toXc1Yc1*X1r1Y1r1toXc1Yc1-X1rY1rtoX1r1Y1r1*X1rY1rtoX1r1Y1r1)/(2*X1rY1rtoX1r1Y1r1*X1r1Y1r1toXc1Yc1));
    MM1(j) = sqrt(1 - MMcos1^2);
    %agen2
    MMcos2 = (-(X2rY2rtoXc2Yc2*X2rY2rtoXc2Yc2-X2r1Y2r1toXc2Yc2*X2r1Y2r1toXc2Yc2-X2rY2rtoX2r1Y2r1*X2rY2rtoX2r1Y2r1)/(2*X2rY2rtoX2r1Y2r1*X2r1Y2r1toXc2Yc2));
    MM2(j) = sqrt(1 - MMcos2^2);
    %agen3
    MMcos3 = (-(X3rY3rtoXc3Yc3*X3rY3rtoXc3Yc3-X3r1Y3r1toXc3Yc3*X3r1Y3r1toXc3Yc3-X3rY3rtoX3r1Y3r1*X3rY3rtoX3r1Y3r1)/(2*X3rY3rtoX3r1Y3r1*X3r1Y3r1toXc3Yc3));
    MM3(j) = sqrt(1 - MMcos3^2);
 
    if j==1,
        h11 =plot(Robot1x1, Robot1y1,'b-','LineWidth',4);
        h12 =plot(Robot2x1, Robot2y1,'r-','LineWidth',1);
        h13 =plot(Robot3x1, Robot3y1,'b-','LineWidth',4);
        h14 =plot(Robot4x1, Robot4y1,'r-','LineWidth',1);
        set(h11,'EraseMode','xor');
        set(h12,'EraseMode','xor');
        set(h13,'EraseMode','xor');
        set(h14,'EraseMode','xor');
    else
        set(h11,'XData', Robot1x1,'YData',Robot1y1)
        set(h12,'XData', Robot2x1,'YData',Robot2y1)
        set(h13,'XData', Robot3x1,'YData',Robot3y1)
        set(h14,'XData', Robot4x1,'YData',Robot4y1)
    end
    
    if j==1,
        h21 =plot(Robot1x2, Robot1y2,'b-','LineWidth',4);
        h22 =plot(Robot2x2, Robot2y2,'r-','LineWidth',1);
        h23 =plot(Robot3x2, Robot3y2,'b-','LineWidth',4);
        h24 =plot(Robot4x2, Robot4y2,'r-','LineWidth',1);
        set(h21,'EraseMode','xor');
        set(h22,'EraseMode','xor');
        set(h23,'EraseMode','xor');
        set(h24,'EraseMode','xor');
    else
        set(h21,'XData', Robot1x2,'YData',Robot1y2)
        set(h22,'XData', Robot2x2,'YData',Robot2y2)
        set(h23,'XData', Robot3x2,'YData',Robot3y2)
        set(h24,'XData', Robot4x2,'YData',Robot4y2)
    end
    
    if j==1,
        h31 =plot(Robot1x3, Robot1y3,'b-','LineWidth',4);
        h32 =plot(Robot2x3, Robot2y3,'r-','LineWidth',1);
        h33 =plot(Robot3x3, Robot3y3,'b-','LineWidth',4);
        h34 =plot(Robot4x3, Robot4y3,'r-','LineWidth',1);
        set(h31,'EraseMode','xor');
        set(h32,'EraseMode','xor');
        set(h33,'EraseMode','xor');
        set(h34,'EraseMode','xor');
    else
        set(h31,'XData', Robot1x3,'YData',Robot1y3)
        set(h32,'XData', Robot2x3,'YData',Robot2y3)
        set(h33,'XData', Robot3x3,'YData',Robot3y3)
        set(h34,'XData', Robot4x3,'YData',Robot4y3)
    end
    
    pause(0.01)    
end

%-----------------------------------------------------------------
% PLOT RPM
%-----------------------------------------------------------------
figure(2)
plot(t2,PWR1A,t2,PWR1B,t2,PWR2A,t2,PWR2B,t2,PWR3A,t2,PWR3B); grid;
legend ('Pwr kanan UGV 1','Pwr kiri UGV 1','Pwr kanan UGV 2','Pwr kiri UGV 2','Pwr kanan UGV 3','Pwr kiri UGV 3')
