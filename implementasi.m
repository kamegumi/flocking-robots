%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Program Name : Gabungan                                                   %
% Author       : Agung Prasdianto & Hafidz Bahtiar                          %
% Version      : 1.1                                                        %
% Description  : Program flocking                                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
 
%% Inisialisasi                                                             
% Membersihkan Jendela
if verLessThan('RWTHMindstormsNXT', '4.01');
    error(strcat('This program requires the RWTH - Mindstorms NXT Toolbox ' ...
    ,'version 4.01 or greater. Go to http://www.mindstorms.rwth-aachen.de ' ...
    ,'and follow the installation instructions!'));
end%if
close all;
clear;
clc;
COM_CloseNXT all
 
%% Memasukan Parameter
% Parameter global 
ft = 16.5; %waktu total
dt = 0.33; %waktu sampling
r = 2.75; %jari-jari roda
l = 5.75; %lebar robot
n = 6.18; %pixel/cm
 
% Parameter leader virtual
xv = 30; %posisi awal x 
yv = 40; %posisi awal y 
vv = 3; %kecepatan awal
av = 0; %percepatan
thv = 0; %posisi sudut awal
wv = 0; %kecepatan sudut awal
 
% Parameter flocking
% Agen1
KPx1 = 0.21; %konstanta proporsional x
KPy1 = 0.21; %konstanta proporsional y
Tix1 = 105; %konstanta integral x
Tiy1 = 105; %konstanta integral y
Tdx1 = 0; %konstanta derivative x
Tdy1 = 0; %konstanta derivative y
% Agen2
KPx2 = 0.21; %konstanta proporsional x
KPy2 = 0.21; %konstanta proporsional y
Tix2 = 105; %konstanta integral x
Tiy2 = 105; %konstanta integral y
Tdx2 = 0; %konstanta derivative x
Tdy2 = 0; %konstanta derivative y
% Agen3
KPx3 = 0.21; %konstanta proporsional x
KPy3 = 0.21; %konstanta proporsional y
Tix3 = 105; %konstanta integral x
Tiy3 = 105; %konstanta integral y
Tdx3 = 0; %konstanta derivative x
Tdy3 = 0; %konstanta derivative y
 
% Konstanta refference
jrk1 = 8;
jrk2 = 8;
jrk3 = 8;
 
min = 20;
 
% Inisialisasi Warna 
redThresh = 0.08; % Threshold merah murni
greenThresh = 0.025; % Threshold hijau murni
blueThresh = 0.1; % Threshold biru murni

%% Pengaturan Awal
% Koneksi Bluetooth
h1 = COM_OpenNXT('bluetooth1.ini');
h2 = COM_OpenNXT('bluetooth2.ini');
h3 = COM_OpenNXT('bluetooth3.ini');
 
% Menyalakan video
vidDevice = imaq.VideoDevice('winvideo', 1, 'YUY2_640x480', ...
            'ROI', [1 1 640 480], ...
            'ReturnedColorSpace', 'rgb');
vidInfo = imaqhwinfo(vidDevice);
hblob = vision.BlobAnalysis('AreaOutputPort', false, ...
            'CentroidOutputPort', true, ... 
            'BoundingBoxOutputPort', true', ...
            'MinimumBlobArea', 600, ...
            'MaximumBlobArea', 3000, ...
            'MaximumCount', 10);
 
% Set kotak
hshapeinsBox = vision.ShapeInserter('BorderColorSource', 'Input port', ...
            'Fill', true, ...
            'FillColorSource', 'Input port', ...
            'Opacity', 0.4);
 
% Set text
htextinsRed = vision.TextInserter('Text', 'Red   : %2d', ...
            'Location',  [5 2], ...
            'Color', [1 0 0], ... // warna merah
            'Font', 'Courier New', ...
            'FontSize', 14);
htextinsGreen = vision.TextInserter('Text', 'Green : %2d', ...
            'Location',  [5 18], ...
            'Color', [0 1 0], ... // warna hijau
            'Font', 'Courier New', ...
            'FontSize', 14);
htextinsBlue = vision.TextInserter('Text', 'Blue  : %2d', ...
            'Location',  [5 34], ...
            'Color', [0 0 1], ... // warna biru
            'Font', 'Courier New', ...
            'FontSize', 14);
 
% Set centroid
htextinsCent = vision.TextInserter('Text', '+      X:%6d, Y:%4d', ...
            'LocationSource', 'Input port', ...
            'Color', [0 0 0], ... // warna hitam
            'Font', 'Courier New', ...
            'FontSize', 14);
        
% Output video
hVideoIn = vision.VideoPlayer('Name', 'Final Video', ...
            'Position', [100 100 vidInfo.MaxWidth+20 vidInfo.MaxHeight+30]);
 
%% Proses Pengolahan                                                              
for i = 1:((ft/dt)+1)
    rgbFrame = step(vidDevice); % Acquire single frame
    
    % Mengambil komponen RGB
    diffFrameRed = imsubtract(rgbFrame(:,:,1), rgb2gray(rgbFrame));
    diffFrameGreen = imsubtract(rgbFrame(:,:,2), rgb2gray(rgbFrame));
    diffFrameBlue = imsubtract(rgbFrame(:,:,3), rgb2gray(rgbFrame));
    
    % Filter noise dengan median filter
    diffFrameRed = medfilt2(diffFrameRed, [3 3]);
    diffFrameGreen = medfilt2(diffFrameGreen, [3 3]);
    diffFrameBlue = medfilt2(diffFrameBlue, [3 3]);
    
    % Mengkonversi image ke binary image
    binFrameRed = im2bw(diffFrameRed, redThresh);
    binFrameGreen = im2bw(diffFrameGreen, greenThresh);
    binFrameBlue = im2bw(diffFrameBlue, blueThresh);
 
    % Nilai centroid dan bounding box
    [centroidRed, bboxRed] = step(hblob, binFrameRed);
    [centroidGreen, bboxGreen] = step(hblob, binFrameGreen);
    [centroidBlue, bboxBlue] = step(hblob, binFrameBlue);
    
    % Mengubah nilai centroid menjadi integer
    centroidRedCam = uint16(centroidRed); 
    centroidGreenCam = uint16(centroidGreen);
    centroidBlueCam = uint16(centroidBlue);
    
    % Pengaturan tampilan bounding box
    rgbFrame(1:100,1:110,:) = 0;
    vidIn = step(hshapeinsBox, rgbFrame, bboxRed, single([1 0 0]));
    vidIn = step(hshapeinsBox, vidIn, bboxGreen, single([0 1 0]));
    vidIn = step(hshapeinsBox, vidIn, bboxBlue, single([0 0 1]));
    
    % Penulisan nilai centroid ke video
    for object = 1:1:length(bboxRed(:,1))
        centXRed = centroidRedCam(object,1); centYRed = centroidRedCam(object,2);
        vidIn = step(htextinsCent, vidIn, [centXRed centYRed], [centXRed-6 centYRed-9]); 
    end
    for object = 1:1:length(bboxGreen(:,1))
        centXGreen = centroidGreenCam(object,1); centYGreen = centroidGreenCam(object,2);
        vidIn = step(htextinsCent, vidIn, [centXGreen centYGreen], [centXGreen-6 centYGreen-9]); 
    end
    for object = 1:1:length(bboxBlue(:,1))
        centXBlue = centroidBlueCam(object,1); centYBlue = centroidBlueCam(object,2);
        vidIn = step(htextinsCent, vidIn, [centXBlue centYBlue], [centXBlue-6 centYBlue-9]); 
    end
    
    % Menghitung nilai blobs
    vidIn = step(htextinsRed, vidIn, uint8(length(bboxRed(:,1)))); 
    vidIn = step(htextinsGreen, vidIn, uint8(length(bboxGreen(:,1))));
    vidIn = step(htextinsBlue, vidIn, uint8(length(bboxBlue(:,1))));
    
    % Output video
    step(hVideoIn, vidIn);
    
    % Definisi awal
    psi1 = 0; %sudut flocking
    psi2 = 45; %sudut flocking
    psi3 = 135; %sudut flocking
    b1 = 0; %jarak flocking
    b2 = 25; %jarak flocking
    b3 = 25; %jarak flocking
    b4 = 25; %jarak flocking
    
    % Leader virtual
    vv(i+1) = vv(i) + av*dt; %kecepatan
    thv(i+1) = thv(i) + wv*dt; %sudut
    dxv(i) = vv(i)*cos(thv(i)*pi/180); %kecepatan sumbu x 
    dyv(i) = vv(i)*sin(thv(i)*pi/180); %kecepatan sumbu y 
    xv(i+1) = xv(i)+(dxv(i)*dt); %posisi x 
    yv(i+1) = yv(i)+(dyv(i)*dt); %posisi y
    
    % UGV
    % Konversi pixel to cm
    cmxb1(i) = centroidGreen(1,1)/n;
    cmyb1(i) = (480-centroidGreen(1,2))/n;
    cmxd1(i) = centroidGreen(2,1)/n;
    cmyd1(i) = (480-centroidGreen(2,2))/n;
    cmxb2(i) = centroidRed(1,1)/n;
    cmyb2(i) = (480-centroidRed(1,2))/n;
    cmxd2(i) = centroidRed(2,1)/n;
    cmyd2(i) = (480-centroidRed(2,2))/n;
    cmxb3(i) = centroidBlue(1,1)/n;
    cmyb3(i) = (480-centroidBlue(1,2))/n;
    cmxd3(i) = centroidBlue(2,1)/n;
    cmyd3(i) = (480-centroidBlue(2,2))/n;
    % Posisi UGV
    xa1(i) = cmxb1(i);
    ya1(i) = cmyb1(i);
    xa2(i) = cmxb2(i);
    ya2(i) = cmyb2(i);
    xa3(i) = cmxb3(i);
    ya3(i) = cmyb3(i);
    % Orientasi UGV
    rxa1(i) = cmxd1(i)-cmxb1(i);
    rya1(i) = cmyd1(i)-cmyb1(i);
    rxa2(i) = cmxd2(i)-cmxb2(i);
    rya2(i) = cmyd2(i)-cmyb2(i);
    rxa3(i) = cmxd3(i)-cmxb3(i);
    rya3(i) = cmyd3(i)-cmyb3(i);
    tha1(i) = (atan((rya1(i))/(rxa1(i))))*180/pi;
    tha2(i) = (atan((rya2(i))/(rxa2(i))))*180/pi;
    tha3(i) = (atan((rya3(i))/(rxa3(i))))*180/pi;
    
    if rxa1(i)>=0 && rya1(i)>=0
        tha1(i)=tha1(i);
    elseif rxa1(i)<0 && rya1(i)>=0
        tha1(i)=180+tha1(i);
    elseif rxa1(i)<0 && rya1(i)<0
        tha1(i)=180+tha1(i);
    else 
        tha1(i)=360+tha1(i);
    end
    if rxa2(i)>=0 && rya2(i)>=0
        tha2(i)=tha2(i);
    elseif rxa2(i)<0 && rya2(i)>=0
        tha2(i)=180+tha2(i);
    elseif rxa2(i)<0 && rya2(i)<0
        tha2(i)=180+tha2(i);
    else
        tha2(i)=360+tha2(i);
    end
    if rxa3(i)>=0 && rya3(i)>=0
        tha3(i)=tha3(i);
    elseif rxa3(i)<0 && rya3(i)>=0
        tha3(i)=180+tha3(i);
    elseif rxa3(i)<0 && rya3(i)<0
        tha3(i)=180+tha3(i);
    else
        tha3(i)=360+tha3(i);
    end
    gam1(i) = tha1(i)*pi/180;
    gam2(i) = tha2(i)*pi/180;
    gam3(i) = tha3(i)*pi/180;
         
    % Trajectory
    % Agen1
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
    % Agen2
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
    % Agen3
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
    
    % Flocking
    % Refference
    % Agen1
    xf1(i)= xa1(i)+(jrk1*cos(gam1(i)));
    yf1(i)= ya1(i)+(jrk1*sin(gam1(i)));
    % Agen2
    xf2(i)= xa2(i)+(jrk2*cos(gam2(i)));
    yf2(i)= ya2(i)+(jrk2*sin(gam2(i)));
    % Agen3
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
    
    % Pengontrol
    % Proporsional
    % Agen1
    Px1(i) = KPx1*ex1(i);
    Py1(i) = KPy1*ey1(i);
    % Agen2
    Px2(i) = KPx2*ex2(i);
    Py2(i) = KPy2*ey2(i);
    % Agen3
    Px3(i) = KPx3*ex3(i);
    Py3(i) = KPy3*ey3(i);
    
    % Integral
    % Agen1
    if i == 1
        Iex1(i) = (exL1(i) + ex1(i)*dt)/2;
        Iey1(i) = (eyL1(i) + ey1(i)*dt)/2;
    else
        Iex1(i) = Iex1(i-1) + ((exL1(i) + ex1(i)*dt)/2);
        Iey1(i) = Iey1(i-1) + ((eyL1(i) + ey1(i)*dt)/2);
    end
    Ix1(i) = KPx1*Iex1(i)/Tix1;
    Iy1(i) = KPy1*Iey1(i)/Tiy1;
    % Agen2
    if i == 1
        Iex2(i) = (exL2(i) + ex2(i)*dt)/2;
        Iey2(i) = (eyL2(i) + ey2(i)*dt)/2;
    else
        Iex2(i) = Iex2(i-1) + ((exL2(i) + ex2(i)*dt)/2);
        Iey2(i) = Iey2(i-1) + ((eyL2(i) + ey2(i)*dt)/2);
    end
    Ix2(i) = KPx2*Iex2(i)/Tix2;
    Iy2(i) = KPy2*Iey2(i)/Tiy2;
    % Agen3
    if i == 1
        Iex3(i) = (exL3(i) + ex3(i)*dt)/2;
        Iey3(i) = (eyL3(i) + ey3(i)*dt)/2;
    else
        Iex3(i) = Iex3(i-1) + ((exL3(i) + ex3(i)*dt)/2);
        Iey3(i) = Iey3(i-1) + ((eyL3(i) + ey3(i)*dt)/2);
    end
    Ix3(i) = KPx3*Iex3(i)/Tix3;
    Iy3(i) = KPy3*Iey3(i)/Tiy3;
    
    % Derivative
    % Agen1
    Dex1(i) = (ex1(i)-exL1(i))/dt;
    Dey1(i) = (ey1(i)-eyL1(i))/dt;
    Dx1(i) = KPx1*Dex1(i)*Tdx1;
    Dy1(i) = KPy1*Dey1(i)*Tdy1;
    % Agen2
    Dex2(i) = (ex2(i)-exL2(i))/dt;
    Dey2(i) = (ey2(i)-eyL2(i))/dt;
    Dx2(i) = KPx2*Dex2(i)*Tdx2;
    Dy2(i) = KPy2*Dey2(i)*Tdy2;
    % Agen3
    Dex3(i) = (ex3(i)-exL3(i))/dt;
    Dey3(i) = (ey3(i)-eyL3(i))/dt;
    Dx3(i) = KPx3*Dex3(i)*Tdx3;
    Dy3(i) = KPy3*Dey3(i)*Tdy3;
    
    % Kecepatan X dan Y UGV setelah dikontrol
    % Agen1
%     dxa1(i)= dxr1(i)+Px1(i);
%     dya1(i)= dyr1(i)+Py1(i);
%     dxa1(i)= dxr1(i)+Px1(i)+Dx1(i);
%     dya1(i)= dyr1(i)+Py1(i)+Dy1(i);
    dxa1(i)= dxr1(i)+Px1(i)+Ix1(i);
    dya1(i)= dyr1(i)+Py1(i)+Iy1(i);
%     dxa1(i)= dxr1(i)+Px1(i)+Ix1(i)+Dx1(i);
%     dya1(i)= dyr1(i)+Py1(i)+Iy1(i)+Dy1(i);
    % Agen2
%     dxa2(i)= dxr2(i)+Px2(i);
%     dya2(i)= dyr2(i)+Py2(i);
%     dxa2(i)= dxr2(i)+Px2(i)+Dx2(i);
%     dya2(i)= dyr2(i)+Py2(i)+Dy2(i);
    dxa2(i)= dxr2(i)+Px2(i)+Ix2(i);
    dya2(i)= dyr2(i)+Py2(i)+Iy2(i);
%     dxa2(i)= dxr2(i)+Px2(i)+Dx2(i)+Ix2(i);
%     dya2(i)= dyr2(i)+Py2(i)+Dy2(i)+Iy2(i);
    % Agen3
%     dxa3(i)= dxr3(i)+Px3(i);
%     dya3(i)= dyr3(i)+Py3(i);
%     dxa3(i)= dxr3(i)+Px3(i)+Dx3(i);
%     dya3(i)= dyr3(i)+Py3(i)+Dy3(i);
    dxa3(i)= dxr3(i)+Px3(i)+Ix3(i);
    dya3(i)= dyr3(i)+Py3(i)+Iy3(i);
%     dxa3(i)= dxr3(i)+Px3(i)+Ix3(i)+Dx3(i);
%     dya3(i)= dyr3(i)+Py3(i)+Iy3(i)+Dy3(i);
    
    % Kecepatan translasi dan rotasi
    % Agen1
    kec1(i) = (dxa1(i)*cos(gam1(i)))+(dya1(i)*sin(gam1(i)));
    omega1(i) = ((-dxa1(i)*sin(gam1(i)))/jrk1)+((dya1(i)*cos(gam1(i))/jrk1));
    % Agen2
    kec2(i) = (dxa2(i)*cos(gam2(i)))+(dya2(i)*sin(gam2(i)));
    omega2(i) = ((-dxa2(i)*sin(gam2(i)))/jrk2)+((dya2(i)*cos(gam2(i))/jrk2));
    % Agen3
    kec3(i) = (dxa3(i)*cos(gam3(i)))+(dya3(i)*sin(gam3(i)));
    omega3(i) = ((-dxa3(i)*sin(gam3(i)))/jrk3)+((dya3(i)*cos(gam3(i))/jrk3));
 
    % Manipulated variable
    % Agen1
    wa1(i) = omega1(i)*180/pi;
    if omega1(i)==0
        va1(i)=kec1(i);
    else
        va1(i) = (sqrt(((kec1(i)*dt)^2)/(2*(1-cos((abs(omega1(i))*dt)))))*2*pi*(abs(omega1(i))*dt/(2*pi)))/dt;
    end
    % Agen2
    wa2(i) = omega2(i)*180/pi;
    if omega2(i)==0
        va2(i)=kec2(i);
    else
        va2(i) = (sqrt(((kec2(i)*dt)^2)/(2*(1-cos((abs(omega2(i))*dt)))))*2*pi*(abs(omega2(i))*dt/(2*pi)))/dt;
    end
    % Agen3
    wa3(i) = omega3(i)*180/pi;
    if omega3(i)==0
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

%     % Power motor
    PWR1A(i) = round((0.64*(RPMr1(i))));
    PWR1B(i) = round((0.64*(RPMl1(i))));
    PWR2A(i) = round((0.64*(RPMr2(i))));
    PWR2B(i) = round((0.64*(RPMl2(i))));
    PWR3A(i) = round((0.64*(RPMr3(i))));
    PWR3B(i) = round((0.64*(RPMl3(i))));
%     
    %%mengukur parameter antar UGV
    %jarak
    jarakmin = 19;
    oa12(i) = sqrt(((xa1(i)-xa2(i))^2)+((ya1(i)-ya2(i))^2)); %hijau-merah
    oa13(i) = sqrt(((xa1(i)-xa3(i))^2)+((ya1(i)-ya3(i))^2)); %hijau-biru
    oa23(i) = sqrt(((xa2(i)-xa3(i))^2)+((ya2(i)-ya3(i))^2)); %merah-biru
    
    %OA baru
    %UGV 1
    if oa12(i) < jarakmin
        if (xa2(i) < xa1(i)) && (ya2(i) < ya1(i))
            PWR2A(i) = -3;
            PWR2B(i) = -5;
        elseif (xa2(i) < xa1(i)) && (ya2(i) > ya1(i))
            PWR2A(i) = -5;
            PWR2B(i) = -3;
        elseif (xa2(i) < xa1(i)) && (ya2(i) == ya1(i))
            PWR2A(i) = -4;
            PWR2B(i) = -4;
        elseif (xa2(i) >= xa1(i)) && (ya2(i) < ya1(i))
            PWR1A(i) = -5;
            PWR1B(i) = -3;
        elseif (xa2(i) >= xa1(i)) && (ya2(i) > ya1(i))
            PWR1A(i) = -3;
            PWR1B(i) = -5;
        elseif (xa2(i) > xa1(i)) && (ya2(i) == ya1(i))
            PWR1A(i) = -4;
            PWR1B(i) = -4;
        end
    end
  
%     UGV 2
    if oa23(i) < jarakmin
        if (xa3(i) < xa2(i)) && (ya3(i) < ya2(i))
            PWR3A(i) = -3;
            PWR3B(i) = -5;
        elseif (xa3(i) < xa2(i)) && (ya3(i) > ya2(i))
            PWR3A(i) = -5;
            PWR3B(i) = -3;
        elseif (xa3(i) < xa2(i)) && (ya3(i) == ya2(i))
            PWR3A(i) = -4;
            PWR3B(i) = -4;
        elseif (xa3(i) >= xa2(i)) && (ya3(i) < ya2(i))
            PWR2A(i) = -5;
            PWR2B(i) = -3;
        elseif (xa3(i) >= xa2(i)) && (ya3(i) > ya2(i))
            PWR2A(i) = -3;
            PWR2B(i) = -5;
        elseif (xa3(i) > xa2(i)) && (ya3(i) == ya2(i))
            PWR2A(i) = -4;
            PWR2B(i) = -4;
        end
    end
   
%     UGV 3
    if oa13(i) < jarakmin
        if (xa3(i) < xa1(i)) && (ya3(i) < ya1(i))
            PWR3A(i) = -3;
            PWR3B(i) = -5;
        elseif (xa3(i) < xa1(i)) && (ya3(i) > ya1(i))
            PWR3A(i) = -5;
            PWR3B(i) = -3;
        elseif (xa3(i) < xa1(i)) && (ya3(i) == ya1(i))
            PWR3A(i) = -4;
            PWR3B(i) = -4;
        elseif (xa3(i) >= xa1(i)) && (ya3(i) < ya1(i))
            PWR1A(i) = -5;
            PWR1B(i) = -3;
        elseif (xa3(i) >= xa1(i)) && (ya3(i) > ya1(i))
            PWR1A(i) = -3;
            PWR1B(i) = -5;
        elseif (xa3(i) > xa1(i)) && (ya3(i) == ya1(i))
            PWR1A(i) = -4;
            PWR1B(i) = -4;
        end
    end
 
 
    %Mengirim data ke UGV 1
    COM_SetDefaultNXT(h1);
    power1A = PWR1A(i);
    power1B = PWR1B(i);
    port1A  = MOTOR_A;
    port1B  = MOTOR_B;
    dist1  = 180;
    
    mUp1A    = NXTMotor(port1A, 'Power', -power1A, 'ActionAtTachoLimit', 'HoldBrake');
    mDown1A  = NXTMotor(port1A, 'Power', power1A, 'ActionAtTachoLimit', 'HoldBrake');
    mUp1B    = NXTMotor(port1B, 'Power', -power1B, 'ActionAtTachoLimit', 'HoldBrake');
    mDown1B  = NXTMotor(port1B, 'Power', power1B, 'ActionAtTachoLimit', 'HoldBrake');
        
    mUp1A.SendToNXT();
    mUp1B.SendToNXT();
    mUp1A.WaitFor();
    mUp1B.WaitFor();
 
    %Mengirim data ke UGV 2
    COM_SetDefaultNXT(h2);
    power2A = PWR2A(i);
    power2B = PWR2B(i);
    port2A  = MOTOR_A;
    port2B  = MOTOR_B;
    dist2  = 180;
    
    mUp2A    = NXTMotor(port2A, 'Power', -power2A, 'ActionAtTachoLimit', 'HoldBrake');
    mDown2A  = NXTMotor(port2A, 'Power', power2A, 'ActionAtTachoLimit', 'HoldBrake');
    mUp2B    = NXTMotor(port2B, 'Power', -power2B, 'ActionAtTachoLimit', 'HoldBrake');
    mDown2B  = NXTMotor(port2B, 'Power', power2B, 'ActionAtTachoLimit', 'HoldBrake');
    
    mUp2A.SendToNXT();
    mUp2B.SendToNXT();
    mUp2A.WaitFor();
    mUp2B.WaitFor();
    
    %Mengirim data UGV 3
    COM_SetDefaultNXT(h3);
    power3A = PWR3A(i);
    power3B = PWR3B(i);
    port3A  = MOTOR_A;
    port3B  = MOTOR_B;
    dist3  = 180;
    
    mUp3A    = NXTMotor(port3A, 'Power', -power3A, 'ActionAtTachoLimit', 'HoldBrake');
    mDown3A  = NXTMotor(port3A, 'Power', power3A, 'ActionAtTachoLimit', 'HoldBrake');
    mUp3B    = NXTMotor(port3B, 'Power', -power3B, 'ActionAtTachoLimit', 'HoldBrake');
    mDown3B  = NXTMotor(port3B, 'Power', power3B, 'ActionAtTachoLimit', 'HoldBrake');
    
    mUp3A.SendToNXT();
    mUp3B.SendToNXT();
    mUp3A.WaitFor();
    mUp3B.WaitFor();
end
 
%% Pembersihan
COM_SetDefaultNXT(h1);
mUp1A.Stop('off');
mUp1B.Stop('off');
COM_SetDefaultNXT(h2);
mUp2A.Stop('off');
mUp2B.Stop('off');
COM_SetDefaultNXT(h3);
mUp3A.Stop('off');
mUp3B.Stop('off');
COM_SetDefaultNXT(h1);
COM_CloseNXT(h1);
COM_SetDefaultNXT(h2);
COM_CloseNXT(h2);
COM_SetDefaultNXT(h3);
COM_CloseNXT(h3);
COM_CloseNXT('all')
release(hVideoIn);
release(vidDevice);
figure(1);
plot(xa1,ya1,xa2,ya2,xa3,ya3,xr1,yr1,xr2,yr2,xr3,yr3);axis equal;axis([0 100 0 75])
t=(0:0.33:16.5);
figure(2);
plot(t,PWR1A,t,PWR1B,t,PWR2A,t,PWR2B,t,PWR3A,t,PWR3B);
