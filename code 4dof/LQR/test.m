
x = [1 1.5 2 4.1 5];
y = [1 -1 1 -1 1];
plot(1,csapi(x,y,1),'k-',x,y,'ro')
title('Cubic Spline Interpolant to Five Points')


%%
AOA_rud = [-50,-45,-40,-35,-30,-25,-20,-15,-12.5,-10,-7.5,-5,-2.5,0,2.5,5,7.5,10,12.5,15,20,25,30,35,40,45,50];
Cl_rud = [-0.6315,-1.3158,-1.3222,-1.3055,-1.3523,-1.5600,-1.7260,-1.5092,-1.3084,-1.0953,-0.8384,-0.5664,-0.2855,0,0.2855,0.5664,0.8384,1.0953,1.3084,1.5092,1.7260,1.5600,1.3523,1.3055,1.3222,1.3158,0.6315];
Cd_rud = [0.41048,0.39972,0.34865,0.26262,0.21667,0.11866,0.04076,0.01607,0.01191,0.00961,0.00801,0.00693,0.00627,0.00609,0.00627,0.00693,0.00801,0.00961,0.01191,0.01607,0.04076,0.11866,0.21667,0.26262,0.34865,0.39972,0.41048];

pcl = polyfit(AOA_rud,Cl_rud,6);
disp('pcl');
disp(pcl(1,1));
disp(pcl(1,2));
disp(pcl(1,3));
disp(pcl(1,4));
disp(pcl(1,5));
disp(pcl(1,6));
disp(pcl(1,7));

pcd = polyfit(AOA_rud,Cd_rud,6);
disp('pcd');
disp(pcd(1,1));
disp(pcd(1,2));
disp(pcd(1,3));
disp(pcd(1,4));
disp(pcd(1,5));
disp(pcd(1,6));
disp(pcd(1,7));

%%


% Sail lift data
Cl_angle = [-180.0,-175.8,-170.1,-165.2,-160.0,-150.9,-140.3,-129.6,-120.2,-110.1,-99.7,-90.3,-79.9,-69.2,-59.7,-50.3,-40.2,-30.2,-25.7,-17.5,-15.0,-12.6,-10.5,-8.0,-6.8,-5.6,-0.2,0.2,5.6,6.8,8.0,10.5,12.6,15.0,17.5,25.7,30.2,40.2,50.3,59.7,69.2,79.9,90.3,99.7,110.1,120.2,129.6,140.3,150.9,160.0,165.2,170.1,175.8,180.0];
Cl_data = [0.464,1.023,1.235,1.046,1.122,1.410,1.471,1.290,0.867,0.565,0.279,-0.038,-0.332,-0.657,-1.034,-1.412,-1.509,-1.426,-1.259,-1.010,-0.987,-1.040,-1.100,-1.085,-1.032,-0.896,-0.185,0.185,0.896,1.032,1.085,1.100,1.040,0.987,1.010,1.259,1.426,1.509,1.412,1.034,0.657,0.332,0.038,-0.279,-0.565,-0.867,-1.290,-1.471,-1.410,-1.122,-1.046,-1.235,-1.023,-0.464];
% Sail drag data
Cd_angle = [-175.5,-170.1,-160.6,-165.4,-150.5,-140.1,-130.3,-120.1,-110.3,-100.4,-90.0,-70.2,-60.3,-50.2,-39.7,-30.0,-24.9,-20.1,-17.4,-15.0,-12.6,-10.2,-7.8,-5.6,-1.9,1.9,5.6,7.8,10.2,12.6,15.0,17.4,20.1,24.9,30.0,39.7,50.2,60.3,70.2,90.0,100.4,110.3,120.1,130.3,140.1,150.5,160.6,165.4,170.1,175.5];
% Cd_data = [-0.039,-0.179,-0.407,-0.300,-0.836,-1.311,-1.652,-1.613,-1.760,-1.881,-1.915,-1.721,-1.735,-1.602,-1.201,-0.800,-0.566,-0.439,-0.372,-0.326,-0.272,-0.172,-0.078,-0.032,-0.025,0.025,0.032,0.078,0.172,0.272,0.326,0.372,0.439,0.566,0.800,1.201,1.602,1.735,1.721,1.915,1.881,1.760,1.613,1.652,1.311,0.836,0.407,0.300,0.179,0.039];
Cd_data = [0.039,0.179,0.407,0.300,0.836,1.311,1.652,1.613,1.760,1.881,1.915,1.721,1.735,1.602,1.201,0.800,0.566,0.439,0.372,0.326,0.272,0.172,0.078,0.032,0.025,0.025,0.032,0.078,0.172,0.272,0.326,0.372,0.439,0.566,0.800,1.201,1.602,1.735,1.721,1.915,1.881,1.760,1.613,1.652,1.311,0.836,0.407,0.300,0.179,0.039];

pcl = polyfit(Cl_angle,Cl_data,7);
disp('pcl');
disp(pcl(1,1));
disp(pcl(1,2));
disp(pcl(1,3));
disp(pcl(1,4));
disp(pcl(1,5));
disp(pcl(1,6));
disp(pcl(1,7));
disp(pcl(1,8));

pcd = polyfit(Cd_angle,Cd_data,6);
disp('pcd');
disp(pcd(1,1));
disp(pcd(1,2));
disp(pcd(1,3));
disp(pcd(1,4));
disp(pcd(1,5));
disp(pcd(1,6));
disp(pcd(1,7));

%%

wind_oriantation = [20,30,35,45,60,75,90,105,120,135,150,165,180];
Cl_ratio_bow = [1.26,1.41,1.38,1.38,1.30,1.26,1.20,1.18,1.13,0.93,0.97,-9.66,0.00];
Cd_ratio_bow = [1.32,1.37,1.43,1.55,1.53,1.49,1.43,1.42,1.32,0.97,1.04,1.00,-0.15];
Cl_ratio_stern = [0.85,1.00,1.10,1.08,1.10,1.13,1.15,1.21,1.14,1.17,1.11,1.07,0.00];
Cd_ratio_stern = [1.40,1.24,1.14,1.25,1.34,1.31,1.20,1.13,1.04,1.18,1.13,1.06,1.06];

pclrbow = polyfit(wind_oriantation,Cl_ratio_bow,4);
disp('pclrbow');
disp(pclrbow(1,1));
disp(pclrbow(1,2));
disp(pclrbow(1,3));
disp(pclrbow(1,4));
disp(pclrbow(1,5));
% disp(pclrbow(1,6));

pcdrbow = polyfit(wind_oriantation,Cd_ratio_bow,4);
disp('pcdrbow');
disp(pcdrbow(1,1));
disp(pcdrbow(1,2));
disp(pcdrbow(1,3));
disp(pcdrbow(1,4));
disp(pcdrbow(1,5));
% disp(pcdrbow(1,6));

pclrstern = polyfit(wind_oriantation,Cl_ratio_stern,4);
disp('pclrstern');
disp(pclrstern(1,1));
disp(pclrstern(1,2));
disp(pclrstern(1,3));
disp(pclrstern(1,4));
disp(pclrstern(1,5));
% disp(pclrstern(1,6));

pcdrstern = polyfit(wind_oriantation,Cd_ratio_stern,4);
disp('pcdrstern');
disp(pcdrstern(1,1));
disp(pcdrstern(1,2));
disp(pcdrstern(1,3));
disp(pcdrstern(1,4));
disp(pcdrstern(1,5));
% disp(pcdrstern(1,6));

%%

wind_oriantation = [20,75,150,165,180];
Cl_ratio_bow = [1.2637,1.2620,0.9655,-9.6571,0.0000];

[pclrbow] = polyfit(wind_oriantation,Cl_ratio_bow,5);
disp('pclrbow');
disp(pclrbow(1,1));
disp(pclrbow(1,2));
disp(pclrbow(1,3));
disp(pclrbow(1,4));
disp(pclrbow(1,5));
disp(pclrbow(1,6));

wind_oriantation = [20,35,105,165,180];
Cd_ratio_bow = [0.8484,1.0964,1.2111,1.0684,0.0000];

[pcdrbow] = polyfit(wind_oriantation,Cd_ratio_bow,5);
disp('pcdrbow');
disp(pcdrbow(1,1));
disp(pcdrbow(1,2));
disp(pcdrbow(1,3));
disp(pcdrbow(1,4));
disp(pcdrbow(1,5));
disp(pcdrbow(1,6));

wind_oriantation = [20,45,120,133,165,180];
Cl_ratio_stern = [1.3243,1.5481,1.3204,0.9738,1.0000,-0.1483];

[pclrstern] = polyfit(wind_oriantation,Cl_ratio_stern,5);
disp('pclrstern');
disp(pclrstern(1,1));
disp(pclrstern(1,2));
disp(pclrstern(1,3));
disp(pclrstern(1,4));
disp(pclrstern(1,5));
disp(pclrstern(1,6));

wind_oriantation = [20,35,60,120,135,180];
Cd_ratio_stern = [1.4034,1.1416,1.3351,1.0432,1.1797,1.0574];

[pcdrstern] = polyfit(wind_oriantation,Cd_ratio_stern,5);
disp('pcdrstern');
disp(pcdrstern(1,1));
disp(pcdrstern(1,2));
disp(pcdrstern(1,3));
disp(pcdrstern(1,4));
disp(pcdrstern(1,5));
disp(pcdrstern(1,6));

