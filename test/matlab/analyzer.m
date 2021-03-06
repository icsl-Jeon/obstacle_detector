candidPolySetX = ...
[5.57499e-10 6.07705e-10  6.5791e-10  5.6273e-10 6.12935e-10  6.6314e-10  5.6796e-10 6.18165e-10 6.68371e-10
          1           1           1           1           1           1           1           1           1
   -2.63554    -2.88225    -3.12896    -1.31185    -1.55856    -1.80527   0.0118392   -0.234872   -0.481584
     2.1162     2.33352     2.55083     1.36078     1.57809      1.7954    0.605348     0.82266     1.03997
  -0.314374   -0.347115   -0.379856   -0.209485   -0.242227   -0.274968   -0.104597   -0.137338    -0.17008
7.75011e-13 1.00238e-12 1.22976e-12 4.33951e-13 6.61324e-13 8.88698e-13 9.28902e-14 3.20264e-13 5.47638e-13];

candidPolySetY = ...
[ 7.23806e-10  5.22985e-10  3.22163e-10  7.29037e-10  5.28215e-10  3.27393e-10  7.34267e-10  5.33445e-10  3.32623e-10
           1            1            1            1            1            1            1            1            1
    0.595392      1.58224      2.56908      1.91908      2.90592      3.89277      3.24277      4.22961      5.21646
    0.501854    -0.367391     -1.23664    -0.253574     -1.12282     -1.99206       -1.009     -1.87825     -2.74749
  -0.0979325    0.0330322     0.163997   0.00695568      0.13792     0.268885     0.111844     0.242809     0.373773
-2.07967e-14 -2.07967e-14 -3.05014e-13  -2.4817e-13 -1.34484e-13 -5.89231e-13  2.06577e-13 -4.75544e-13 -7.02918e-13];

L = size(candidPolySetX,2);
horizon = 3;
t_eval = linspace(0,horizon,20);

 
waypoints = ...
    [1 2 3  11 12 13 ; ...  % x
     4 5 6  14 10 6];      % y

figure(1)
clf
hold on

for l = 1: L
    xs = polyval(flipud(candidPolySetX(:,l)),t_eval);
    ys = polyval(flipud(candidPolySetY(:,l)),t_eval);
    plot([0 waypoints(1,:)],[0 waypoints(2,:)],'ks')
    plot(xs,ys,'k-')
    grid on
end

%% Delimeter (for copy and paste to c++ script)
px = [-0.0347    0.3304   -0.8155   -0.0000    1.0000    6.0000];
py = [0.0609   -0.5404    1.3265   -0.0000    0.0000    0.1000];

allOneString = sprintf('%.4f,' , fliplr(py));
allOneString = allOneString(1:end-1) % strip final comma



