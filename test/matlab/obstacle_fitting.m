% This script tests the minimum volume ellipsoid extraction. 

% Load dataset /sensor_msgs/LaserScan
bag_path = 'forest2DScan.bag'; 

bSel = select(rosbag(bag_path),'Topic','/camera/scan');
msgStructs = readMessages(bSel,'DataFormat','struct');
angles = msgStructs{1}.AngleMin:msgStructs{1}.AngleIncrement:msgStructs{1}.AngleMax;

%% Visualize min volume ellipsoid 
figure(1)
xLim = [0 30];
yLim = [-20 20];

endSeq = length(msgStructs);
% endSeq = 1;

for i = 1:endSeq
    points = []; % we will collect points inside of a box bound 
    ranges = msgStructs{i}.Ranges;
    x = ranges.*cos(angles)';
    y = ranges.*sin(angles)';
    for j = 1:length(x)
        if x(j) < xLim(2) && x(j) > xLim(1) && y(j) < yLim(2) && y(j) > yLim(1)
            points = [points; [x(j) y(j)]];
        end
    end
    validPoints = points(~isinf(points(:,1)),:);   
    eps = 1.5; Nmin = 5;
    IDX =DBSCAN(validPoints,eps,Nmin);
    nCluster = max(IDX);
    figure(1)
    cla
    hold on 
    for n = 1:nCluster
        curClusterPoints = validPoints(IDX == n,:);
        [A,c]=MinVolEllipse(curClusterPoints',0.01);
        [U,Q,V]=svd(A);
        r1 = 1/sqrt(Q(1,1)); 
        r2 = 1/sqrt(Q(2,2));
        R = V ;
        draw_ellipse(R,[r1 r2],c,'b','k')
    end
    plot(validPoints(:,1),validPoints(:,2),'ko')
    axis equal
    axis([0 30 -20 20])
    pause(1e-2)
end

%% DBSCAN for valid points 




