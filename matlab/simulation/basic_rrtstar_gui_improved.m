function basic_rrtstar_gui
% FINAL VERSION: RRT* with UI + polygon obstacles + reset + safe indexing

clc; close all;

%% APP STATE
app.mapSize = 20;
app.numObstacles = 10;
app.stepSize = 1.0;
app.maxIterations = 1500;

app.start = [];
app.goal = [];
app.obstacles = [];

%% FIGURE
figure('Name','RRT* Planner','Color','w');
app.ax = axes; hold on; grid on; axis equal;

set(gcf,'WindowButtonDownFcn',@clickCallback);

%% UI CONTROLS
uicontrol('Style','pushbutton','String','Plan Path',...
    'Units','normalized','Position',[0.8 0.01 0.15 0.05],...
    'Callback',@planPath);

uicontrol('Style','pushbutton','String','Reset Map',...
    'Units','normalized','Position',[0.6 0.01 0.15 0.05],...
    'Callback',@(~,~) resetMap());

uicontrol('Style','edit','String','20',...
    'Units','normalized','Position',[0.05 0.01 0.1 0.05],...
    'Callback',@(src,~) setMapSize(src));

uicontrol('Style','edit','String','10',...
    'Units','normalized','Position',[0.2 0.01 0.1 0.05],...
    'Callback',@(src,~) setObstacleCount(src));

resetMap();

%% CLICK HANDLER
function clickCallback(~,~)
    pt = get(app.ax,'CurrentPoint');
    p = pt(1,1:2);

    if isempty(app.start)
        app.start = p;
    elseif isempty(app.goal)
        app.goal = p;
    else
        app.start = p;
        app.goal = [];
    end

    plotScene();
end

%% PLAN PATH
function planPath(~,~)
    if isempty(app.start) || isempty(app.goal)
        disp('Select start and goal'); return;
    end

    [path, nodePos, parent] = rrtStar(app.start, app.goal);

    plotScene();

    % draw tree
    for i = 2:size(nodePos,1)
        p = parent(i);
        if p <= 0
            continue;
        end
        plot([nodePos(i,1) nodePos(p,1)],...
             [nodePos(i,2) nodePos(p,2)], 'b');
    end

    % draw path
    if ~isempty(path)
        plot(path(:,1), path(:,2),'r','LineWidth',2);
    else
        disp('No path found');
    end
end

%% RRT*
function [path, nodePos, parent] = rrtStar(start, goal)

nodePos = zeros(app.maxIterations,2);
parent = zeros(app.maxIterations,1);
cost = inf(app.maxIterations,1);

nodePos(1,:) = start;
cost(1)=0;
n=1;

for i=1:app.maxIterations

    qRand = rand(1,2)*app.mapSize;

    d = vecnorm(nodePos(1:n,:) - qRand,2,2);
    [~,idx] = min(d);
    qNear = nodePos(idx,:);

    dir = qRand - qNear;
    qNew = qNear + app.stepSize*dir/norm(dir);

    if ~collisionFree(qNear,qNew)
        continue;
    end

    n = n+1;
    nodePos(n,:) = qNew;
    parent(n) = idx;
    cost(n) = cost(idx) + norm(qNew-qNear);

    % rewire
    for j=1:n-1
        if norm(nodePos(j,:)-qNew)<2
            newCost = cost(n)+norm(nodePos(j,:)-qNew);
            if newCost<cost(j) && collisionFree(qNew,nodePos(j,:))
                parent(j)=n;
                cost(j)=newCost;
            end
        end
    end

    if norm(qNew-goal)<1
        path = backtrack(nodePos,parent,n,goal);
        return;
    end
end

path = [];
nodePos = nodePos(1:n,:);
parent = parent(1:n);
end

%% BACKTRACK
function path = backtrack(nodePos,parent,idx,goal)
path = goal;
while idx>0
    path = [nodePos(idx,:); path]; %#ok
    idx = parent(idx);
end
end

%% COLLISION (POLYGONS)
function tf = collisionFree(p1,p2)
tf = true;

for i=1:length(app.obstacles)
    verts = app.obstacles(i).verts;
    for j=1:size(verts,1)
        v1 = verts(j,:);
        v2 = verts(mod(j,size(verts,1))+1,:);

        if segmentsIntersect(p1,p2,v1,v2)
            tf = false;
            return;
        end
    end
end
end

function flag = segmentsIntersect(p1,p2,q1,q2)
flag = ccw(p1,q1,q2) ~= ccw(p2,q1,q2) && ...
       ccw(p1,p2,q1) ~= ccw(p1,p2,q2);
end

function val = ccw(A,B,C)
val = (C(2)-A(2))*(B(1)-A(1)) > (B(2)-A(2))*(C(1)-A(1));
end

%% OBSTACLES (POLYGONS)
function obs = generateObstacles()
obs = struct('verts',{});

for i=1:app.numObstacles
    n = randi([4,8]);
    center = rand(1,2)*app.mapSize;
    angles = sort(rand(1,n)*2*pi);
    radius = 1 + rand;

    verts = zeros(n,2);
    for k=1:n
        r = radius*(0.5+rand);
        verts(k,:) = center + [r*cos(angles(k)), r*sin(angles(k))];
    end

    obs(i).verts = verts;
end
end

%% PLOT
function plotScene()
    cla; hold on;
    axis([0 app.mapSize 0 app.mapSize]);

    for i=1:length(app.obstacles)
        v = app.obstacles(i).verts;
        fill(v(:,1), v(:,2), [0.3 0.3 0.3]);
    end

    if ~isempty(app.start)
        plot(app.start(1),app.start(2),'go','MarkerFaceColor','g');
    end

    if ~isempty(app.goal)
        plot(app.goal(1),app.goal(2),'ro','MarkerFaceColor','r');
    end
end

%% RESET
function resetMap()
    app.start = [];
    app.goal = [];
    app.obstacles = generateObstacles();
    cla;
    plotScene();
end

%% PARAM SETTERS
function setMapSize(src)
    app.mapSize = str2double(src.String);
    resetMap();
end

function setObstacleCount(src)
    app.numObstacles = round(str2double(src.String));
    resetMap();
end

end
