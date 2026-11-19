%% eep55c21 assignment 3 - a* pathfinding simulation
% this is one self-contained matlab r2024b script with no toolbox or gui needed.
% it uses the prescribed assignment 3 map and the a* method introduced in lab 10.
%
% you can run the whole file, or step through the sections with ctrl+enter.
% run part 1 before part 2. parts 3 to 7 can each run on their own.
%
% the map uses 0 for free cells and 1 for blocked cells.
% the start is [15,2], the goal is [15,15], and row 1 appears at the top.
% movement uses four neighbours with a cost of 1 for every step.
% a* uses f(n) = g(n) + h(n), where h(n) is manhattan distance.
% each path is stored as [row,column], and every obstacle case starts fresh.

%% part 1 - load the prescribed 16 x 16 maze
clearvars; close all; clc;
animateScenarios = ~strcmp(getenv('CPS_NO_ANIMATION'),'1');
[map,startRC,goalRC] = cpsPrescribedMap();
fprintf('Original map loaded. S=[%d,%d], G=[%d,%d].\n', ...
    startRC(1),startRC(2),goalRC(1),goalRC(2));

%% part 2 - calculate and validate the initial a* route
[found,path] = cpsAStar(map,startRC,goalRC);
assert(found,'A* did not find a route on the original map.');
[valid,reason] = cpsValidateRoute(map,path,startRC,goalRC);
assert(valid,reason);
moveCount = size(path,1)-1;
assert(moveCount == 25,'Original map should have an optimal 25-move route.');
fprintf('Initial A* route: %d moves.\n',moveCount);
figure('Name','Assignment 3 A* - Initial route');
cpsDrawGrid(gca,map,startRC,goalRC,path,startRC);
title(sprintf('A* shortest path: %d moves',moveCount));

%% part 3 - run case 1 and replan only if the route is blocked
cpsRunPrescribedScenario(1,animateScenarios,[9,7]);

%% part 4 - reset the map and run case 2 independently
cpsRunPrescribedScenario(2,animateScenarios,[9,7]);

%% part 5 - reset the map and run case 3 independently
cpsRunPrescribedScenario(3,animateScenarios,[9,7]);

%% part 6 - check that a route is kept when the obstacle is behind the agent
% case 1 appears after the agent reaches [9,10], so the remaining route is safe.
cpsRunPrescribedScenario(1,animateScenarios,[9,10]);

%% part 7 - check that blocking every crossing gives no route
% this is just a robustness check, not another prescribed obstacle case.
[baseMap,s,g] = cpsPrescribedMap();
noRouteMap = baseMap;
noRouteMap([2,5,9],8:9) = 1;
[found,failedPath] = cpsAStar(noRouteMap,s,g);
assert(~found && isempty(failedPath), ...
    'A* should report no route when all crossings are blocked.');
fprintf('Unreachable goal: A* correctly returned no route.\n');
figure('Name','Assignment 3 A* - Unreachable goal test');
cpsDrawGrid(gca,noRouteMap,s,g,[],[],noRouteMap==1 & baseMap==0);
title('Extra test: no route exists');

%% local functions used by the sections above

function [map,startRC,goalRC] = cpsPrescribedMap()
    % we build the exact map from figure 1, with row 1 shown at the top.
    map = ones(16,16);
    map(2:15,2)  = 0;      % this opens the left corridor
    map(2:15,15) = 0;      % this opens the right corridor
    map(2,2:15)  = 0;      % this is the top crossing
    map(5,2:15)  = 0;      % this is the middle crossing
    map(9,2:15)  = 0;      % this is the lower crossing
    startRC = [15,2];
    goalRC  = [15,15];
end

function updatedMap = cpsApplyPrescribedObstacle(originalMap,scenario)
    % each case starts from the original map and adds only its prescribed cells.
    updatedMap = originalMap;
    switch scenario
        case 1
            updatedMap(9,8:9) = 1;
        case 2
            updatedMap(9,8:9) = 1;
            updatedMap(5,8:9) = 1;
        case 3
            updatedMap(9,8:9) = 1;
            updatedMap(2,8:9) = 1;
        otherwise
            error('Scenario must be 1, 2 or 3.');
    end
end

function [found,path] = cpsAStar(map,startRC,goalRC)
    % this is lab 10 a* adapted to our four-connected grid.
    % g stores the cost travelled, h estimates what is left, and f is their sum.
    % a logical open set keeps this small 16 x 16 implementation easy to follow.
    found = false;
    path = [];
    [nRows,nCols] = size(map);

    % we check the endpoints first so bad inputs never reach matrix indexing.
    if numel(startRC)~=2 || numel(goalRC)~=2 || ...
            any(~isfinite([startRC goalRC])) || ...
            any([startRC goalRC] ~= round([startRC goalRC])) || ...
            any(startRC<1) || any(goalRC<1) || ...
            startRC(1)>nRows || startRC(2)>nCols || ...
            goalRC(1)>nRows || goalRC(2)>nCols
        return;
    end
    if map(startRC(1),startRC(2))~=0 || map(goalRC(1),goalRC(2))~=0
        return;
    end

    gScore = inf(nRows,nCols);
    fScore = inf(nRows,nCols);
    inOpen = false(nRows,nCols);
    closed = false(nRows,nCols);
    parentRow = zeros(nRows,nCols);
    parentCol = zeros(nRows,nCols);
    neighbours = [-1 0; 1 0; 0 -1; 0 1];

    gScore(startRC(1),startRC(2)) = 0;
    fScore(startRC(1),startRC(2)) = sum(abs(startRC-goalRC));
    inOpen(startRC(1),startRC(2)) = true;

    while any(inOpen(:))
        % we always expand the open cell with the lowest estimated total cost.
        candidates = find(inOpen);
        bestF = min(fScore(candidates));
        ties = candidates(fScore(candidates)==bestF);
        % when f ties, a larger g gently favours progress without changing optimality.
        [~,choice] = max(gScore(ties));
        currentIdx = ties(choice);
        [r,c] = ind2sub(size(map),currentIdx);
        inOpen(r,c) = false;
        closed(r,c) = true;

        if isequal([r,c],goalRC)
            found = true;
            break;
        end

        for k = 1:4
            next = [r,c] + neighbours(k,:);
            nr = next(1); nc = next(2);
            if nr<1 || nr>nRows || nc<1 || nc>nCols
                continue;
            end
            if map(nr,nc)~=0
                continue;
            end

            tentativeG = gScore(r,c)+1;
            if tentativeG < gScore(nr,nc)
                parentRow(nr,nc) = r;
                parentCol(nr,nc) = c;
                gScore(nr,nc) = tentativeG;
                h = abs(nr-goalRC(1)) + abs(nc-goalRC(2));
                fScore(nr,nc) = tentativeG+h;
                % we reopen a cell if this route reaches it more cheaply.
                closed(nr,nc) = false;
                inOpen(nr,nc) = true;
            end
        end
    end

    if ~found
        return; % an empty path is how we report an unreachable goal.
    end

    % now we walk backwards through the saved parents to rebuild the route.
    current = goalRC;
    path = zeros(gScore(goalRC(1),goalRC(2))+1,2);
    for pathIndex = size(path,1):-1:1
        path(pathIndex,:) = current;
        if pathIndex > 1
            r = current(1); c = current(2);
            current = [parentRow(r,c),parentCol(r,c)];
            if any(current==0)
                found = false; path = []; return;
            end
        end
    end
end

function [ok, reason] = cpsValidateRoute(map,path,startRC,goalRC)
    % this check keeps plotting and movement separate from trusting the planner.
    ok = false;
    if isempty(path) || size(path,2) ~= 2
        reason = 'Path is empty or incorrectly shaped.'; return;
    end
    if ~isequal(path(1,:),startRC) || ~isequal(path(end,:),goalRC)
        reason = 'Path does not start and end at requested cells.'; return;
    end
    if any(~isfinite(path(:))) || any(path(:) ~= round(path(:)))
        reason = 'Path includes invalid/noninteger indices.'; return;
    end
    if any(path(:,1)<1 | path(:,1)>size(map,1) | ...
           path(:,2)<1 | path(:,2)>size(map,2))
        reason = 'Path leaves the grid.'; return;
    end
    idx = sub2ind(size(map),path(:,1),path(:,2));
    if any(map(idx) ~= 0)
        reason = 'Path goes through a blocked cell.'; return;
    end
    stepCosts = sum(abs(diff(path,1,1)),2);
    if any(stepCosts ~= 1)
        reason = 'Path uses illegal diagonal or multi-cell movement.'; return;
    end
    ok = true; reason = 'Route valid.';
end

function cpsDrawGrid(ax,map,startRC,goalRC,path,currentRC,newBlocked, ...
        invalidRoute,travelledRoute,replanRC)
    % paths use [row,column], so we swap them when plotting x and y.
    if nargin<5, path=[]; end
    if nargin<6, currentRC=[]; end
    if nargin<7, newBlocked=false(size(map)); end
    if nargin<8, invalidRoute=[]; end
    if nargin<9, travelledRoute=[]; end
    if nargin<10, replanRC=[]; end

    cla(ax);
    imagesc(ax,map);
    colormap(ax,[1 1 1;0 0 0]);
    clim(ax,[0 1]);
    axis(ax,'image');
    xlim(ax,[0.5 size(map,2)+0.5]);
    ylim(ax,[0.5 size(map,1)+0.5]);
    set(ax,'YDir','reverse','XTick',1:size(map,2),'YTick',1:size(map,1));
    hold(ax,'on');

    % half-integer lines put the grid around cells rather than through their centres.
    for k=0.5:1:size(map,2)+0.5
        plot(ax,[k k],[0.5 size(map,1)+0.5], ...
            'Color',[0.7 0.7 0.7],'LineWidth',0.35);
    end
    for k=0.5:1:size(map,1)+0.5
        plot(ax,[0.5 size(map,2)+0.5],[k k], ...
            'Color',[0.7 0.7 0.7],'LineWidth',0.35);
    end

    % red cells make the new obstruction easy to separate from the original walls.
    [newR,newC] = find(newBlocked);
    for j=1:numel(newR)
        rectangle(ax,'Position',[newC(j)-0.5,newR(j)-0.5,1,1], ...
            'FaceColor',[0.85 0.2 0.2],'EdgeColor',[0.7 0.7 0.7]);
    end
    if ~isempty(travelledRoute)
        % grey shows where the agent travelled before the map changed.
        plot(ax,travelledRoute(:,2),travelledRoute(:,1),'-', ...
            'Color',[0.55 0.55 0.55],'LineWidth',5);
    end
    if ~isempty(invalidRoute)
        % dashed red shows the discarded route, so it is never mistaken for a safe one.
        plot(ax,invalidRoute(:,2),invalidRoute(:,1),'--', ...
            'Color',[0.9 0.2 0.2],'LineWidth',2);
    end
    if ~isempty(path)
        plot(ax,path(:,2),path(:,1),'-', ...
            'Color',[0.05 0.5 0.9],'LineWidth',2.6);
    end
    text(ax,startRC(2),startRC(1),'S','Color',[0.85 0.05 0.05], ...
        'FontWeight','bold','HorizontalAlignment','center', ...
        'VerticalAlignment','middle');
    text(ax,goalRC(2),goalRC(1),'G','Color',[0 0.45 0.1], ...
        'FontWeight','bold','HorizontalAlignment','center', ...
        'VerticalAlignment','middle');
    if ~isempty(currentRC)
        plot(ax,currentRC(2),currentRC(1),'mo', ...
            'MarkerFaceColor','none','LineWidth',2,'MarkerSize',10);
    end
    if ~isempty(replanRC)
        plot(ax,replanRC(2),replanRC(1),'d', ...
            'MarkerEdgeColor',[0.85 0.3 0], ...
            'MarkerFaceColor',[1 0.7 0.15],'LineWidth',1.5,'MarkerSize',8);
    end
    xlabel(ax,'Column (x)'); ylabel(ax,'Row (y, increasing downwards)');
    hold(ax,'off');
end

function cpsRunPrescribedScenario(scenario,animate,stopRC)
    % every case begins again on the original map.
    % the new obstacle only appears after the agent reaches stoprc.
    [baseMap,startRC,goalRC] = cpsPrescribedMap();
    [found,originalRoute] = cpsAStar(baseMap,startRC,goalRC);
    assert(found,'No initial route found.');
    [ok,why] = cpsValidateRoute(baseMap,originalRoute,startRC,goalRC);
    assert(ok,why);

    % we choose the stopping point on the original route before changing the map.
    stopIdx = find(all(originalRoute==stopRC,2),1,'first');
    assert(~isempty(stopIdx) && stopIdx>1 && stopIdx<size(originalRoute,1), ...
        'Intermediate position must be on the initial route before the goal.');

    fig = figure('Name',sprintf('A* Case %d, stop [%d,%d]', ...
        scenario,stopRC(1),stopRC(2)));
    ax = axes('Parent',fig);
    cpsDrawGrid(ax,baseMap,startRC,goalRC,originalRoute,startRC);
    title(ax,sprintf('Case %d: initial A* path (%d moves)', ...
        scenario,size(originalRoute,1)-1));
    drawnow;

    % first we follow the route that was planned on the unchanged map.
    for k = 2:stopIdx
        if animate
            cpsDrawGrid(ax,baseMap,startRC,goalRC,originalRoute,originalRoute(k,:));
            title(ax,sprintf('Case %d: initial progress (%d/%d)', ...
                scenario,k-1,stopIdx-1));
            drawnow; pause(0.08);
        end
    end
    currentRC = originalRoute(stopIdx,:);
    travelledRoute = originalRoute(1:stopIdx,:);

    % at the stopping point, the complete prescribed map update becomes known.
    updatedMap = cpsApplyPrescribedObstacle(baseMap,scenario);
    newBlocked = updatedMap==1 & baseMap==0;
    assert(updatedMap(currentRC(1),currentRC(2))==0, ...
        'The intermediate position became blocked.');

    % we only replan if the part still ahead has genuinely become invalid.
    remaining = originalRoute(stopIdx:end,:);
    [remainingValid,reason] = cpsValidateRoute( ...
        updatedMap,remaining,currentRC,goalRC);
    if remainingValid
        newRoute = remaining;
        outcome = 'Route remains valid. No replanning required.';
    else
        [found,newRoute] = cpsAStar(updatedMap,currentRC,goalRC);
        if found
            [ok,why] = cpsValidateRoute(updatedMap,newRoute,currentRC,goalRC);
            assert(ok,why);
            outcome = 'Route blocked. Replanned from current position.';
        else
            outcome = 'No route exists. Agent stops safely.';
        end
    end

    fprintf('\nCase %d, stop [%d,%d]\n',scenario,currentRC(1),currentRC(2));
    fprintf('  Initial route: %d moves.\n',size(originalRoute,1)-1);
    fprintf('  Remaining route valid: %s (%s)\n', ...
        mat2str(remainingValid),reason);
    fprintf('  %s\n',outcome);

    if remainingValid
        cpsDrawGrid(ax,updatedMap,startRC,goalRC,newRoute,currentRC, ...
            newBlocked,[],travelledRoute,currentRC);
    else
        % the dashed red line is only evidence of why replanning was needed.
        cpsDrawGrid(ax,updatedMap,startRC,goalRC,[],currentRC, ...
            newBlocked,remaining,travelledRoute,currentRC);
    end
    title(ax,sprintf('Case %d: %s',scenario,outcome));
    drawnow;
    if animate, pause(0.5); end

    if isempty(newRoute)
        cpsDrawGrid(ax,updatedMap,startRC,goalRC,[],currentRC,newBlocked);
        title(ax,sprintf('Case %d: no route. Agent stopped.',scenario));
        return;
    end

    movesBefore = stopIdx-1;
    movesAfter = size(newRoute,1)-1;
    fprintf('  Remaining route: %d moves.\n',movesAfter);
    fprintf('  Total travel: %d moves.\n',movesBefore+movesAfter);
    cpsDrawGrid(ax,updatedMap,startRC,goalRC,newRoute,currentRC, ...
        newBlocked,[],travelledRoute,currentRC);
    title(ax,sprintf('Case %d: valid continuation (%d moves remaining)', ...
        scenario,movesAfter));
    drawnow;

    for k = 2:size(newRoute,1)
        nextRC = newRoute(k,:);
        assert(updatedMap(nextRC(1),nextRC(2))==0, ...
            'Agent tried to move through an obstacle.');
        if animate
            cpsDrawGrid(ax,updatedMap,startRC,goalRC,newRoute,nextRC, ...
                newBlocked,[],travelledRoute,currentRC);
            title(ax,sprintf('Case %d: continuing (%d/%d)', ...
                scenario,k-1,movesAfter));
            drawnow; pause(0.08);
        end
    end
    cpsDrawGrid(ax,updatedMap,startRC,goalRC,newRoute,goalRC, ...
        newBlocked,[],travelledRoute,currentRC);
    title(ax,sprintf('Case %d: GOAL reached. %d total moves.', ...
        scenario,movesBefore+movesAfter));
end
