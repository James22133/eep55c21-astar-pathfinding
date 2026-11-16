%% EEP55C21 ASSIGNMENT 3 - A* PATHFINDING SIMULATION
% MATLAB R2024b. One self-contained script. No toolbox or GUI required.
% Based on the prescribed Assignment 3 map and the A* method in Lab 10.
%
% Run the full file or work through the numbered sections with Ctrl+Enter.
% Run Part 1 before Parts 2 and 3. Parts 4-8 work independently.
%
% Map: map(row,column), 0 = free, 1 = blocked.
% Start: [15,2]. Goal: [15,15]. Row 1 is displayed at the top.
% Movement: four neighbours only (up/down/left/right), cost 1 per step.
% A*: f(n) = g(n) + h(n), with Manhattan distance as h(n).
% This heuristic is admissible and consistent for unit-cost 4-way movement.
% Path is N x 2 [row,column]; cost is size(path,1)-1 moves.
% Each prescribed obstacle case begins with the ORIGINAL map.

%% PART 1 - Reproduce the prescribed 16 x 16 maze (Figure 1)
clearvars; close all; clc;
animateScenarios = ~strcmp(getenv('CPS_NO_ANIMATION'),'1');
[map,startRC,goalRC] = cpsPrescribedMap();
figure('Name','Assignment 3 A* - Original map');
cpsDrawGrid(gca,map,startRC,goalRC);
title('Original prescribed map');
fprintf('Original map loaded. S=[%d,%d], G=[%d,%d].\n', ...
    startRC(1),startRC(2),goalRC(1),goalRC(2));

%% PART 2 - Calculate and validate the initial A* route
[found,path,searchStats] = cpsAStar(map,startRC,goalRC);
assert(found,'A* did not find a route on the original map.');
[valid,reason] = cpsValidateRoute(map,path,startRC,goalRC);
assert(valid,reason);
moveCount = size(path,1)-1;
assert(moveCount == 25,'Original map should have an optimal 25-move route.');
fprintf('Initial A* route: %d moves, %d cells expanded.\n', ...
    moveCount,searchStats.expanded);
figure('Name','Assignment 3 A* - Initial route');
cpsDrawGrid(gca,map,startRC,goalRC,path,startRC);
title(sprintf('A* shortest path: %d moves',moveCount));
cpsMaybeExport(gcf,'original-route.png');

%% PART 3 - Inspect all three prescribed obstacle maps
% Preview only. The actual simulation changes the map AFTER movement.
figure('Name','Assignment 3 A* - Prescribed maps', ...
    'Position',[100 100 1200 420]);
for scenario = 1:3
    updated = cpsApplyPrescribedObstacle(map,scenario);
    added = updated==1 & map==0;
    subplot(1,3,scenario);
    cpsDrawGrid(gca,updated,startRC,goalRC,[],[],added);
    title(sprintf('Case %d, %d new blocked cells',scenario,nnz(added)));
end
cpsMaybeExport(gcf,'scenario-maps.png');

%% PART 4 - Case 1: interrupt journey and replan if necessary
cpsRunPrescribedScenario(1,animateScenarios,[9,7]);

%% PART 5 - Case 2: reset original map and run independently
cpsRunPrescribedScenario(2,animateScenarios,[9,7]);

%% PART 6 - Case 3: reset original map and run independently
cpsRunPrescribedScenario(3,animateScenarios,[9,7]);

%% PART 7 - Extra validation: obstacle is BEHIND the robot
% Case 1 is unchanged, but the obstacle appears when the robot reaches
% [9,10], after it has passed the newly blocked cells. No replan needed.
cpsRunPrescribedScenario(1,animateScenarios,[9,10]);

%% PART 8 - Extra validation: all crossings blocked means NO ROUTE
% This is a robustness test, NOT a fourth prescribed obstacle scenario.
[baseMap,s,g] = cpsPrescribedMap();
noRouteMap = baseMap;
noRouteMap([2,5,9],8:9) = 1;
[found,failedPath,testStats] = cpsAStar(noRouteMap,s,g);
assert(~found && isempty(failedPath), ...
    'A* should report no route when all crossings are blocked.');
fprintf('Unreachable goal: A* correctly returned no route (%d cells expanded).\n', ...
    testStats.expanded);
figure('Name','Assignment 3 A* - Unreachable goal test');
cpsDrawGrid(gca,noRouteMap,s,g,[],[],noRouteMap==1 & baseMap==0);
title('Extra test: no route exists');
cpsMaybeExport(gcf,'unreachable-goal.png');

%% LOCAL FUNCTIONS - these are called by the sections above

function [map,startRC,goalRC] = cpsPrescribedMap()
    % Assignment 3 Fig. 1, MATLAB row 1 at TOP of display.
    map = ones(16,16);
    map(2:15,2)  = 0;      % Left vertical corridor
    map(2:15,15) = 0;      % Right vertical corridor
    map(2,2:15)  = 0;      % Top crossing
    map(5,2:15)  = 0;      % Middle crossing
    map(9,2:15)  = 0;      % Lower crossing
    startRC = [15,2];
    goalRC  = [15,15];
end

function updatedMap = cpsApplyPrescribedObstacle(originalMap,scenario)
    % Assignment 3 Fig. 2: independent modifications to original map.
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

function [found,path,stats] = cpsAStar(map,startRC,goalRC)
    % A* from Lab 10 sections 1.5 and 2, implemented for our 4-way grid.
    % g: actual cost from start. h: Manhattan distance to goal.
    % f = g+h. Lowest-f open cell is explored next.
    % A logical open set is sufficient on a small 16x16 grid.
    found = false;
    path = [];
    stats.expanded = 0;
    stats.discovered = 0;
    [nRows,nCols] = size(map);

    % Reject invalid or blocked start/goal cells before indexing the map.
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
    stats.discovered = 1;

    while any(inOpen(:))
        % Find the open cell with the smallest f value.
        candidates = find(inOpen);
        bestF = min(fScore(candidates));
        ties = candidates(fScore(candidates)==bestF);
        % Tie-break: favour larger g. This does not affect optimality.
        [~,choice] = max(gScore(ties));
        currentIdx = ties(choice);
        [r,c] = ind2sub(size(map),currentIdx);
        inOpen(r,c) = false;
        closed(r,c) = true;
        stats.expanded = stats.expanded + 1;

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
                if ~inOpen(nr,nc) && ~closed(nr,nc)
                    stats.discovered = stats.discovered + 1;
                end
                % Reopening is safe if a shorter route is later found.
                closed(nr,nc) = false;
                inOpen(nr,nc) = true;
            end
        end
    end

    if ~found
        return; % Empty path means goal is unreachable.
    end

    % Follow the stored predecessors backwards from goal to start.
    current = goalRC;
    path = current;
    while ~isequal(current,startRC)
        r = current(1); c = current(2);
        current = [parentRow(r,c),parentCol(r,c)];
        if any(current==0)
            found = false; path = []; return;
        end
        path = [current;path]; %#ok<AGROW>
    end
end

function [ok, reason] = cpsValidateRoute(map,path,startRC,goalRC)
    % Check endpoints, in-bounds, free cells, and single-step movements.
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

function cpsDrawGrid(ax,map,startRC,goalRC,path,currentRC,newBlocked,invalidRoute)
    % Map coordinates in [row,col]; plot coordinates in (column,row).
    if nargin<5, path=[]; end
    if nargin<6, currentRC=[]; end
    if nargin<7, newBlocked=false(size(map)); end
    if nargin<8, invalidRoute=[]; end

    cla(ax);
    imagesc(ax,map);
    colormap(ax,[1 1 1;0 0 0]);
    clim(ax,[0 1]);
    axis(ax,'image');
    xlim(ax,[0.5 size(map,2)+0.5]);
    ylim(ax,[0.5 size(map,1)+0.5]);
    set(ax,'YDir','reverse','XTick',1:size(map,2),'YTick',1:size(map,1));
    hold(ax,'on');

    % TRUE cell boundaries at half-integers, unlike the misleading grid on ticks.
    for k=0.5:1:size(map,2)+0.5
        plot(ax,[k k],[0.5 size(map,1)+0.5], ...
            'Color',[0.7 0.7 0.7],'LineWidth',0.35);
    end
    for k=0.5:1:size(map,1)+0.5
        plot(ax,[0.5 size(map,2)+0.5],[k k], ...
            'Color',[0.7 0.7 0.7],'LineWidth',0.35);
    end

    % Clearly distinguish newly blocked cells from original black cells.
    [newR,newC] = find(newBlocked);
    for j=1:numel(newR)
        rectangle(ax,'Position',[newC(j)-0.5,newR(j)-0.5,1,1], ...
            'FaceColor',[0.85 0.2 0.2],'EdgeColor',[0.7 0.7 0.7]);
    end
    if ~isempty(invalidRoute)
        % Dashed red means INVALIDATED route; never followed after update.
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
    xlabel(ax,'Column (x)'); ylabel(ax,'Row (y, increasing downwards)');
    hold(ax,'off');
end

function cpsRunPrescribedScenario(scenario,animate,stopRC)
    % The obstacle is applied ONLY AFTER the robot reaches stopRC.
    % The initial route is computed on the original map each time.
    [baseMap,startRC,goalRC] = cpsPrescribedMap();
    [found,originalRoute,initialStats] = cpsAStar(baseMap,startRC,goalRC);
    assert(found,'No initial route found.');
    [ok,why] = cpsValidateRoute(baseMap,originalRoute,startRC,goalRC);
    assert(ok,why);

    % Choose the stopping position independently of the future map update.
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

    % Move along the initial route until the intermediate free cell.
    for k = 2:stopIdx
        if animate
            cpsDrawGrid(ax,baseMap,startRC,goalRC,originalRoute,originalRoute(k,:));
            title(ax,sprintf('Case %d: initial progress (%d/%d)', ...
                scenario,k-1,stopIdx-1));
            drawnow; pause(0.08);
        end
    end
    currentRC = originalRoute(stopIdx,:);

    % Introduce only the actual prescribed obstacle AFTER movement.
    updatedMap = cpsApplyPrescribedObstacle(baseMap,scenario);
    newBlocked = updatedMap==1 & baseMap==0;
    assert(updatedMap(currentRC(1),currentRC(2))==0, ...
        'The intermediate position became blocked.');

    % Validate the remaining original route before deciding to replan.
    remaining = originalRoute(stopIdx:end,:);
    [remainingValid,reason] = cpsValidateRoute( ...
        updatedMap,remaining,currentRC,goalRC);
    if remainingValid
        newRoute = remaining;
        outcome = 'Route remains valid. No replanning required.';
        expandedAfter = 0;
    else
        [found,newRoute,replanStats] = cpsAStar(updatedMap,currentRC,goalRC);
        expandedAfter = replanStats.expanded;
        if found
            [ok,why] = cpsValidateRoute(updatedMap,newRoute,currentRC,goalRC);
            assert(ok,why);
            outcome = 'Route blocked. Replanned from current position.';
        else
            outcome = 'No route exists. Agent stops safely.';
        end
    end

    fprintf('\nCase %d, stop [%d,%d]\n',scenario,currentRC(1),currentRC(2));
    fprintf('  Initial route: %d moves, %d cells expanded.\n', ...
        size(originalRoute,1)-1,initialStats.expanded);
    fprintf('  Remaining route valid: %s (%s)\n', ...
        mat2str(remainingValid),reason);
    fprintf('  %s\n',outcome);

    if remainingValid
        cpsDrawGrid(ax,updatedMap,startRC,goalRC,newRoute,currentRC,newBlocked);
    else
        % Red dashed means the discarded route, NOT a usable path.
        cpsDrawGrid(ax,updatedMap,startRC,goalRC,[],currentRC, ...
            newBlocked,remaining);
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
    fprintf('  Remaining route: %d moves, %d A* cells expanded.\n', ...
        movesAfter,expandedAfter);
    fprintf('  Total travel: %d moves.\n',movesBefore+movesAfter);
    cpsDrawGrid(ax,updatedMap,startRC,goalRC,newRoute,currentRC,newBlocked);
    title(ax,sprintf('Case %d: valid continuation (%d moves remaining)', ...
        scenario,movesAfter));
    drawnow;

    for k = 2:size(newRoute,1)
        nextRC = newRoute(k,:);
        assert(updatedMap(nextRC(1),nextRC(2))==0, ...
            'Agent tried to move through an obstacle.');
        if animate
            cpsDrawGrid(ax,updatedMap,startRC,goalRC,newRoute,nextRC,newBlocked);
            title(ax,sprintf('Case %d: continuing (%d/%d)', ...
                scenario,k-1,movesAfter));
            drawnow; pause(0.08);
        end
    end
    cpsDrawGrid(ax,updatedMap,startRC,goalRC,newRoute,goalRC,newBlocked);
    title(ax,sprintf('Case %d: GOAL reached. %d total moves.', ...
        scenario,movesBefore+movesAfter));
    if isequal(stopRC,[9,7])
        cpsMaybeExport(fig,sprintf('case-%d-result.png',scenario));
    else
        cpsMaybeExport(fig,'route-retention-test.png');
    end
end

function cpsMaybeExport(fig,fileName)
    % Set CPS_EXPORT_ASSETS=1 to reproduce the README figures.
    if ~strcmp(getenv('CPS_EXPORT_ASSETS'),'1')
        return;
    end
    drawnow;
    assetsDir = fullfile(fileparts(mfilename('fullpath')),'assets');
    if ~isfolder(assetsDir)
        mkdir(assetsDir);
    end
    exportgraphics(fig,fullfile(assetsDir,fileName),'Resolution',180);
end
