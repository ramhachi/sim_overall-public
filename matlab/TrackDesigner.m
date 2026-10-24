function TrackDesigner()
    % TrackDesigner - FSAE Track Design Tool
    % Allows interactive placement of waypoints and exports to CSV for laptime simulation.
    % Features: Spline/Straight segments, Corner Fillets, Slalom Generation, CSV Export
    
    % Global state
    state.waypoints = zeros(0, 2); % [x, y]
    state.segmentTypes = zeros(0, 1); % 0=Spline, 1=Straight (Applied to segment AFTER point)
    state.mode = 'add'; % 'add', 'move', 'delete'
    state.trackWidth = 3.0;
    state.isClosed = false;
    state.selectedIdx = -1;
    state.fig = [];
    state.ax = [];
    state.ui = []; % Handle structure
    
    % Create UI
    createUI();
    
    function createUI()
        state.fig = figure('Name', 'FSAE Track Designer V3 (Slalom)', ...
            'NumberTitle', 'off', ...
            'MenuBar', 'none', ...
            'ToolBar', 'figure', ...
            'Position', [100, 100, 1200, 800], ...
            'WindowButtonDownFcn', @onMouseDown, ...
            'WindowButtonMotionFcn', @onMouseMove, ...
            'WindowButtonUpFcn', @onMouseUp, ...
            'KeyPressFcn', @onKeyPress);
        
        % Layout
        panelControls = uipanel(state.fig, 'Position', [0 0 0.25 1], 'Title', 'Controls');
        state.ax = axes(state.fig, 'Position', [0.3 0.1 0.65 0.85]);
        grid(state.ax, 'on');
        hold(state.ax, 'on');
        axis(state.ax, 'equal');
        xlabel(state.ax, 'X [m]');
        ylabel(state.ax, 'Y [m]');
        title(state.ax, 'Track Layout');
        
        % -- Controls --
        yPos = 0.96;
        h = 0.04;
        
        % Mode Selection
        uicontrol(panelControls, 'Style', 'text', 'String', 'Mode:', 'Units', 'normalized', ...
            'Position', [0.05 yPos-0.02 0.9 0.03], 'HorizontalAlignment', 'left', 'FontWeight', 'bold');
        yPos = yPos - 0.05;
        
        bg = uibuttongroup(panelControls, 'Position', [0.05 yPos-0.12 0.9 0.12], 'SelectionChangedFcn', @onModeChange);
        uicontrol(bg, 'Style', 'radiobutton', 'String', 'Add Points (Spline)', 'Units', 'normalized', 'Position', [0.1 0.7 0.8 0.2], 'Tag', 'add');
        uicontrol(bg, 'Style', 'radiobutton', 'String', 'Move / Edit', 'Units', 'normalized', 'Position', [0.1 0.45 0.8 0.2], 'Tag', 'move');
        uicontrol(bg, 'Style', 'radiobutton', 'String', 'Delete Points', 'Units', 'normalized', 'Position', [0.1 0.2 0.8 0.2], 'Tag', 'delete');
        yPos = yPos - 0.15;
        
        % Segment Type (Selected Point)
        uicontrol(panelControls, 'Style', 'text', 'String', 'Selected Segment Type:', 'Units', 'normalized', ...
            'Position', [0.05 yPos 0.9 0.03], 'HorizontalAlignment', 'left', 'FontWeight', 'bold');
        yPos = yPos - 0.04;
        
        state.ui.segSpline = uicontrol(panelControls, 'Style', 'radiobutton', 'String', 'Spline to Next', ...
            'Units', 'normalized', 'Position', [0.1 yPos 0.8 0.04], 'Callback', {@onSegTypeChange, 0});
        yPos = yPos - 0.04;
        state.ui.segLine = uicontrol(panelControls, 'Style', 'radiobutton', 'String', 'Straight to Next', ...
            'Units', 'normalized', 'Position', [0.1 yPos 0.8 0.04], 'Callback', {@onSegTypeChange, 1});
        yPos = yPos - 0.06;
        
        % Fillet Tool
        uicontrol(panelControls, 'Style', 'text', 'String', 'Corner Fillet (Arc):', 'Units', 'normalized', ...
            'Position', [0.05 yPos 0.9 0.03], 'HorizontalAlignment', 'left', 'FontWeight', 'bold');
        yPos = yPos - 0.04;
        
        uicontrol(panelControls, 'Style', 'text', 'String', 'Radius [m]:', 'Units', 'normalized', ...
            'Position', [0.05 yPos 0.4 0.03], 'HorizontalAlignment', 'left');
        state.ui.filletRadius = uicontrol(panelControls, 'Style', 'edit', 'String', '5.0', ...
            'Units', 'normalized', 'Position', [0.5 yPos 0.4 0.04]);
        yPos = yPos - 0.05;
        
        uicontrol(panelControls, 'Style', 'pushbutton', 'String', 'Apply Fillet', ...
            'Units', 'normalized', 'Position', [0.05 yPos 0.9 0.05], 'Callback', @onFillet, 'BackgroundColor', [0.9 0.9 1.0]);
        yPos = yPos - 0.08;
        
        % Slalom Tool
        uicontrol(panelControls, 'Style', 'text', 'String', 'Slalom Generator:', 'Units', 'normalized', ...
            'Position', [0.05 yPos 0.9 0.03], 'HorizontalAlignment', 'left', 'FontWeight', 'bold');
        yPos = yPos - 0.04;
        
        % Slalom Params
        uicontrol(panelControls, 'Style', 'text', 'String', 'Cones:', 'Units', 'normalized', 'Position', [0.05 yPos 0.25 0.03]);
        state.ui.slalomN = uicontrol(panelControls, 'Style', 'edit', 'String', '5', 'Units', 'normalized', 'Position', [0.3 yPos 0.15 0.04]);
        
        uicontrol(panelControls, 'Style', 'text', 'String', 'Gap[m]:', 'Units', 'normalized', 'Position', [0.5 yPos 0.25 0.03]);
        state.ui.slalomGap = uicontrol(panelControls, 'Style', 'edit', 'String', '10', 'Units', 'normalized', 'Position', [0.75 yPos 0.15 0.04]);
        yPos = yPos - 0.05;
        
        uicontrol(panelControls, 'Style', 'text', 'String', 'Amp[m]:', 'Units', 'normalized', 'Position', [0.05 yPos 0.25 0.03]);
        state.ui.slalomAmp = uicontrol(panelControls, 'Style', 'edit', 'String', '2.0', 'Units', 'normalized', 'Position', [0.3 yPos 0.15 0.04]);
        
        uicontrol(panelControls, 'Style', 'pushbutton', 'String', 'Append Slalom', ...
            'Units', 'normalized', 'Position', [0.5 yPos 0.45 0.05], 'Callback', @onSlalom, 'BackgroundColor', [1.0 0.9 0.9]);
        yPos = yPos - 0.08;
        
        % Global Settings
        uicontrol(panelControls, 'Style', 'text', 'String', 'Settings:', 'Units', 'normalized', ...
            'Position', [0.05 yPos 0.9 0.03], 'HorizontalAlignment', 'left', 'FontWeight', 'bold');
        yPos = yPos - 0.04;
        
        uicontrol(panelControls, 'Style', 'text', 'String', 'Width [m]:', 'Units', 'normalized', ...
            'Position', [0.05 yPos 0.4 0.03], 'HorizontalAlignment', 'left');
        uicontrol(panelControls, 'Style', 'edit', 'String', '3.0', 'Units', 'normalized', ...
            'Position', [0.5 yPos 0.4 0.04], 'Callback', @onWidthChange);
        yPos = yPos - 0.05;
        
        state.chkClosed = uicontrol(panelControls, 'Style', 'checkbox', 'String', 'Closed Loop', ...
            'Units', 'normalized', 'Position', [0.05 yPos 0.9 0.04], 'Callback', @onClosedChange);
        yPos = yPos - 0.08;
        
        % File Ops
        uicontrol(panelControls, 'Style', 'pushbutton', 'String', 'Clear All', 'Units', 'normalized', ...
            'Position', [0.05 yPos 0.9 0.05], 'Callback', @onClear);
        yPos = yPos - 0.06;
        
        uicontrol(panelControls, 'Style', 'pushbutton', 'String', 'Load CSV...', 'Units', 'normalized', ...
            'Position', [0.05 yPos 0.4 0.05], 'Callback', @onLoad);
        uicontrol(panelControls, 'Style', 'pushbutton', 'String', 'Save CSV...', 'Units', 'normalized', ...
            'Position', [0.55 yPos 0.4 0.05], 'Callback', @onSave, 'FontWeight', 'bold', 'BackgroundColor', [0.8 1.0 0.8]);
        
        % Update initial state
        updateUIState();
        updatePlot();
    end

    % --- Interactions ---
    
    function onMouseDown(~, ~)
        pt = get(state.ax, 'CurrentPoint');
        x = pt(1,1);
        y = pt(1,2);
        
        idx = findNearestPoint(x, y);
        
        switch state.mode
            case 'add'
                % Add new point
                state.waypoints(end+1, :) = [x, y];
                % Default segment type: Spline (0)
                state.segmentTypes(end+1, 1) = 0; 
                
                state.selectedIdx = size(state.waypoints, 1);
                updateUIState();
                updatePlot();
                
            case 'move'
                if idx > 0
                    state.selectedIdx = idx;
                    updateUIState();
                    updatePlot();
                else
                    % Clicked empty space clears selection
                    state.selectedIdx = -1;
                    updateUIState();
                    updatePlot();
                end
                
            case 'delete'
                if idx > 0
                    state.waypoints(idx, :) = [];
                    state.segmentTypes(idx) = [];
                    state.selectedIdx = -1;
                    updatePlot();
                end
        end
    end

    function onMouseMove(~, ~)
        if strcmp(state.mode, 'move') && state.selectedIdx > 0
            pt = get(state.ax, 'CurrentPoint');
            state.waypoints(state.selectedIdx, :) = [pt(1,1), pt(1,2)];
            updatePlot();
        end
    end

    function onMouseUp(~, ~)
        % Keep selection in move mode, just stop dragging
    end

    function onKeyPress(~, event)
        if strcmp(event.Key, 'delete') || strcmp(event.Key, 'backspace')
            if state.selectedIdx > 0
                state.waypoints(state.selectedIdx, :) = [];
                state.segmentTypes(state.selectedIdx) = [];
                state.selectedIdx = -1;
                updatePlot();
            end
        end
    end

    % --- Logic ---
    
    function onSlalom(~, ~)
        if isempty(state.waypoints)
            warndlg('Create a start point first.'); return;
        end
        
        % Params
        nCones = str2double(state.ui.slalomN.String);
        gap = str2double(state.ui.slalomGap.String);
        amp = str2double(state.ui.slalomAmp.String);
        
        if isnan(nCones) || isnan(gap) || isnan(amp), warndlg('Invalid slalom params.'); return; end
        
        % Determine direction
        n = size(state.waypoints, 1);
        lastPt = state.waypoints(end, :);
        
        dir = [1, 0];
        if n >= 2
            v = lastPt - state.waypoints(end-1, :);
            if norm(v) > 1e-6, dir = v / norm(v); end
        end
        
        % Normal vector (Left)
        normal = [-dir(2), dir(1)];
        
        % Generate sine wave points
        % We want to weave around cones positioned at: lastPt + dir * (i * gap)
        % Path should oscillate.
        % First cone at distance 'gap' from lastPt.
        % We go LEFT of first cone? Or Right?
        % Let's assume start going Left.
        
        newPoints = zeros(0, 2);
        
        % We generate Apexes.
        % Cone 1 at dist=gap. Apex 1 is at dist=gap, offset=+amp (Left) ? 
        % Or Apex is between cones?
        % Standard slalom path: Apex is usually AT the longitudinal position of the cone.
        
        for i = 1:nCones
            dist = i * gap;
            
            % Direction of weave: Odd -> Left (+), Even -> Right (-)
            sign = 1;
            if mod(i, 2) == 0, sign = -1; end
            
            pt = lastPt + dir * dist + normal * (sign * amp);
            newPoints(end+1, :) = pt;
        end
        
        % Exit point: return to center?
        finalDist = (nCones + 1) * gap;
        exitPt = lastPt + dir * finalDist;
        newPoints(end+1, :) = exitPt;
        
        % Append
        state.waypoints = [state.waypoints; newPoints];
        state.segmentTypes = [state.segmentTypes; zeros(size(newPoints, 1), 1)]; % Spline default
        
        state.selectedIdx = size(state.waypoints, 1);
        updatePlot();
    end

    function onFillet(~, ~)
        if state.selectedIdx <= 0 || isempty(state.waypoints)
            warndlg('Select a corner point first (Move/Edit mode).');
            return;
        end
        
        idx = state.selectedIdx;
        n = size(state.waypoints, 1);
        
        % Identify Prev and Next points
        if idx == 1
            if state.isClosed, prevIdx = n; else, warndlg('Cannot fillet start point of open track.'); return; end
        else
            prevIdx = idx - 1;
        end
        
        if idx == n
            if state.isClosed, nextIdx = 1; else, warndlg('Cannot fillet end point of open track.'); return; end
        else
            nextIdx = idx + 1;
        end
        
        P_curr = state.waypoints(idx, :);
        P_prev = state.waypoints(prevIdx, :);
        P_next = state.waypoints(nextIdx, :);
        
        R = str2double(state.ui.filletRadius.String);
        if isnan(R) || R <= 0, warndlg('Invalid radius.'); return; end
        
        % Vectors
        v1 = P_curr - P_prev;
        v2 = P_next - P_curr;
        len1 = norm(v1);
        len2 = norm(v2);
        
        if len1 < 1e-3 || len2 < 1e-3, return; end
        
        v1 = v1 / len1;
        v2 = v2 / len2;
        
        % Bisector vector
        bisector = (v1 - v2);
        if norm(bisector) < 1e-6, return; end % Straight
        bisector = bisector / norm(bisector);
        
        halfAngle = acos(dot(-v1, v2)) / 2;
        if abs(halfAngle) < 1e-3, return; end
        
        d_tangent = R / tan(halfAngle);
        
        if d_tangent > len1 || d_tangent > len2
            warndlg(sprintf('Radius too large.\nMax tangent: %.1f, %.1f\nReq: %.1f', len1, len2, d_tangent));
            return;
        end
        
        % Tangent Points
        P_t1 = P_curr - v1 * d_tangent;
        P_t2 = P_curr + v2 * d_tangent;
        
        % Center and Orientation
        crossZ = v1(1)*v2(2) - v1(2)*v2(1);
        isLeftTurn = crossZ > 0;
        
        if isLeftTurn
            n1 = [-v1(2), v1(1)];
        else
            n1 = [v1(2), -v1(1)];
        end
        
        Center = P_t1 + n1 * R;
        
        % Angles
        a1 = atan2(P_t1(2)-Center(2), P_t1(1)-Center(1));
        a2 = atan2(P_t2(2)-Center(2), P_t2(1)-Center(1));
        
        if isLeftTurn
            if a2 < a1, a2 = a2 + 2*pi; end
        else
            if a1 < a2, a1 = a1 + 2*pi; end
        end
        
        % Generate Arc Points
        nArc = ceil(abs(a2-a1) * R / 0.5); 
        if nArc < 3, nArc = 3; end
        
        theta = linspace(a1, a2, nArc);
        theta = theta(2:end-1); 
        
        arcPoints = [Center(1) + R*cos(theta'), Center(2) + R*sin(theta')];
        
        % Replace
        newPoints = [P_t1; arcPoints; P_t2];
        newTypes = zeros(size(newPoints, 1), 1);
        
        state.segmentTypes(prevIdx) = 1; % Force straight enter
        newTypes(end) = 1; % Force straight exit
        
        state.waypoints = [state.waypoints(1:idx-1, :); newPoints; state.waypoints(idx+1:end, :)];
        state.segmentTypes = [state.segmentTypes(1:idx-1); newTypes; state.segmentTypes(idx+1:end)];
        
        state.selectedIdx = idx + size(newPoints,1) - 1;
        updateUIState();
        updatePlot();
    end

    function pts = generateTrackPoints(waypoints, types, closed)
        % Main generation function handling hybrid Spline/Line
        if size(waypoints, 1) < 2
            pts = waypoints;
            return;
        end
        
        pts = zeros(0, 2);
        
        n = size(waypoints, 1);
        numSegments = n;
        if ~closed, numSegments = n - 1; end
        
        for i = 1:numSegments
            pStart = waypoints(i, :);
            idxNext = mod(i, n) + 1;
            pEnd = waypoints(idxNext, :);
            
            segType = types(i); % 0=Spline, 1=Straight
            
            if segType == 1
                % STRAIGHT
                numSub = ceil(norm(pEnd - pStart) / 0.5); 
                t = linspace(0, 1, max(2, numSub));
                segPts = pStart + (pEnd - pStart) .* t(:);
                if i < numSegments
                    segPts = segPts(1:end-1, :); 
                end
                pts = [pts; segPts];
            else
                % SPLINE (Catmull-Rom)
                if i == 1 && ~closed
                    p0 = pStart - (pEnd - pStart);
                else
                    idxPrev = mod(i-2, n) + 1;
                    p0 = waypoints(idxPrev, :);
                end
                
                if idxNext == n && ~closed
                    p3 = pEnd + (pEnd - pStart);
                else
                    idxNext2 = mod(idxNext, n) + 1;
                    p3 = waypoints(idxNext2, :);
                end
                
                p1 = pStart;
                p2 = pEnd;
                
                m1 = 0.5 * (p2 - p0);
                m2 = 0.5 * (p3 - p1);
                
                numSub = ceil(norm(pEnd - pStart) / 0.2); 
                t = linspace(0, 1, max(2, numSub));
                t = t(:);
                
                h00 = 2*t.^3 - 3*t.^2 + 1;
                h10 = t.^3 - 2*t.^2 + t;
                h01 = -2*t.^3 + 3*t.^2;
                h11 = t.^3 - t.^2;
                
                segPts = h00.*p1 + h10.*m1 + h01.*p2 + h11.*m2;
                if i < numSegments
                    segPts = segPts(1:end-1, :);
                end
                pts = [pts; segPts];
            end
        end
        
        if ~closed
            pts = [pts; waypoints(end, :)];
        end
    end

    function updatePlot()
        cla(state.ax);
        
        % Plot generated track
        if size(state.waypoints, 1) >= 2
            trackPts = generateTrackPoints(state.waypoints, state.segmentTypes, state.isClosed);
            plot(state.ax, trackPts(:,1), trackPts(:,2), 'b-', 'LineWidth', 2);
            
            % Boundaries
            dx = gradient(trackPts(:,1));
            dy = gradient(trackPts(:,2));
            len = sqrt(dx.^2 + dy.^2);
            len(len<1e-6) = 1;
            nx = -dy ./ len;
            ny = dx ./ len;
            w = state.trackWidth / 2;
            plot(state.ax, trackPts(:,1)+nx*w, trackPts(:,2)+ny*w, 'k--', 'Color', [0.5 0.5 0.5]);
            plot(state.ax, trackPts(:,1)-nx*w, trackPts(:,2)-ny*w, 'k--', 'Color', [0.5 0.5 0.5]);
        end
        
        % Plot Control Points
        if ~isempty(state.waypoints)
            for i = 1:size(state.waypoints, 1)
                pt = state.waypoints(i, :);
                col = 'r';
                if i == state.selectedIdx, col = 'g'; end
                marker = 'o';
                if i <= length(state.segmentTypes) && state.segmentTypes(i) == 1, marker = 's'; end
                plot(state.ax, pt(1), pt(2), [marker '-'], 'MarkerFaceColor', col, 'Color', 'k', 'MarkerSize', 8);
                text(state.ax, pt(1), pt(2)+0.5, num2str(i), 'FontSize', 8, 'Color', 'k');
            end
            text(state.ax, state.waypoints(1,1), state.waypoints(1,2)-1, ' Start', 'Color', 'blue', 'FontWeight', 'bold');
        end
    end

    function updateUIState()
        idx = state.selectedIdx;
        if idx > 0 && idx <= length(state.segmentTypes)
            type = state.segmentTypes(idx);
            if type == 0
                state.ui.segSpline.Value = 1;
            else
                state.ui.segLine.Value = 1;
            end
        else
            state.ui.segSpline.Value = 1;
        end
    end

    % --- Callbacks ---
    function onModeChange(~, event)
        state.mode = event.NewValue.Tag;
    end

    function onSegTypeChange(~, ~, typeVal)
        if state.selectedIdx > 0
            state.segmentTypes(state.selectedIdx) = typeVal;
            updatePlot();
        end
    end

    function onWidthChange(src, ~)
        val = str2double(src.String);
        if ~isnan(val) && val > 0; state.trackWidth = val; updatePlot(); end
    end

    function onClosedChange(src, ~)
        state.isClosed = src.Value; updatePlot();
    end

    function onClear(~, ~)
        state.waypoints = zeros(0, 2);
        state.segmentTypes = zeros(0,1);
        state.selectedIdx = -1;
        updatePlot();
    end

    function onLoad(~, ~)
        [file, path] = uigetfile('*.csv', 'Load Track CSV');
        if isequal(file, 0), return; end
        try
            data = readmatrix(fullfile(path, file));
            if size(data, 1) > 200
                answer = questdlg('Large file. Import as dense polygon (Straight)?', 'Import', 'Yes', 'No', 'Yes');
                if strcmp(answer, 'Yes')
                    state.waypoints = data(:, 1:2);
                    state.segmentTypes = ones(size(data, 1), 1);
                else
                    state.waypoints = data(:, 1:2);
                    state.segmentTypes = zeros(size(data, 1), 1);
                end
            else
                state.waypoints = data(:, 1:2);
                state.segmentTypes = zeros(size(data, 1), 1);
            end
            if size(data, 2) >= 3; state.trackWidth = data(1, 3); end
            updatePlot();
        catch e, errordlg(e.message); end
    end

    function onSave(~, ~)
        if isempty(state.waypoints), return; end
        [file, path] = uiputfile('*.csv', 'Save Track CSV');
        if isequal(file, 0), return; end
        pts = generateTrackPoints(state.waypoints, state.segmentTypes, state.isClosed);
        outputData = [pts, repmat(state.trackWidth, size(pts,1), 1)];
        writematrix(outputData, fullfile(path, file));
        msgbox(['Track saved: ' file], 'Success');
    end

    function idx = findNearestPoint(x, y)
        if isempty(state.waypoints), idx = 0; return; end
        d = sum((state.waypoints - [x, y]).^2, 2);
        [minD, minIdx] = min(d);
        if minD < 100, idx = minIdx; else, idx = 0; end
    end
end
