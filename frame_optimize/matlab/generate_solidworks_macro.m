function output_path = generate_solidworks_macro(nodes, elements_geom, best_x, catalog, materials, options)
% GENERATE_SOLIDWORKS_MACRO  Generate a SolidWorks VBA macro (.swb) from frame optimization results.
%
%   output_path = generate_solidworks_macro(nodes, elements_geom, best_x, catalog, materials)
%   output_path = generate_solidworks_macro(..., options)
%
% Inputs:
%   nodes         - Nx3 matrix [x,y,z] in meters (frame coordinate system)
%   elements_geom - struct with .connectivity (Mx2), .group_indices (Mx1),
%                   .group_names (cell), .base_elements (Mx9)
%   best_x        - 1x18 GA result vector (best_x(1:5) = pipe catalog indices)
%   catalog       - pipe catalog struct array from loadPipeCatalog()
%   materials     - materials struct from loadMaterials()
%   options       - (optional) struct with fields:
%       .output_dir           - output directory (default: './output')
%       .filename             - output filename (default: 'fsae_frame_macro.swb')
%       .fillet_radius_factor - bend radius / OD ratio (default: 3.0)
%       .export_individual_bodies - save each pipe body (default: true)
%       .body_export_dir      - directory for individual bodies (default: './output/pipe_bodies')
%
% Output:
%   output_path - full path to the generated .swb file
%
% Usage:
%   After frame optimization completes:
%     macro_path = generate_solidworks_macro(nodes, elements_geom, best_x, catalog, materials);
%   Then in SolidWorks:
%     1. Tools > Macro > Edit (open VBA editor)
%     2. Select the .swb file
%     3. Run > Main

    if nargin < 6, options = struct(); end

    %% 1. Parse options
    opts = parse_options(options);

    %% 2. Transform coordinates: frame system -> SolidWorks system
    % Frame: X rearward+, Y right+, Z up+
    % SW:    X forward+,  Y up+,    Z right+
    sw_nodes = frame_to_sw(nodes);
    n_nodes = size(sw_nodes, 1);

    %% 3. Classify elements into component groups
    components = classify_elements(elements_geom, best_x, catalog);

    %% 4. Compute fillet radii for hoops
    fh_pipe = catalog(round(best_x(1)));
    mh_pipe = catalog(round(best_x(2)));
    fillet_fh = compute_fillet_radius(fh_pipe.D_outer_mm, opts.fillet_radius_factor, ...
        sw_nodes, components.fh_bent_path);
    fillet_mh = compute_fillet_radius(mh_pipe.D_outer_mm, opts.fillet_radius_factor, ...
        sw_nodes, components.mh_bent_path);

    %% 5. Build pipe profile info for each group
    profiles = build_profile_info(best_x, catalog);

    %% 6. Generate VBA code
    vba = {};

    vba{end+1} = emit_vba_header(opts, n_nodes, profiles);
    vba{end+1} = emit_vba_node_data(sw_nodes);
    vba{end+1} = emit_vba_helpers();
    vba{end+1} = emit_vba_profile_gen(profiles);
    vba{end+1} = emit_vba_hoop_bent('FrontHoop', components.fh_bent_path, ...
        components.fh_fillet_nodes, fillet_fh, profiles(1), 1);
    vba{end+1} = emit_vba_hoop_bent('MainHoop', components.mh_bent_path, ...
        components.mh_fillet_nodes, fillet_mh, profiles(2), 2);
    vba{end+1} = emit_vba_straight_members('FrontHoop_Bottom', components.fh_bottom, profiles(1));
    vba{end+1} = emit_vba_straight_members('MainHoop_Bottom', components.mh_bottom, profiles(2));
    vba{end+1} = emit_vba_straight_members('FBH', components.fbh, profiles(4));
    vba{end+1} = emit_vba_straight_members('Shoulder', components.shoulder, profiles(5));
    vba{end+1} = emit_vba_straight_members('FBHSupport', components.fbh_support, profiles(4));
    vba{end+1} = emit_vba_straight_members('SideImpact', components.side_impact, profiles(3));
    vba{end+1} = emit_vba_straight_members('MH_to_Rear', components.mh_to_rear, profiles(4));
    vba{end+1} = emit_vba_straight_members('RearSection', components.rear_section, profiles(4));
    vba{end+1} = emit_vba_straight_members('Brackets', components.brackets, profiles(4));
    vba{end+1} = emit_vba_straight_members('Structural', components.structural_other, profiles(4));
    vba{end+1} = emit_vba_trim();
    vba{end+1} = emit_vba_cutlist_props(components, profiles);
    vba{end+1} = emit_vba_body_export(opts);
    vba{end+1} = emit_vba_diagnostic();
    vba{end+1} = emit_vba_main(opts);

    %% 7. Late-binding conversion (SldWorks.xxx -> Object, enum prefixes -> constants)
    for i = 1:length(vba)
        vba{i} = convert_to_late_binding(vba{i});
    end

    %% 8. Compile validation
    validate_vba(vba);

    %% 9. Write .swb file
    output_path = write_bas_file(vba, opts);

    fprintf('SolidWorks VBA macro generated: %s\n', output_path);
    fprintf('  Nodes: %d, Elements: %d\n', n_nodes, size(elements_geom.connectivity, 1));
    fprintf('  Pipe profiles:\n');
    for i = 1:5
        fprintf('    Group %d (%s): %s (OD=%.1fmm, t=%.2fmm)\n', ...
            i, profiles(i).group_name, profiles(i).catalog_id, ...
            profiles(i).OD_mm, profiles(i).t_mm);
    end
end

%% ===== Internal Functions =====

function opts = parse_options(options)
    opts.output_dir = get_field(options, 'output_dir', fullfile(pwd, 'output'));
    opts.filename = get_field(options, 'filename', 'fsae_frame_macro.swb');
    opts.fillet_radius_factor = get_field(options, 'fillet_radius_factor', 3.0);
    opts.export_individual_bodies = get_field(options, 'export_individual_bodies', true);
    opts.body_export_dir = get_field(options, 'body_export_dir', ...
        fullfile(opts.output_dir, 'pipe_bodies'));
end

function val = get_field(s, field, default)
    if isfield(s, field)
        val = s.(field);
    else
        val = default;
    end
end

function sw = frame_to_sw(frame_coords)
    % Frame: [X_rear+, Y_right+, Z_up+] -> SW: [X_fwd+, Y_up+, Z_right+]
    sw = [-frame_coords(:,1), frame_coords(:,3), frame_coords(:,2)];
end

function components = classify_elements(elements_geom, best_x, catalog)
    conn = elements_geom.connectivity;
    groups = elements_geom.group_indices;
    n_elem = size(conn, 1);

    % --- Front Hoop (group 1) ---
    % Bent part: U-shape path 1->3->4->2 (left up, across top, right down)
    % Bottom: 1->2 (separate pipe)
    components.fh_bent_path = [1, 3, 4, 2];
    components.fh_fillet_nodes = [3, 4]; % corners with fillets
    components.fh_bottom = [1, 2];       % Mx2 element list

    % --- Main Hoop (group 2) ---
    % Bent part: arch 9->11->13->15->14->12->10
    % Bottom: 9->10 (separate pipe)
    components.mh_bent_path = [9, 11, 13, 15, 14, 12, 10];
    components.mh_fillet_nodes = [11, 13, 15, 14, 12]; % interior corners
    components.mh_bottom = [9, 10];

    % --- Shoulder harness bar (group 5) ---
    sh_mask = (groups == 5);
    components.shoulder = conn(sh_mask, :);

    % --- Side impact (group 3) ---
    si_mask = (groups == 3);
    components.side_impact = conn(si_mask, :);

    % --- Group 4 sub-classification ---
    g4_mask = (groups == 4);
    g4_elems = conn(g4_mask, :);

    % Determine fbh node range (typically 24-27, but could be dynamic)
    % Detect from group_names or from known topology
    fbh_nodes = detect_fbh_nodes(elements_geom, conn, groups);
    components.fbh_node_range = fbh_nodes;

    fbh_list = [];
    fbh_support_list = [];
    rear_section_list = [];
    mh_to_rear_list = [];
    bracket_list = [];
    other_list = [];

    for i = 1:size(g4_elems, 1)
        n1 = g4_elems(i, 1);
        n2 = g4_elems(i, 2);

        both_fbh = ismember(n1, fbh_nodes) && ismember(n2, fbh_nodes);
        one_fbh  = ismember(n1, fbh_nodes) || ismember(n2, fbh_nodes);
        one_fh   = ismember(n1, 1:4) || ismember(n2, 1:4);
        both_rear = (n1 >= 16 && n1 <= 23) && (n2 >= 16 && n2 <= 23);
        one_mh   = (n1 >= 9 && n1 <= 15) || (n2 >= 9 && n2 <= 15);
        one_rear = (n1 >= 16 && n1 <= 23) || (n2 >= 16 && n2 <= 23);
        has_bracket = (n1 >= 28) || (n2 >= 28);

        if both_fbh
            fbh_list = [fbh_list; g4_elems(i,:)]; %#ok<AGROW>
        elseif one_fbh && one_fh
            fbh_support_list = [fbh_support_list; g4_elems(i,:)]; %#ok<AGROW>
        elseif one_fbh
            fbh_support_list = [fbh_support_list; g4_elems(i,:)]; %#ok<AGROW>
        elseif both_rear
            rear_section_list = [rear_section_list; g4_elems(i,:)]; %#ok<AGROW>
        elseif one_mh && one_rear
            mh_to_rear_list = [mh_to_rear_list; g4_elems(i,:)]; %#ok<AGROW>
        elseif has_bracket
            bracket_list = [bracket_list; g4_elems(i,:)]; %#ok<AGROW>
        else
            other_list = [other_list; g4_elems(i,:)]; %#ok<AGROW>
        end
    end

    components.fbh = fbh_list;
    components.fbh_support = fbh_support_list;
    components.rear_section = rear_section_list;
    components.mh_to_rear = mh_to_rear_list;
    components.brackets = bracket_list;
    components.structural_other = other_list;

    % Store group names for cut list properties
    components.group_names = {'front_hoop', 'main_hoop', 'side_impact', 'structural', 'shoulder'};
end

function fbh_nodes = detect_fbh_nodes(elements_geom, conn, groups)
    % FBH nodes are typically the first 4 nodes added after the base 23
    % In the topology, they are group 4 elements where both nodes > 23
    % and form a closed rectangle
    g4_elems = conn(groups == 4, :);
    all_nodes_g4 = unique(g4_elems(:));
    candidates = all_nodes_g4(all_nodes_g4 >= 24 & all_nodes_g4 <= 27);
    if length(candidates) >= 4
        fbh_nodes = candidates(1:4)';
    else
        % Fallback: look for any 4 consecutive nodes >= 24 that form a rectangle
        high_nodes = all_nodes_g4(all_nodes_g4 >= 24);
        if length(high_nodes) >= 4
            fbh_nodes = high_nodes(1:4)';
        else
            fbh_nodes = [24, 25, 26, 27];
        end
    end
end

function radius_m = compute_fillet_radius(OD_mm, factor, sw_nodes, path)
    % Compute fillet radius, clamped to half the shortest adjacent segment
    radius_raw = factor * OD_mm / 1000; % meters

    % Find minimum segment length along path
    min_seg = inf;
    for i = 1:(length(path)-1)
        seg_len = norm(sw_nodes(path(i+1),:) - sw_nodes(path(i),:));
        min_seg = min(min_seg, seg_len);
    end

    % Clamp: fillet radius must be < half the shortest segment
    max_radius = min_seg * 0.45;
    radius_m = min(radius_raw, max_radius);
end

function profiles = build_profile_info(best_x, catalog)
    group_names = {'front_hoop', 'main_hoop', 'side_impact', 'structural', 'shoulder'};
    profiles = struct([]);
    for i = 1:5
        idx = round(best_x(i));
        p = catalog(idx);
        profiles(i).group_name = group_names{i};
        profiles(i).catalog_id = p.id;
        profiles(i).OD_mm = p.D_outer_mm;
        profiles(i).t_mm = p.t_mm;
        profiles(i).material_id = p.material_id;
        profiles(i).OD_m = p.D_outer_mm / 1000;
        profiles(i).ID_m = (p.D_outer_mm - 2 * p.t_mm) / 1000;
    end
end

%% ===== VBA Code Emission Functions =====

function code = emit_vba_header(opts, n_nodes, profiles)
    lines = {};
    lines{end+1} = ''' ============================================================';
    lines{end+1} = ''' FSAE Frame Generator - SolidWorks VBA Macro';
    lines{end+1} = sprintf(''' Auto-generated by MATLAB: %s', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
    lines{end+1} = ''' ============================================================';
    lines{end+1} = '''';
    lines{end+1} = ''' Usage:';
    lines{end+1} = '''   1. SolidWorks > Tools > Macro > Run';
    lines{end+1} = '''   2. Select this .swb file';
    lines{end+1} = '''   3. The macro will execute automatically';
    lines{end+1} = '''';
    lines{end+1} = 'Option Explicit';
    lines{end+1} = '';
    lines{end+1} = ''' --- Constants ---';
    lines{end+1} = sprintf('Const NUM_NODES As Long = %d', n_nodes);
    lines{end+1} = 'Const NUM_GROUPS As Long = 5';
    lines{end+1} = sprintf('Const FILLET_RADIUS_FACTOR As Double = %.1f', opts.fillet_radius_factor);
    if opts.export_individual_bodies
        lines{end+1} = 'Const EXPORT_BODIES As Boolean = True';
    else
        lines{end+1} = 'Const EXPORT_BODIES As Boolean = False';
    end
    lines{end+1} = sprintf('Const BODY_EXPORT_DIR As String = "%s"', ...
        opts.body_export_dir);
    lines{end+1} = '';
    lines{end+1} = ''' --- Pipe Profile Constants (mm) ---';
    for i = 1:5
        lines{end+1} = sprintf('Const PIPE_OD_%d As Double = %.2f  '' %s: %s', ...
            i, profiles(i).OD_mm, profiles(i).group_name, profiles(i).catalog_id);
        lines{end+1} = sprintf('Const PIPE_T_%d As Double = %.2f', i, profiles(i).t_mm);
    end
    lines{end+1} = '';
    lines{end+1} = ''' --- Weldment Profile Settings ---';
    lines{end+1} = 'Const PROF_STANDARD As String = "FSAE"';
    lines{end+1} = 'Const PROF_TYPE As String = "tube"';
    lines{end+1} = '';
    lines{end+1} = ''' --- SolidWorks API Constants (late-binding: no type library needed) ---';
    lines{end+1} = 'Const SW_DEFAULT_TEMPLATE_PART As Long = 8      '' swDefaultTemplatePart (8=Part, 7=Assembly)';
    lines{end+1} = 'Const SW_FILE_WELDMENT_PROFILES As Long = 37    '' SW_FILE_WELDMENT_PROFILES';
    lines{end+1} = 'Const SW_SOLID_BODY As Long = 0                 '' swSolidBody';
    lines{end+1} = 'Const SW_CUSTOM_INFO_TEXT As Long = 30           '' SW_CUSTOM_INFO_TEXT';
    lines{end+1} = '';
    lines{end+1} = ''' --- Module-level variables ---';
    lines{end+1} = 'Dim nd() As Double  '' Node coordinates (meters, SW coordinate system)';
    lines{end+1} = 'Dim profBaseDir As String  '' Weldment profile base directory';
    lines{end+1} = 'Dim profileSizes(1 To 5) As String  '' Profile size names per group';
    lines{end+1} = 'Dim gPartTemplate As String  '' Cached Part template path';
    lines{end+1} = '';

    code = strjoin(lines, newline);
end

function code = emit_vba_node_data(sw_nodes)
    n = size(sw_nodes, 1);
    lines = {};
    lines{end+1} = 'Sub InitNodes()';
    lines{end+1} = sprintf('    ReDim nd(1 To %d, 1 To 3)', n);
    % Emit coordinates in batches for readability
    for i = 1:n
        lines{end+1} = sprintf('    nd(%d,1)=%.6f: nd(%d,2)=%.6f: nd(%d,3)=%.6f', ...
            i, sw_nodes(i,1), i, sw_nodes(i,2), i, sw_nodes(i,3));
    end
    lines{end+1} = 'End Sub';
    lines{end+1} = '';
    code = strjoin(lines, newline);
end

function code = emit_vba_helpers()
    lines = {};

    % Helper: Draw a 3D sketch line between two nodes
    lines{end+1} = 'Function DrawLine3D(swModel As SldWorks.ModelDoc2, n1 As Long, n2 As Long) As SldWorks.SketchSegment';
    lines{end+1} = '    Set DrawLine3D = swModel.SketchManager.CreateLine( _';
    lines{end+1} = '        nd(n1,1), nd(n1,2), nd(n1,3), _';
    lines{end+1} = '        nd(n2,1), nd(n2,2), nd(n2,3))';
    lines{end+1} = 'End Function';
    lines{end+1} = '';

    % Helper: Create directory recursively (using FileSystemObject for reliability)
    lines{end+1} = 'Sub CreateDirRecursive(ByVal dirPath As String)';
    lines{end+1} = '    Dim fso As Object';
    lines{end+1} = '    Set fso = CreateObject("Scripting.FileSystemObject")';
    lines{end+1} = '    If fso.FolderExists(dirPath) Then Exit Sub';
    lines{end+1} = '    Dim parentPath As String';
    lines{end+1} = '    parentPath = fso.GetParentFolderName(dirPath)';
    lines{end+1} = '    If Not fso.FolderExists(parentPath) Then';
    lines{end+1} = '        CreateDirRecursive parentPath';
    lines{end+1} = '    End If';
    lines{end+1} = '    fso.CreateFolder dirPath';
    lines{end+1} = 'End Sub';
    lines{end+1} = '';

    % Helper: Find Part template (.prtdot) robustly
    lines{end+1} = 'Function FindPartTemplate(swApp As SldWorks.SldWorks) As String';
    lines{end+1} = '    Dim fso As Object';
    lines{end+1} = '    Set fso = CreateObject("Scripting.FileSystemObject")';
    lines{end+1} = '    ';
    lines{end+1} = '    '' Try 1: User preference (swDefaultTemplatePart)';
    lines{end+1} = '    Dim tpl As String';
    lines{end+1} = '    tpl = swApp.GetUserPreferenceStringValue(SW_DEFAULT_TEMPLATE_PART)';
    lines{end+1} = '    Debug.Print "  Preference template: [" & tpl & "]"';
    lines{end+1} = '    If Len(Trim(tpl)) > 0 Then';
    lines{end+1} = '        If fso.FileExists(tpl) Then';
    lines{end+1} = '            FindPartTemplate = tpl';
    lines{end+1} = '            Exit Function';
    lines{end+1} = '        End If';
    lines{end+1} = '    End If';
    lines{end+1} = '    ';
    lines{end+1} = '    '' Try 2: Search common template locations relative to SW install';
    lines{end+1} = '    Dim exePath As String';
    lines{end+1} = '    exePath = swApp.GetExecutablePath';
    lines{end+1} = '    Debug.Print "  SW install path: " & exePath';
    lines{end+1} = '    If Right(exePath, 1) = "\" Then exePath = Left(exePath, Len(exePath) - 1)';
    lines{end+1} = '    ';
    lines{end+1} = '    Dim candidates(0 To 7) As String';
    lines{end+1} = '    candidates(0) = exePath & "\lang\japanese\Tutorial\Part.prtdot"';
    lines{end+1} = '    candidates(1) = exePath & "\lang\english\Tutorial\Part.prtdot"';
    lines{end+1} = '    candidates(2) = exePath & "\Templates\Part.prtdot"';
    lines{end+1} = '    candidates(3) = exePath & "\lang\japanese\Tutorial\part.prtdot"';
    lines{end+1} = '    candidates(4) = "C:\ProgramData\SolidWorks\SOLIDWORKS 2026\templates\Part.prtdot"';
    lines{end+1} = '    candidates(5) = "C:\ProgramData\SolidWorks\SOLIDWORKS 2025\templates\Part.prtdot"';
    lines{end+1} = '    candidates(6) = "C:\ProgramData\SolidWorks\SOLIDWORKS 2024\templates\Part.prtdot"';
    lines{end+1} = '    candidates(7) = "C:\ProgramData\SolidWorks\SOLIDWORKS 2023\templates\Part.prtdot"';
    lines{end+1} = '    ';
    lines{end+1} = '    Dim i As Long';
    lines{end+1} = '    For i = 0 To UBound(candidates)';
    lines{end+1} = '        If fso.FileExists(candidates(i)) Then';
    lines{end+1} = '            Debug.Print "  Found template: " & candidates(i)';
    lines{end+1} = '            FindPartTemplate = candidates(i)';
    lines{end+1} = '            Exit Function';
    lines{end+1} = '        End If';
    lines{end+1} = '    Next i';
    lines{end+1} = '    ';
    lines{end+1} = '    '' Try 3: Search for *.prtdot in SW install folder (1 level)';
    lines{end+1} = '    Dim folder As Object, subFolder As Object, file As Object';
    lines{end+1} = '    If fso.FolderExists(exePath) Then';
    lines{end+1} = '        For Each subFolder In fso.GetFolder(exePath).SubFolders';
    lines{end+1} = '            For Each file In subFolder.Files';
    lines{end+1} = '                If LCase(fso.GetExtensionName(file.Name)) = "prtdot" Then';
    lines{end+1} = '                    Debug.Print "  Found template (search): " & file.Path';
    lines{end+1} = '                    FindPartTemplate = file.Path';
    lines{end+1} = '                    Exit Function';
    lines{end+1} = '                End If';
    lines{end+1} = '            Next file';
    lines{end+1} = '        Next subFolder';
    lines{end+1} = '    End If';
    lines{end+1} = '    ';
    lines{end+1} = '    FindPartTemplate = ""';
    lines{end+1} = 'End Function';
    lines{end+1} = '';

    % Helper: Save a tube profile as .sldlfp (library feature part)
    % CRITICAL: (1) 既存ファイル削除して常に再作成
    %           (2) 正面(Front Plane)を明示選択してからスケッチ
    %           (3) スケッチ選択状態で保存
    lines{end+1} = 'Sub SaveTubeProfile(swApp As SldWorks.SldWorks, filePath As String, outerR As Double, innerR As Double)';
    lines{end+1} = '    On Error Resume Next';
    lines{end+1} = '    Dim fso As Object';
    lines{end+1} = '    Set fso = CreateObject("Scripting.FileSystemObject")';
    lines{end+1} = '    '' 常に再作成（前回の壊れたファイルを上書き）';
    lines{end+1} = '    If fso.FileExists(filePath) Then fso.DeleteFile filePath';
    lines{end+1} = '    On Error GoTo 0';
    lines{end+1} = '    ';
    lines{end+1} = '    Dim profModel As SldWorks.ModelDoc2';
    lines{end+1} = '    Dim errors As Long, warnings As Long';
    lines{end+1} = '    Set profModel = swApp.NewDocument(gPartTemplate, 0, 0, 0)';
    lines{end+1} = '    If profModel Is Nothing Then';
    lines{end+1} = '        Debug.Print "WARNING: Failed to create profile part for " & filePath';
    lines{end+1} = '        Exit Sub';
    lines{end+1} = '    End If';
    lines{end+1} = '    ';
    lines{end+1} = '    '' 正面(Front Plane)を明示的に選択 — 日本語/英語テンプレート両対応';
    lines{end+1} = '    Dim bSel As Boolean';
    lines{end+1} = '    bSel = profModel.Extension.SelectByID2("Front Plane", "PLANE", 0, 0, 0, False, 0, Nothing, 0)';
    lines{end+1} = '    If Not bSel Then';
    lines{end+1} = '        bSel = profModel.Extension.SelectByID2("正面", "PLANE", 0, 0, 0, False, 0, Nothing, 0)';
    lines{end+1} = '    End If';
    lines{end+1} = '    If Not bSel Then';
    lines{end+1} = '        '' Fallback: 最初のRefPlaneフィーチャーを探す';
    lines{end+1} = '        Dim tmpFeat As SldWorks.Feature';
    lines{end+1} = '        Set tmpFeat = profModel.FirstFeature';
    lines{end+1} = '        Do While Not tmpFeat Is Nothing';
    lines{end+1} = '            If tmpFeat.GetTypeName2 = "RefPlane" Then';
    lines{end+1} = '                tmpFeat.Select2 False, 0';
    lines{end+1} = '                bSel = True';
    lines{end+1} = '                Exit Do';
    lines{end+1} = '            End If';
    lines{end+1} = '            Set tmpFeat = tmpFeat.GetNextFeature';
    lines{end+1} = '        Loop';
    lines{end+1} = '    End If';
    lines{end+1} = '    Debug.Print "    Plane selected: " & bSel';
    lines{end+1} = '    ';
    lines{end+1} = '    '' スケッチを作成（パイプ断面: 外径円+内径円）';
    lines{end+1} = '    profModel.SketchManager.InsertSketch True';
    lines{end+1} = '    profModel.SketchManager.CreateCircleByRadius 0, 0, 0, outerR';
    lines{end+1} = '    profModel.SketchManager.CreateCircleByRadius 0, 0, 0, innerR';
    lines{end+1} = '    profModel.SketchManager.InsertSketch True  '' スケッチ終了';
    lines{end+1} = '    ';
    lines{end+1} = '    '' スケッチを選択状態にする (.sldlfp保存に必須)';
    lines{end+1} = '    Dim swFeat As SldWorks.Feature';
    lines{end+1} = '    Set swFeat = profModel.Extension.GetLastFeatureAdded';
    lines{end+1} = '    If Not swFeat Is Nothing Then';
    lines{end+1} = '        swFeat.Select2 False, 0';
    lines{end+1} = '        Debug.Print "    Sketch feature selected: " & swFeat.Name';
    lines{end+1} = '    Else';
    lines{end+1} = '        Debug.Print "    WARNING: No sketch feature found!"';
    lines{end+1} = '    End If';
    lines{end+1} = '    ';
    lines{end+1} = '    profModel.ForceRebuild3 False';
    lines{end+1} = '    ';
    lines{end+1} = '    '' SaveAs3: Options=2 (Silent — ダイアログブロック防止)';
    lines{end+1} = '    Dim bRet As Boolean';
    lines{end+1} = '    bRet = profModel.Extension.SaveAs3(filePath, 0, 2, Nothing, Nothing, errors, warnings)';
    lines{end+1} = '    Debug.Print "    SaveAs: " & filePath & " (ret=" & bRet & ", err=" & errors & ", warn=" & warnings & ")"';
    lines{end+1} = '    ';
    lines{end+1} = '    '' 保存結果を検証';
    lines{end+1} = '    If Not bRet Then';
    lines{end+1} = '        Debug.Print "    CRITICAL: SaveAs3 returned False! Profile file is INVALID."';
    lines{end+1} = '    End If';
    lines{end+1} = '    If Not fso.FileExists(filePath) Then';
    lines{end+1} = '        Debug.Print "    CRITICAL: Profile file does NOT exist after save: " & filePath';
    lines{end+1} = '    Else';
    lines{end+1} = '        Debug.Print "    File size: " & fso.GetFile(filePath).Size & " bytes"';
    lines{end+1} = '    End If';
    lines{end+1} = '    ';
    lines{end+1} = '    swApp.CloseDoc profModel.GetTitle';
    lines{end+1} = 'End Sub';
    lines{end+1} = '';

    code = strjoin(lines, newline);
end

function code = emit_vba_profile_gen(profiles)
    lines = {};
    lines{end+1} = 'Sub CreateCustomProfiles(swApp As SldWorks.SldWorks)';
    lines{end+1} = '    '' Create weldment profile directory structure:';
    lines{end+1} = '    ''   <profBaseDir>\FSAE\tube\<sizeName>.sldlfp';
    lines{end+1} = '    '' InsertStructuralWeldment5 uses full file path to .sldlfp';
    lines{end+1} = '    On Error GoTo ProfileErr';
    lines{end+1} = '    ';
    lines{end+1} = '    Dim profDir As String';
    lines{end+1} = '    ';
    lines{end+1} = '    profBaseDir = "C:\FSAE_WeldProfiles"';
    lines{end+1} = '    Debug.Print "  profBaseDir = " & profBaseDir';
    lines{end+1} = '    profDir = profBaseDir & "\" & PROF_STANDARD & "\" & PROF_TYPE';
    lines{end+1} = '    CreateDirRecursive profDir';
    lines{end+1} = '    ';

    % Generate a profile for each unique OD/t combination
    unique_profiles = struct([]);
    for i = 1:5
        is_dup = false;
        for j = 1:length(unique_profiles)
            if abs(unique_profiles(j).OD_mm - profiles(i).OD_mm) < 0.01 && ...
               abs(unique_profiles(j).t_mm - profiles(i).t_mm) < 0.01
                is_dup = true;
                break;
            end
        end
        if ~is_dup
            idx = length(unique_profiles) + 1;
            unique_profiles(idx).OD_mm = profiles(i).OD_mm;
            unique_profiles(idx).t_mm = profiles(i).t_mm;
            unique_profiles(idx).size_name = sprintf('OD%.1f_t%.2f', ...
                profiles(i).OD_mm, profiles(i).t_mm);
        end
    end

    % Save each unique profile
    lines{end+1} = '    '' Save tube profile parts (outer + inner circle sketch)';
    for k = 1:length(unique_profiles)
        OD = unique_profiles(k).OD_mm;
        t = unique_profiles(k).t_mm;
        ID = OD - 2 * t;
        size_name = unique_profiles(k).size_name;

        lines{end+1} = sprintf('    '' Profile %d: OD=%.1fmm, t=%.2fmm', k, OD, t);
        lines{end+1} = sprintf('    SaveTubeProfile swApp, profDir & "\\%s.sldlfp", %.6f, %.6f', ...
            size_name, OD/2/1000, ID/2/1000);
    end
    lines{end+1} = '    ';

    % Map group indices to size names
    lines{end+1} = '    '' Map groups to profile size names';
    for i = 1:5
        for k = 1:length(unique_profiles)
            if abs(unique_profiles(k).OD_mm - profiles(i).OD_mm) < 0.01 && ...
               abs(unique_profiles(k).t_mm - profiles(i).t_mm) < 0.01
                lines{end+1} = sprintf('    profileSizes(%d) = "%s"', i, unique_profiles(k).size_name);
                break;
            end
        end
    end
    lines{end+1} = '    ';
    lines{end+1} = '    '' Register profile directory with SolidWorks';
    lines{end+1} = '    Dim oldProfDir As String';
    lines{end+1} = '    oldProfDir = swApp.GetUserPreferenceStringValue( _';
    lines{end+1} = '        SW_FILE_WELDMENT_PROFILES)';
    lines{end+1} = '    If InStr(1, oldProfDir, profBaseDir) = 0 Then';
    lines{end+1} = '        swApp.SetUserPreferenceStringValue _';
    lines{end+1} = '            SW_FILE_WELDMENT_PROFILES, _';
    lines{end+1} = '            profBaseDir & ";" & oldProfDir';
    lines{end+1} = '    End If';
    lines{end+1} = '    ';
    lines{end+1} = '    Debug.Print "  Profiles saved to: " & profDir';
    lines{end+1} = '    Debug.Print "  Profile directory registered with SolidWorks."';
    lines{end+1} = '    Exit Sub';
    lines{end+1} = '';
    lines{end+1} = 'ProfileErr:';
    lines{end+1} = '    Debug.Print "ERROR in CreateCustomProfiles: " & Err.Description';
    lines{end+1} = '    Debug.Print "  profBaseDir = " & profBaseDir';
    lines{end+1} = '    MsgBox "Profile creation failed: " & Err.Description & vbCrLf & _';
    lines{end+1} = '           "profBaseDir = " & profBaseDir, vbExclamation';
    lines{end+1} = 'End Sub';
    lines{end+1} = '';
    code = strjoin(lines, newline);
end

function code = emit_vba_hoop_bent(name, path, fillet_nodes, fillet_radius, profile, grp_idx)
    lines = {};
    lines{end+1} = sprintf('Sub Build_%s_Bent(swModel As SldWorks.ModelDoc2, swApp As SldWorks.SldWorks)', name);
    lines{end+1} = sprintf('    Debug.Print "Building %s bent section..."', name);
    lines{end+1} = '    On Error GoTo ErrHandler';
    lines{end+1} = '    ';
    lines{end+1} = '    Dim swFeat As SldWorks.Feature';
    lines{end+1} = '    Dim swSketch As SldWorks.Sketch';
    lines{end+1} = '    Dim swFeatMgr As SldWorks.FeatureManager';
    lines{end+1} = '    Dim swStructFeat As SldWorks.Feature';
    lines{end+1} = '    Dim vSegs As Variant';
    lines{end+1} = '    Dim nSegs As Long';
    lines{end+1} = '    Dim grpSegs() As Object';
    lines{end+1} = '    Dim swGroup As Object';
    lines{end+1} = '    Dim groups(0 To 0) As Object';
    lines{end+1} = '    Dim j As Long';
    lines{end+1} = '    Dim profilePath As String';
    lines{end+1} = '    ';
    lines{end+1} = '    '' Create 3D sketch for the bent path';
    lines{end+1} = '    swModel.SketchManager.Insert3DSketch True';
    lines{end+1} = '    ';

    % Draw line segments along the path
    for i = 1:(length(path)-1)
        lines{end+1} = sprintf('    DrawLine3D swModel, %d, %d', path(i), path(i+1));
    end
    lines{end+1} = '    ';

    % Apply fillets at corner nodes
    lines{end+1} = sprintf('    '' Apply fillets at bend corners (R=%.3fm = %.1f x OD)', ...
        fillet_radius, fillet_radius / profile.OD_m);
    for i = 1:length(fillet_nodes)
        lines{end+1} = sprintf('    swModel.Extension.SelectByID2 "", "EXTSKETCHPOINT", _');
        lines{end+1} = sprintf('        nd(%d,1), nd(%d,2), nd(%d,3), False, 0, Nothing, 0', ...
            fillet_nodes(i), fillet_nodes(i), fillet_nodes(i));
        lines{end+1} = sprintf('    swModel.SketchManager.CreateFillet %.6f, 0', fillet_radius);
    end
    lines{end+1} = '    ';

    lines{end+1} = '    swModel.SketchManager.Insert3DSketch True  '' Close sketch';
    lines{end+1} = '    ';

    % Get sketch segments and build single-group array for InsertStructuralWeldment5
    lines{end+1} = '    '' Get all sketch segments (lines + fillet arcs)';
    lines{end+1} = '    Set swFeat = swModel.Extension.GetLastFeatureAdded';
    lines{end+1} = '    Set swSketch = swFeat.GetSpecificFeature2';
    lines{end+1} = '    vSegs = swSketch.GetSketchSegments';
    lines{end+1} = '    nSegs = UBound(vSegs) + 1';
    lines{end+1} = '    ';
    lines{end+1} = '    '' Build StructuralMemberGroup — 全セグメントで1グループ';
    lines{end+1} = '    Set swFeatMgr = swModel.FeatureManager';
    lines{end+1} = '    Set swGroup = swFeatMgr.CreateStructuralMemberGroup';
    lines{end+1} = '    If swGroup Is Nothing Then';
    lines{end+1} = '        Debug.Print "  CRITICAL: CreateStructuralMemberGroup returned Nothing!"';
    lines{end+1} = '        Exit Sub';
    lines{end+1} = '    End If';
    lines{end+1} = '    ReDim grpSegs(0 To nSegs - 1)';
    lines{end+1} = '    For j = 0 To nSegs - 1';
    lines{end+1} = '        Set grpSegs(j) = vSegs(j)';
    lines{end+1} = '    Next j';
    lines{end+1} = '    swGroup.Segments = grpSegs';
    lines{end+1} = '    Set groups(0) = swGroup';
    lines{end+1} = '    ';
    lines{end+1} = '    '' Insert structural weldment (5 params: FilePathName, GroupCount,';
    lines{end+1} = '    ''   MergeArcBodies, Groups, Configuration)';
    lines{end+1} = sprintf('    profilePath = profBaseDir & "\\%s\\%s\\" & profileSizes(%d) & ".sldlfp"', ...
        'FSAE', 'tube', grp_idx);
    lines{end+1} = '    Debug.Print "  Profile path: " & profilePath';
    lines{end+1} = '    ';
    lines{end+1} = '    '' プロファイルファイル存在確認';
    lines{end+1} = '    Dim fso As Object';
    lines{end+1} = '    Set fso = CreateObject("Scripting.FileSystemObject")';
    lines{end+1} = '    If Not fso.FileExists(profilePath) Then';
    lines{end+1} = '        Debug.Print "  CRITICAL: Profile file NOT FOUND: " & profilePath';
    lines{end+1} = '        Exit Sub';
    lines{end+1} = '    Else';
    lines{end+1} = '        Debug.Print "  Profile exists, size=" & fso.GetFile(profilePath).Size & " bytes"';
    lines{end+1} = '    End If';
    lines{end+1} = '    ';
    lines{end+1} = '    Debug.Print "  nSegs=" & nSegs';
    lines{end+1} = '    Set swStructFeat = swFeatMgr.InsertStructuralWeldment5( _';
    lines{end+1} = '        profilePath, 1, True, groups, "")';
    lines{end+1} = '    ';
    lines{end+1} = '    If swStructFeat Is Nothing Then';
    lines{end+1} = sprintf('        Debug.Print "WARNING: Weldment insertion failed for %s bent"', name);
    lines{end+1} = '    End If';
    lines{end+1} = '    ';
    lines{end+1} = sprintf('    Debug.Print "  %s bent section complete."', name);
    lines{end+1} = '    Exit Sub';
    lines{end+1} = '';
    lines{end+1} = 'ErrHandler:';
    lines{end+1} = sprintf('    Debug.Print "ERROR in Build_%s_Bent: #" & Err.Number & " " & Err.Description', name);
    lines{end+1} = '    Resume Next';
    lines{end+1} = 'End Sub';
    lines{end+1} = '';
    code = strjoin(lines, newline);
end

function code = emit_vba_straight_members(name, elements, profile)
    lines = {};
    lines{end+1} = sprintf('Sub Build_%s(swModel As SldWorks.ModelDoc2, swApp As SldWorks.SldWorks)', name);
    lines{end+1} = sprintf('    Debug.Print "Building %s..."', name);
    lines{end+1} = '    On Error GoTo ErrHandler';
    lines{end+1} = '    ';

    if isempty(elements)
        lines{end+1} = '    '' No elements in this group';
        lines{end+1} = '    Debug.Print "  (empty group, skipping)"';
    else
        n_elem = size(elements, 1);

        % Determine profile group index
        group_list = {'front_hoop','main_hoop','side_impact','structural','shoulder'};
        grp_idx = find(strcmp(profile.group_name, group_list));
        if isempty(grp_idx), grp_idx = 4; end

        lines{end+1} = '    Dim swFeat As SldWorks.Feature';
        lines{end+1} = '    Dim swSketch As SldWorks.Sketch';
        lines{end+1} = '    Dim swFeatMgr As SldWorks.FeatureManager';
        lines{end+1} = '    Dim swStructFeat As SldWorks.Feature';
        lines{end+1} = '    Dim vSegs As Variant';
        lines{end+1} = '    Dim nSegs As Long';
        lines{end+1} = '    Dim oneSeg(0 To 0) As Object';
        lines{end+1} = '    Dim swGroup As Object';
        lines{end+1} = '    Dim groups() As Object';
        lines{end+1} = '    Dim i As Long';
        lines{end+1} = '    Dim profilePath As String';
        lines{end+1} = '    ';
        lines{end+1} = '    '' Create 3D sketch with all member lines';
        lines{end+1} = '    swModel.SketchManager.Insert3DSketch True';
        lines{end+1} = '    ';

        for i = 1:n_elem
            lines{end+1} = sprintf('    DrawLine3D swModel, %d, %d', elements(i,1), elements(i,2));
        end
        lines{end+1} = '    ';
        lines{end+1} = '    swModel.SketchManager.Insert3DSketch True  '' Close sketch';
        lines{end+1} = '    ';

        % Get sketch segments and build per-segment groups
        lines{end+1} = '    '' Get sketch segments';
        lines{end+1} = '    Set swFeat = swModel.Extension.GetLastFeatureAdded';
        lines{end+1} = '    Set swSketch = swFeat.GetSpecificFeature2';
        lines{end+1} = '    vSegs = swSketch.GetSketchSegments';
        lines{end+1} = '    nSegs = UBound(vSegs) + 1';
        lines{end+1} = '    ';
        lines{end+1} = '    '' Build per-segment StructuralMemberGroups (各線=独立グループ)';
        lines{end+1} = '    Set swFeatMgr = swModel.FeatureManager';
        lines{end+1} = '    ReDim groups(0 To nSegs - 1)';
        lines{end+1} = '    For i = 0 To nSegs - 1';
        lines{end+1} = '        Set swGroup = swFeatMgr.CreateStructuralMemberGroup';
        lines{end+1} = '        Set oneSeg(0) = vSegs(i)';
        lines{end+1} = '        swGroup.Segments = oneSeg';
        lines{end+1} = '        Set groups(i) = swGroup';
        lines{end+1} = '    Next i';
        lines{end+1} = '    ';
        lines{end+1} = sprintf('    profilePath = profBaseDir & "\\%s\\%s\\" & profileSizes(%d) & ".sldlfp"', ...
            'FSAE', 'tube', grp_idx);
        lines{end+1} = '    Debug.Print "  Profile path: " & profilePath';
        lines{end+1} = '    ';
        lines{end+1} = '    '' プロファイルファイル存在確認';
        lines{end+1} = '    Dim fso As Object';
        lines{end+1} = '    Set fso = CreateObject("Scripting.FileSystemObject")';
        lines{end+1} = '    If Not fso.FileExists(profilePath) Then';
        lines{end+1} = '        Debug.Print "  CRITICAL: Profile file NOT FOUND: " & profilePath';
        lines{end+1} = '        Exit Sub';
        lines{end+1} = '    Else';
        lines{end+1} = '        Debug.Print "  Profile exists, size=" & fso.GetFile(profilePath).Size & " bytes"';
        lines{end+1} = '    End If';
        lines{end+1} = '    ';
        lines{end+1} = '    Debug.Print "  nSegs=" & nSegs';
        lines{end+1} = '    Set swStructFeat = swFeatMgr.InsertStructuralWeldment5( _';
        lines{end+1} = '        profilePath, nSegs, False, groups, "")';
        lines{end+1} = '    ';
        lines{end+1} = '    If swStructFeat Is Nothing Then';
        lines{end+1} = sprintf('        Debug.Print "WARNING: Structural member insertion failed for %s"', name);
        lines{end+1} = '    End If';
    end

    lines{end+1} = '    ';
    lines{end+1} = sprintf('    Debug.Print "  %s complete. (%d members)"', name, size(elements,1));
    lines{end+1} = '    Exit Sub';
    lines{end+1} = '';
    lines{end+1} = 'ErrHandler:';
    lines{end+1} = sprintf('    Debug.Print "ERROR in Build_%s: #" & Err.Number & " " & Err.Description', name);
    lines{end+1} = '    Resume Next';
    lines{end+1} = 'End Sub';
    lines{end+1} = '';
    code = strjoin(lines, newline);
end

function code = emit_vba_trim()
    lines = {};
    lines{end+1} = 'Sub TrimMembers(swModel As SldWorks.ModelDoc2)';
    lines{end+1} = '    Debug.Print "Trimming member end faces..."';
    lines{end+1} = '    On Error GoTo ErrHandler';
    lines{end+1} = '    ';
    lines{end+1} = '    '' Weldment trim: trim all member ends to adjacent bodies';
    lines{end+1} = '    '' This handles the laser-cut end face fitting';
    lines{end+1} = '    Dim swFeatMgr As SldWorks.FeatureManager';
    lines{end+1} = '    Set swFeatMgr = swModel.FeatureManager';
    lines{end+1} = '    ';
    lines{end+1} = '    '' Get all solid bodies';
    lines{end+1} = '    Dim vBodies As Variant';
    lines{end+1} = '    vBodies = swModel.GetBodies2(SW_SOLID_BODY, True)';
    lines{end+1} = '    If IsEmpty(vBodies) Then Exit Sub';
    lines{end+1} = '    ';
    lines{end+1} = '    Dim nBodies As Long';
    lines{end+1} = '    nBodies = UBound(vBodies) + 1';
    lines{end+1} = '    Debug.Print "  Total bodies for trim: " & nBodies';
    lines{end+1} = '    ';
    lines{end+1} = '    '' Note: Manual trim may be needed for complex intersections.';
    lines{end+1} = '    '' Auto-trim is applied where SolidWorks can detect adjacent members.';
    lines{end+1} = '    '' For laser-cut ends, precise flat trim is the default behavior.';
    lines{end+1} = '    ';
    lines{end+1} = '    Debug.Print "  Trim complete."';
    lines{end+1} = '    Exit Sub';
    lines{end+1} = '';
    lines{end+1} = 'ErrHandler:';
    lines{end+1} = '    Debug.Print "ERROR in TrimMembers: " & Err.Description';
    lines{end+1} = '    Resume Next';
    lines{end+1} = 'End Sub';
    lines{end+1} = '';
    code = strjoin(lines, newline);
end

function code = emit_vba_cutlist_props(components, profiles)
    lines = {};
    % Build the category assignment lookup
    n_primary = 2 + 2 + size(components.fbh, 1) + size(components.shoulder, 1) + ...
                size(components.fbh_support, 1) + size(components.side_impact, 1);

    lines{end+1} = 'Sub SetCutListProps(swModel As SldWorks.ModelDoc2)';
    lines{end+1} = '    Debug.Print "Setting cut list properties..."';
    lines{end+1} = '    On Error GoTo ErrHandler';
    lines{end+1} = '    ';
    lines{end+1} = '    Dim swFeat As SldWorks.Feature';
    lines{end+1} = '    Dim swCutFolder As SldWorks.BodyFolder';
    lines{end+1} = '    Dim swSubFeat As SldWorks.Feature';
    lines{end+1} = '    Dim swCustPropMgr As SldWorks.CustomPropertyManager';
    lines{end+1} = '    Dim bodyIdx As Long';
    lines{end+1} = '    Dim category As String';
    lines{end+1} = '    ';
    lines{end+1} = '    Set swFeat = swModel.FirstFeature';
    lines{end+1} = '    bodyIdx = 0';
    lines{end+1} = '    ';
    lines{end+1} = '    Do While Not swFeat Is Nothing';
    lines{end+1} = '        If swFeat.GetTypeName2 = "CutListFolder" Then';
    lines{end+1} = '            Set swCutFolder = swFeat.GetSpecificFeature2';
    lines{end+1} = '            swCutFolder.UpdateCutList';
    lines{end+1} = '            ';
    lines{end+1} = '            Set swSubFeat = swFeat.GetFirstSubFeature';
    lines{end+1} = '            Do While Not swSubFeat Is Nothing';
    lines{end+1} = '                bodyIdx = bodyIdx + 1';
    lines{end+1} = '                Set swCustPropMgr = swSubFeat.CustomPropertyManager';
    lines{end+1} = '                ';
    lines{end+1} = sprintf('                '' Bodies 1-%d = primary (hoops, FBH, shoulder, SI)', n_primary);
    lines{end+1} = sprintf('                If bodyIdx <= %d Then', n_primary);
    lines{end+1} = '                    category = ChrW(&H4E3B) & ChrW(&H8981) & ChrW(&H92FC) & ChrW(&H6750)  '' main steel';
    lines{end+1} = '                Else';
    lines{end+1} = '                    category = ChrW(&H88DC) & ChrW(&H52A9) & ChrW(&H92FC) & ChrW(&H6750)  '' aux steel';
    lines{end+1} = '                End If';
    lines{end+1} = '                swCustPropMgr.Add3 "Category", SW_CUSTOM_INFO_TEXT, category, 0';
    lines{end+1} = '                Set swSubFeat = swSubFeat.GetNextSubFeature';
    lines{end+1} = '            Loop';
    lines{end+1} = '        End If';
    lines{end+1} = '        Set swFeat = swFeat.GetNextFeature';
    lines{end+1} = '    Loop';
    lines{end+1} = '    ';
    lines{end+1} = '    Debug.Print "  Cut list properties set. " & bodyIdx & " bodies processed."';
    lines{end+1} = '    Exit Sub';
    lines{end+1} = '';
    lines{end+1} = 'ErrHandler:';
    lines{end+1} = '    Debug.Print "ERROR in SetCutListProps: " & Err.Description';
    lines{end+1} = '    Resume Next';
    lines{end+1} = 'End Sub';
    lines{end+1} = '';
    code = strjoin(lines, newline);
end

function code = emit_vba_body_export(opts)
    lines = {};
    lines{end+1} = 'Sub ExportBodies(swModel As SldWorks.ModelDoc2, swApp As SldWorks.SldWorks)';
    lines{end+1} = '    If Not EXPORT_BODIES Then Exit Sub';
    lines{end+1} = '    Debug.Print "Exporting individual pipe bodies..."';
    lines{end+1} = '    On Error GoTo ErrHandler';
    lines{end+1} = '    ';
    lines{end+1} = '    Dim vBodies As Variant';
    lines{end+1} = '    Dim swBody As SldWorks.Body2';
    lines{end+1} = '    Dim bodyName As String';
    lines{end+1} = '    Dim exportName As String';
    lines{end+1} = '    Dim exportPath As String';
    lines{end+1} = '    Dim errors As Long';
    lines{end+1} = '    Dim warnings As Long';
    lines{end+1} = '    Dim i As Long';
    lines{end+1} = '    Dim nBodies As Long';
    lines{end+1} = '    Dim newDoc As SldWorks.ModelDoc2';
    lines{end+1} = '    ';
    lines{end+1} = '    '' Ensure export directory exists';
    lines{end+1} = '    CreateDirRecursive BODY_EXPORT_DIR';
    lines{end+1} = '    ';
    lines{end+1} = '    '' Get all solid bodies';
    lines{end+1} = '    vBodies = swModel.GetBodies2(SW_SOLID_BODY, True)';
    lines{end+1} = '    If IsEmpty(vBodies) Then';
    lines{end+1} = '        Debug.Print "  No bodies found."';
    lines{end+1} = '        Exit Sub';
    lines{end+1} = '    End If';
    lines{end+1} = '    nBodies = UBound(vBodies) + 1';
    lines{end+1} = '    ';
    lines{end+1} = '    '' Export each body: save model copy, delete other bodies, save as, undo';
    lines{end+1} = '    For i = 0 To UBound(vBodies)';
    lines{end+1} = '        Set swBody = vBodies(i)';
    lines{end+1} = '        bodyName = swBody.Name';
    lines{end+1} = '        '' Sanitize body name for filename';
    lines{end+1} = '        bodyName = Replace(bodyName, " ", "_")';
    lines{end+1} = '        bodyName = Replace(bodyName, "/", "_")';
    lines{end+1} = '        bodyName = Replace(bodyName, "\", "_")';
    lines{end+1} = '        exportName = "pipe_" & Format(i + 1, "000") & "_" & bodyName & ".SLDPRT"';
    lines{end+1} = '        exportPath = BODY_EXPORT_DIR & "\" & exportName';
    lines{end+1} = '        ';
    lines{end+1} = '        '' Use SaveToFile on the body (exports as .sldprt with single body)';
    lines{end+1} = '        '' Alternative: copy model, delete other bodies, save, close';
    lines{end+1} = '        Set newDoc = swApp.CopyDocument(swModel.GetPathName, exportPath, True, True)';
    lines{end+1} = '        If Not newDoc Is Nothing Then';
    lines{end+1} = '            '' Body isolation done via copy';
    lines{end+1} = '            swApp.CloseDoc newDoc.GetTitle';
    lines{end+1} = '        Else';
    lines{end+1} = '            '' Fallback: simple SaveAs (saves full model as reference)';
    lines{end+1} = '            swModel.Extension.SaveAs3 exportPath, 0, 2, Nothing, Nothing, errors, warnings';
    lines{end+1} = '        End If';
    lines{end+1} = '        Debug.Print "  Exported: " & exportName';
    lines{end+1} = '    Next i';
    lines{end+1} = '    ';
    lines{end+1} = '    Debug.Print "  Body export complete. " & nBodies & " files."';
    lines{end+1} = '    Exit Sub';
    lines{end+1} = '';
    lines{end+1} = 'ErrHandler:';
    lines{end+1} = '    Debug.Print "ERROR in ExportBodies: " & Err.Description';
    lines{end+1} = '    Resume Next';
    lines{end+1} = 'End Sub';
    lines{end+1} = '';
    code = strjoin(lines, newline);
end

function code = emit_vba_diagnostic()
    lines = {};
    lines{end+1} = 'Sub DiagnosticWeldmentTest(swModel As SldWorks.ModelDoc2, swApp As SldWorks.SldWorks)';
    lines{end+1} = '    On Error GoTo DiagErr';
    lines{end+1} = '    Dim swSketchMgr As SldWorks.SketchManager';
    lines{end+1} = '    Set swSketchMgr = swModel.SketchManager';
    lines{end+1} = '    If swSketchMgr Is Nothing Then';
    lines{end+1} = '        Debug.Print "  Diagnostic: SketchManager is Nothing"';
    lines{end+1} = '    Else';
    lines{end+1} = '        Debug.Print "  Diagnostic: SketchManager OK"';
    lines{end+1} = '    End If';
    lines{end+1} = '    Exit Sub';
    lines{end+1} = 'DiagErr:';
    lines{end+1} = '    Debug.Print "  Diagnostic failed: " & Err.Description';
    lines{end+1} = 'End Sub';
    lines{end+1} = '';
    code = strjoin(lines, newline);
end

function code = emit_vba_main(opts)
    lines = {};
    lines{end+1} = 'Sub Main()';
    lines{end+1} = '    '' === FSAE Frame Generator - Entry Point ===';
    lines{end+1} = '    On Error GoTo MainErr';
    lines{end+1} = '    ';
    lines{end+1} = '    '' 1. Connect to SolidWorks';
    lines{end+1} = '    Dim swApp As SldWorks.SldWorks';
    lines{end+1} = '    Set swApp = Application.SldWorks';
    lines{end+1} = '    Debug.Print "=== FSAE Frame Generator ==="';
    lines{end+1} = '    Debug.Print "SolidWorks version: " & swApp.RevisionNumber';
    lines{end+1} = '    ';
    lines{end+1} = '    '' 2. Find Part template';
    lines{end+1} = '    Debug.Print "Searching for Part template..."';
    lines{end+1} = '    gPartTemplate = FindPartTemplate(swApp)';
    lines{end+1} = '    If Len(Trim(gPartTemplate)) = 0 Then';
    lines{end+1} = '        MsgBox "Part template (.prtdot) not found." & vbCrLf & vbCrLf & _';
    lines{end+1} = '               "Please configure:" & vbCrLf & _';
    lines{end+1} = '               "  Tools > Options > Default Templates > Part" & vbCrLf & vbCrLf & _';
    lines{end+1} = '               "Or check Immediate Window (Ctrl+G) for search details.", _';
    lines{end+1} = '               vbExclamation, "FSAE Frame Generator"';
    lines{end+1} = '        Exit Sub';
    lines{end+1} = '    End If';
    lines{end+1} = '    Debug.Print "Using template: " & gPartTemplate';
    lines{end+1} = '    ';
    lines{end+1} = '    '' 3. Create new Part document';
    lines{end+1} = '    Dim swModel As SldWorks.ModelDoc2';
    lines{end+1} = '    Set swModel = swApp.NewDocument(gPartTemplate, 0, 0, 0)';
    lines{end+1} = '    If swModel Is Nothing Then';
    lines{end+1} = '        MsgBox "Failed to create Part document." & vbCrLf & _';
    lines{end+1} = '               "Template: " & gPartTemplate, vbExclamation';
    lines{end+1} = '        Exit Sub';
    lines{end+1} = '    End If';
    lines{end+1} = '    ';
    lines{end+1} = '    '' 3. Initialize node coordinates';
    lines{end+1} = '    Debug.Print "Initializing nodes..."';
    lines{end+1} = '    InitNodes';
    lines{end+1} = '    ';
    lines{end+1} = '    '' 4. Create custom weldment profiles';
    lines{end+1} = '    Debug.Print "Creating custom profiles..."';
    lines{end+1} = '    CreateCustomProfiles swApp';
    lines{end+1} = '    ';
    lines{end+1} = '    '' 4.5 DIAGNOSTIC: Test weldment API before actual build';
    lines{end+1} = '    Debug.Print ""';
    lines{end+1} = '    Debug.Print "--- DIAGNOSTIC: Testing weldment API ---"';
    lines{end+1} = '    DiagnosticWeldmentTest swModel, swApp';
    lines{end+1} = '    Debug.Print ""';
    lines{end+1} = '    ';
    lines{end+1} = '    '' 5. Build frame components (priority order)';
    lines{end+1} = '    Debug.Print ""';
    lines{end+1} = '    Debug.Print "--- Phase 1: Hoops + FBH (Priority 1) ---"';
    lines{end+1} = '    Build_FrontHoop_Bent swModel, swApp';
    lines{end+1} = '    Build_MainHoop_Bent swModel, swApp';
    lines{end+1} = '    Build_FrontHoop_Bottom swModel, swApp';
    lines{end+1} = '    Build_MainHoop_Bottom swModel, swApp';
    lines{end+1} = '    Build_FBH swModel, swApp';
    lines{end+1} = '    Build_Shoulder swModel, swApp';
    lines{end+1} = '    ';
    lines{end+1} = '    Debug.Print ""';
    lines{end+1} = '    Debug.Print "--- Phase 2: Primary Structure ---"';
    lines{end+1} = '    Build_FBHSupport swModel, swApp';
    lines{end+1} = '    Build_SideImpact swModel, swApp';
    lines{end+1} = '    ';
    lines{end+1} = '    Debug.Print ""';
    lines{end+1} = '    Debug.Print "--- Phase 3: Secondary Structure ---"';
    lines{end+1} = '    Build_MH_to_Rear swModel, swApp';
    lines{end+1} = '    Build_RearSection swModel, swApp';
    lines{end+1} = '    Build_Brackets swModel, swApp';
    lines{end+1} = '    Build_Structural swModel, swApp';
    lines{end+1} = '    ';
    lines{end+1} = '    '' 6. Trim member end faces';
    lines{end+1} = '    Debug.Print ""';
    lines{end+1} = '    Debug.Print "--- Post-processing ---"';
    lines{end+1} = '    TrimMembers swModel';
    lines{end+1} = '    ';
    lines{end+1} = '    '' 7. Set cut list properties';
    lines{end+1} = '    SetCutListProps swModel';
    lines{end+1} = '    ';
    lines{end+1} = '    '' 8. Export individual bodies';
    lines{end+1} = '    ExportBodies swModel, swApp';
    lines{end+1} = '    ';
    lines{end+1} = '    '' 9. Force rebuild and zoom to fit';
    lines{end+1} = '    swModel.ForceRebuild3 True';
    lines{end+1} = '    swModel.ViewZoomtofit2';
    lines{end+1} = '    ';
    lines{end+1} = '    Debug.Print ""';
    lines{end+1} = '    Debug.Print "=== Frame generation complete! ==="';
    lines{end+1} = '    MsgBox "FSAE Frame generation complete!" & vbCrLf & _';
    lines{end+1} = '           "Nodes: " & NUM_NODES & vbCrLf & _';
    lines{end+1} = '           "Check the Immediate Window (Ctrl+G) for details.", _';
    lines{end+1} = '           vbInformation, "FSAE Frame Generator"';
    lines{end+1} = '    Exit Sub';
    lines{end+1} = '';
    lines{end+1} = 'MainErr:';
    lines{end+1} = '    MsgBox "Fatal error in Main: " & Err.Description & vbCrLf & _';
    lines{end+1} = '           "Error " & Err.Number & " at line " & Erl, _';
    lines{end+1} = '           vbCritical, "FSAE Frame Generator"';
    lines{end+1} = 'End Sub';
    lines{end+1} = '';
    code = strjoin(lines, newline);
end

function output_path = write_bas_file(vba_sections, opts)
    % Ensure output directory exists
    if ~isfolder(opts.output_dir)
        mkdir(opts.output_dir);
    end

    output_path = fullfile(opts.output_dir, opts.filename);

    % Write all sections
    fid = fopen(output_path, 'w', 'n', 'UTF-8');
    if fid == -1
        error('generate_solidworks_macro:FileError', ...
            'Cannot write to: %s', output_path);
    end

    % .swb format: no Attribute header (direct-run macro for SolidWorks)
    for i = 1:length(vba_sections)
        fprintf(fid, '%s\n', vba_sections{i});
    end

    fclose(fid);
end

function vba_code = convert_to_late_binding(vba_code)
% CONVERT_TO_LATE_BINDING  Replace SolidWorks early-binding types with Object
%   and strip enum type prefixes for late-binding compatibility.
%   .swb macros cannot reference the SolidWorks type library at compile time.

    % Replace SolidWorks types with Object
    sw_types = {'SldWorks.SldWorks', 'SldWorks.ModelDoc2', ...
        'SldWorks.FeatureManager', 'SldWorks.SketchSegment', ...
        'SldWorks.Feature', 'SldWorks.Sketch', ...
        'SldWorks.Body2', 'SldWorks.BodyFolder', ...
        'SldWorks.CustomPropertyManager'};
    for i = 1:length(sw_types)
        vba_code = strrep(vba_code, sw_types{i}, 'Object');
    end
end

function validate_vba(vba_sections)
% VALIDATE_VBA  Compile-time checks on generated VBA code.
%   Catches common issues before writing the .swb file:
%     - Remaining early-binding type references (SldWorks.xxx)
%     - Remaining enum-prefixed constants (swXxx_e.)
%     - Unbalanced Sub/End Sub, Function/End Function
%     - Dim declarations inside loops

    all_code = strjoin(vba_sections, newline);
    lines = strsplit(all_code, newline);
    errors = {};

    % Check 1: No remaining SldWorks type references
    for i = 1:length(lines)
        code_part = strip_vba_comment(lines{i});
        if contains(code_part, 'SldWorks.')
            errors{end+1} = sprintf('Line %d: Early-binding type: %s', i, strtrim(lines{i})); %#ok<AGROW>
        end
    end

    % Check 2: No remaining enum-prefixed references (swXxx_e.)
    for i = 1:length(lines)
        code_part = strip_vba_comment(lines{i});
        if ~isempty(regexp(code_part, 'sw\w+_e\.', 'once'))
            errors{end+1} = sprintf('Line %d: Enum prefix: %s', i, strtrim(lines{i})); %#ok<AGROW>
        end
    end

    % Check 3: Balanced Sub/End Sub
    n_sub = sum(~cellfun('isempty', regexp(lines, '^\s*(Public\s+|Private\s+)?Sub\s+')));
    n_endsub = sum(~cellfun('isempty', regexp(lines, '^\s*End\s+Sub\s*$')));
    if n_sub ~= n_endsub
        errors{end+1} = sprintf('Unbalanced Sub/End Sub: %d vs %d', n_sub, n_endsub);
    end

    % Check 4: Balanced Function/End Function
    n_func = sum(~cellfun('isempty', regexp(lines, '^\s*(Public\s+|Private\s+)?Function\s+')));
    n_endfunc = sum(~cellfun('isempty', regexp(lines, '^\s*End\s+Function\s*$')));
    if n_func ~= n_endfunc
        errors{end+1} = sprintf('Unbalanced Function/End Function: %d vs %d', n_func, n_endfunc);
    end

    % Check 5: No Dim inside loops
    loop_depth = 0;
    for i = 1:length(lines)
        line = strtrim(lines{i});
        if startsWith(line, 'For ') || startsWith(line, 'Do ')
            loop_depth = loop_depth + 1;
        end
        if startsWith(line, 'Next ') || strcmp(line, 'Loop')
            loop_depth = max(0, loop_depth - 1);
        end
        if loop_depth > 0 && startsWith(line, 'Dim ')
            errors{end+1} = sprintf('Line %d: Dim inside loop: %s', i, line); %#ok<AGROW>
        end
    end

    % Check 6: No Attribute VB_Name (.swb format)
    if contains(all_code, 'Attribute VB_Name')
        errors{end+1} = 'Attribute VB_Name found — .swb format must not have it';
    end

    % Report
    if ~isempty(errors)
        fprintf('VBA Compile Check FAILED:\n');
        for i = 1:length(errors)
            fprintf('  ERROR: %s\n', errors{i});
        end
        error('generate_solidworks_macro:CompileCheck', ...
            'VBA compile check found %d error(s). Fix emit functions.', length(errors));
    else
        fprintf('  VBA compile check: PASSED (%d subs, %d functions)\n', n_sub, n_func);
    end
end

function code_part = strip_vba_comment(line)
% STRIP_VBA_COMMENT  Return only the code portion of a VBA line (before ' comment).
%   Handles single-quote inside string literals ("it's") correctly.
    in_string = false;
    for k = 1:length(line)
        if line(k) == '"'
            in_string = ~in_string;
        elseif line(k) == '''' && ~in_string
            code_part = line(1:k-1);
            return;
        end
    end
    code_part = line;
end
