classdef test_VehicleSimRunner_unit < matlab.unittest.TestCase
    % Unit tests for VehicleSimRunner helpers that do NOT require a live Docker
    % socket.  Run with:
    %   runtests('matlab/tests/test_VehicleSimRunner_unit.m')
    %
    % All tests exercise parseJsonResult logic and the hostToContainerPath
    % mapping by replicating the exact same logic in a self-contained helper
    % (so no Docker socket is needed and no VehicleSimRunner constructor fires).

    % -------------------------------------------------------------------------
    % parseJsonResult
    % -------------------------------------------------------------------------
    methods (Test)
        function testDelimitedJsonIsParsed(tc)
            raw = sprintf('startup log\n%%%%JSON_BEGIN%%%%\n{"status":"success","lap_time":42.1}\n%%%%JSON_END%%%%\ntrailing noise\n');
            result = tc.parseHelper(raw);
            tc.verifyEqual(result.status, 'success');
            tc.verifyEqual(result.lap_time, 42.1, 'AbsTol', 1e-9);
        end

        function testFallbackGreedyBraces(tc)
            % No delimiters: should still extract JSON between first { and last }.
            raw = 'Startup log {"status":"success","lap_time":7.5} some more output';
            result = tc.parseHelper(raw);
            tc.verifyEqual(result.status, 'success');
            tc.verifyEqual(result.lap_time, 7.5, 'AbsTol', 1e-9);
        end

        function testDelimitedBeatsGreedy(tc)
            % When delimiters are present the content inside them must win,
            % even if stray curly braces appear elsewhere in the output.
            raw = sprintf('{"noise":1}\n%%%%JSON_BEGIN%%%%\n{"status":"ok","lap_time":5.0}\n%%%%JSON_END%%%%\n{"noise":2}\n');
            result = tc.parseHelper(raw);
            tc.verifyEqual(result.status, 'ok');
            tc.verifyEqual(result.lap_time, 5.0, 'AbsTol', 1e-9);
        end

        function testNoJsonThrows(tc)
            raw = 'No JSON here at all';
            tc.verifyError(@() tc.parseHelper(raw), 'VehicleSimRunner:noJson');
        end

        function testFailedResultJsonWithDelimiters(tc)
            raw = sprintf('%%%%JSON_BEGIN%%%%\n{"status":"failed","lap_time":0,"lap_completed":false,"failure_code":"early_stall","failure_message":"stalled"}\n%%%%JSON_END%%%%\n');
            result = tc.parseHelper(raw);
            tc.verifyEqual(result.status, 'failed');
            tc.verifyFalse(result.lap_completed);
            tc.verifyEqual(result.failure_code, 'early_stall');
        end

        function testMultipleDelimiterBlocksUsesLast(tc)
            % If somehow two blocks are emitted (e.g. stderr flush), last wins.
            raw = sprintf('%%%%JSON_BEGIN%%%%\n{"lap_time":1}\n%%%%JSON_END%%%%\n%%%%JSON_BEGIN%%%%\n{"lap_time":2}\n%%%%JSON_END%%%%\n');
            result = tc.parseHelper(raw);
            tc.verifyEqual(result.lap_time, 2, 'AbsTol', 1e-9);
        end
    end

    % -------------------------------------------------------------------------
    % hostToContainerPath
    % -------------------------------------------------------------------------
    methods (Test)
        function testContainerPathMapping(tc)
            % Verify the /workspace/ mapping for a path under ProjectRoot.
            root = '/fake/project';
            host = '/fake/project/output/runtime/params_abc.json';
            got  = tc.containerPathHelper(root, host);
            tc.verifyEqual(got, '/workspace/output/runtime/params_abc.json');
        end

        function testContainerPathMappingNested(tc)
            root = '/fake/project';
            host = '/fake/project/data/vehicle.json';
            got  = tc.containerPathHelper(root, host);
            tc.verifyEqual(got, '/workspace/data/vehicle.json');
        end

        function testContainerPathOutsideRootErrors(tc)
            root = '/fake/project';
            host = '/other/path/params.json';
            tc.verifyError(@() tc.containerPathHelper(root, host), ...
                'VehicleSimRunner:invalidPath');
        end
    end

    % -------------------------------------------------------------------------
    % appendSampleLog
    % -------------------------------------------------------------------------
    methods (Test)
        function testAppendSampleLogCreatesAndAppends(tc)
            log_path = fullfile(tempdir, sprintf('vsr_test_log_%d.jsonl', randi(1e8)));
            cleanup = onCleanup(@() deleteIfExists(log_path));

            result1 = struct('status', 'success', 'lap_time', 10.0);
            result2 = struct('status', 'success', 'lap_time', 11.0);

            tc.appendLogHelper(log_path, result1, true,  '/p/a.json');
            tc.appendLogHelper(log_path, result2, false, '/p/b.json');

            tc.verifyTrue(isfile(log_path));
            lines = readlines_helper(log_path);
            tc.verifyEqual(numel(lines), 2);

            e1 = jsondecode(lines{1});
            e2 = jsondecode(lines{2});
            tc.verifyTrue(e1.success);
            tc.verifyFalse(e2.success);
            tc.verifyEqual(e1.result.lap_time, 10.0, 'AbsTol', 1e-9);
        end
    end

    % -------------------------------------------------------------------------
    % Private helpers (replicate internal logic without touching real class)
    % -------------------------------------------------------------------------
    methods (Access = private)
        function result = parseHelper(~, cmdout)
            begin_tok = '%%JSON_BEGIN%%';
            end_tok   = '%%JSON_END%%';
            b_idx = strfind(cmdout, begin_tok);
            e_idx = strfind(cmdout, end_tok);
            if ~isempty(b_idx) && ~isempty(e_idx)
                json_start   = b_idx(end) + length(begin_tok);
                json_end_pos = e_idx(end) - 1;
                json_content = strtrim(cmdout(json_start:json_end_pos));
                result = jsondecode(json_content);
                return;
            end
            json_start = strfind(cmdout, '{');
            json_end   = strfind(cmdout, '}');
            if isempty(json_start) || isempty(json_end)
                error('VehicleSimRunner:noJson', 'No JSON found in output');
            end
            result = jsondecode(cmdout(json_start(1):json_end(end)));
        end

        function container_path = containerPathHelper(~, root, host_path)
            host_abs = host_path;   % already absolute in these tests
            root_abs = root;
            if ~startsWith(host_abs, root_abs)
                error('VehicleSimRunner:invalidPath', ...
                    'params_file_host must be under ProjectRoot (%s), got: %s', ...
                    root_abs, host_path);
            end
            rel = host_abs(length(root_abs)+2:end);
            container_path = ['/workspace/' strrep(rel, '\', '/')];
        end

        function appendLogHelper(~, log_file, result, success, params_file)
            try
                entry = struct();
                entry.timestamp = datestr(now, 'yyyy-mm-ddTHH:MM:SS');
                entry.success   = success;
                if ~isempty(params_file)
                    entry.params_file = params_file;
                end
                if ~isempty(result) && isstruct(result)
                    entry.result = result;
                end
                fid = fopen(log_file, 'a');
                if fid ~= -1
                    fprintf(fid, '%s\n', jsonencode(entry));
                    fclose(fid);
                end
            catch
            end
        end
    end
end

function lines = readlines_helper(path)
    fid = fopen(path, 'r');
    lines = {};
    tline = fgetl(fid);
    while ischar(tline)
        if ~isempty(strtrim(tline))
            lines{end+1} = tline; %#ok<AGROW>
        end
        tline = fgetl(fid);
    end
    fclose(fid);
end

function deleteIfExists(p)
    if isfile(p), delete(p); end
end
