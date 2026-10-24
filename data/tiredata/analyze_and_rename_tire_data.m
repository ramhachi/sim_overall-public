%% analyze_and_rename_tire_data.m
% .datファイル → CSV変換 & 統一命名リネーム
%
% ■ 使い方（想定手順）
%   1. Cornering の .dat ファイルをこのディレクトリ直下に置く
%   2. file_prefix = 'Cr' に設定して実行 → csv/ に変換される
%   3. 直下の .dat を削除（delete_dat_after = true で自動削除も可）
%   4. DriveBrake の .dat ファイルを直下に置き直す
%   5. file_prefix = 'Db' に設定して実行 → csv/ に追加される
%   6. 直下の .dat を削除
%
% ■ 出力ファイル名の形式
%   {prefix}_{TireName}_{Compound}_Run{XXX}.csv
%   例: Cr_Hoosier_16p0X7p5-10_R20_C2000_Run004.csv
%       Db_Hoosier_18p0X6p0-10_R20_Run068.csv

clear; clc;

%% ============================================================
%% ユーザ設定（ここを変えて使う）
%% ============================================================

% ファイル名のプレフィックス（テスト種別を示す）
%   'Cr'  : Cornering（コーナリング試験）
%   'Db'  : DriveBrake（駆動制動試験）
%   'Cmb' : Combined（複合試験）
%   ''    : プレフィックスなし
file_prefix = 'Db';

% ドライラン（true: 確認のみ / false: 実際に変換）
dry_run = false;

% 変換後に .dat ファイルを自動削除するか
delete_dat_after = false;

%% ============================================================
%% 実行
%% ============================================================

fprintf('=================================================================\n');
fprintf(' Tire Data: .dat → CSV 変換ツール\n');
fprintf('=================================================================\n');
fprintf('  プレフィックス : "%s"\n', file_prefix);
fprintf('  ドライラン     : %s\n', mat2str(dry_run));
fprintf('  .dat自動削除   : %s\n', mat2str(delete_dat_after));
fprintf('=================================================================\n\n');

% csvフォルダがない場合は作成
if ~exist('csv', 'dir')
    mkdir('csv');
    fprintf('  csv/ フォルダを作成しました\n\n');
end

% .datファイルを検出
dat_files = dir('*.dat');
n_dat = length(dat_files);
fprintf('検出した .dat ファイル: %d 個\n\n', n_dat);

if n_dat == 0
    fprintf('  .dat ファイルが見つかりません。\n');
    fprintf('  このディレクトリに .dat ファイルを置いて再実行してください。\n\n');
    return;
end

% --- 変換ループ ---
converted_count = 0;
converted_files = {};

for i = 1:n_dat
    dat_name = dat_files(i).name;
    
    % .datファイルの1行目からメタデータを解析
    info = parse_dat_metadata(dat_name);
    
    % ファイル名を生成
    new_csv_name = generate_csv_filename(info, file_prefix);
    out_path = fullfile('csv', new_csv_name);
    
    % 表示
    fprintf('  %2d. %s\n', i, dat_name);
    fprintf('      タイヤ     : %s\n', info.tire_name);
    if ~isempty(info.compound)
        fprintf('      コンパウンド: %s\n', info.compound);
    end
    fprintf('      Run        : %s\n', info.run_number);
    fprintf('      → %s\n', out_path);
    
    if dry_run
        fprintf('      [DRY RUN - スキップ]\n\n');
        continue;
    end
    
    % 既に同名のCSVが存在する場合
    if exist(out_path, 'file')
        fprintf('      [SKIP] 既に存在します\n\n');
        continue;
    end
    
    % 変換実行
    try
        convert_dat_to_csv(dat_name, out_path);
        fprintf('      [OK] 変換完了\n\n');
        converted_count = converted_count + 1;
        converted_files{end+1} = dat_name; %#ok<SAGROW>
    catch ME
        fprintf('      [ERROR] %s\n\n', ME.message);
    end
end

% --- .dat 削除 ---
if ~dry_run && delete_dat_after && converted_count > 0
    fprintf('--- .dat ファイル削除 ---\n');
    for i = 1:length(converted_files)
        delete(converted_files{i});
        fprintf('  削除: %s\n', converted_files{i});
    end
    fprintf('\n');
end

% --- サマリー ---
fprintf('=================================================================\n');
fprintf(' 完了\n');
fprintf('=================================================================\n');
fprintf('  検出    : %d 個\n', n_dat);
fprintf('  変換済み: %d 個\n', converted_count);
fprintf('  出力先  : csv/\n');
fprintf('  プレフィックス: %s\n', file_prefix);
fprintf('=================================================================\n\n');

if dry_run
    fprintf('※ ドライランモードです。\n');
    fprintf('  実行するには dry_run = false にしてください。\n\n');
end

% csv/ フォルダ内のファイル一覧を表示
csv_files = dir('csv/*.csv');
if ~isempty(csv_files)
    fprintf('--- csv/ フォルダの内容 (%d ファイル) ---\n', length(csv_files));
    for i = 1:length(csv_files)
        fprintf('  %s\n', csv_files(i).name);
    end
    fprintf('\n');
end

%% ============================================================
%% ローカル関数
%% ============================================================

function info = parse_dat_metadata(dat_filename)
    % .datファイルの1行目からメタデータを抽出
    %
    % 1行目の形式例:
    %   TIRF Data File: Project 2356; Run 004; Date 01-18-2022;
    %   Tire_Name=Hoosier 16.0X7.5-10, R20, C2000; Rim_Width=7.0; ...
    
    info = struct();
    info.tire_name = 'Unknown';
    info.compound = '';
    info.run_number = '';
    
    fid = fopen(dat_filename, 'r');
    if fid == -1
        return;
    end
    
    try
        metadata = fgetl(fid);
        fclose(fid);
        
        % Run番号を抽出（例: "Run 004" → "004"）
        run_match = regexp(metadata, 'Run\s+(\d+)', 'tokens');
        if ~isempty(run_match)
            info.run_number = run_match{1}{1};
        end
        
        % Tire_Name を抽出（セミコロンまで）
        tire_match = regexp(metadata, 'Tire_Name=([^;]+)', 'tokens');
        if ~isempty(tire_match)
            tire_full = strtrim(tire_match{1}{1});
            
            % カンマで分割: "Hoosier 16.0X7.5-10, R20, C2000"
            parts = strsplit(tire_full, ',');
            if ~isempty(parts)
                info.tire_name = strtrim(parts{1});
                
                for j = 2:length(parts)
                    part = strtrim(parts{j});
                    % リム径（R20 など）→ タイヤ名に追加
                    if ~isempty(regexp(part, '^R\d{2}$', 'once'))
                        info.tire_name = [info.tire_name ' ' part];
                    end
                    % コンパウンド（C2000 など）
                    if ~isempty(regexp(part, '^C\d{2,4}$', 'once'))
                        info.compound = part;
                    end
                end
            end
        end
    catch
        if fid ~= -1
            fclose(fid);
        end
    end
end

function new_name = generate_csv_filename(info, prefix)
    % ファイル名を生成
    % 形式: {prefix}_{TireName}_{Compound}_Run{XXX}.csv
    
    % プレフィックス
    if nargin < 2 || isempty(prefix)
        prefix_part = '';
    else
        prefix_part = prefix;
        if ~endsWith(prefix_part, '_')
            prefix_part = [prefix_part '_'];
        end
    end
    
    % タイヤ名の正規化
    tire_name = info.tire_name;
    tire_name = strrep(tire_name, ' ', '_');
    tire_name = strrep(tire_name, ',', '');
    tire_name = strrep(tire_name, '.', 'p');
    tire_name = strrep(tire_name, '×', 'X');
    
    % コンパウンド
    if ~isempty(info.compound)
        compound_part = ['_' info.compound];
    else
        compound_part = '';
    end
    
    % Run番号
    if ~isempty(info.run_number)
        run_part = ['_Run' info.run_number];
    else
        run_part = '';
    end
    
    new_name = sprintf('%s%s%s%s.csv', prefix_part, tire_name, compound_part, run_part);
end

function convert_dat_to_csv(dat_filename, csv_filename)
    % TIRF形式の.datファイルをCSVに変換
    %
    % 構造:
    %   1行目: メタデータ（スキップ）
    %   2行目: カラム名（スペース区切り）
    %   3行目: 単位（スペース区切り）
    %   4行目以降: データ（スペース区切り）
    
    fid = fopen(dat_filename, 'r');
    if fid == -1
        error('ファイルを開けません: %s', dat_filename);
    end
    
    try
        % 1行目: メタデータ（スキップ）
        fgetl(fid);
        
        % 2行目: カラム名
        header_line = strtrim(regexprep(fgetl(fid), '\s+', ' '));
        
        % 3行目: 単位
        unit_line = strtrim(regexprep(fgetl(fid), '\s+', ' '));
        
        % カンマ区切りに変換
        header_csv = strrep(header_line, ' ', ',');
        unit_csv = strrep(unit_line, ' ', ',');
        
        % CSVファイルを作成
        fid_out = fopen(csv_filename, 'w');
        if fid_out == -1
            error('出力ファイルを作成できません: %s', csv_filename);
        end
        
        fprintf(fid_out, '%s\n', header_csv);
        fprintf(fid_out, '%s\n', unit_csv);
        
        % データ行を変換
        while ~feof(fid)
            line = fgetl(fid);
            if ischar(line)
                line = strtrim(regexprep(line, '\s+', ' '));
                fprintf(fid_out, '%s\n', strrep(line, ' ', ','));
            end
        end
        
        fclose(fid_out);
        fclose(fid);
        
    catch ME
        fclose(fid);
        if exist('fid_out', 'var') && fid_out ~= -1
            fclose(fid_out);
        end
        rethrow(ME);
    end
end
