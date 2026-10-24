# `suspension_kinematics_design/plot_a_arm_geometry.m`

既定の車両 artifact から `FL` hardpoints を読み、A アーム、キングピン、ホイール中心、タイヤ半径を 3D で可視化する確認スクリプトです。  
`resolve_default_vehicle_json(..., 'RequiredHardpointCorners', {'FL'})` を使うため、固定ファイル名に依存しません。
