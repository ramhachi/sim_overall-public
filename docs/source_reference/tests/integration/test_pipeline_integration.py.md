# tests/integration/test_pipeline_integration.py

`optimize_pid_gains.m` が組み立てる `docker compose ... laptime_sim` コマンドを Python から再現し、ASE Autocross の統合実行経路が通るかを確認するテストです。  
本質は数値精度ではなく、**MATLAB -> Docker -> C++** のコマンド結線が崩れていないかを見ることにあります。
