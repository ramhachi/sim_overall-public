classdef FSAESimClient < handle
    % FSAESIMCLIENT MATLABからDocker内のC++ RLサーバーにTCP接続するクライアント
    % 高速化のため、tcpclientの代わりにjava.net.Socketを使用
    
    properties (Constant)
        CMD_INIT  = uint8(0x01)
        CMD_STEP  = uint8(0x02)
        CMD_RESET = uint8(0x03)
        CMD_PARAM = uint8(0x04)
        CMD_QUIT  = uint8(0xFF)
        
        RSP_OK    = uint8(0x01)
        RSP_OBS   = uint8(0x02)
        RSP_ERR   = uint8(0xFF)
        
        OBS_DIM   = 34
        INFO_DIM  = 12
    end
    
    properties
        Host = 'localhost'
        Port = 22345
        Timeout = 30  % 秒
    end
    
    properties (Access = private)
        Socket
        InputStream
        OutputStream
        DataInputStream
        DataOutputStream
        Connected = false
    end
    
    methods
        function obj = FSAESimClient(host, port)
            if nargin >= 1, obj.Host = host; end
            if nargin >= 2
                obj.Port = port;
            else
                obj.Port = get_rl_server_host_port();
            end
            
            % Javaクラスパス確認（必要ならロード）不要な場合が多い
        end
        
        function connect(obj)
            if obj.Connected
                return;
            end
            try
                % JavaのSocketを使用
                obj.Socket = java.net.Socket(java.lang.String(obj.Host), obj.Port);
                obj.Socket.setSoTimeout(obj.Timeout * 1000);
                obj.Socket.setTcpNoDelay(true); % 必須: Nagleアルゴリズム無効化
                
                % ストリーム準備
                obj.InputStream = obj.Socket.getInputStream();
                obj.OutputStream = obj.Socket.getOutputStream();
                obj.DataInputStream = java.io.DataInputStream(obj.InputStream);
                obj.DataOutputStream = java.io.DataOutputStream(obj.OutputStream);
                
                obj.Connected = true;
                fprintf('Connected to %s:%d (Java Socket TCP_NODELAY)\n', obj.Host, obj.Port);
            catch e
                error('Java Socket Connect Failed: %s', e.message);
            end
        end
        
        function disconnect(obj)
            if ~obj.Connected, return; end
            try
                obj.DataOutputStream.writeByte(int8(obj.CMD_QUIT));
                obj.DataOutputStream.flush();
                pause(0.01);
                
                obj.DataInputStream.close();
                obj.DataOutputStream.close();
                obj.Socket.close();
            catch
            end
            obj.Connected = false;
        end
        
        function disconnectQuiet(obj)
            % CMD_QUITを送らずにソケットのみ閉じる（サーバーは終了しない）
            if ~obj.Connected, return; end
            try
                obj.DataInputStream.close();
                obj.DataOutputStream.close();
                obj.Socket.close();
            catch
            end
            obj.Connected = false;
        end
        
        function trackLength = init(obj, trackName, paramsJson)
            if ~obj.Connected, obj.connect(); end
            if nargin < 3, paramsJson = ''; end
            
            % CMD_INIT
            obj.DataOutputStream.writeByte(int8(obj.CMD_INIT));
            
            % トラック名送信
            obj.sendString(trackName);
            
            % パラメータJSON送信
            obj.sendString(paramsJson);
            obj.DataOutputStream.flush();
            
            % レスポンス受信
            rsp = typecast(int8(obj.DataInputStream.readByte()), 'uint8');
            if rsp ~= obj.RSP_OK
                error('INIT failed: response=0x%02X', rsp);
            end
            
            % double 1つ(8バイト)を受信
            trackLength = obj.readDoubleArray(1);
        end
        
        function [obs, info] = step(obj, steer, throttle, brake)
            % CMD_STEP + 3 doubles
            obj.DataOutputStream.writeByte(int8(obj.CMD_STEP));
            obj.sendDoubleArray([steer, throttle, brake]);
            obj.DataOutputStream.flush();
            
            % レスポンス受信
            rsp = typecast(int8(obj.DataInputStream.readByte()), 'uint8');
            if rsp ~= obj.RSP_OBS
                error('STEP failed: response=0x%02X', rsp);
            end
            
            % OBSとINFOをまとめて受信
            data = obj.readDoubleArray(obj.OBS_DIM + obj.INFO_DIM);
            obs = data(1:obj.OBS_DIM);
            info = data(obj.OBS_DIM+1:end);
        end
        
        function obs = reset(obj)
            obj.DataOutputStream.writeByte(int8(obj.CMD_RESET));
            obj.DataOutputStream.flush();
            
            rsp = typecast(int8(obj.DataInputStream.readByte()), 'uint8');
            if rsp ~= obj.RSP_OBS
                error('RESET failed: response=0x%02X', rsp);
            end
            obs = obj.readDoubleArray(obj.OBS_DIM);
        end
        
        function setParams(obj, paramsJson)
            obj.DataOutputStream.writeByte(int8(obj.CMD_PARAM));
            obj.sendString(paramsJson);
            obj.DataOutputStream.flush();
            
            rsp = typecast(int8(obj.DataInputStream.readByte()), 'uint8');
            if rsp ~= obj.RSP_OK
                error('PARAM failed');
            end
        end
        
        function delete(obj)
            if obj.Connected
                obj.disconnect();
            end
        end
    end
    
    methods (Access = private)
        function sendString(obj, s)
            len = uint32(length(s));
            % unsigned int32をビッグエンディアン(Java標準)で送信するには逆順かDataOutputStream.writeIntを使うが、
            % サーバー側は送信順(LEかBEか)に依存。
            % 元のtcpclientはシステムエンディアン(X86/ARMではリトルエンディアン)
            
            bytes = typecast(len, 'int8');
            obj.DataOutputStream.write(bytes, 0, 4); % 4バイト
            
            if len > 0
                strBytes = int8(s);
                obj.DataOutputStream.write(strBytes, 0, length(strBytes));
            end
        end
        
        function sendDoubleArray(obj, vals)
            % valsをシステムエンディアンのバイト列にし送信
            bytes = typecast(double(vals), 'int8');
            obj.DataOutputStream.write(bytes, 0, length(bytes));
        end
        
        function vals = readDoubleArray(obj, count)
            % count個のdoubleベクトルを受信
            % MATLABはJavaメソッドにint8配列を値渡しするため、
            % java.nio.channels を使い全データをJava内で保持する。
            byteCount = count * 8;
            
            % Java NIO ByteBuffer（ネイティブバイトオーダー）
            byteBuf = java.nio.ByteBuffer.allocate(byteCount);
            byteBuf.order(java.nio.ByteOrder.nativeOrder());
            
            % InputStreamをChannelでラップし、ByteBufferへ直接読み込み
            chan = java.nio.channels.Channels.newChannel(obj.InputStream);
            while byteBuf.hasRemaining()
                n = chan.read(byteBuf);
                if n < 0
                    error('Stream closed unexpectedly');
                end
            end
            
            % DoubleBufferとして読み出し
            byteBuf.flip();
            doubleBuf = byteBuf.asDoubleBuffer();
            vals = zeros(1, count);
            for i = 1:count
                vals(i) = doubleBuf.get();
            end
        end
    end
end
