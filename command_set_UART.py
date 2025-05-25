import serial
import matplotlib.pyplot as plt
import time

# --- 設定 ---
SERIAL_PORT = 'COM3'  # マイコンが接続されているCOMポート名 (Windowsの場合)
                      # Linuxの場合は '/dev/ttyUSB0' や '/dev/ttyACM0' など
                      # macOSの場合は '/dev/cu.usbserial-XXXX' など
BAUD_RATE = 115200    # マイコンのボーレートに合わせてください
TIMEOUT = 1           # 読み取りタイムアウト（秒）

# マイコンへ送信するコマンド
COMMAND_TO_SEND = "LISTEN SINC=3 OV=125 IOSR=1 CLKDIV=8 RBS=2"

header = "Sending audio data..."
footer = "Data printing complete. Ready for next command."
audio_data_values = []
sample_prefix = "Sample "

collecting_data = False
samples_collected = 0
max_samples_to_collect = 16000 # まずは8個のデータ

ser = None # シリアルポートオブジェクトを初期化

try:
    # シリアルポートを開く
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
    print(f"シリアルポート {SERIAL_PORT} を開きました。ボーレート: {BAUD_RATE}")
    time.sleep(2) # ポートが開くのを少し待ち、マイコンがリセットされる可能性に備える

    # マイコンにコマンドを送信
    print(f"コマンドを送信します: '{COMMAND_TO_SEND}'")
    ser.write((COMMAND_TO_SEND + '\n').encode('utf-8')) # コマンドの終端に改行を追加し、UTF-8でエンコードして送信
    # マイコンがコマンドを処理するのに少し時間を与える (必要に応じて調整)
    time.sleep(0.5)

    print("データ受信待機中...")
    # データ受信ループ
    # ループの終了条件をより明確にするため、フッター受信かタイムアウトを考慮
    # ここでは、データ収集が完了したらループを抜けるようにします。
    # 実際の運用では、より堅牢なタイムアウト処理が必要になる場合があります。
    
    # 受信開始時刻
    start_time_receiving = time.time()
    max_wait_time_after_command = 30 # コマンド送信後、データ受信を待つ最大時間（秒）

    while True:
        # タイムアウトチェック (コマンド送信後、長時間データが来ない場合)
        if not collecting_data and (time.time() - start_time_receiving > max_wait_time_after_command):
            print(f"{max_wait_time_after_command}秒経過しましたが、ヘッダー '{header}' を受信できませんでした。")
            break

        if ser.in_waiting > 0: # 受信バッファにデータがあれば
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line: # 空行はスキップ
                    continue
                print(f"受信: {line}") # デバッグ用に受信行を表示
            except Exception as e: # readlineやdecodeで予期せぬエラーが発生した場合
                print(f"行の読み取り/デコード中にエラー: {e}")
                continue

            if header in line:
                if not collecting_data: # まだ収集中でなければ開始
                    collecting_data = True
                    audio_data_values = [] # 新しいデータセットのためにリストをクリア
                    samples_collected = 0
                    print("ヘッダーを検出しました。データ収集を開始します。")
                continue # ヘッダー行自体はデータではないのでスキップ

            if collecting_data:
                if footer in line:
                    print("フッターを検出しました。データ収集を終了します。")
                    collecting_data = False
                    break # データ収集が完了したのでループを抜ける

                if line.startswith(sample_prefix) and samples_collected < max_samples_to_collect:
                    try:
                        parts = line.split(':')
                        value_str = parts[1].strip()
                        audio_value = int(value_str)
                        audio_data_values.append(audio_value)
                        samples_collected += 1
                        print(f"受信データ {samples_collected}/{max_samples_to_collect}: {audio_value}")
                    except (IndexError, ValueError) as e:
                        print(f"エラー: オーディオデータの解析に失敗しました。行: '{line}', エラー: {e}")
                
                if samples_collected >= max_samples_to_collect:
                    print(f"{max_samples_to_collect}個のデータを収集しました。フッターを待ちます（または次のデータ処理へ）。")
                    # 8個集めたら、フッターを待つか、ここで抜けても良い。
                    # ここではフッターも待つ設計とするが、もしフッターが来ない場合はタイムアウトが必要
                    # collecting_data = False # フッターを待たずに終了する場合はコメントアウトを解除
                    # break
        else:
            # データがない場合は少し待つ
            time.sleep(0.01)
            # もし収集中にデータが途絶えた場合のタイムアウト処理もここに追加できる
            if collecting_data and samples_collected > 0:
                # 例えば、最後のデータ受信から一定時間経過したら終了など
                pass


except serial.SerialException as e:
    print(f"シリアルポートエラー: {e}")
except KeyboardInterrupt:
    print("手動で中断しました。")
finally:
    if ser and ser.is_open:
        ser.close()
        print(f"シリアルポート {SERIAL_PORT} を閉じました。")

# --- 収集したデータをプロット ---
if audio_data_values:
    print("\n収集したオーディオデータ:")
    print(audio_data_values[0:10])
    plt.figure(figsize=(10, 6))
    plt.plot(audio_data_values, marker='o')
    # プロットするサンプル数をタイトルに動的に表示
    plt.title(f'Received Audio Data (First {len(audio_data_values)} Samples)')
    plt.xlabel('Sample Index (0-indexed)')
    plt.ylabel('Audio Value')
    plt.grid(True)
    plt.show()
else:
    print("プロットするオーディオデータがありません。")