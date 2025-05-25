import serial
import matplotlib.pyplot as plt
import time
import numpy as np

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
FS = 16000

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
                # print(f"受信: {line}") # デバッグ用に受信行を表示
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
                        # print(f"受信データ {samples_collected}/{max_samples_to_collect}: {audio_value}")
                        print(f"\r受信データ {samples_collected}/{max_samples_to_collect}: {audio_value}            ", end="")
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
    print(f"\n収集したオーディオデータの総数: {len(audio_data_values)}件")
    
    num_samples_to_show_each_end = 5
    if len(audio_data_values) > (2 * num_samples_to_show_each_end):
        print(f"  最初の{num_samples_to_show_each_end}件: {audio_data_values[:num_samples_to_show_each_end]}")
        print(f"  最後の{num_samples_to_show_each_end}件: {audio_data_values[-num_samples_to_show_each_end:]}")
        omitted_count = len(audio_data_values) - (2 * num_samples_to_show_each_end)
        if omitted_count > 0:
            print(f"  (他 {omitted_count} 件のデータは省略)")
    else:
        print(f"  全データ: {audio_data_values}")

    # --- グラフ描画 ---
    N = len(audio_data_values)
    signal = np.array(audio_data_values)
    FS = 16000  # サンプリング周波数 (Hz)

    # FigureオブジェクトとAxesオブジェクトのリストを作成 (2行1列)
    fig, axs = plt.subplots(2, 1, figsize=(10, 9)) # figsizeを調整して2つのグラフを見やすくする

    # 1. 時間領域のオーディオデータプロット
    axs[0].plot(signal, marker='.', linestyle='-') # マーカーを小さくするオプション
    axs[0].set_title(f'Received Audio Data (Total {N} Samples)')
    axs[0].set_xlabel('Sample Index (0-indexed)')
    axs[0].set_ylabel('Audio Value')
    axs[0].grid(True)

    # 2. FFTマグニチュードスペクトルプロット
    if N > 0: # データが1点以上ないとFFTできない/意味がない
        # FFT計算
        yf = np.fft.fft(signal)
        xf = np.fft.fftfreq(N, d=1/FS) # 周波数軸の計算 (dはサンプリング間隔)

        # 正の周波数成分のみ取得 (0Hzからナイキスト周波数まで)
        # Nが偶数の場合 xf_positive は N/2 点、奇数の場合は (N+1)/2 点
        positive_freq_indices = np.where(xf >= 0)[0] 
        # 一般的には N//2 までで十分なことが多いが、fftfreq の出力を正しく扱う
        # xf_positive = xf[positive_freq_indices]
        # magnitude_spectrum = np.abs(yf[positive_freq_indices])

        # より一般的な片側スペクトルの取得 (0からN//2-1番目の要素まで)
        xf_plot = xf[:N//2]
        magnitude_plot = np.abs(yf[:N//2])
        # オプション: スケーリングして振幅スペクトルにする場合
        # magnitude_plot = 2.0/N * np.abs(yf[0:N//2])
        # magnitude_plot[0] = magnitude_plot[0] / 2.0 # DC成分は半分に (上記スケーリングの場合)

        axs[1].plot(xf_plot, magnitude_plot)
        axs[1].set_title('FFT Magnitude Spectrum')
        axs[1].set_xlabel('Frequency (Hz)')
        axs[1].set_ylabel('Magnitude')
        axs[1].grid(True)
        # axs[1].set_xlim([0, FS/2]) # 表示範囲をナイキスト周波数までに制限（通常は自動で良い感じになる）

        # ピーク周波数の検出とマーキング
        if len(magnitude_plot) > 0: # スペクトルデータが存在する場合
            peak_index = np.argmax(magnitude_plot)
            peak_frequency = xf_plot[peak_index]
            peak_magnitude = magnitude_plot[peak_index]

            axs[1].plot(peak_frequency, peak_magnitude, 'x', color='red', markersize=10, markeredgewidth=2,
                        label=f'Peak: {peak_frequency:.2f} Hz')
            axs[1].legend() # ピーク情報のラベルを凡例として表示
            
            # ピーク周波数をテキストで表示する場合（凡例と重複する可能性あり）
            # axs[1].text(peak_frequency, peak_magnitude, f' {peak_frequency:.2f} Hz', 
            #             color='red', va='bottom', ha='left')


    else:
        axs[1].set_title('FFT Magnitude Spectrum')
        axs[1].text(0.5, 0.5, 'No data for FFT', horizontalalignment='center', verticalalignment='center', transform=axs[1].transAxes)


    plt.tight_layout() # サブプロット間のレイアウトを自動調整
    plt.show()
else:
    print("プロットするオーディオデータがありません。")