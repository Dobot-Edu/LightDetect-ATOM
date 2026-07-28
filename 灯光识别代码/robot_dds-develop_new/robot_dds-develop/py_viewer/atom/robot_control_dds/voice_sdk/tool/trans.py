import librosa
import soundfile as sf

# 加载原始音频文件
original_audio, sr = librosa.load('dong.wav', sr=None)

# 转换采样率
converted_audio = librosa.resample(original_audio, orig_sr=sr, target_sr=16000)

# 保存转换后的音频文件
sf.write('dong2.wav', converted_audio, 16000)