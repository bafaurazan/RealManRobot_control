import wave
import os
import numpy as np

script_dir = os.path.dirname(os.path.abspath(__file__))
generated_folder = os.path.join(script_dir, 'generated')
os.makedirs(generated_folder, exist_ok=True)
output_file_path = os.path.join(generated_folder, 'servo_movements.txt')
vector_output_file_path = os.path.join(generated_folder, 'servo_angles.txt')
time_output_file_path = os.path.join(generated_folder, 'timestamps.txt')

audio_folder = os.path.join(script_dir, 'audio')

wav_files = [f for f in os.listdir(audio_folder) if f.endswith('.wav')]

if not wav_files:
    raise FileNotFoundError("Brak plików .wav w folderze 'audio/'")

wav_path = os.path.join(audio_folder, wav_files[0])

audio = wave.open(wav_path, 'rb')

print(f"Otwarty plik: {wav_path}")
print(audio.getparams())

# Podstawowe parametry audio z pliku
number_of_channels = audio.getnchannels()
audio_width = audio.getsampwidth()  # 3 bajty = 24 bity
audio_sample_rate = audio.getframerate()  # frequency
audio_num_frames = audio.getnframes()
audio_wave = audio.readframes(-1)

print(f"channels: {number_of_channels}, sample rate: {audio_sample_rate}, num frames: {audio_num_frames}, sample width: {audio_width}")

audio_time = audio_num_frames / audio_sample_rate
print(f"time: {audio_time} seconds")

# Szybsza konwersja 24-bitowych danych audio za pomocą NumPy
if audio_width == 3:
    raw_bytes = np.frombuffer(audio_wave, dtype=np.uint8)
    # Wektorowa konwersja 24-bitowych próbek
    signal_array = (
        raw_bytes[0::3].astype(np.int32) << 8 |
        raw_bytes[1::3].astype(np.int32) << 16 |
        raw_bytes[2::3].astype(np.int32) << 24
    ).view(np.int32) >> 8  # Przesunięcie bitowe i konwersja na int32
    signal_array = signal_array.reshape(-1, number_of_channels)[:, 0]  # Wybierz jeden kanał
else:
    raise ValueError("Skrypt obsługuje tylko sampwidth=3 (24 bity) na chwilę obecną.")

# Sprawdzenie zgodności rozmiaru
if len(signal_array) != audio_num_frames:
    raise ValueError(f"Rozmiar signal_array ({len(signal_array)}) nie zgadza się z audio_num_frames ({audio_num_frames})")

# Usunięto generowanie wykresu dla przyspieszenia
# times = np.linspace(0, audio_time, num=audio_num_frames)
# plt.plot(times, signal_array)
# plt.xlim(0, audio_time)
# plt.show()

# Funkcja do mapowania amplitudy na kąt serwa (0-180 stopni)
def map_amplitude_to_angle(amplitude, min_amplitude, max_amplitude):
    return np.interp(amplitude, [min_amplitude, max_amplitude], [0, 180])

# Definiowanie zakresów czasowych do ustawienia domyślnej pozycji serwa
default_angle = 60
excluded_intervals = [(0, 11.80),
                      (30.80, 39.20),
                      (65.90, 71.20),
                      (90.00, 110.20)]

chunk_size = int(audio_sample_rate * 0.1)  # Przetwarzaj co 100 ms
min_amplitude = np.min(signal_array)
max_amplitude = np.max(signal_array)

# Wektorowe obliczenia dla czasów i kątów
times = np.arange(0, len(signal_array), chunk_size) / audio_sample_rate
angles = np.full(len(times), default_angle, dtype=np.float32)

# Wektorowe obliczanie amplitudy dla chunków
for i in range(len(times)):
    if not any(start <= times[i] <= end for start, end in excluded_intervals):
        chunk = signal_array[i * chunk_size:(i + 1) * chunk_size]
        if len(chunk) > 0:
            avg_amplitude = np.mean(np.abs(chunk))
            angles[i] = map_amplitude_to_angle(avg_amplitude, min_amplitude, max_amplitude)

# Przygotowanie danych do zapisu
output_lines = [f"Czas: {t:.2f} s, Pozycja serwa: {a:.2f} stopni" for t, a in zip(times, angles)]

# Zapis do plików
print("\nSymulacja ruchów serwa:")
with open(output_file_path, 'w') as output_file, open(time_output_file_path, 'w') as time_file:
    output_file.write("\n".join(output_lines) + "\n")
    time_file.write("\n".join(f"{t:.2f}" for t in times) + "\n")

# Zapis wektora kątów do pliku tekstowego
np.savetxt(vector_output_file_path, angles, fmt='%.2f')

print(f"Wyniki zapisano w {output_file_path}")
print(f"Wektor kątów zapisano w {vector_output_file_path}")
print(f"Czasy zapisano w {time_output_file_path}")