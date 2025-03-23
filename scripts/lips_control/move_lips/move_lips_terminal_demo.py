import os
import numpy as np
import time
import argparse
from pydub import AudioSegment
import simpleaudio as sa

# Konfiguracja parsera argumentów
parser = argparse.ArgumentParser(description="Odtwarzanie audio i wyświetlanie pozycji serwa.")
parser.add_argument(
    '--start_time', type=float, default=0,
    help="Czas w sekundach, od którego ma rozpocząć się odtwarzanie audio (domyślnie 0)."
)
args = parser.parse_args()

# Pobranie ścieżki do pliku
script_dir = os.path.dirname(os.path.abspath(__file__))
generated_folder = os.path.join(script_dir, '../generate_servo_setup/generated')
audio_folder = os.path.join(script_dir, '../generate_servo_setup/audio')
vector_file_path = os.path.join(generated_folder, 'servo_angles.txt')
time_file_path = os.path.join(generated_folder, 'timestamps.txt')

# Znalezienie pliku audio
wav_files = [f for f in os.listdir(audio_folder) if f.endswith('.wav')]
if not wav_files:
    raise FileNotFoundError("Brak plików .wav w folderze 'audio'")

wav_path = os.path.join(audio_folder, wav_files[0])

# Sprawdzenie, czy pliki istnieją
if not os.path.exists(vector_file_path):
    raise FileNotFoundError(f"Brak pliku: {vector_file_path}")
if not os.path.exists(time_file_path):
    raise FileNotFoundError(f"Brak pliku: {time_file_path}")

# Wczytanie danych do wektora
angles = np.loadtxt(vector_file_path)
times = np.loadtxt(time_file_path)

# Sprawdzenie, czy liczba kątów i czasów jest zgodna
if len(angles) != len(times):
    raise ValueError("Liczba kątów i czasów musi się zgadzać!")

# Funkcja do znalezienia indeksu pierwszego czasu, który jest >= start_time
def find_start_index(start_time, times):
    for i, t in enumerate(times):
        if t >= start_time:
            print(f"indeks: {i}, linijka: {i + 1}")
            return i
    return len(times) - 1  # Jeśli nie znaleziono, zwróć ostatni indeks

# Znajdź pierwszy czas, który jest >= start_time
start_index = find_start_index(args.start_time, times)

# Wczytanie pliku audio przy użyciu pydub
audio = AudioSegment.from_wav(wav_path)

# Konwersja do 16-bitowego formatu, aby uniknąć problemów z 24-bitowym audio
audio = audio.set_sample_width(2)  # Ustawienie szerokości próbki na 2 bajty (16 bitów)

# Przycięcie audio od wskazanej sekundy
start_ms = int(args.start_time * 1000)  # Konwersja na milisekundy
audio = audio[start_ms:]

# Diagnostyka parametrów audio
print(f"Channels: {audio.channels}, Sample width: {audio.sample_width}, Frame rate: {audio.frame_rate}")

# Odtworzenie audio
play_obj = sa.play_buffer(
    audio.raw_data,
    num_channels=audio.channels,
    bytes_per_sample=audio.sample_width,
    sample_rate=audio.frame_rate
)

# Wyświetlenie zawartości wektora w odpowiednich czasach
print(f"Rozpoczęcie odtwarzania od {args.start_time} sekund audio...")
start_time = time.time()  # Czas początkowy dla synchronizacji

# Synchronizacja z wybranym czasem startowym
for i in range(start_index, len(angles)):
    # Czas, kiedy należy wyświetlić kąt
    expected_time = times[i] - args.start_time  # Dostosowanie czasu do start_time
    
    # Czekaj, aż nadejdzie odpowiedni czas
    sleep_time = expected_time - (time.time() - start_time)
    
    if sleep_time > 0:
        time.sleep(sleep_time)  # Czekaj, aż nadejdzie odpowiedni czas
    
    # Wyświetl kąt serwa
    print(f"Czas: {times[i]:.2f} s, Pozycja serwa: {angles[i]:.2f} stopni")

# Oczekiwanie na zakończenie odtwarzania
play_obj.wait_done()