import os
import numpy as np
import time

# Pobranie ścieżki do pliku
script_dir = os.path.dirname(os.path.abspath(__file__))
generated_folder = os.path.join(script_dir, '../generate_servo_setup/generated')
vector_file_path = os.path.join(generated_folder, 'servo_angles.txt')
time_file_path = os.path.join(generated_folder, 'timestamps.txt')

# Sprawdzenie, czy pliki istnieją
if not os.path.exists(vector_file_path):
    raise FileNotFoundError(f"Brak pliku: {vector_file_path}")
if not os.path.exists(time_file_path):
    raise FileNotFoundError(f"Brak pliku: {time_file_path}")

# Wczytanie danych z plików
angles = np.loadtxt(vector_file_path)
times = np.loadtxt(time_file_path)

# Sprawdzenie, czy liczba kątów i czasów jest zgodna
if len(angles) != len(times):
    raise ValueError("Liczba kątów i czasów musi się zgadzać!")

# Wyświetlenie zawartości wektora w odpowiednich czasach
print("Wyświetlanie kątów serwa w odpowiednich czasach:")
start_time = time.time()  # Czas początkowy dla synchronizacji
for i in range(len(angles)):
    # Czas, kiedy należy wyświetlić kąt
    expected_time = times[i]
    # Obliczenie opóźnienia, które zapewni synchronizację
    sleep_time = expected_time - (time.time() - start_time)
    
    if sleep_time > 0:
        time.sleep(sleep_time)  # Czekaj, aż nadejdzie odpowiedni czas
    
    print(f"Czas: {expected_time:.2f} s, Pozycja serwa: {angles[i]:.2f} stopni")
