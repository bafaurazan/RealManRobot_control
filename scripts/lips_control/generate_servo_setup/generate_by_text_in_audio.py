import wave
import os
import numpy as np
import matplotlib.pyplot as plt
import speech_recognition as sr
import nltk
from nltk.corpus import cmudict

# Pobranie słownika fonetycznego CMU
nltk.download('cmudict')
phonetic_dict = cmudict.dict()

# Pobranie ścieżki do folderu skryptu
script_dir = os.path.dirname(os.path.abspath(__file__))

# Pobranie listy plików .wav w folderze audio
audio_folder = os.path.join(script_dir, 'audio')
wav_files = [f for f in os.listdir(audio_folder) if f.endswith('.wav')]

if not wav_files:
    raise FileNotFoundError("Brak plików .wav w folderze 'audio/'")

wav_path = os.path.join(audio_folder, wav_files[0])

print(f"Otwarty plik: {wav_path}")

# Przetwarzanie mowy na tekst
recognizer = sr.Recognizer()
with sr.AudioFile(wav_path) as source:
    audio_data = recognizer.record(source)
    try:
        recognized_text = recognizer.recognize_google(audio_data, language="pl-PL")  # Rozpoznawanie tekstu
        print(f"Rozpoznany tekst: {recognized_text}")
    except sr.UnknownValueError:
        print("Nie udało się rozpoznać mowy.")
        recognized_text = ""
    except sr.RequestError:
        print("Błąd połączenia z usługą rozpoznawania.")
        recognized_text = ""

# Konwersja tekstu na fonetykę
def text_to_phonemes(text):
    words = text.lower().split()
    phonemes = []
    for word in words:
        if word in phonetic_dict:
            phonemes.extend(phonetic_dict[word][0])  # Wybór pierwszej wymowy słowa
        else:
            phonemes.append(word)  # Jeśli brak w słowniku, dodaj jako tekst
    return phonemes

phonetic_representation = text_to_phonemes(recognized_text)
print(f"Reprezentacja fonetyczna: {phonetic_representation}")

# Liczenie sylab (przybliżenie na podstawie fonemów)
def count_syllables(phonemes):
    return sum(1 for phoneme in phonemes if phoneme[-1].isdigit())

num_syllables = count_syllables(phonetic_representation)
print(f"Liczba sylab: {num_syllables}")

# Mapowanie liczby sylab na kąt serwa (przykładowa skala)
def map_syllables_to_angle(syllables, min_syllables=1, max_syllables=20):
    return np.interp(syllables, [min_syllables, max_syllables], [0, 180])

servo_angle = map_syllables_to_angle(num_syllables)
print(f"Pozycja serwa: {servo_angle:.2f} stopni")
