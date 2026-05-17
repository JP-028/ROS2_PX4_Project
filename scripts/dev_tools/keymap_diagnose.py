#!/usr/bin/env python3
import sys
import termios
import tty
import time

KEY_NAMES = {
    "w": "w = wahrscheinlich vorwärts",
    "a": "a = wahrscheinlich links",
    "s": "s = wahrscheinlich rückwärts",
    "d": "d = wahrscheinlich rechts",
    "i": "i = eventuell vorwärts",
    "j": "j = eventuell drehen links",
    "k": "k = eventuell stop / zurück",
    "l": "l = eventuell drehen rechts",
    " ": "Leertaste = Stop / Not-Aus möglich",
    "\r": "Enter",
    "\n": "Enter",
    "\x03": "CTRL+C",
    "\x1b[A": "Pfeil hoch",
    "\x1b[B": "Pfeil runter",
    "\x1b[C": "Pfeil rechts",
    "\x1b[D": "Pfeil links",
}

def read_key():
    first = sys.stdin.read(1)

    # Pfeiltasten starten meistens mit ESC
    if first == "\x1b":
        second = sys.stdin.read(1)
        third = sys.stdin.read(1)
        return first + second + third

    return first

def pretty_key(key):
    if key in KEY_NAMES:
        return KEY_NAMES[key]
    return f"Unbekannte Taste: {repr(key)} / ASCII: {[ord(c) for c in key]}"

print("")
print("======================================")
print(" KEYMAP DIAGNOSE")
print("======================================")
print("")
print("Drücke jetzt nacheinander deine Steuer-Tasten.")
print("Empfohlen:")
print("w a s d")
print("i j k l")
print("Leertaste")
print("Pfeiltasten")
print("")
print("Mit q beenden.")
print("")

fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
pressed = []

try:
    tty.setraw(fd)

    while True:
        key = read_key()

        if key == "q":
            break

        pressed.append(key)

        print("")
        print("--------------------------------------")
        print("Erkannt:", pretty_key(key))
        print("Raw:", repr(key))
        print("ASCII:", [ord(c) for c in key])
        print("--------------------------------------")

finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

print("")
print("======================================")
print(" ZUSAMMENFASSUNG")
print("======================================")

if not pressed:
    print("Keine Tasten erkannt.")
else:
    unique = []
    for key in pressed:
        if key not in unique:
            unique.append(key)

    for key in unique:
        print(f"{repr(key):12} -> {pretty_key(key)}")

print("")
print("Fertig.")
