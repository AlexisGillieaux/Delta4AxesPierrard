import serial
import time

# Configuration du port série du Raspberry Pi
# /dev/serial0 pointe automatiquement vers le port UART matériel principal
ser = serial.Serial('/dev/serial0', baudrate=115200, timeout=1)

time.sleep(2) # Attendre que la connexion s'initialise

ser.write(f"Ecrivez un nombre à envoyer au Teensy : ".encode('utf-8'))
votre_nombre=input()  # Lire le nombre depuis l'entrée standard
try:
    print("Envoi du nombre converti en texte avec un saut de ligne (\\n)")
    ser.write(f"{votre_nombre}\n".encode('utf-8'))
    print(f"Nombre {votre_nombre} envoyé avec succès !")
except serial.SerialException as e:
    print(f"Erreur série : {e}")
finally:
    ser.close()