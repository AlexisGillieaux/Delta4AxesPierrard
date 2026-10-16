import serial
import time
teensy = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)  # laisse le temps au Teensy de démarrer
def OuverturePince(pourcentage_ouverture, temps_ms):
    pourcentage_ouverture = max(0, min(100, pourcentage_ouverture))
    temps_ms = max(0, temps_ms)

    message = f"{pourcentage_ouverture},{temps_ms}\n"
    teensy.write(message.encode('utf-8'))

    print("Commande envoyée :", message.strip())
    
def FermeturePince(pourcentage_ouverture, temps_ms):
    pourcentage_ouverture = max(0, min(100, pourcentage_ouverture))
    temps_ms = max(0, temps_ms)

    message = f"{pourcentage_ouverture},{temps_ms}\n"
    teensy.write(message.encode('utf-8'))

    print("Commande envoyée :", message.strip())