
import serial
import struct

# Configuration du port série
ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)

def envoyer_matrice_trajectoire(points):

    nb_points = len(points)
    if nb_points > 255:
        raise ValueError("La matrice ne peut pas dépasser 255 points d'un coup.")

    # 1. En-tête de début (Header) + Nombre de points
    payload = bytearray([0xAA, 0xBB, nb_points])

    # 2. Ajout des coordonnées de la matrice (conversion des floats en 4 octets chacun)
    donnees_brutes = bytearray()
    for pt in points:
        # 'ffff' signifie 4 floats (X, Y, Z, T)
        donnees_brutes += struct.pack('<ffff', pt[0], pt[1], pt[2], pt[3])
    
    payload += donnees_brutes

    # 3. Calcul du Checksum (somme simple de toutes les données brutes)
    checksum = sum(donnees_brutes) & 0xFF
    payload.append(checksum)

    # 4. En-tête de fin (Footer)
    payload += bytearray([0xCC, 0xDD])

    # Envoi de la trame complète sur la liaison série
    ser.write(payload)