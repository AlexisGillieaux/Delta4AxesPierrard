import argparse

import serial


def demander_entier(invite, minimum, maximum):
    while True:
        try:
            valeur = int(input(f"{invite} > "), 10)
            if minimum <= valeur <= maximum:
                return valeur
        except ValueError:
            pass
        print(f"Entrez un entier entre {minimum} et {maximum}.")


def main():
    parser = argparse.ArgumentParser(description="Commander un variateur via la Teensy.")
    parser.add_argument("--port", default="/dev/serial0", help="Port UART du Raspberry Pi")
    args = parser.parse_args()

    try:
        with serial.Serial(args.port, baudrate=115200, timeout=10, write_timeout=2) as liaison:
            print(f"Connecté à {args.port}. Ctrl+C pour quitter.")
            while True:
                moteur = demander_entier("Numéro du moteur (adresse Modbus)", 1, 247)
                position = demander_entier("Position absolue (pas du variateur)", -(2**31), 2**31 - 1)
                vitesse = demander_entier("Vitesse (tr/min)", 1, 8000)
                acceleration = demander_entier("Accélération (ms)", 0, 65535)
                deceleration = demander_entier("Décélération (ms)", 0, 65535)

                commande = f"DEPLACER_ABSOLU {moteur} {position} {vitesse} {acceleration} {deceleration}\n"
                liaison.write(commande.encode("ascii"))
                reponse = liaison.readline()
                if not reponse.endswith(b"\n"):
                    print("Réponse complète non reçue. Vérifiez l’état du moteur avant de renvoyer une commande.")
                    return 1
                print(reponse.decode("ascii", errors="replace").strip())
    except (KeyboardInterrupt, EOFError):
        print()
        return 0
    except serial.SerialException as erreur:
        print(f"Erreur de liaison série : {erreur}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
