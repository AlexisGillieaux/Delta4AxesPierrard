
"""SAUVEGARDE DES FONCTIONS POUR LA GESTION DES POINTS"""
class Point :
    def Add_Point(self, nom, Theta1, Theta2, Theta3, Theta4):
        self.nom = nom
        self.Theta1 = Theta1
        self.Theta2 = Theta2
        self.Theta3 = Theta3
        self.Theta4 = Theta4
        
    
    """
    Maxendre 13/04/26

    La fonction servira à créer un point à partir de ses coordonnées et de son nom.
    ensuite à l'aide du MGD on pourra calculer les coordonnées cartésiennes du point.
    ------------------------------------------------------------------------------------
    Elle aura en entrée :
        - Les Coordonnées Théta1, Théta2, Théta3, Théta4
        - Le nom du point enregistré
    ------------------------------------------------------------------------------------
    Elle aura en sortie :
        - Un message de confirmation "Point 'XXXXX' enregistré"
    ------------------------------------------------------------------------------------
    """
    pass

    def Affichage_Noms_Points(self):
        """
    Maxendre 13/04/26

    La fonction servira à afficher tous les points enregistrés puis permettre à l'utilisateur d'en choisir un pour afficher ses caractéristiques.
    Liaision avec "Affichage_Point" pour afficher les caractéristiques du point choisi.
    Utilisation d'une boucle for :

    for nom in self.points:
            print(nom)

    ------------------------------------------------------------------------------------
    Elle aura en entrée :
        - rien
    ------------------------------------------------------------------------------------
    Elle aura en sortie :
        - rien, juste l'affichage de tous les points et de leurs caractéristiques
    ------------------------------------------------------------------------------------
    """

    pass
        
    def Affichage_Point(self):
        """
    Maxendre 13/04/26

    La fonction servira à afficher un point particulier et toutes ses caractéristiques
    Utilisation d'une reconnaissance de nom pour afficher le point voulu. (Affichage_Noms_Points)
    Affichage Print des valeurs.
    ------------------------------------------------------------------------------------
    Elle aura en entrée :
        - le nom du point à afficher grâce à la reconnaissance de nom (Affichage_Noms_Points)
    ------------------------------------------------------------------------------------
    Elle aura en sortie :
        - rien, juste l'affichage du point et de ses caractéristiques
    ------------------------------------------------------------------------------------
    """
    
    pass

    def Suppression_Point(self):
        """
    Maxendre 13/04/26

    La fonction servira à supprimer un point particulier et toutes ses caractéristiques
    Utilisation d'une reconnaissance de nom pour supprimer le point voulu. (Affichage_Noms_Points)
    Utilisation de la fonction .pop() pour supprimer le point de la liste.
    ------------------------------------------------------------------------------------
    Elle aura en entrée :
        - le nom du point à supprimer grâce à la reconnaissance de nom (Affichage_Noms_Points)
    ------------------------------------------------------------------------------------
    Elle aura en sortie :
        - un message de confirmation "Point 'XXXXX' supprimé"
    ------------------------------------------------------------------------------------
    """
    pass



class User_Frame:
    def User_Frame(self):

        """Apprentissage du user frame
    On utilisera la méthode des 2 points en forcant le Z vertical et vers le haut car Delta ne peut pas se tourner en X et Y
    Pour la méthode des 2 points on va sauver 1 point (ORIGINE) et 1 point (X) ensuite on va "normer" le vecteur X à 1 unité.
    Ensuite on va pouvoir avoir l'axe Z en créant une perpendiculaire à l'axe X pour aller vers le haut.
    Finalement calculer L'axe Y en faisant le produit vectoriel de Z et X.
    On aura donc les 3 axes X Y Z avec 2 points.
    ------------------------------------------------------------------------------------
    Ensuite utiliser ce frame pour faire les calculs de MGD et MGI en utilisant les coordonnées du user frame au lieu du frame de base.
    ------------------------------------------------------------------------------------
    """

    def Affichage_User_Frame(self):
        """Affichage du user frame
    Il aura la liste de tous les userframe et pourra les ressortir en sortie lors de l'appel de la fonction.
    ------------------------------------------------------------------------------------
    Elle aura en entrée :
        - rien
    ------------------------------------------------------------------------------------
    Elle aura en sortie :
        - l'affichage de tous les user frame enregistrés
    ------------------------------------------------------------------------------------
    """
        

 class Tool_Frame:
    def Tool_Frame(self):
        """Apprentissage du tool frame
    On utilisera la méthode des 3 points 
    ------------------------------------------------------------------------------------
    Ensuite utiliser ce frame pour faire les calculs de MGD et MGI en utilisant les coordonnées du tool frame au lieu du frame de base.
    ------------------------------------------------------------------------------------
    """

    def Affichage_Tool_Frame(self):
        """Affichage du tool frame
    Il aura la liste de tous les toolframe et pourra les ressortir en sortie lors de l'appel de la fonction.
    ------------------------------------------------------------------------------------
    """

    def update_TCP(self):
        """Update du TCP
    Permettra de recalculer les coordonnées par rapport à l'outil et pas au niveau du préhenseur.
    ------------------------------------------------------------------------------------
    Elle aura en entrée :
        - les coordonnées du TCP
    ------------------------------------------------------------------------------------
    Elle aura en sortie :
        - un message de confirmation "TCP mis à jour"
        - Les nouveles coordonnées du TCP
    ------------------------------------------------------------------------------------
    elle aura évidemment les décalages pour chaque type d'outil ///// A VOIR SI ON RAJOUTE DES FONCTIONS AVEC AJOUT D OUTIL OU SI C EST DEJA PROGRAMMé!!!!
    Mais comme on est dans la classe normakement on aura l'outil déjà enregistré et on pourra faire les calculs de décalage à l'intérieur de la fonction update_TCP
    """
        

    def Check_Boudaries(self):
        """Check des limites de travail
    Permettra de vérifier que les coordonnées du TCP sont dans les limites de travail du robot.
    ------------------------------------------------------------------------------------
    Elle aura en entrée :
        - les coordonnées du TCP
    ------------------------------------------------------------------------------------
    Elle aura en sortie :
        - un message de confirmation "TCP dans les limites de travail" ou "TCP hors des limites de travail"
    ------------------------------------------------------------------------------------
    """
        

    def Check_Payload(self):
        """Check de la charge utile
    Permettra de vérifier que la charge utile de l'outil est dans les limites de charge du robot.
    Elle modifiera l'accélération et la vitesse maximale du robot en fonction de la charge utile de l'outil.
    ------------------------------------------------------------------------------------
    Elle aura en entrée :
        - la charge utile de l'outil
    ------------------------------------------------------------------------------------
    Elle aura en sortie :
        - un message de confirmation "Charge utile dans les limites" ou "Charge utile hors des limites"
        - les nouvelles valeurs d'accélération et de vitesse maximale du robot en fonction de la charge utile de l'outil
    ------------------------------------------------------------------------------------
    """
        

class Free_Drive:
    def Free_Drive(self):
        """Free Drive
    Permettra de déplacer le robot manuellement en mode free drive pour apprendre des points ou pour faire des ajustements.
    Le robot en free drive ne répondra pas aux commandes de mouvement et ne fera que suivre les mouvements manuels de l'utilisateur. 
    De plus il commandera les moteurs pour le garder dans les airs à la position souhaitée et éviter qu'il ne tombe ou ne se déplace de manière incontrôlée.
    ------------------------------------------------------------------------------------
    Elle aura en entrée :
        - rien, juste l'activation du mode free drive
    ------------------------------------------------------------------------------------
    Elle aura en sortie :
        - un message de confirmation "Mode Free Drive activé"
    ------------------------------------------------------------------------------------
    """

    def Ajustement_Moteurs (self):
        """Ajustement des moteurs
    Permettra de comparer les capteur de courant de chaque moteur pour calculer à quel point on force sur chaque moteur, prendre en compte cette valeur et bouger le robot en conséquence pour compenser les forces et éviter de forcer sur les moteurs.
    Ensuite, on pourra faire une fonction d'ajustement automatique qui va faire bouger le robot dans les 3 axes et ajuster les moteurs en fonction des forces ressenties pour trouver la position optimale du robot.   
    ------------------------------------------------------------------------------------
    Elle aura en entrée :
        - les valeurs des capteurs de courant de chaque moteur
    ------------------------------------------------------------------------------------
    Elle aura en sortie :
        - l'activation des moteurs pour compenser les forces ressenties
        - les nouvelles positions du robot en fonction des ajustements 
    ------------------------------------------------------------------------------------
    """
        


        















