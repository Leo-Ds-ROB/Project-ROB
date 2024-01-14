# Project-INFO

Voici notre proposition pour le projet d'informatique de robot pathfinder.
Le principe de notre algorithme est de se faire déplacer le robot dans un premier temps par un principe de wall following , dans notre cas le mur de gauche.
Si le robot ne trouve pas la case noir par ce principe au bout d'un certain temps celui-ci passe en mode mouvement aléatoire où ses déplacements sont désormais à la manière d'une bille qui rebondirait sur chaque mur.
La variable "Taille" est à changer suivant le labyrinthe , exemple pour un labyrinthe de taille 5x5 affectez la variable à 35. En clair affectez la variable avec le résultat de (largeur * longueur) + 10.
