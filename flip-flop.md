## Mesure de la période d'exécution des opérations - mise en oeuvre d'un signal de type « flip flop »
Opération réalisé en septembre 2022

Nous avion commencé la mise en oeuvre de ce projet lors de l'une de nos rencontre du **mois de septembre**.<br>
À cette occasion nous avons installé le premier « flipflop » au début de la boucle principale **loop()** dans le module **kva.ino** en dirigeant la sortie du flip flop vers le port 37 du Uno. Pour chaque mode d'opération (STANDBY, FREE_RUN et TELEOP), nous avons mesuré les statistiques de la bascule du port 37 à l'aide d'un oscilloscope en portant notre attention sur la « période » p du signal.<br><br>
Nous avons trouvé que p semble dépendre essentiellement de la valeur du délai d em ms, ```delay(d);``` introduit à la toute fin de la boucle loop() et ce, **peu importe de mode d'opération**. En d'autres termes, **p** suivait de très près la valeur de **d** introduit dans la boucle. Par ailleurs, nous avons trouvé un léger excédent (une fraction de ms) entre p et d et nous avons conclus que cette différence représente la temps nécessaire pour exécuter les opérations en rapport au mode d'opération en cours.<br><br>
Nous avons aussi découvert que le mode TELEOP de kva devient inopérant si un délai inférieur à 20 ms est introduit. Nous n'avons pas déterminé l'effet qui se produit pour les autres modes d'opération.

Il semble donc que l'introduction d'un flip flop est une bonne façon d'évaluer les temps d'exécution des différents modes d'opérations. Nous aimerions rafiner cette procédure afin de mesurer les temps d'exécution des différentes fonctionnalités de kva :
* rafraichissement et lecture du HMI
* traitement du mode TELEOP

Ceci en vu de déterminer les mise en oeuvre suivantes :
* traitement des données des capteurs de proximité
* traitrement des données du télémètre
* 
... à suivre :
