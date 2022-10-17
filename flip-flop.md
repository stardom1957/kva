## Mesure de la période d'exécution des opérations - mise en oeuvre d'un signal de type « flip flop »
Opération réalisé en septembre 2022

Nous avion commencé la mise en oeuvre de ce projet lors de l'une de nos rencontre du **mois de septembre**.<br>
À cette occasion nous avons installé le premier « flipflop » au début de la boucle principale **loop()** dans le module **kva.ino** en dirigeant la sortie du flip flop vers le port 37 du Uno. Pour chaque mode d'opération (STANDBY, FREE_RUN et TELEOP), nous avons mesuré les statistiques de la bascule du port 37 à l'aide d'un oscilloscope en portant notre attention sur la « période » du signal.<br>
Nous avons trouvé qu'à l'exception de quelques variations mineures, la période du signal dépend essentiellement de la valeur du délai ```delay(10);``` introduit à la toute fin de la boucle loop() et ce, **peu importe de mode d'opération**. En d'autres termes, la période du signal suivait de très près la valeur du délai introduit dans la boucle. Nous avons trouvé une fraction de ms de différence entre la valeur du délai et cette mesurée. Nous pouvons conclure que cette différence représente la temps d'exécution de la boucle.<br>
Nous avons aussi découvert que le mode TELEOP de kva devient inopérant si un délai inférieur à 20 ms est introduit.

... à suivre.
