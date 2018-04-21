TODO
====

* gérer collisions :
    * calculer point de contact et frame lors de collision Box-Box et lors de l'utilisation de GJK.
    * améliorer critère d'arrêt GJK.
    * Où définir les coefficients de restitution ?
    
* fonctionnalité UI :
    * charger une scène depuis preset.
    * décider la méthode d'intégration numérique depuis l'UI.
    * décider des paramètres physiques (gravité, viscosité) du monde depuis l'UI.

* visu :
    * texturer les objets pour faciliter visualisation du mouvement.

* mettre en place multithreading (1 thread UI et 1 thread physics).
* supprimer utilisation des pointeurs intelligents (inutile et couteux).
