# Trajektorienplanung

Benutzung des Hochzeitsprojekts --> verheiraten von Trajektorienberechnung und Regelung von Leader- und Followerrobotern:

- erst roscore und rosrun unicyclesim unicyclesim_node starten

- Trajektorienplanung

    --> findet in leader_move_node.cpp statt: Aufruf mit rosrun hochzeit leader_move_node

    --> Soll-Position wird in den jeweiligen Funktionen berechnet und dort direkt gepublished

    --> bei Trajektorien-Start die Trajektorie initialisieren --> diese werden dann an die jeweiligen Funktionen übertragen

    --> zum Testen am besten keine Funktion laufen lassen, sondern mit "stationären Koordinaten" arbeiten


- Soll-Ist-Regler

    --> findet in hochzeit.cpp statt: Aufruf mit rosrun hochzeit hochzeit_node

    --> hier wird der Regler aufgerufen, der die Soll-Position mit der Ist-Position vergleicht

    --> Soll-Position heißt: targetValLeader

    --> Ist-Position heißt: currentValLeader
    
    - TwistLeader --> hier wird angular.z und linear.x gespeichert und gepublished
    - P2P_controller.cpp --> hier ist der Punkt zu Punkt Regler Code zu finden
    - init_leader.cpp --> hier kann man die Leader-Spawn Funktion aufrufen und so den Leader spawnen
    - utility.cpp --> hier sind die Hilfsfunktionen zu finden, die z.B. für Umrechnungen von Rad zu Degree benötigt werden
    - trajec.h --> hier sind die Header-Dateien der Funktionen zu finden, die die Soll-Positionen des Leaders berechnen und auch publishen (Linie, Kreis und Ellipse)


Am besten erst die Trajektorienplanung im Terminal starten und dann erst die Soll-Ist-Regelung
