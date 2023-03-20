# Trajektorienplanung

Benutzung des Hochzeitsprojekts --> verheiraten von Trajektorienberechnung und Regelung von Leader- und Followerrobotern:


- erst *roscore* und *rosrun unicyclesim unicyclesim_node* starten


- Trajektorienplanung
	
    --> in leader_move_node.cpp: Aufruf mit *rosrun hochzeit leader_move_node*
	
	--> soll position wird in den jeweiligen Funktionen berechnet und auch dort direkt gepublished
	
    --> bei Trajektorien-Start die Trajektorie initialisieren --> die werden an die jeweiligen Funktionen übertragen
	
    --> zum Testen am besten keine Funktion laufen lassen, sondern mit "stationären Koordinaten" arbeiten
	
	
	
- Soll-Ist-Regler
	
    --> in hochzeit.cpp: Aufruf mit *rosrun hochzeit hochzeit_node*
	
	--> hier wird der Regler aufgerufen, der die Soll-Position mit der Ist-Position vergleicht
	
    --> Soll-Position heißt: targetValLeader
	
    --> Ist-Position heißt: currentValLeader
	




	
- TwistLeader --> hier wird angular.z und linear.x gespeichert und gepublished
- init.cpp --> hier kann man die Leader-Spawn Funktion aufrufen und so den Leader spawnen
- trajec.cpp --> hier sollen die Funktionen aufgerufen werden, die die Soll-Positionen des Leaders berechnen (und auch publishen)
	
	
Am besten erst die Trajektorienplanung im Terminal starten und dann erst die Soll-Ist-Regelung