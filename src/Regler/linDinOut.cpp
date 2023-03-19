#include "linDinOut.h"


/*
theta => Ausrichtungswinkel
dtheta => 1.Ableitung des Ausrichtungswinkel
ddtheta => 2.Ableitung des Ausrichtungswinkel
x => soll X-Werte der Trajektorie
y => soll Y-Werte der Trajektorie
dx => 1.Ableitung der soll X-Werte der Trajektorie
dy => 1.Ableitung der soll Y-Werte der Trajektorie
ddx => 2.Ableitung der soll X-Werte der Trajektorie
ddy => 2.Ableitung der soll Y-Werte der Trajektorie
kp1 => Einstellparameter für x-Werte
kp2 => Einstellparameter für y-Werte
kd1=> Einstellparameter für x-Werte
kd2 => Einstellparameter für y-Werte
T => Periodendauer
i_x => ist X-Werte der Trajektorie
i_y => ist Y-Werte der Trajektorie
i_dx => 1.Ableitung der ist X-Werte der Trajektorie
i_dy => 1.Ableitung der ist Y-Werte der Trajektorie
*/


float ableitung(float x, float x1){
float dx = (x - x1)/50;
return dx;
}

vector<float> leader(float theta, float x, float y, float i_x, float i_y, float i_theta, float kp1, float kp2, float kd1, float kd2, float T) {

    static float dx = 0;
    static float dy = 0;
    static float ddx = 0;
    static float ddy = 0;
    //static float dtheta = 0; 
    //static float ddtheta = 0; 
    static float v_alt = 0.1;
    static float a_alt = 0;
    static float i_dx = 0;
    static float i_dy = 0;



    //Ausgabewerte Zustandsregler
    float u1 = ddx + kp1 * (x - i_x) + kd1 * (dx - i_dx);
    float u2 = ddy + kp2 * (y - i_y) + kd2 * (dy - i_dy);

    //Beschleunigung
    float a = u1 * cos (theta) + u2 * sin (theta);

    //Winkelgeschwindigkeit
    float w = (-u1 * sin (theta) + u2 * cos (theta)) / v_alt;

    //Geschwindigkeit
    float v = (T * a + T * a_alt + 2 * v_alt) / 2;



    //ungültige Werte abfangen:
    if(!std::isfinite(v)){
        ROS_ERROR("not finite value for v");
        v = 0.0;
    }
    if(!std::isfinite(w)){
        ROS_ERROR("not finite value for w");
        w = 0.0;
    }
    //Stellbegrenzung:
    if(v > 10){
        v = 10;
    }else if(v < -10){
        v = -10;
    }

    if(w > 10){
        w = 10;
    }else if(w < -10){
        w = -10;
    }



    //Parameter umspeichern fuer naechsten Durchlauf
    if(v == 0)
        v = 0.01;

    v_alt = v;
    a_alt = a;






    // Abspeichern der Sollposition in eine neue Datenstruktur i_pos_d
	//Trajectory i_pos_d;
    dx = v_alt * cos(i_theta);     //berechnen der Ableitungen dx (aus vorwärtsgeschwindigkeit v und winkel, siehe Matlab grüner Block)
    dy = v_alt * sin(i_theta);		// berechnen der Ableitung dy (-"-)
    
    x = i_x;	// Umspeichern der Position
    y = i_y;	// umspeichern der y-Position
    theta = i_theta; // Umspeichern des Winkels










    //ddx = ableitung(dx, ddx);
    //ddy = ableitung(dy, ddy);
    ////ddtheta = ableitung(dtheta, ddtheta);
    //dx = ableitung(x, dx);
    //dy = ableitung(y, dy);
    ////dtheta = ableitung(theta, dtheta);
    //i_dx = ableitung(i_x, i_dx);
    //i_dy = ableitung(i_y, i_dy);



    cout << "v: " << v << "   w: " << w << endl;
    cout << "dx: " << dx << "   ddx: " << ddx << endl;





    //BERNHARD
    ///@brief first element := linear velocity; second := angular velocity
    vector<float> returnVelocities;
    returnVelocities.push_back(v);
    returnVelocities.push_back(w);
    
    return returnVelocities;
}
