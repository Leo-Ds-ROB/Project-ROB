#define LIGHT_SENSOR_PIN A0
#define BLACK_THRESHOLD 100 //valeur définissant le seuil de couleur noir
#define RED 310 ////valeur définissant le seuil de couleur rouge
#define PROX_SENSOR_L_PIN A1
#define PROX_SENSOR_R_PIN 6
#define PROX_SENSOR_FL_PIN A2
#define PROX_SENSOR_FR_PIN A3
#define PROX_SENSOR_RL_PIN A4
#define PROX_SENSOR_RR_PIN 12
#define PROX_SENSOR_DL_PIN A5
#define PROX_SENSOR_DR_PIN 9
#define STOP_TIME 200 
#define MOTOR_RF_PIN 2
#define MOTOR_RB_PIN 4
#define MOTOR_R_SPEED 3
#define MOTOR_LF_PIN 7
#define MOTOR_LB_PIN 8
#define MOTOR_L_SPEED 5
bool lightSensorActive = true; // définition de l'état du capteur
unsigned long blackDetectionStartTime = 0; // temps de détection de la couleur noire
unsigned long redDetectionStartTime = 0; // temps de détection de la couleur rouge
bool Time_up = false; // false = wall following , true = bouncing on walls
int temps = 0; 
int Taille = 35; // A CHANGER SUIVANT LA TAILLE DU LABYRINTHE


void hardware_setup() {
  new DCMotor_Hbridge(MOTOR_RF_PIN, MOTOR_RB_PIN, MOTOR_R_SPEED, "ePuck_rightJoint", 2.5, 3 * 3.14159, 1);
  new DCMotor_Hbridge(MOTOR_LF_PIN, MOTOR_LB_PIN, MOTOR_L_SPEED, "ePuck_leftJoint", 2.5, 3 * 3.14159, 1);

  new VisionSensor(LIGHT_SENSOR_PIN, "ePuck_lightSensor", 0.1);

  new ProximitySensor(PROX_SENSOR_FL_PIN, "ePuck_proxSensor3", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_FR_PIN, "ePuck_proxSensor4", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_L_PIN, "ePuck_proxSensor1", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_R_PIN, "ePuck_proxSensor6", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_RL_PIN, "ePuck_proxSensor7", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_RR_PIN, "ePuck_proxSensor8", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_DL_PIN, "ePuck_proxSensor2", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_DR_PIN, "ePuck_proxSensor5", 0.1, 1);
}

void setup() {
  Serial.begin(4800);

  pinMode(MOTOR_RF_PIN, OUTPUT);
  pinMode(MOTOR_RB_PIN, OUTPUT);
  pinMode(MOTOR_R_SPEED, OUTPUT);
  pinMode(MOTOR_LF_PIN, OUTPUT);
  pinMode(MOTOR_LB_PIN, OUTPUT);
  pinMode(MOTOR_L_SPEED, OUTPUT);

  // Set speed to max
  analogWrite(MOTOR_R_SPEED, 255);
  analogWrite(MOTOR_L_SPEED, 255);

}
                                                           //DEFINITION DES MOUVEMENTS SIMPLES DU ROBOT//
void stopRobot() {
  digitalWrite(MOTOR_RF_PIN, LOW);
  digitalWrite(MOTOR_RB_PIN, LOW);
  digitalWrite(MOTOR_LF_PIN, LOW);
  digitalWrite(MOTOR_LB_PIN, LOW);
  delay(100);
}

void avancer() {  //Mouvement vers l'avant en ligne droite
  digitalWrite(MOTOR_RF_PIN, HIGH);
  digitalWrite(MOTOR_RB_PIN, LOW);
  digitalWrite(MOTOR_LF_PIN, HIGH);
  digitalWrite(MOTOR_LB_PIN, LOW);
  digitalWrite(MOTOR_R_SPEED, HIGH);
  digitalWrite(MOTOR_L_SPEED, HIGH);


}

void droite() {  //Tourne à droite
  digitalWrite(MOTOR_RF_PIN, LOW);
  digitalWrite(MOTOR_RB_PIN, HIGH);
  digitalWrite(MOTOR_LF_PIN, HIGH);
  digitalWrite(MOTOR_LB_PIN, LOW);
  analogWrite(MOTOR_R_SPEED, 150);
  analogWrite(MOTOR_L_SPEED, 150);
}

void gauche() {  //Tourne à gauche
  digitalWrite(MOTOR_RF_PIN, HIGH);
  digitalWrite(MOTOR_RB_PIN, LOW);
  digitalWrite(MOTOR_LF_PIN, LOW);
  digitalWrite(MOTOR_LB_PIN, HIGH);
  analogWrite(MOTOR_R_SPEED, 150);
  analogWrite(MOTOR_L_SPEED, 150);

}

void fin(){ // Case rouge trouvée donc fin du déplacement
  analogWrite(MOTOR_R_SPEED, 0);
  analogWrite(MOTOR_L_SPEED, 0);
  delay(100000);
  
}
//fonction permettant , au bout de "" secondes sans avoir trouvé la case noire en suivant le mur de droite , de basculer le robot en mode déplacement aléatoire
void parcours() { 
  temps = millis();
  if (temps/1000 < Taille){
    Time_up = false;
  }
  else{
    Time_up = true;
  }
}


// fonction permettant de détecter la case rouge qui ne s'execute que lorsque la case noir à été trouvée
void checkRedCase() {
int lightSensorValue = analogRead(LIGHT_SENSOR_PIN); // affectation de la valeur de la couleur de 0 à 1023
  if (lightSensorActive == false) {

    if ((lightSensorValue < RED) && (lightSensorValue > 270)) { // on rentre dans la boucle si la couleur lue par le lightSensor se trouve dans la gamme du rouge
      // le robot est sur une case rouge
      if (redDetectionStartTime == 0) {
        // début du timer
        redDetectionStartTime = millis();
      } else {
        // on regarde si le rouge à été détecté pendant une certaine durée
        if (millis() - redDetectionStartTime >= STOP_TIME) {
          Serial.println("Case rouge Trouvée !");
          fin();
          lightSensorActive = true;
        }
      }
    } else {
      // Reset le timer si la case n'est pas rouge
      redDetectionStartTime = 0;
    }
  }  
}

// fonction permettant de détecter la case noire
void checkBlackCase() {
int lightSensorValue = analogRead(LIGHT_SENSOR_PIN); // affectation de la valeur de la couleur de 0 à 1023
  if (lightSensorActive == true) {

    if (lightSensorValue < BLACK_THRESHOLD) { // on rentre dans la boucle si la couleur lue par le lightSensor se trouve dans la gamme du noir
      // le robot est sur une case noire
      if (blackDetectionStartTime == 0) {
        //  début du timer
        blackDetectionStartTime = millis();
      } else {
        // on regarde si le noir à été détecté pendant une certaine durée
        if (millis() - blackDetectionStartTime >= STOP_TIME) {
          stopRobot();
          Serial.println("Case noir Trouvée !"); 
          lightSensorActive = false; // bascule le lightsensor en false pour passer à la détection de rouge
          Time_up = false; // rebascule le type de mouvement du robot en wallfollowing 
          
        }
      }
    } else {
      // Reset le timer si la case n'est pas rouge
      blackDetectionStartTime = 0;
    }
  }  
}

void loop() {

parcours();
                                  // BOUCLE DE MOUVEMENT TYPE WALL FOLLOWING // 
if (Time_up == false ) {  
int captFL = analogRead(PROX_SENSOR_FL_PIN);
int captFR = analogRead(PROX_SENSOR_FR_PIN);
int captL = analogRead(PROX_SENSOR_L_PIN);
int captR = digitalRead(PROX_SENSOR_R_PIN);
int captRL = analogRead(PROX_SENSOR_RL_PIN); // définition de certains capteurs en analogue pour gérer les distances par rapport aux murs
int captRR = digitalRead(PROX_SENSOR_RR_PIN);
int captDL = analogRead(PROX_SENSOR_DL_PIN);
int captDR = digitalRead(PROX_SENSOR_DR_PIN);
int captNOIR = digitalRead(LIGHT_SENSOR_PIN);

if ((captFR > 500) && (captFL > 500)) {  // aucun obstacle proche de face donc le robot peut avancer
   avancer();
}
if ((captFR < 500) || (captFL < 500)) {  //le robot tourne à droite pour s'éloigner du mur de gauche
      droite();
    }
if ((captDL > 800) && (captL > 700) && (captRL > 800)) {  //le robot tourne à gauche si il s'éloigne trop du mur
      gauche();
}
    
checkBlackCase();
checkRedCase();

}


                                                          // BOUCLE DE MOUVEMENT DE TYPE ALEATOIRE //
if (Time_up == true) {
int captFL = digitalRead(PROX_SENSOR_FL_PIN);
int captFR = digitalRead(PROX_SENSOR_FR_PIN);
int captL = digitalRead(PROX_SENSOR_L_PIN);
int captR = digitalRead(PROX_SENSOR_R_PIN);
int captRL = digitalRead(PROX_SENSOR_RL_PIN);
int captRR = digitalRead(PROX_SENSOR_RR_PIN);
int captDL = digitalRead(PROX_SENSOR_DL_PIN);
int captDR = digitalRead(PROX_SENSOR_DR_PIN);
int captNOIR = digitalRead(LIGHT_SENSOR_PIN);


 if( captFL == 0 && captFR == 0){         // aucun obstacle de face donc marche avant
  digitalWrite(MOTOR_RF_PIN, HIGH);
  digitalWrite(MOTOR_RB_PIN, LOW);
  digitalWrite(MOTOR_LF_PIN, HIGH);
  digitalWrite(MOTOR_LB_PIN, LOW);
  delay(20);
}
 if( captFL == 1 || captDL == 1 || captL == 1 && captDR == 0 || captR == 0 || captFR == 0 ){   // obstacle à gauche , direction à droite
  analogWrite(MOTOR_RF_PIN, 20);
  analogWrite(MOTOR_RB_PIN, 150);
  analogWrite(MOTOR_LF_PIN, 150);
  analogWrite(MOTOR_LB_PIN, 0);
  delay(20);
}
 if( captFL == 0 || captDL == 0 || captL == 0 && captDR == 1 || captR == 1 || captFR == 1 ){   // obstacle à droite , direction à gauche
  analogWrite(MOTOR_RF_PIN, 150); 
  analogWrite(MOTOR_RB_PIN, 0);
  analogWrite(MOTOR_LF_PIN, 20);
  analogWrite(MOTOR_LB_PIN, 150);
  delay(20);
}
 if (captFR == 1){                    // obstacle de face donc leger demi-tour
  digitalWrite(MOTOR_RF_PIN, HIGH);
  digitalWrite(MOTOR_RB_PIN, LOW);
  digitalWrite(MOTOR_LF_PIN, LOW);
  digitalWrite(MOTOR_LB_PIN, HIGH);
  delay(20);
}
 if ( captFL == 1 ){                 // obstacle de face donc leger demi-tour
  digitalWrite(MOTOR_RF_PIN, LOW); 
  digitalWrite(MOTOR_RB_PIN, HIGH);
  digitalWrite(MOTOR_LF_PIN, HIGH);
  digitalWrite(MOTOR_LB_PIN, LOW);
  delay(20);
}
  checkBlackCase();
  checkRedCase();
}
}
