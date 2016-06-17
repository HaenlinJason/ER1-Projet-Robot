///////////////////////////////////////////////
//      projet robot prototype de library Robot
//      Prototypes et Class
//      date de création    : 15/10/2015
//      date de mise à jour : 24/01/2016
///////////////////////////////////////////////
#ifndef MBED_TEST_H
#define MBED_TEST_H

#include "mbed.h"

//class
class robot
{

//public: accée à l'éxtérieur de la class
public:
    //PinNames
    robot(PinName C1,PinName C2,PinName C3,PinName C4,PinName MG, PinName MD,
     PinName SENSMD,PinName SENSMG, PinName RCPT1, PinName RCPT2);
    //prototypes
    void initprgm();
    //=> appel: Name.PWMoteurConfettis();
    void PWMoteurConfettis();
    //=> appel: Name.PWMoteur0();
    void PWMoteur0(); 
    //=> appel: int = Name.fctlignedroite();                              
    int  fctlignedroite(); 
    //=> appel: Name.etatmoteurs(float);                         
    void etatmoteurs1(float);                 
    float readCapteurs();               
    void moteurSense(int);
    int square_fonction(float);                      
    void rightprio();
    float load_vitesse();
    void moteur_90d();
//protected: seulement à l'intérieur de la class
protected:
    typedef struct
    {
        float v1,v2,v3,v4;
    }PWMspeed;
    PWMspeed vitesse;
    
    void speedinvers(PWMspeed*);                          
    void leftshortcut();  
    void checkdist();                            
    void _trigger1();
    void _trigger2();
    int checkerror();
    int error,stat,moteur_Stat,nombres_virages;
    float val1,val2,val3,val4;
    Ticker testdist;

//déclaration des ports de communications
    Ticker _task; 
    AnalogIn _C1;      
    AnalogIn _C2;       
    AnalogIn _C3;       
    AnalogIn _C4;       
    PwmOut   _MG;       // MLI moteur gauche
    PwmOut   _MD;       // MLI moteur droite
    DigitalOut _SENSMD;  //sense du moteur
    DigitalOut _SENSMG; 
    InterruptIn _RCPT1; //compte tour moteur 
    InterruptIn _RCPT2; 
};
#endif

