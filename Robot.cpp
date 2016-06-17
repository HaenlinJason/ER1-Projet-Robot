///////////////////////////////////////////////
//      projet robot fonction de la library Robot
//      Fichier fonctions avec utilisation de class
//      date de création    : 15/10/2015
//      date de mise à jour : 09/01/2016
///////////////////////////////////////////////

#define PRG
#ifdef PRG

//Include(s)
#include "mbed.h"
#include "Robot.h"

//constante
#define PERIOD 500
#define VMAX 100
#define VMAX2 275
#define VMIN PERIOD
#define M_PI 3.141592654
#define PAS_ROUE ((2*M_PI*2.6)/40) //cm

//variables globales
volatile int g_cpt1;
volatile int g_cpt2;
int g_dist1;
int g_dist2;
float g_speed;

//Communication PC
//Serial pc(USBTX, USBRX);

//fonction(s)
robot::robot(PinName C1,PinName C2,PinName C3,PinName C4,PinName MG, PinName MD,
             PinName SENSMD,PinName SENSMG, PinName RCPT1, PinName RCPT2):
    _C1(C1),_C2(C2),_C3(C3),_C4(C4),_MG(MG),_MD(MD),_SENSMD(SENSMD),
    _SENSMG(SENSMG),_RCPT1(RCPT1),_RCPT2(RCPT2)
{
    error=0;
    stat=0;
    g_speed=0;
    g_dist1=0;
    g_dist2=0;
    g_cpt1=0;
    g_cpt2=0;
    moteur_Stat=0;
    nombres_virages=0;
}
void robot::initprgm()
{
    _task.attach(this,&robot::checkdist,0.2);
    _RCPT1.rise(this,&robot::_trigger1);//rise=> compte sur front montant
    _RCPT2.rise(this,&robot::_trigger2);
    _MG.period_us(PERIOD); //initialise la periode à PERIOD en us
    _MD.period_us(PERIOD);
}
void robot::checkdist()
{
    g_speed=(g_cpt1*PAS_ROUE)/0.200;
    g_cpt1=0;
}
float robot::load_vitesse()
{
    //pc.printf("%g cm/s\n\r",g_speed);
    return(g_speed);
}
//fonctions compte tours moteur 1 et 2
void robot::_trigger1()
{
    g_cpt1++;
    g_dist1++;
}
void robot::_trigger2()
{
    g_cpt2++;
    g_dist2++;
}
//prends la valeur des capteurs et calcul la diff
float robot::readCapteurs()
{
    float diff;
    val1=_C3.read();   // || [] || || placements capteurs
    val2=_C1.read();   // || || [] ||
    val3=_C4.read();   // [] || || ||
    val4=_C2.read();   // || || || []
    diff = val1-val2;  //diff entre les deux capteurs interieurs
    //test les capteurs
    //pc.printf("\n\rC3= %g\tC2= %g\tDiff1= %g\t",val1,val2,diff);
    //wait(0.5);
    return(diff);
}
//-------Fonction ligne droite------------------//
void robot::PWMoteurConfettis()
{
    _SENSMG=0;
    _SENSMD=0;
    _MG.pulsewidth_us(VMAX);
    _MD.pulsewidth_us(VMAX);
}
int robot::fctlignedroite()
{
    if(_C1.read() < 0.2 && _C3.read() < 0.2 )
        return(1);
    else return(0);
}
//------------etats des moteurs-----------------//
void robot::PWMoteur0()
{
    _MG.pulsewidth(VMIN); //éteint 100% du temps
    _MD.pulsewidth(VMIN);
    _SENSMD.write(0);
    _SENSMG.write(0);
}
void robot::etatmoteurs1(float Diff1)
{
    vitesse.v1=(VMAX2+abs(20*Diff1));
    vitesse.v2=(VMAX2+abs(95*Diff1));
    vitesse.v3=VMIN;
    vitesse.v4=VMAX2;
    //speedinvers(&vitesse);//sens = 1 => inverse rapport cyclique
    if(_C2.read()<0.7) {
        //printf("\n\rv3= %g\tv4= %g",vitesse.v3,vitesse.v4);
        _MG.pulsewidth_us(vitesse.v3);
        _MD.pulsewidth_us(vitesse.v4);
    } else if(_C4.read()<0.7) {
        //printf("\n\rv3= %g\tv4= %g",vitesse.v3,vitesse.v4);
        _MG.pulsewidth_us(vitesse.v4);
        _MD.pulsewidth_us(vitesse.v3);
    } else  if(_C2.read()<0.6&&_C3.read()<0.3&&_C1.read()<0.3||_C4.read()<0.6&&_C3.read()<0.3&&_C1.read()<0.3||_C4.read()<0.6&&_C3.read()<0.2&&_C2.read()<0.5&&_C1.read()<0.6) {
        _MG.pulsewidth_us(vitesse.v4);
        _MD.pulsewidth_us(vitesse.v4);
    } else {
        //printf("\n\rv1= %g\tv2= %g",vitesse.v1,vitesse.v2);
        if(Diff1 < 0) {
            _MG.pulsewidth_us(vitesse.v1);
            _MD.pulsewidth_us(vitesse.v2);
        } else {
            _MD.pulsewidth_us(vitesse.v1);
            _MG.pulsewidth_us(vitesse.v2);
        }
    }
//wait(0.5);
}
int robot::square_fonction(float square)
{
    square=square*100;
    float distance;
    switch(stat) {
        case 0:
            //pc.printf("case 0\n\r");
            g_dist1=0;
            g_dist2=0;
            stat=1;
            break;
        case 1:
            //pc.printf("case 1\n\r");
            PWMoteurConfettis();
            distance=PAS_ROUE*g_dist1;
            if(distance>=square) {
                stat=2;
                nombres_virages++;
            }
            if(nombres_virages==4) {
                nombres_virages=0;
                stat=0;
                return 1;
            }
            break;
        case 2:
            //pc.printf("case 2\n\r");
            g_dist1=0;
            g_dist2=0;
            stat=3;
            PWMoteur0();
            wait(1);
            PWMoteurConfettis();
            break;
        case 3:
            //pc.printf("case 3\n\r");
            moteur_90d();
            if(g_dist1>=63&&g_dist2>=63)
                stat=0;
            break;
    }
    return 0;
}
void robot::moteur_90d()
{
    _SENSMG=1;
    _MG.pulsewidth(VMAX);
}
#endif //PRG