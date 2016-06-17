///////////////////////////////////////////////
//      projet robot programme principale
//      nom de code: Warbot
//      date de création    : 15/10/2015
//      date de mise à jour : 24/01/2016
///////////////////////////////////////////////

#define PRG
#ifdef PRG

//Library
#include "mbed.h"
#include "Robot.h"
#include "TextLCD.h"
#include "ultrasonic.h"

//Communication PC
//Serial pc(USBTX, USBRX);

//Communications
robot MCbot(A0,A1,A2,A3,D6,D7,D14,D15,D4,D5);   //C1, C2, C3, C4, MG, MD, SENSMD,SENSMG, RCPT1, RCPT2
TextLCD lcd (D8,D9,D10,D11,D12,D13);   //rs, e, d4-d7,lcdtype

//prototypes
void dist(int);
void affiche_vitesse(void);

//ultrason
//ultrasonic mu(PTC1,PTC2,0.1,1,&dist);
//Set the trigger pin and the echo pin
//have updates every .1 seconds and a timeout after 1
//second, and call dist when the distance changes

DigitalIn fdc(D3);      //fin de course
DigitalIn jack(D2);     //jack de demarrage
DigitalIn bp1(PTC2);      //Bouton poussoir de choix
DigitalIn bp2(D1);      //Bouton poussoir de demarrage

//programme principal
int main()
{
//initialisation
    MCbot.initprgm();
    MCbot.PWMoteur0();
    bp1.mode(PullNone);
    bp2.mode(PullNone);
    float diff1,square_dim=0.5;
    int etat=0,choix=10,bp1_av=0,bp2_av=0,fdc_av=0,last=0;
    lcd.cls();      //efface LCD
    lcd.printf("Bienvenue");
    wait(1);
    while(1) {

//-----------------machine à états--------//

//****************************************//
        switch(etat) {
                //---------etat 1:éteint----------//
            case 0:
                lcd.cls();
                lcd.printf("1:confettis");
                choix=10;
                etat=1;
            case 1: //choix programmes
                switch(choix) {
                    case 10:
                        MCbot.PWMoteur0();
                        if(bp1.read()<bp1_av) { //vérifie si il y a un changement sur le bouton 1
                            lcd.cls();
                            lcd.printf("2:suiveur ligne");
                            choix=11;
                        }
                        if(bp2.read()<bp2_av) { //vérifie si il y a un changement sur le bouton 2
                            lcd.cls();
                            lcd.printf("=>confettis");
                            etat=2;
                        }
                        break;
                    case 11:
                        if(bp1.read()<bp1_av) {
                            lcd.cls();
                            lcd.printf("3:carre         programmable");
                            choix=12;
                        }
                        if(bp2.read()<bp2_av) {
                            lcd.cls();
                            lcd.printf("=>suiveur ligne");
                            etat=3;
                        }
                        break;
                    case 12:
                        if(bp1.read()<bp1_av) {
                            lcd.cls();
                            lcd.printf("4:autres");
                            choix=13;
                        }
                        if(bp2.read()<bp2_av) {
                            lcd.cls();
                            lcd.printf("=>carre         programmable");
                            etat=4;
                            square_dim=0.6;
                        }
                        break;
                    case 13:
                        if(bp1.read()<bp1_av) {
                            lcd.cls();
                            lcd.printf("1:confettis");
                            choix=10;
                        }
                        if(bp2.read()<bp2_av) {
                            lcd.cls();
                            lcd.printf("=>autres");
                            etat=5;
                        }
                        break;
                }
                break;
                //---------etat 2-5:marche---------------//
                //--programme confettis------------------//
            case 2://check
                if(jack.read()==1) {
                    lcd.locate(0,1);
                    lcd.printf("GO");
                    affiche_vitesse();
                    MCbot.PWMoteurConfettis();
                }
                if(fdc.read()<fdc_av || MCbot.fctlignedroite()==1)
                    etat=0;
                break;
                //--programme suiveur de ligne-----------//
            case 3://not yet
                if(jack.read()==1) {
                    lcd.locate(0,1);
                    lcd.printf("GO");
                    /*lcd.cls();
                    lcd.printf("v1= %g",(500-(100-abs(20*diff1))));
                    lcd.locate(0,1);
                    lcd.printf("v2= %g",(500-(100-abs(90*diff1))));
                    wait(0.5);*/
                    affiche_vitesse();
                    diff1=MCbot.readCapteurs();
                    MCbot.etatmoteurs1(diff1);
                }
                if(fdc.read()<fdc_av)
                    etat=0;
                break;
                //--programme carre programmable---------//
            case 4://to check
                if(bp1.read()<bp1_av) {
                    square_dim+=0.1;
                    lcd.cls();
                    lcd.printf("=> carre : %g",square_dim);
                    if(square_dim>2)
                        square_dim=0.5;
                }
                if(jack.read()==1) {
                    lcd.locate(0,1);
                    lcd.printf("GO");
                    affiche_vitesse();
                    last=MCbot.square_fonction(square_dim);
                }
                if(fdc.read()<fdc_av||last==1) {
                    etat=0;
                    last=0;
                }
                break;
                //--programme autres---------------------//
            case 5://Check
                if(jack.read()==1) {
                    lcd.locate(0,1);
                    lcd.printf("GO");
                    MCbot.moteur_90d();
                }
                if(fdc.read()<fdc_av)
                    etat=0;
                break;
        }
//****************************************//

//------//---------fin machine à états----//
        fdc_av=fdc.read();
        bp1_av=bp1.read();
        bp2_av=bp2.read();
        wait(0.001); //cycle du programme
    }
}
void affiche_vitesse(void)
{
    float vit=MCbot.load_vitesse();
    lcd.locate(4,1);
    lcd.printf("%.2fcm/s",vit);
}
void dist(int distance)
{
    printf("Distance changed to %dmm\r\n", distance);
}
#endif //PRG
