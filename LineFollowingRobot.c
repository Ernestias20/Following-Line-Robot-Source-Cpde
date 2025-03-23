 #include <18F25K50.h> // Le PIC à programmer
#device ADC = 10 // Nombre de bits pour les PINS utilisés pour le CAN
#include <math.h> // Pour utilisations de fonctions math
#use delay (internal=16MHz) // Spécifie l'horloge à quartz à 16MHz

#build(reset=0x1000, interrupt=0x1008) // Démarrer le programme à l'adresse 0x1000
#org 0,0x0FFF {}

// Configuration de l'écran LCD
#define LCD_ENABLE_PIN PIN_A1
#define LCD_RS_PIN PIN_A0
#define LCD_DATA4 PIN_B4
#define LCD_DATA5 PIN_B5
#define LCD_DATA6 PIN_B6
#define LCD_DATA7 PIN_B7

// Définition des capteurs et moteurs
defined rightSensor PIN_B2 // Capteur de ligne à droite
defined leftSensor PIN_B3 // Capteur de ligne à gauche
defined centerSensor PIN_C7 // Capteur de ligne au centre
defined motor1 PIN_C3 // Moteur 1 DIR1
defined motor2 PIN_C4 // Moteur 2 DIR2
defined buttonStart PIN_C6
defined BP1 PIN_B1 // Interruption 1 : concours robot Cachan
defined BP0 PIN_B0 // Interruption 2 : concours robot Cachan

#include <lcd_EB005.c>

// Variables globales
unsigned int8 contBarsRight = 0;
unsigned int8 contBarsLeft = 0;
int contLineTour = 0;
int contLineTour1 = 0;
int contLineTour2 = 0;
int globalCont = 0;
int obstacle = 0;
unsigned long DATA = 0;
unsigned int robotSpeed1 = 0;
unsigned int robotSpeed2 = 0;
unsigned int control = 1;

// Interruption Capteur gauche
#INT_EXT
void barLeft_BP0(void) {
    contBarsLeft++;
}

// Interruption Capteur droite
#INT_EXT1
void barRight_BP1(void) {
    contBarsRight++;
}

// Fonctions globales
void stop() {
    output_low(PIN_A3);
    output_low(PIN_A4);
    robotSpeed1 = 0;
    robotSpeed2 = 0;
    set_pwm1_duty(robotSpeed1);
    set_pwm2_duty(robotSpeed2);
}

void stopGabarit() {
    delay_ms(300);
    stop();
}

void turnLeft() {
    robotSpeed1 = 40;
    robotSpeed2 = 15;
    output_high(PIN_A3);
    output_high(PIN_A4);
    set_pwm1_duty(robotSpeed1);
    set_pwm2_duty(robotSpeed2);
}

void turnRight() {
    robotSpeed1 = 15;
    robotSpeed2 = 40;
    output_high(PIN_A3);
    output_high(PIN_A4);
    set_pwm1_duty(robotSpeed1);
    set_pwm2_duty(robotSpeed2);
}

void goForward() {
    robotSpeed1 = 60;
    robotSpeed2 = 60;
    output_high(PIN_A3);
    output_high(PIN_A4);
    set_pwm1_duty(robotSpeed1);
    set_pwm2_duty(robotSpeed2);
}

int obstacleBool() {
    if (obstacle == 0) {
        goForward();
        return 0;
    } else {
        return 1;
    }
}

void distanceSensor() {
    set_adc_channel(2);
    delay_us(10);
    DATA = read_adc();
    delay_ms(50);
    obstacle = (DATA > 200) ? 1 : 0;
}

void startRobot() {
    if (input(PIN_C6) && control == 1) {
        while (input(buttonStart)) {
            delay_ms(10);
        }
        output_high(PIN_C0);
        control = 0;
    }
}

void statesInfo() {
    lcd_gotoxy(4, 1);
    printf(lcd_putc, "D:%i M:%i G:%i ", input(rightSensor), input(centerSensor), input(leftSensor));
    lcd_gotoxy(1,2);
    printf(lcd_putc, "OB:%i BL:%i BR:%i", obstacle, contBarsLeft, contBarsRight);
}

void correctionRight() {
    robotSpeed1 = 40;
    robotSpeed2 = 50;
    output_high(PIN_A3);
    output_high(PIN_A4);
    set_pwm1_duty(robotSpeed1);
    set_pwm2_duty(robotSpeed2);
}

void correctionLeft() {
    robotSpeed1 = 50;
    robotSpeed2 = 40;
    output_high(PIN_A3);
    output_high(PIN_A4);
    set_pwm1_duty(robotSpeed1);
    set_pwm2_duty(robotSpeed2);
}

void main() {
    // Initialisations
    control = 1;
    enable_interrupts(GLOBAL);
    enable_interrupts(INT_EXT);
    enable_interrupts(INT_EXT1);
    ext_int_edge(0, L_TO_H);
    ext_int_edge(1, L_TO_H);

    contBarsRight = 0;
    contBarsLeft = 0;
    obstacle = 0;
    globalCont = 0;

    output_low(PIN_C0);
    output_low(PIN_A3);
    output_low(PIN_A4);
    set_pwm1_duty(0);
    set_pwm2_duty(0);
    setup_ccp1(CCP_PWM);
    setup_ccp2(CCP_PWM);
    setup_timer_2(T2_DIV_BY_16, 124, 1);

    lcd_init();
    setup_adc(ADC_CLOCK_INTERNAL);
    setup_adc_ports(sAN2);

    lcd_gotoxy(4, 1);
    printf(lcd_putc, "ROBOT STOP");
    lcd_gotoxy(1, 2);
    printf(lcd_putc, "LONGWYBOT *.*");

    while (true) {
        startRobot();
        distanceSensor();
        statesInfo();

        if (obstacle == 1) {
            stop();
        } else {
            if (!input(rightSensor) && !input(centerSensor) && !input(leftSensor)) stop();
            else if (input(rightSensor) && !input(centerSensor) && !input(leftSensor)) turnRight();
            else if (!input(rightSensor) && input(centerSensor) && !input(leftSensor)) goForward();
            else if (input(rightSensor) && input(centerSensor) && !input(leftSensor)) correctionLeft();
            else if (!input(rightSensor) && input(centerSensor) && input(leftSensor)) correctionRight();
            else if (input(rightSensor) && input(centerSensor) && input(leftSensor)) goForward();
            else if (!input(rightSensor) && !input(centerSensor) && input(leftSensor)) turnLeft();
            else {
                globalCont++;
                goForward();
            }
        }
    }
}
