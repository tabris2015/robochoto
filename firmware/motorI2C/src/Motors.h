/**
 * Motors - 
 * Clase para controlar 2 motores DC con un driver que acepta 3 pines por motor:
 * PWM, IN1, IN2 y ademas tiene un pin de standby de todo el driver.
 * Esta clase ha sido probada con el driver TB6612.
 * @author Jose Laruta
 * @version 1.0
 */

#ifndef MOTORS_H
#define MOTORS_H
#include <Arduino.h>

class Motors
{
    public:
    /** Constructor
     * Inicializa los pines
     * @param StdbyPin numero del pin conectado a STDBY del driver
     * @param LPwmPin pin para el control de velocidad del motor izquierdo
     * @param L1Pin pin 1 para el control del sentido de giro del motor izquierdo
     * @param L2Pin pin 2 para el control del sentido de giro del motor izquierdo
     * @param RPwmPin pin para la velocidad del motor derecho
     * @param R1Pin pin 1 para el control del sentido de giro del motor derecho
     * @param R2Pin pin 2 para el control del sentido de giro del motor derecho
     */
    Motors(int StdbyPin, int LPwmPin, int L1Pin, int L2Pin, int RPwmPin, int R1Pin, int R2Pin);
    // metodos publicos
    /** 
     * Inicializa el driver escribiendo 1 en el pin STDBY
     */
    void start(void);
    /** 
     * Conduce los motores con distintas velocidades
     * @param l_vel velocidad del motor izquierdo, si es negativo va hacia atras.
     * @param r_vel velocidad del motor derecho, si es negativo va hacia atras.
     */ 
    void drive(int l_vel, int r_vel);
    /**
     * Detiene los motores desactivando el pin STDBY
     */
    void stop(void);
    /**
     * Frena ambos motores activando ambos pines de control.
     * La disponibilidad de esta funcion depende del driver.
     */
    void brake(void);
    private:
    void leftForward(int vel);
    void leftBackward(int vel);
    void rightForward(int vel);
    void rightBackward(int vel);

    int _stdbypin;
    int _lpwmpin;
    int _l1pin;
    int _l2pin;
    int _rpwmpin;
    int _r1pin;
    int _r2pin;
};

Motors::Motors(int StdbyPin, int LPwmPin, int L1Pin, int L2Pin, int RPwmPin, int R1Pin, int R2Pin):
_stdbypin(StdbyPin),
_lpwmpin(LPwmPin),
_l1pin(L1Pin),
_l2pin(L2Pin),
_rpwmpin(RPwmPin),
_r1pin(R1Pin),
_r2pin(R2Pin)
{
    pinMode(_stdbypin, OUTPUT);
    pinMode(_lpwmpin, OUTPUT);
    pinMode(_l1pin, OUTPUT);
    pinMode(_l2pin, OUTPUT);
    pinMode(_rpwmpin, OUTPUT);
    pinMode(_r1pin, OUTPUT);
    pinMode(_r2pin, OUTPUT);
    digitalWrite(_stdbypin, 0);
    
}

void Motors::start()
{
    digitalWrite(_stdbypin, 1);
}
void Motors::stop()
{
    digitalWrite(_stdbypin, 0);
}
void Motors::brake()
{
    digitalWrite(_l1pin, 1);
    digitalWrite(_l2pin, 1);
    digitalWrite(_r1pin, 1);
    digitalWrite(_r2pin, 1);
}

void Motors::leftBackward(int vel)
{
    digitalWrite(_l1pin, 0);
    digitalWrite(_l2pin, 1);
    analogWrite(_lpwmpin, vel);
}
void Motors::leftForward(int vel)
{
    digitalWrite(_l1pin, 1);
    digitalWrite(_l2pin, 0);
    analogWrite(_lpwmpin, vel);
}


void Motors::rightBackward(int vel)
{
    digitalWrite(_r1pin, 0);
    digitalWrite(_r2pin, 1);
    analogWrite(_rpwmpin, vel);
}
void Motors::rightForward(int vel)
{
    digitalWrite(_r1pin, 1);
    digitalWrite(_r2pin, 0);
    analogWrite(_rpwmpin, vel);
}

void Motors::drive(int l_vel, int r_vel)
{
    if(l_vel < 0)
    {
        leftBackward(abs(l_vel));
    }
    else
    {
        leftForward(l_vel);
    }

    if(r_vel < 0)
    {
        rightBackward(abs(r_vel));
    }
    else
    {
        rightForward(r_vel);
    }
}

#endif