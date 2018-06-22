/** Sensors.h
 * Libreria para usar un array de sensores tipo pololu QRT8A. 
 * Se puede conectar hasta 8 sensores infrarrojos analogicos a los pines
 * A0 - A7 (en el arduino nano, pro mini) y hasta 6 sensores A0 - A5 
 * en el Arduino UNO.
 */
#ifndef SENSORS_H
#define SENSORS_H
#include <Arduino.h>

#define MAX_SENSORS 8       /* numero de sensores conectados*/
#define MAX_DEFAULT 1023    /* maximo por defecto sin usar la calibracion */
#define MIN_DEFAULT 0       

//#define DEBUG
class Sensors
{
    public:
    /** Contructor
     * Inicializa el pin de activacion de los sensores y diversas variables
     * @param NSensors numero de sensores a utilizarse
     * @param ONPin pin de activacion de los LEDs
     * @param reverse (default: false) introduzca true si se usara una linea 
     *                                  negra sobre una superficie blanca
     */
    Sensors(int16_t NSensors, int16_t ONPin, bool reverse=false);
    
    /** getRaw
     * Obtiene los valores 'raw' de los sensores. Si no se usa el metodo Sensors::calibrate() antes
     * copia los valores directamente de los puertos analogicos. Si se hace la calibracion antes
     * copia un array con valores del 0 al 100.
     * @param buffer Un puntero al array a donde se copiaran los valores de cada sensor.
     */
    void getRaw(uint16_t * buffer);
    /** getBool
     * Obtiene un byte que representa el valor binario de los sensores.
     * @return uint8_t byte con los valores binarios
     */
    uint8_t getBool(void);
    /** getDensity()
     * Devuelve la cantidad de sensores que detectan la linea. Esto puede se util para 
     * detectar cruces, o angulos rectos.
     * @return uint8_t el numero de sensores activos o en la linea.
     */
    uint8_t getDensity();
    /** calibrate
     * Rutina de calibracion de los sensores.
     * La calibracion se realiza midiendo cada sensor y almacenando el nivel maximo y el minimo 
     * en 2 arrays donde cada sensor tiene un rango individual, esto ayudaria al caso en 
     * el que exista un ligero desbalance en el array de sensores.
     */
    void calibrate();
    /** getPosition
     * Devuelve la posicion de la linea relativa al array de sensores.
     * Se usa un algoritmo de promedio ponderado para obtener un numero que corresponde
     *  a la posicion de la linea con respecto al array de sensores.
     * @return int16_t valor de la posicion de la linea. 0 corresponde a la linea en el sensor 
     *                 mas lejano de la izquierda y el valor (nSensors * 100) corresponde al sensor
     *                  mas lejano a la derecha.
     */ 
    int16_t getPosition(void);
    /** setReverse
     * Selecciona el tipo de linea que se usara, linea negra sobre blanco o linea blanca sobre negro.
     * @param reverse valor para detectar la linea. 0 o false significa detectar una linea negra sobre blanco.
     */
    void setReverse(uint8_t reverse);
    /** update
     * Realiza la lectura de los sensores y su posterior adecuacion de rango de acuerdo a calibracion.
     * Se debe usar esta funcion siempre que se desee acceder a algun valor actual de los sensores.
     */
    void update(void);
    /** getMaxvalues
     * Copia los valores maximos de calibracion a un array
     * @param maxs puntero a un array para almacenar los valores maximos de calibracion
     */
    void getMaxvalues(uint16_t * maxs);
    /** getMinvalues
     * Copia los valores minimos de calibracion a un array
     * @param mins puntero a un array para almacenar los valores minimos de calibracion
     */
    void getMinvalues(uint16_t * mins);
    

    private:
    /** _readSensors
     * Realiza la lectura de los sensores.
     */
    void _readSensors();
    int16_t _position;              /* posicion de la linea */
    int16_t _lastPosition;          /* posicion anterior de la linea */
    int16_t _onPin;                 /* pin para la activacion de los sensores */
    uint8_t _nSensors;              /* numero de sensores a usar */
    uint8_t _reverse;               /* si se lee una linea negra o blanca */
    uint16_t _threshold;            /* valor de umbral para la deteccion binaria */
    uint16_t _buffer[MAX_SENSORS];  /* array con los valores leidos y adecuados */
    uint8_t _boolBuffer[MAX_SENSORS];   /* no se usa */
    uint8_t _boolValues;                /* valores binarios de los sensores */
    uint16_t _maxValues[MAX_SENSORS];   /* valores maximos de calibracion */
    uint16_t _minValues[MAX_SENSORS];   /* valores minimos de calibracion */
};

Sensors::Sensors(int16_t NSensors, int16_t ONPin, bool reverse):
_onPin(ONPin), 
_nSensors(NSensors),
_reverse(reverse)
{
    pinMode(_onPin, OUTPUT);
    digitalWrite(_onPin, 0);
    
    for(int8_t i = 0; i < MAX_SENSORS; i++)
    {
        _maxValues[i] = MAX_DEFAULT;
        _minValues[i] = MIN_DEFAULT;
    }
    _threshold = 50;
}

void Sensors::_readSensors()
{
    digitalWrite(_onPin, 1);
    delayMicroseconds(100);
    _boolValues = 0;
    for(uint8_t i = 0; i < _nSensors; i++)
    {
        int32_t value = analogRead(i) - _minValues[i];
        uint16_t den = _maxValues[i] - _minValues[i];
        

        value = (den != 0 || den < 0) ? (value * 100) / den : 0;
        
        value = constrain(value, 0, 100);
        
        #ifdef DEBUG
            Serial.print(value);
            Serial.print(' ');
        #endif
        _buffer[i] = value;
        // check reverse
        _boolValues |=  (_reverse ? _buffer[i] < _threshold : _buffer[i] > _threshold) << i;
    }
    #ifdef DEBUG
    Serial.println(' ');
    #endif
    digitalWrite(_onPin, 0);
}
void Sensors::update()
{
    _readSensors();
}

void Sensors::getRaw(uint16_t * buffer)
{
    for(int8_t i = 0; i < _nSensors; i++)
    {
        buffer[i] = _buffer[i];
    }
}

uint8_t Sensors::getDensity()
{
    uint8_t density = 0;
    for(int i = 0; i < _nSensors; i++)
    {
        density += (_boolValues >> i) & 0x01;
    }
    return density;
}
uint8_t Sensors::getBool()
{
    return _boolValues;
}

int16_t Sensors::getPosition()
{
    uint8_t onLine = 0;
    uint32_t avg = 0;
    uint16_t sum = 0;

    for(int8_t i = 0; i < _nSensors; i++)
    {
        uint16_t value = _reverse ? 100 - _buffer[i]: _buffer[i];

        if(value > 40)
        {
            onLine = 1;
        }
        if(value > 8)
        {
            avg += (uint32_t)(value) * (i * 100);
            sum += value;
        }
    }
    if(!onLine)
    {
        if(_lastPosition < (_nSensors - 1)*100 / 2)
        {
            return 0;
        }
        else 
        {
            return (_nSensors - 1) * 100;
        }
    }
    
    _lastPosition = avg / sum;
    return _lastPosition;
}

void Sensors::setReverse(uint8_t reverse)
{
    _reverse = reverse;
}

void Sensors::calibrate()
{
    
    for(int i = 0; i < _nSensors; i++)
    {
        _maxValues[i] = MIN_DEFAULT;
        _minValues[i] = MAX_DEFAULT;
    }
    for(int i = 0; i < 100; i++)
    {
        digitalWrite(_onPin, 1);
        for(int s = 0; s < _nSensors; s++)
        {
            uint16_t value = analogRead(s);
            #ifdef DEBUG
            Serial.print(value);
            Serial.print(' ');
            #endif
            if(value > _maxValues[s])
            {
                _maxValues[s] = value;
            }
            if(value < _minValues[s])
            {
                _minValues[s] = value;
            }
        }
        #ifdef DEBUG
            
            Serial.println(' ');
        #endif
        delay(50);
    }
    digitalWrite(_onPin, 0);
}

void Sensors::getMaxvalues(uint16_t * maxs)
{
    for(int i = 0; i < _nSensors; i++)
    {
        maxs[i] = _maxValues[i];
    }
}
void Sensors::getMinvalues(uint16_t * mins)
{
    for(int i = 0; i < _nSensors; i++)
    {
        mins[i] = _minValues[i];
    }
}
#endif