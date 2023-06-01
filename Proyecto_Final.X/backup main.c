/* 
 * File:   Proyecto_Micros.c
 * Author: pablo
 *
 * Created on 9 de mayo de 2023, 2:52 p.m.
 * 
 * Este proyecto tiene como finalidad mostrar todas las competencias logradas durante el curso
 * 
 * Temática del Proyecto - Brazo Robótico
 * Funciones:
 * 
- Tendrá estados o posiciones que puede grabar y reproducir desde la EEPROM.
- Podrá controlar al menos 4 servos o motores DC.
- Deberá utilizar al menos 1 señal de PWM con el módulo de CCP del pic.
- Utiliza 4 potenciómetros o entradas analógicas para poder controlar los servos.
- Puede tener un modo manual, modo EEPROM, modo UART y modo Adafruit.
- Manual: puede ajustar la posición de los servos y guardarlo.
- EEPROM: reproduce las posiciones grabadas en EEPROM.
- UART: ejecuta secuencias de movimientos controlados desde una interfaz en la
computadora.
- Adafruit: controla los servos o posiciones desde Adafruit IO
- Deberá tener algún indicador del modo de funcionamiento
- Controlará los movimientos y estados del proyecto desde un dashboard en Adafruit. Al
seleccionar una secuencia desde Adafruit, usted deberá enviar los valores al pic de
manera serial.

 */


// PIC16F887 Configuration Bit Settings
// 'C' source line config statements
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF       // RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
#pragma config CP = OFF          // Code Protection bit (Program memory code protection is enabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.


/**********LIBRERIAS**********/

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <pic16f887.h>
#include "PWM_Library.h"

/**********DEFINIR CONSTANTES**********/

#define _XTAL_FREQ 500000
unsigned int numero;
unsigned int centena; // Almacena las centenas en  ASCII
unsigned int decena; 
unsigned int unidad; 
uint8_t contador;

uint8_t modo_funcionamiento = 0; //Registro para guardar el modo en funcionamiento.
uint8_t activador_modo = 0;
uint8_t val_potenciometro = 0;
uint8_t selector_servoN = 0;
uint8_t selector_servoA = 0;
uint8_t lectura_escritura_ENABLE = 0;

/**********VARIABLES GLOBALES**********/

int adc00 = 0, adc01 = 0,  adc11 = 0, adc10=0;
uint8_t ondaPWM;
int valor_mapeado0, valor_mapeado1, valor_mapeado2, valor_mapeado3;
uint8_t address0 = 0x04; // Direccion de los datos en EEPROM
uint8_t address1 = 0x05; // Direccion de los datos en EEPROM
uint8_t address2 = 0x06; // Direccion de los datos en EEPROM
uint8_t address3 = 0x07; // Direccion de los datos en EEPROM

/**********PROTOTIPOS**********/
uint8_t map(uint8_t valor, uint8_t rango_min, uint8_t rango_max, uint8_t nuevo_min, uint8_t nuevo_max);

void modo_automatico(void);
void modo_adafruit(void);
void modo_manual(void);
void modo_uart(void);
void write_EEPROM(uint8_t address, uint8_t data); //Escritura de EEPROM
uint8_t read_EEPROM(uint8_t address); //lectura del EEPROM

void setup(void);
void initUART(void);
void enviar_numero_decimal(unsigned int numero);

/**********INTERRUPCIONES**********/

void __interrupt() isr(void) {

        /*Tabla Guia de los motores:
          - 00 - Servo base  (Azul 1)
          - 01 - Servo pinza (Azul 2)
          - 10 - Servo codo  (Negro 1)
          - 11 - Servo brazo (Negro 2) */
   
    /**************************************************************************************/
    
    if (ADIF){                                              // Interrupción del ADC
        if (ADCON0bits.CHS == 0b0000) {         // Canal 0
            adc00 = ADRESH;
            ADCON0bits.CHS = 0b0001;            // Cambia a canal 1
        } else if (ADCON0bits.CHS == 0b0001){   // Canal 1
            adc01 = ADRESH;
            ADCON0bits.CHS = 0b0010;            // Cambia a canal 2
        } else if (ADCON0bits.CHS == 0b0010){   // Canal 2
            adc10 = ADRESH;
            ADCON0bits.CHS = 0b0011;           // Cambia a canal 3
        } else if (ADCON0bits.CHS == 0b0011){   // Canal 3
            adc11 = ADRESH;
            ADCON0bits.CHS = 0b0000;           // Cambia a canal 0
        }    
        PIR1bits.ADIF = 0;                                  //Se limpia la bandera de Interrupción del ADC   
    }  
    
    /**************************************************************************************/
        
    if(RCIF == 1){          //Si se prende la bandera de recepción de datos
        PORTD = RCREG;
        __delay_ms(5);
        PIR1bits.RCIF = 0; // Borrar el indicador
    }
 
    /**************************************************************************************/
    
        /*Tabla Guia de los estados:
          - 00 - Modo Automatico
          - 01 - Modo Manual
          - 10 - Modo UART
          - 11 - Modo Adafruit  */
    
    if (RBIF) { // interrupción en PORTB  
                
        if (RB0 == 0) { 
            modo_funcionamiento ++; // Cambio de modo de funcionamiento
            if (modo_funcionamiento == 4){
                modo_funcionamiento = 0;
            }
        }
        if (RB1 == 0) { 
            activador_modo = activador_modo ^ 1; // Cambio de estado bandera
        }
        if (RB2 == 0) { 
            selector_servoN = selector_servoN ^ 1; // Cambio de estado Servos Negros
        }
        if (RB3 == 0) { 
            selector_servoA = selector_servoA ^ 1; // Cambio de estado Servos Azules
        }
        if (RB4 == 0) { 
            if(modo_funcionamiento == 0 && activador_modo == 1){
                PORTD = 255;
                valor_mapeado0 = read_EEPROM(address0); //Mostrar el valor guardado en la dirección establecida de la EEPROM
                valor_mapeado1 = read_EEPROM(address1); //Mostrar el valor guardado en la dirección establecida de la EEPROM
                valor_mapeado2 = read_EEPROM(address2); //Mostrar el valor guardado en la dirección establecida de la EEPROM
                valor_mapeado3 = read_EEPROM(address3); //Mostrar el valor guardado en la dirección establecida de la EEPROM
                PORTD = 0;
                
            } else if(modo_funcionamiento == 1 && activador_modo == 1){
                //Guardar los valores de los potenciometros
                write_EEPROM(address0, valor_mapeado0); //Escribir el valor del potenciómetro en la EEPROM
                write_EEPROM(address1, valor_mapeado1); //Escribir el valor del potenciómetro en la EEPROM
                write_EEPROM(address2, valor_mapeado2); //Escribir el valor del potenciómetro en la EEPROM
                write_EEPROM(address3, valor_mapeado3); //Escribir el valor del potenciómetro en la EEPROM
                               
            } else if(modo_funcionamiento == 2 && activador_modo == 1){
                
            } else if(modo_funcionamiento == 3 && activador_modo == 1){
                
            }
        }
    }
       
    /**************************************************************************************/
    
        centena = (contador/100);       // Calcular las centenas 
        decena = ((contador/10)%10);    // Calcular las decenas 
        unidad = (contador%10);         // Calcular las unidades 
        

        RBIF = 0;        // Limpiar la bandera de interrupción de PORTB 
         __delay_ms(5);
        return;
}
    


/**********CÓDIGO PRINCIPAL**********/


/*
 * 
 */
void main(void) 
{
    setup();

    while(1)   //loop principal
    {        
        if (modo_funcionamiento == 0){
            PORTE = 0b111;
            if (activador_modo == 1){
                //LEER Y ACTIVAR
                OSCCONbits.IRCF = 0b111;    //8Mhz
                valor_mapeado0 = read_EEPROM(address0); //Mostrar el valor guardado en la dirección establecida de la EEPROM
                valor_mapeado1 = read_EEPROM(address1); //Mostrar el valor guardado en la dirección establecida de la EEPROM
                valor_mapeado2 = read_EEPROM(address2); //Mostrar el valor guardado en la dirección establecida de la EEPROM
                valor_mapeado3 = read_EEPROM(address3); //Mostrar el valor guardado en la dirección establecida de la EEPROM
                PWM_config(1, 20);          //Configuración CCP1
                PWM_config(2, 20);          //Configuración CCP2
                PWM_manual_config(15);      //Configuración PWM manula 10ms
                
                modo_automatico();          //MODO AUTOMATICO - SUBLOOP
                
                //DESACTIVAR
                T2CONbits.TMR2ON = 0;       //Apagar TMR2
                OSCCONbits.IRCF = 0b111;    //8Mhz
                PORTC = 0;
            }
        }
        else if (modo_funcionamiento == 1){
            PORTE = 0b001;
            if (activador_modo == 1){
                //ACTIVAR
                OSCCONbits.IRCF = 0b111;    //8Mhz
                ADCON0bits.ADON = 1;        // ADC enable
                PWM_config(1, 20);          //Configuración CCP1
                PWM_config(2, 20);          //Configuración CCP2
                PWM_manual_config(15);      //Configuración PWM manula 10ms
                
                modo_manual();              //Modo MANUAL SUB-LOOP
                
                //DESACTIVAR
                ADCON0bits.ADON = 0;        // ADC enable
                T2CONbits.TMR2ON = 0;       //Apagar TMR2
                OSCCONbits.IRCF = 0b111;    //8Mhz
                PORTC = 0;
            }
        }
        else if (modo_funcionamiento == 2){
            PORTE = 0b010;
            if (activador_modo == 1){
                initUART();
                modo_uart();
            }
        }
        else if (modo_funcionamiento == 3){
            PORTE = 0b100;
            if (activador_modo == 1){
                initUART();
                modo_adafruit();
            }
        }
        
    }  
}


/**********FUNCIONES**********/
void modo_automatico(void){
    while (1){  
        __delay_ms(10);
        
        T2CONbits.TMR2ON = 0;       //Apagar TMR2
        OSCCONbits.IRCF = 0b111;    //8Mhz
        PWM_manual(valor_mapeado2, 0b1);
        PWM_manual(valor_mapeado3, 0b1000);

        OSCCONbits.IRCF = 0b011;    //5Khz
        T2CONbits.TMR2ON = 1;       //Encender TMR2
        PWM_duty(2,valor_mapeado0);
        PWM_duty(1,valor_mapeado1);    

        if (activador_modo == 0){
            break;
        }
    }
}

void modo_manual(void){
    while (1){
        if (ADCON0bits.GO == 0){
            ADCON0bits.GO = 1;      //Iniciar a convertir
            __delay_us(2);
        }
        valor_mapeado0 = map(adc00,0,255,30,65);     //Para varial el DutyCycle entre 1ms y 2ms el mapeo debe de ser de 30 a 65
        valor_mapeado1 = map(adc01,0,255,30,65);
        valor_mapeado2 = map(adc10,0,255,25,65);
        valor_mapeado3 = map(adc11,0,255,25,65);
        
        T2CONbits.TMR2ON = 0;       //Apagar TMR2
        OSCCONbits.IRCF = 0b111;    //8Mhz
        PWM_manual(valor_mapeado2, 0b1);
        PWM_manual(valor_mapeado3, 0b1000);  

        OSCCONbits.IRCF = 0b011;    //5Khz
        T2CONbits.TMR2ON = 1;       //Encender TMR2
        PWM_duty(2,valor_mapeado0);
        PWM_duty(1,valor_mapeado1);        
                
        if (activador_modo == 0){
            break;
        }
    }
}
void modo_uart(void){
    while (1){
        TXREG = centena + 48; // Enviar el valor de las centenas
        __delay_ms(5);        
        TXREG = decena + 48; // Enviar el valor de las decenas
        __delay_ms(5);                 
        TXREG = unidad + 48; // Enviar el valor de las unidades
        __delay_ms(5);     
        TXREG = '\n'; // Enviar nueva línea
        __delay_ms(5);
        if (activador_modo == 0){
            break;
        }
    }
}

void modo_adafruit(void){
    while (1){
        TXREG = centena + 48; // Enviar el valor de las centenas
        __delay_ms(5);        
        TXREG = decena + 48; // Enviar el valor de las decenas
        __delay_ms(5);                 
        TXREG = unidad + 48; // Enviar el valor de las unidades
        __delay_ms(5);     
        TXREG = '\n'; // Enviar nueva línea
        __delay_ms(5);
        if (activador_modo == 0){
            break;
        }
    }
    
}

void initUART(void){
    /*
     * 12.1.1.6 Asynchronous Transmission Set-up:
    1. Initialize the SPBRGH, SPBRG register pair and
    the BRGH and BRG16 bits to achieve the desired
    baud rate (see Section 12.3 ?EUSART Baud
    Rate Generator (BRG)?).
    2. Enable the asynchronous serial port by clearing
    the SYNC bit and setting the SPEN bit.
    3. If 9-bit transmission is desired, set the TX9 control bit. A set ninth data bit will indicate that the 8
    Least Significant data bits are an address when
    the receiver is set for address detection.
    4. Enable the transmission by setting the TXEN
    control bit. This will cause the TXIF interrupt bit
    to be set.
    5. If interrupts are desired, set the TXIE interrupt
    enable bit. An interrupt will occur immediately
    provided that the GIE and PEIE bits of the
    INTCON register are also set.
    6. If 9-bit transmission is selected, the ninth bit
    should be loaded into the TX9D data bit.
    7. Load 8-bit data into the TXREG register. This
    will start the transmission.
     */
    
    //paso1
    SPBRG = 12;     //SPBRGH:SPBRG = (8MHz/9600)/64 -1 = 12 ---> Real 9615.38
    SPBRGH = 0;     //%error = (9615.38-9600)/9600*100 = 0.16%
    BRGH = 0;
    BRG16 = 0;
    
    //paso2
    SYNC = 0;   //Modo asincronico habilitado
    SPEN = 1;   //Habilitación del modulo UART
    
    //paso3, habilitacion de los 9 bits.
    
    
    //paso4 Habilitacion de la transmicion
    TXEN = 1; //Habilitar transmision & TXIF = 1
    TXIF = 0; //Apagar la bandera TX
    // Asynchronous Reception Set-up
    CREN = 1;   //habilita la recepcion
    RCIF = 0;   //limpia bandera de interrupcion de datos recibidos
    
    //Paso 5> Interrupciones
    
    //Paso 6> cargar el 9no bit.
    
    //Paso7> cargar los 8 bits
    
    
    
}    
   
void setup(void)
{
    //configuracion de entradas y salidas 
    ANSELH = 0;
    
    TRISB = 0b11111;  // Input los primeros 5 bits del PORTB
    TRISD = 0;      // Puertos como salida
    TRISE = 0;
    
    PORTD = 0;   //Se inicializan todos los puertos en 0
    PORTE = 0;
    PORTA = 0;
        
    //Configuracion PULL-UP
    OPTION_REGbits.nRBPU = 0;   //0 = PORTB pull-ups are enabled by individual PORT latch values
    WPUB = 0b11111;   // Pull-up Enable para los primeros 2 bits del PORTB
    IOCB = 0b11111; //Interrupt-on-change enabled para los primeros 2 bits del PORTB

    //Configuración del Reloj del PIC
    OSCCONbits.IRCF = 0b111;    //8Mhz
    OSCCONbits.SCS = 1;         //Utilización del oscilador Interno
    
    //Configuración de las interrupciones
    PIR1bits.ADIF = 0;   // limpiar la bandera de interrupcion del ADC
    PIE1bits.ADIE = 1;   // habilitar interrupciones de ADC
    PIE1bits.RCIE = 1;   // Interrupcion RX
    INTCONbits.GIE = 1;  // Usualmente la global se enciende de [ultimo]
    INTCONbits.PEIE = 1; // habilitar interrupciones perifericas
    INTCONbits.RBIF = 0; // Limpiar la bandera de interrupcion del PORTB
    INTCONbits.RBIE = 1; // Habilitar la interrupcion del PORTB  
    
    //Configuracion ADC
    ANSEL = 0b1111;
    TRISA = 0b1111;
    ADCON0bits.ADCS = 0b10;     //Conversion Clock
    ADCON0bits.CHS = 0b0000;    //Canal 0
    ADCON1bits.ADFM = 0;    //justificado a la izquierda
    ADCON1bits.VCFG0 = 0;   //Vdd
    ADCON1bits.VCFG1 = 0;   //Vss
    ADCON0bits.ADON = 0;    //ADC enable

    return;
}

uint8_t map(uint8_t valor, uint8_t rango_min, uint8_t rango_max, uint8_t nuevo_min, uint8_t nuevo_max){
    uint8_t nuevo_valor = nuevo_min + (valor - rango_min)*(nuevo_max - nuevo_min)/(rango_max - rango_min);
    return nuevo_valor;
}

void write_EEPROM(uint8_t address, uint8_t data){
    while (WR){;}
    EEADR = address;//Dirección de memoria a Escribir
    EEDAT = data;   //Dato a escribir
    
    EECON1bits.EEPGD = 0; //Acceso a memoria de datos en la EEPROM
    EECON1bits.WREN = 1; // Habilitacion de escritura en la EEPROM
    INTCONbits.GIE = 0; //deshabilita las interrupciones 
    
    //Secuencia de escritura
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1; //Iniciamos la Escritura
    EECON1bits.WREN = 0; //Deshabilitamos la escritura en la EEPROM
    
    INTCONbits.RBIF = 0; // limpiamos bandera en el puerto b
    INTCONbits.GIE = 1; //habilita interrupciones globales 
            
}

uint8_t read_EEPROM(uint8_t address){
    while (WR||RD){;}   //Verifica WR, RD para saber si hay un proceso de escritura o lectura en proceso
    EEADR = address ;   //Asigna la dirección de memoria a leer
    EECON1bits.EEPGD = 0;//Leectrua a la EEPROM
    EECON1bits.RD = 1;  //Se obtien el dato de la EEPROM
    return EEDAT;       //Retorno del valor Leido
}
