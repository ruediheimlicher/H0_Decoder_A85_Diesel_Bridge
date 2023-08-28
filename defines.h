//
//  defines.h
//  H0_Decoder
//
//  Created by Ruedi Heimlicher on 11.09.2020.
//

#ifndef defines_h
#define defines_h


#define LOOPLEDPORT      PORTB
#define LOOPLEDDDR      DDRB
#define LOOPLED         3 // wie Motoraux
#define LEDPWM          50

#define INT0_RISING        0
#define INT0_FALLING       1


#define SHORT 0 // Abstand im doppelpaket
#define LONG 2 // Abstand zwischen Daten

#define OSZIPORT        PORTB      // Ausgang 
#define OSZIDDR         DDRB

#define OSZIA 0           // Lampe A


#define PAKETA          0
#define PAKETB          1

#define OSZIALO OSZIPORT &= ~(1<<OSZIA)
#define OSZIAHI OSZIPORT |= (1<<OSZIA)
#define OSZIATOG OSZIPORT ^= (1<<OSZIA)




#define MOTORPORT   PORTB
#define MOTORDDR    DDRB
#define MOTORPIN    PINB

#define LAMPEPORT    PORTB
#define LAMPEDDR     DDRB
#define LAMPEPIN     PINB


//PINS
// von Decoder84


#define MOTORA_PIN      3
#define MOTORB_PIN      4


#define SNIFFPORT    PORTB 
#define SNIFFDDR     DDRB
#define SNIFFPIN     PINB


#define SNIFF_PIN      0 // used as OSZI

#define LAMPEB_PIN      1


#define MOTORDIR        0
#define MOTOROUT        1
#define MOTORINT0       2
#define MOTORAUX        3
#define LAMPE           4



#define MOTORTEILER     1

// Bits
#define FUNKTIONOK      2
#define ADDRESSOK       3
#define DATAOK          4


// lokstatus-Bits
#define FUNKTION        0
#define OLDFUNKTION     1
#define FUNKTIONSTATUS  2






#define ADDRESSBIT      0
#define DATABIT         2
#define VORBIT          3 // eingestellte richtung
#define FUNKTIONBIT     4
#define RUNBIT  5
#define RICHTUNGBIT     6
#define CHANGEBIT       7  

#define STARTDELAY      100

#define TRIT0 0
#define TRIT1 1
#define TRIT2 2
#define TRIT3 3
#define TRIT4 4

//#define HI_IMPULSDAUER 10
//#define LO_IMPULSDAUER 20

#define INPIN     PINB
#define INT0_START      0
#define INT0_END        1
#define INT0_WAIT       2

#define INT0_PAKET_A    4
#define INT0_PAKET_B    5

//#define INT0_RUN        2 // detektiert Datenfluss

#define LAMPEMAX 0x40 // 50%

#define FIRSTRUN_END 80


#endif /* defines_h */
