# V25_Semester_Prosjekt
Design og utvikling av en prototype for en avstandsmåler som brukes som ryggesensor i bil

Arduino 1 (Hovedenhet) "M1":
- TO DO: Viser avstand på en LCD-skjerm ved aktivert rygge sensor
            Avstandsvisning 0x9 - 0x15 for høyre visning, en x = 20 cm
            |          xxxxxxx | Høyre avstand er under 20 cm

            Avstandsvisning 0x0 - 0x6 for venstre visning, en x = 20 cm
            | xxxxxxx          | Venstre avstand under 20 cm

            | xxxxxxx  xxxxxxx | Begge avstandene er under 20 cm

            Bitvisning 16x2 LCD skjerm
            | 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 | (Y = 0)
            | 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 | (Y = 1)

            Avlesning av høyre 1x13 - 1x15 og enhet vising på 1x7 - 1x8
            |          xxxxxxx | Høyre avstand er under 20 cm
            |        cm    000 | Høyre avstand er under 20 cm
            
            Avlesning av venstre 1x00 - 1x2 og enhet vising på 1x7 - 1x8
            | xxxxxxx          | Venstre avstand under 20 cm
            | 000    cm        | Venstre avstand under 20 cm

            LCD visning 16x2         
            | xxxxxxx  xxxxxxx | Begge avstandene er under 20 cm
            | 000    cm    000 | Begge avstandene er 0 cm

            Klokkeslett i 0x0 - 0x4 hh:mm og temperatur.
            | hh:mm      exx*C | hh = time, mm = minut, e = fortegn, xx = temperatur
            |                  | 
        
- TO DO: Lyd ved 50 cm avstand og stigene lyd til 0 cm.
- TO DO: PDC Knapp med LED indikasjon når PDC er aktivert. Knapp kan skru av PDC selv om gir spaken er i revers.
- TO DO: Gir spake sensor for revers, aktiverer PDC.
- DONE: Bruker LED-lys (Rød < 50cm, 50cm =< Gul >= 100cm,100cm =< Grønn >= 150cm) for visuell varsling basert på avstand

Arduino 2 (Slaveenhet) "S1":
- Måler avstand til hindringer med 2 x HC-SR04 ultralydsensor.
          Høyre 
          Venstre
- DONE: Sender måledata til Arduino 1 - Master ehneten via UART
- TO DO: Hengerfeste sensor for disabling av ryggesensor.
 
Kode struktur:

- Master.ino
    - Communication.ino
        - Mottar data fra slave via serial protokol.
            - Avstandsmålinger; Høyre , Venstre
    - Distance_Indication_LED.ino
        - Kaller opp communication og avstand målingene som blir mottatt fra slave enhet.
            - Styrer LED lys bastert på avstandskriteriner.
    - ID_module.ino
        - ID visning på matric visning M1 eller S1 osv ved oppstart.
