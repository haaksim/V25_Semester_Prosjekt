/*
Tittel: 
Semesterprosjekt – Ryggesensor Prototype

------------------- Beskrivelse av system -------------------:
Semesterprosjektet omhandler utviklingen av en ryggesensor-prototype for bil, bestående av to Arduino-enheter: en hovedenhet (M1) og en slaveenhet (S1). 
Slaveenheten måler avstand til hindringer på høyre og venstre side med ultralydsensorer og sender data via UART til hovedenheten, 
som viser informasjonen på en LCD-skjerm og gir visuelle varsler med LED-lys. Systemet aktiveres automatisk ved reversgir eller manuelt via en knapp, og 
deaktiveres hvis henger er tilkoblet. Brukeren får informasjon om både avstand og klokkeslett, 
og systemet benytter sanntidsklokke og temperaturmåling for utvidet funksjonalitet.

------------------- Arduino 1 (Hovedenhet) "M1" -------------------:
- DONE: Viser avstand på en LCD-skjerm ved aktivert rygge sensor styres av Parkingsknappen og Revers gir.
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

- DONE:  Klokkeslett i 0x0 - 0x4 hh:mm og Dato.
            | hh:mm     mnd dd | hh = time, mm = minut, mnd = måned eks Feb, dd = dag
            |                  | 

- TO DO: Klokken og dato oppdaters automatisk når tilkoblet til PC. 
- DONE: Parking Knapp med LED indikasjon når PDC er aktivert. Knapp kan skru av PDC selv om gir spaken er i revers.
- DONE: Gir spake sensor for revers, aktiverer Parking.
- DONE: Bruker LED-lys (Rød < 50cm, 50cm =< Gul >= 100cm,100cm =< Grønn >= 150cm) for visuell varsling basert på avstand

------------------- Arduino 2 (Slaveenhet) "S1" -------------------:
- Måler avstand til hindringer med 2 x HC-SR04 ultralydsensor.
          Høyre 
          Venstre
- DONE: Sender måledata til Arduino 1 - Master ehneten via UART
- DONE: Hengerfeste sensor for disabling av ryggesensor.
- TO DO: Temperatur avlesning og sending til Arduino 1 - Master ehneten for visning på LCD når bilen ikke har aktivert parking.
- TO DO: Lyd ved 50 cm avstand og stigene lyd til 0 cm, funksjonstesting.

------------------- Kode struktur -------------------:
Prosjektets kode er strukturert med fokus på lesbarhet, modularitet og videreutvikling. Koden er delt inn i tydelige seksjoner: 
inkludering av nødvendige biblioteker, definisjon av pinner og globale variabler, samt funksjonelle blokker for oppsett og logikk. 
Oppstartskoden (setup()) initialiserer alle maskinvarekomponenter, inkludert LCD-skjerm, LED-matrise, knapper, LED-varsler og sanntidsklokke (RTC). 
Hovedløkken (loop()) kaller på dedikerte funksjoner som håndterer tid, skjermvisning, UART-kommunikasjon, knappestatus og reverssensor.

Kommunikasjonen mellom hovedenheten og slaveenheten skjer via UART, hvor en buffer-struktur brukes for å samle og tolke innkommende data. 
Visningsfunksjonen lcdView() er ansvarlig for dynamisk oppdatering av LCD-skjermen basert på parkeringsmodus, mens ledIndication() kontrollerer LED-belysning avhengig av avstandsdata. 
Koden benytter også hjelpefunksjoner for blant annet datoformattering og avstandshåndtering, som gjør den enkel å vedlikeholde og utvide.

Dato: 14. mars 2025
Forfatter: Håkon Simonsen
Aktiv Revisjon: 2.0.14
Revision Dato: 18. April 2025

------------------- Version Endringer -------------------:

v2.0.14
- Forbedre koden for hastighet og gjenntagelser.

v2.0.13
- Lag til sendings protokol for status på parkering til slave

v2.0.12
- La til henger status og styring av parkingen, hvis henger er tilkoble skal ikke parkering virke.
- Oppdaterte kommunikasjonskoden, slette serial print koder.

v2.0.11
- La til klokkeslett og dato visning i display når bilen ikke er i parkering.

v2.0.10
- Forenkling av parkingsbryterene og la til styring av parkeringsLEDs.
- Fjernet settings.h

v2.0.9
- Lag til Parking bryter, og revers signal fra girkasse som aktivere parkerings delen av LCDen.

v2.0.8
- Endret Display koden til å bruke en enkel ternary-operator (? :) for å bestemme riktig plassering basert på antall siffer. 

v2.0.7
- Endret visningen på 0x0 - 0x6 og 0x15 - 0x9 fra x til full byte "hel firkant".

v2.0.6 
- Forbedring av kommunikasjons koden

v2.0.5 
- Endre på pin nave fra lowercase to uppercase og LED_PIN, _PIN, BUTTON_PIN som denne standared

v2.0.4 
- Lag til temperature sensor THD 34 tilkoblet på til Master Unit "M1"

v2.0.3
- Lag til Settings.h, slik at alle instilling kan endres fra en plass.

v2.0.2 
- Lag til kode for bruk av LCD "carLCD" tilkyttet fysisk Master Unit "M1"

v2.0.1
- Endring i av koden ved bruk av moderne C++-best praktis på baud rate.

*/

// Legger til de ulike bibliotekene vi trenger for at kode skal virke
#include "ArduinoGraphics.h"                                                                        // Bibliotek som gir grafiske funksjoner som tekst og former til LED Matrix
#include "Arduino_LED_Matrix.h"                                                                     // Bibliotek som brukes til å styre den innebygde LED-matrisen på Arduino Nano 33 BLE Sense
#include <LiquidCrystal.h>                                                                          // Bibliotek for å styre et 16x2 LCD-display via 4-bit datalinjer
#include "RTC.h"                                                                                    // Bibliotek for å hente tid og dato fra RTC-modulen (Real Time Clock)

// Sette opp seriell & UART kommunikasjon
constexpr unsigned long BAUD_RATE_M = 115600;                                                       // Baudrate for kommunikasjon med seriell monitor (for debugging)
constexpr unsigned long BAUD_RATE_C = 9600;                                                         // Baudrate for UART-kommunikasjon mellom hovedenhet og slave

// Lager objekt for LED-matrisen
ArduinoLEDMatrix matrix;                                                                            // Oppretter et objekt for å kunne vise tekst eller grafikk på LED-matrisen

// Innstillinger for bruk med ArduinoGraphics biblioteket
int br = BAUD_RATE_M;                                                                               // Baudrate-variabel brukt i grafikkbiblioteket (her satt lik seriell-monitor hastighet)
int dt = 100;                                                                                       // Delay-tid (100 ms) mellom visningsoppdateringer på LED-matrisen

// Navn som skal vises på LED Matrixen for å lettere identifisere Arduino-enheten visuelt
String unitId = "M1";                                                                               // Enhets-ID (f.eks. "M1" for Master) som kan vises på LED-matrisen

// Analog-pinner

// LED-pinner (digital output)
const int RED_LED_PIN = 13;                                                                         // Rød LED – lyser når avstand er kritisk (nærme hinder)
const int YELLOW_LED_PIN = 12;                                                                      // Gul LED – lyser når avstand er mellomlang
const int GREEN_LED_PIN = 11;                                                                       // Grønn LED – lyser når det er god avstand
const int PARKING_LED_PIN = 10;                                                                     // Indikerer om parkeringssystemet er aktivert

// Bryter-pinner (digital input)
const int PARKING_BUTTON_PIN = 9;                                                                   // Pinne til parkeringsknapp – aktiverer eller deaktiverer parkeringsmodus
const int REVERS_BUTTON_PIN = 8;                                                                    // Pinne koblet til revers-signal eller bryter – aktiverer automatisk parkeringsmodus

// LCD-pinner (tilkobling til LiquidCrystal display)
const int RS_PIN = 7;                                                                               // Register Select – styrer kommando vs data
const int ENABLE_PIN = 6;                                                                           // Enable – aktiverer LCD for å lese data
const int D4_PIN = 5;                                                                               // Data pin 4
const int D5_PIN = 4;                                                                               // Data pin 5
const int D6_PIN = 3;                                                                               // Data pin 6
const int D7_PIN = 2;                                                                               // Data pin 7

// Initialiserer LCD med valgt pinnekonfigurasjon (4-bits modus)
LiquidCrystal carLCD(RS_PIN, ENABLE_PIN, D4_PIN, D5_PIN, D6_PIN, D7_PIN);                           // Oppretter et LCD-objekt for å kunne skrive til displayet

// Lag et fullt blokk-tegn for bruk som status bar (spesialtegn på LCD)
byte fullBlock[8] = {                                                                               // Definerer 5x8 piksler for fullt rektangel (brukes til visuell statusbar)
  B11111,                                                                                           // Topp rad: alle piksler på
  B11111,                                                                                           // ...
  B11111,                                                                                           // ...
  B11111,                                                                                           // ...
  B11111,                                                                                           // ...
  B11111,                                                                                           // ...
  B11111,                                                                                           // ...
  B11111                                                                                            // Bunn rad: alle piksler på
};

 
// Kommunikasjon UART fra slave
String buffer = "";                                                                                 // Midlertidig tekstbuffer for mottatt data fra UART (f.eks. "L:120 R:110 T:1")

int rightDistanceInt = 0;                                                                           // Høyre avstand avrundet til nærmeste 10 cm, brukt til visning
int rightDistance = 0;                                                                              // Høyre avstand i cm (råverdi uten desimaler) mottatt fra slave

int leftDistanceInt = 0;                                                                            // Venstre avstand avrundet til nærmeste 10 cm, brukt til visning
int leftDistance = 0;                                                                               // Venstre avstand i cm (råverdi uten desimaler) mottatt fra slave

bool trailerStatus = false;                                                                         // Status på tilkoblet henger – true hvis henger er tilkoblet (mottatt fra slave)

// Tid for display
String currentDisplayTime = "";                                                                     // Klokkeslett i format HH:MM som vises på displayet
String currentDisplayDay = "";                                                                      // Dato (dag i måneden) som vises på displayet
String currentDisplayMonth = "";                                                                    // Måned (forkortet navn) som vises på displayet

// Brytervariabler
bool parkingLcdState = false;                                                                       // Styrer om LCD skal vise parkeringsvisning (true når aktiv)
bool parkingLedState = false;                                                                       // Styrer om LED skal vises (true når aktiv)
bool lastParkingButtonState = false;                                                                // For å sjekke endring i status på parkeringsknappen (brukes til å oppdage trykk)
bool lastReversState = false;                                                                       // For å sjekke endring i reversstatus (unngå gjentatt trigging)


// RTC variabel
RTCTime currentTime;                                                                                // Global variabel for lagring av sanntid, brukes i hele prosjektet

// Funksjon som returnerer månedsnavn (kort format) basert på månedens nummer
String getMonthShortName(int month) {
  switch (month) {                                                                                  // Bruker switch-setning for å sammenligne variabelen "month" med flere faste verdier (1–12)
    case 1:  return "Jan";                                                                          // Hvis month == 1, returner "Jan" og avslutt funksjonen
    case 2:  return "Feb";                                                                          // Hvis month == 2, returner "Feb" og avslutt funksjonen
    case 3:  return "Mar";                                                                          // Hvis month == 3, returner "Mar" og avslutt funksjonen
    case 4:  return "Apr";                                                                          // Hvis month == 4, returner "Apr" og avslutt funksjonen
    case 5:  return "May";                                                                          // Hvis month == 5, returner "May" og avslutt funksjonen
    case 6:  return "Jun";                                                                          // Hvis month == 6, returner "Jun" og avslutt funksjonen
    case 7:  return "Jul";                                                                          // Hvis month == 7, returner "Jul" og avslutt funksjonen
    case 8:  return "Aug";                                                                          // Hvis month == 8, returner "Aug" og avslutt funksjonen
    case 9:  return "Sep";                                                                          // Hvis month == 9, returner "Sep" og avslutt funksjonen
    case 10: return "Oct";                                                                          // Hvis month == 10, returner "Oct" og avslutt funksjonen
    case 11: return "Nov";                                                                          // Hvis month == 11, returner "Nov" og avslutt funksjonen
    case 12: return "Dec";                                                                          // Hvis month == 12, returner "Dec" og avslutt funksjonen
    default: return "   ";                                                                          // Hvis verdien ikke er mellom 1 og 12, returner tom tekst ("   ") som standardtilfelle
  }
}



//--------------------------------------------------------------------------------------------------// Start av voids

// Setup kjøres én gang ved oppstart        
void setup() {
  Serial.begin(BAUD_RATE_M);                                                                        // Start seriell kommunikasjon med PC
  Serial1.begin(BAUD_RATE_C);                                                                       // Start seriell kommunikasjon med slave "UART"

// Oppsett for LED Matrix
  matrix.begin();                                                                                   // Initialiserer LED-matrisen
  matrix.beginDraw();                                                                               // Starter å tegne på matrisen
  matrix.textFont(Font_5x7);                                                                        // Bruker standard 5x7 font                                      
  matrix.beginText(1, 1, 255, 0, 0);                                                                // Skriver teksten med rød farge, men har ikke noen å si pga LED er ikke RGB
  matrix.println(unitId);                                                                           // Skriver enhets IDen
  matrix.endText();                                                                                 // Avslutter tekstblokken
  matrix.endDraw();                                                                                 // Avslutter tegningen og viser resultatet

// Oppsett for knappestyring
  pinMode(PARKING_BUTTON_PIN, INPUT_PULLUP);                                                        // PTC-knapp (aktiv lav)
  pinMode(REVERS_BUTTON_PIN, INPUT_PULLUP);                                                         // Revers-bryter (aktiv lav)
  pinMode(PARKING_LED_PIN, OUTPUT);                                                                 // LED for parkeringsindikasjon

// Oppsett for LCD visningen
  carLCD.begin(16, 2);                                                                              // Set up the number of columns and rows on the LCD.
  carLCD.createChar(0, fullBlock);                                                                  // Lag fullt rektangel på plass 0

// Oppsett av LED Status Lys
  pinMode(RED_LED_PIN, OUTPUT);                                                                     // Setter (rød LED) som utgang slik at den kan styres av mikrokontrolleren.
  pinMode(YELLOW_LED_PIN, OUTPUT);                                                                  // Setter (gul LED) som utgang.
  pinMode(GREEN_LED_PIN, OUTPUT);                                                                   // Setter (grønn LED) som utgang.
  pinMode(PARKING_LED_PIN, OUTPUT);                                                                 // Setter (park LED) som utgang.

// Oppsett av RTC-modul ved oppstart
  RTC.begin();                                                                                      // Initialiserer RTC-modulen

// Setter starttid: 17. april 2025, 12:18:00, onsdag, sommertid aktiv
  // RTCTime startTime(18, Month::APRIL, 2025, 12, 16, 30, DayOfWeek::WEDNESDAY, SaveLight::SAVING_TIME_ACTIVE);
  // RTC.setTime(startTime);                                                                        // Lagrer starttiden i RTC

}

// Looper som kjøres kontinuerlig
void loop() {
  rtcTime();                                                                                        // Henter og oppdaterer tid og dato fra RTC-modulen
  lcdView();                                                                                        // Viser informasjon på LCD-skjermen (klokke, dato eller avstand)
  ledIndication();                                                                                  // Styrer LED-lysene basert på avstand til hindringer
  communicationUART();                                                                              // Sender og mottar avstandsdata mellom Arduino-enheter via UART
  parkingButton();                                                                                  // Leser status fra parkeringsknappen og oppdaterer parkeringstilstand
  reversState();                                                                                    // Sjekker om bilen er i revers og håndterer eventuell overstyring
  
}

// Kommunikasjon med Slave via UART seriall link
void communicationUART() {
  while (Serial1.available()) {                                                                     // Sjekk om det er data tilgjengelig fra slave
    char c = Serial1.read();                                                                        // Les ett og ett tegn fra UART, eks på kommunikasjon fra slaven enhet er R:121L:98\n

    if (c == '\n') {                                                                                // Hvis linjeslutt "\n" er nådd (hele meldingen er mottatt), utføres kommandoen. Info: Slave enhet sender siste beskjed i en serial.println(siste beskjed);
      rawCommData(buffer);                                                                          // Sender rå data mottatt videre til void rawCommData som henter ut avstandene og annen info, eks på buffer data er R:121L:98
      buffer = "";                                                                                  // Nullstill buffer for neste melding
    } else {
      buffer += c;                                                                                  // Legg til tegn for tegn i bufferen
      Serial.print("buffer: ");
      Serial.println(buffer);
    }
  }

  delay(10);                                                                                        // Liten pause for å stabilisere mottak

  // Send data til master i formatet "R:123 L:45"
  Serial1.print("P:");                                                                              // Skriver "P:"
  Serial1.println(parkingLcdState);                                                                 // Sender venstre avstand + linjeskift (viktig for parsing)

  // Debug output
  Serial.print("Sendt: P:");                                                                        // Skriver hele strengen som ble sendt for debugging
  Serial.println(parkingLcdState);                                                                  //
}

// Her analysers sensor dataen og henter ut høyre & venstre avstander.
void rawCommData(String data) {                                                                     // Mottar rawCommData fra buffer i void commicationUART og skriver det til String "data"
  int distanceSensor_R = data.indexOf("R:");                                                        // Finn posisjonen til "R:" i rå data mottatt fra slave enheten, hvis posisjon ikke ble funnet returners -1
  int distanceSensor_L = data.indexOf("L:");                                                        // Finn posisjonen til "L:" i rå data mottatt fra slave enheten, hvis posisjon ikke ble funnet returners -1
  int trailerSensor = data.indexOf("T:");                                                           // Finn posisjonen til "T:" i rå data mottatt fra slave enheten, hvis posisjon ikke ble funnet returners -1

  // Høyre avstandsensor data prosesering
  if (distanceSensor_R != -1 && distanceSensor_L != -1 && trailerSensor != -1) {                    // Hvis alle nøkkelord er funnet
    String distanceSensor_RVal = data.substring(distanceSensor_R + 2, distanceSensor_L);            // Hent verdien mellom R: og L:
    String distanceSensor_LVal = data.substring(distanceSensor_L + 2, trailerSensor);               // Hent verdien etter L: og T:
    String trailerSensorVal = data.substring(trailerSensor + 2);                                    // Hent verdien etter L: og T:

    distanceSensor_RVal.trim();                                                                     // Fjern eventuelle mellomrom og linjeskift fra høyreverdi
    distanceSensor_LVal.trim();                                                                     // Fjern eventuelle mellomrom og linjeskift fra venstreverdi
    trailerSensorVal.trim();                                                                        // Fjern eventuelle mellomrom og linjeskift fra venstreverdi

    rightDistance = distanceSensor_RVal.toInt();                                                    // Konverter høyre verdi til helt tall, fjerner desimal pga Int veriable
    leftDistance = distanceSensor_LVal.toInt();                                                     // Konverter venstre verdi til helt tall, fjerner desimal pga Int veriable
    trailerStatus = trailerSensorVal.toInt();                                                       // Konverter henger status til hel verdi.

    rightDistanceInt = (constrain((int)rightDistance, 2, 400) / 10) * 10;                           // Begrens mellom 2 – 400 cm og rund ned til nærmeste 10, eks 118 / 10 * 10 blir 110 fordi Int veriable lager bare hele tall så 118 / 10 blir 11 og ikke 11.8
    leftDistanceInt = (constrain((int)leftDistance, 2, 400) / 10) * 10;                             // Begrens mellom 2 – 400 cm og rund ned til nærmeste 10, eks 118 / 10 * 10 blir 110 fordi Int veriable lager bare hele tall så 118 / 10 blir 11 og ikke 11.8
 
  } else {
    // Feilmelding hvis nødvendig informasjon ikke ble funnet
    Serial.println("Feil i datamottak, sjekk kommunikasjonen");
  }
    
}

// Denne voiden lager LCD visningen 
void lcdView() {
                                                                                                    // Leser status fra parkeringsknapp
                                                                                                    // Oppdaterer avstandsdata fra sensorer via UART
  carLCD.clear();                                                                                   // Tømmer LCD-skjermen før ny informasjon skrives

  if (parkingLcdState) {                                                                            // Hvis bilen er i parkeringsmodus, vis avstandsinformasjon på skjermen

    // Venstre side status bar visning og avstandsvisning
    if (leftDistance >= 0 && leftDistance <= 150) {                                                 // Hvis venstre avstand er mellom 0 og 150 cm
      int numberOfX = (140 - leftDistance) / 20;                                                    // Regner ut antall blokker som skal vises for status bar (hver blokk ~20 cm)
      for (int i = 0; i <= numberOfX && i < 7; i++) {                                               // Loop for å skrive ut blokker fra posisjon 0 til maks 6
        carLCD.setCursor(i, 0);                                                                     // Setter markøren i kolonne i, rad 0 (øverst til venstre)
        carLCD.write(byte(0));                                                                      // Skriver spesialtegn (fullt rektangel) som status bar
      }

      carLCD.setCursor(0, 1);                                                                       // Setter markøren nederst til venstre (kolonne 0, rad 1)
      carLCD.print(leftDistanceInt);                                                                // Skriver avstand i cm (avrundet) fra venstre sensor

      carLCD.setCursor(7, 1);                                                                       // Setter markøren midt på nederste linje (kolonne 7, rad 1)
      carLCD.print("cm");                                                                           // Skriver enheten "cm"
    }

    // Høyre side status bar visning og avstandsvisning
    if (rightDistance >= 0 && rightDistance <= 150) {                                               // Hvis høyre avstand er mellom 0 og 150 cm
      int numberOfX = (140 - rightDistance) / 20;                                                   // Regner ut antall blokker for høyre side basert på avstand
      for (int i = 0; i <= numberOfX && i < 7; i++) {                                               // Loop for å skrive ut blokker fra posisjon 15 ned til maks 9
        carLCD.setCursor(15 - i, 0);                                                                // Setter markøren fra høyre mot venstre (kolonne 15 til 9)
        carLCD.write(byte(0));                                                                      // Skriver blokk som representerer avstand på høyre side
      }

      int rightCursorPosision = (rightDistance < 10) ? 15 : (rightDistance < 100) ? 14 : 13;        // Beregner riktig posisjon for å høyrejustere tallvisning (én, to eller tre siffer)

      carLCD.setCursor(rightCursorPosision, 1);                                                     // Setter markøren nederst til høyre (kolonne avhengig av sifferlengde, rad 1)
      carLCD.print(rightDistanceInt);                                                               // Skriver høyre avstand i cm (avrundet)

      carLCD.setCursor(7, 1);                                                                       // Setter markøren midt på nederste linje (kolonne 7, rad 1)
      carLCD.print("cm");                                                                           // Skriver enheten "cm"
    }
  }
  else {                                                                                            // Hvis bilen ikke er i parkeringsmodus, vis klokke og dato

    carLCD.setCursor(0, 0);                                                                         // Setter markøren øverst til venstre (kolonne 0, rad 0)
    carLCD.print(currentDisplayTime);                                                               // Skriver klokkeslett i format HH:MM

    carLCD.setCursor(10, 0);                                                                        // Setter markøren øverst mot høyre (kolonne 10, rad 0)
    carLCD.print(currentDisplayMonth);                                                              // Skriver måned i kort tekstformat (f.eks. "Apr")

    carLCD.setCursor(14, 0);                                                                        // Setter markøren helt til høyre (kolonne 14, rad 0)
    carLCD.print(currentDisplayDay);                                                                // Skriver dagens dato (dag i måneden)
  }

  delay(50);                                                                                        // Liten pause for å unngå for hyppig oppdatering
}

// Denne voiden styrer LCD visning om den skal være i revers modus eller ikke
void reversState() {
  bool isReversActive = !digitalRead(REVERS_BUTTON_PIN);                                            // Leser reversknappen: aktiv (true) når knappen er trykket (lavt signal pga en pullup motstand)

  // Hvis revers akkurat ble aktivert, skrur på parking modus
  if (isReversActive && !lastReversState && !trailerStatus) {
    parkingLcdState = true;                                                                         // Skru på LCD for parkering
    parkingLedState = true;                                                                         // Skru på LED for parkering
  }

  // Hvis revers akkurat ble deaktivert, skrus av parking modus
  if (!isReversActive && lastReversState) {                                                         // Revers er nå deaktivert, men var aktiv forrige gang
    parkingLcdState = false;                                                                        // Skru av LCD for parkering
    parkingLedState = false;                                                                        // Skru av LED for parkering
  }

  digitalWrite(PARKING_LED_PIN, parkingLedState ? HIGH : LOW);                                      // Oppdater LED-utgang basert på parkeringstilstand
  lastReversState = isReversActive;                                                                 // Husk revers-status til neste syklus
}

void parkingButton() {
  bool currentParkingButtonState = !digitalRead(PARKING_BUTTON_PIN);                                // Leser knappens status (aktiv lav, altså true når trykket)

  // Kan bare brukes når bilens girkasse ikke er i revers
  if (!lastReversState && !trailerStatus) {                                                                           // Hvis revers ikke er aktiv nå
    if (currentParkingButtonState && !lastParkingButtonState) {                                     // Hvis knappen akkurat ble trykket (fra lav til høy)
      parkingLcdState = !parkingLcdState;                                                           // Toggle LCD-tilstanden
      parkingLedState = parkingLcdState;                                                            // Synkroniser LED med LCD

      digitalWrite(PARKING_LED_PIN, parkingLedState ? HIGH : LOW);                                  // Oppdater LED basert på ny tilstand
    }
  }

  lastParkingButtonState = currentParkingButtonState;                                               // Husk knappens tilstand til neste syklus
}

// void loop for LED-styring basert på avstand
void ledIndication() {                                                                              // Oppdatere LED-status i sanntid.

  if (parkingLedState) {                                                                            // Hvis bilen er i parkeringsmodus kjøres kommandoene og lysen tennes når avstanden er riktig
  int minDistance = min(rightDistance, leftDistance);                                               // Beregner nærmeste avstand mellom høyre og venstre sensor ved å bruke funksjonen min().

  digitalWrite(RED_LED_PIN, minDistance < 50);                                                      // Hvis den nærmeste avstanden er under 50 cm, tennes den røde LED-en (logisk HIGH).
  digitalWrite(YELLOW_LED_PIN, minDistance >= 50 && minDistance < 100);                             // Tenn gul LED hvis avstanden er mellom 50 og 99 cm.
  digitalWrite(GREEN_LED_PIN, minDistance >= 100 && minDistance < 150);                             // Tenn grønn LED hvis avstanden er mellom 100 og 149 cm.
  }
  else {
  digitalWrite(RED_LED_PIN, LOW);                                                                   // Hvis bilen ikke er i parkeringsmodus blir rødt lysen slått av
  digitalWrite(YELLOW_LED_PIN, LOW);                                                                // Hvis bilen ikke er i parkeringsmodus blir gult lysen slått av
  digitalWrite(GREEN_LED_PIN, LOW);                                                                 // Hvis bilen ikke er i parkeringsmodus blir grønt lysen slått av
  }
}

void rtcTime() {
  RTC.getTime(currentTime);                                                                         // Henter aktuell tid fra RTC

  // Setter tid i format HH:MM som streng
  currentDisplayTime = String(currentTime.getHour()) + ":" + String(currentTime.getMinutes());      // Lager klokkeslett-streng i format HH:MM

  // Henter og lagrer dag i måneden som streng
  currentDisplayDay = String(currentTime.getDayOfMonth());                                          // Henter dagsdato som streng (f.eks. "17")

  // Henter månedsnummer og konverterer til kort tekst (f.eks. "Apr")
  int monthInt = Month2int(currentTime.getMonth());                                                 // Konverterer månedsverdi til heltall
  currentDisplayMonth = getMonthShortName(monthInt);                                                // Henter forkortet månedsnavn (f.eks. "Apr")

  delay(50);                                                                                        // Liten pause for å unngå spam i seriell monitor
}
