/*
Tittel: 
Semesterprosjekt – Ryggesensor Prototype

------------------- Beskrivelse av system -------------------:
Semesterprosjektet omhandler utviklingen av en ryggesensor-prototype for bil, basert på to Arduino-enheter: en hovedenhet (M1) og en slaveenhet (S1).
Slaveenheten (S1) måler avstand til hindringer på høyre og venstre side ved hjelp av ultralydsensorer, og sender kontinuerlig avstandsdata, temperatur og sanntidsklokkeinformasjon til hovedenheten via UART-kommunikasjon.
Hovedenheten (M1) mottar og tolker disse dataene, og presenterer informasjonen på en LCD-skjerm, i tillegg til å varsle brukeren med LED-indikatorer og buzzer-lyd basert på avstand.

Systemet aktiveres automatisk ved deteksjon av reversignal, eller kan aktiveres manuelt via knappetrykk. Hvis en henger er tilkoblet, overstyres systemet automatisk for å unngå feiltolkning.
Brukeren får sanntidsvisning av avstander, klokkeslett og temperatur, samt tydelig feilhåndtering hvis en sensor eller kommunikasjon feiler.
Løsningen kombinerer presis måleteknologi, responsiv varsling og brukervennlig grensesnitt for å støtte trygg rygging under varierte forhold.

------------------- Kode struktur -------------------:
Koden er modulært bygget opp i tydelige seksjoner:
- Biblioteker, pin-definisjoner og variabler
- Funksjoner for initiering, visning, feilhåndtering, kommunikasjon og knappelesing
- Loop() er strukturert for kontinuerlig oppdatering av skjerm, feilkoder og buzzer
- Kommentert for lesbarhet og fremtidig vedlikehold.

Dato: 14. mars 2025
Forfatter: Håkon Simonsen
Aktiv Revisjon: 2.0.23
Revisjon Dato: 30. april 2025
Enhet ID: Master Unit "M1"  

Kompanjongkode for slaveenhet: Versjon 2.0.13

------------------- Versjon Endringer -------------------

v2.0.24
- Endre fra 400cm til 450cm

v2.0.23
- La til feil handtering av temperatur sensor, Error E7 ved mottatt verdi av 600. 

v2.0.22
- Forbedringer i PDC-aktivering:
  - PDC deaktiveres automatisk dersom begge sensorer på en side feiler.
  - LCD viser dynamisk "PDC Unavailable" med feilkoder hvis parkering ikke kan aktiveres.
  - Automatisk deaktivering av parkerings-LED og buzzer ved feil.
- Feilhåndtering:
  - Redusert modus vises ("Left Reduced" / "Right Reduced") dersom kun én sensor på en side feiler.
- Kommunikasjon:
  - Bedre håndtering av UART-feil og forbedret sekvensiell feilmelding på LCD.
- LCD-Visning:
  - Forbedret feilmeldingssekvens ved kommunikasjonsfeil mellom M1 og S1.
  - Smidigere initial PDC-velkomstvisning inkludert redusert sensorstatus.
- Kode- og strukturforbedringer:
  - Ryddet og omstrukturert `error()`, `lcdView()` og feilhåndteringslogikk.
  - Kommentarer og dokumentasjon oppdatert for klarhet og vedlikehold.

v2.0.21
- Bugfix: 
  - Kombinerte avstander (`combinedLeft`/`combinedRight`) velger nå laveste gyldige sensorverdi selv ved feil, for mer pålitelig visning.

- Kommunikasjon:
  - Ny LCD-feilmelding for kommunikasjonsfeil ("Comms Error" vises sekvensielt).
  - Robust timeout-håndtering for både RX og TX.

- Forbedringer:
  - Smidigere buzzerstyring ved hindringer nærmere enn 50 cm.
  - Forbedret feilhåndtering i meny (Error Menu) med enklere navigasjon.
  - Økt stabilitet i UART-datahåndtering og avstandsvisning på LCD.

- Dokumentasjon: 
  - Oppdatert og utvidet kommentarer for enklere vedlikehold.

v2.0.20 - Revisjonsendring:
- Implementert komplett feilmeldingsmeny (Error Menu) på LCD med navigering via knapper.  
- Innført individuelle feilvariabler for hver sensor og kommunikasjonsretning (TX/RX).  
- Timeout-overvåking på UART-kommunikasjon, med egne feilkoder ved feil (E5/E6).  
- Forbedret grafisk visning av parkering og normalmodus på LCD (klokke, dato, temperatur).  
- Dynamisk visning av aktive feil på skjerm, eller "No Active Error" hvis ingen feil.  
- Buzzerstyring forbedret med trinnløs frekvens basert på avstand (innen 50 cm).  
- Periodisk "TX TEST" for kontroll av UART-forbindelse mot slaveenhet.  
- Omstrukturert kode for bedre lesbarhet, responsivitet og stabilitet.  
- Fjernet ubrukt EEPROM-støtte fra prosjektet.  
- Kommentarer og dokumentasjon oppdatert for enklere vedlikehold.

v2.0.19
- Forbedringer
  - Utvidet og forbedret håndtering av høyre og venstre sensorer
    - Ny og robust logikk for å kombinere `rightDistance` og `centreRightDistance`, samt `leftDistance` og `centreLeftDistance`:
      - Systemet velger nå automatisk den **minste og gyldige** verdien mellom hovedsensor og midtsensor for hver side.
      - Dette forbedrer nøyaktigheten når ett av sensorparene oppdager hindringer tidligere enn det andre.
    - Feilkoder `E1` til `E4` (sensorfeil) brukes nå som filtrering for ugyldige verdier i visningslogikken.

  - Oppdatert `lcdView()` med forbedret visning for høyre og venstre side:**
    - LCD-grafikk og tallvisning oppdateres nå kun hvis gyldige måledata er mottatt.
    - Viser enten tallverdi (i cm) eller tekst ("Left"/"Right") basert på avstand til hindring.
    - Bedre tilpasning til skjermens plass for verdier med 1, 2 eller 3 siffer.

- Andre endringer
  - Lagt til kommentarforbedringer i `lcdView()` for klarere vedlikehold og dokumentasjon.
  - Fjernet duplikatvisning for `leftDistanceInt` og `rightDistanceInt` i serial debug under `lcdView()`.

v2.0.18
- Ny funksjonalitet
  - Lagt til støtte for buzzer-varsling ved parkering:
    - Aktiverer buzzer med stigende lydfrekvens når avstand til hindring er mindre enn 50 cm.
    - Buzzer-lyden øker i intensitet etter hvert som avstanden minsker mot 0 cm.
    - Ny funksjon `parkingBuzzer()` implementert i hovedløkken (`loop()`).

- Forbedringer
  - Optimalisert parsing av UART-data fra slaveenheten (S1):
    - Implementert mer robust datahåndtering av innkommende sensordata (`CR:`, `CL:`).
    - Lagt til midtre ultralydsensorer (`centreRightDistance`, `centreLeftDistance`) med tilhørende variabler og logikk.
    - Forbedret feilhåndtering ved ufullstendige eller feilaktige data, inkludert tydeligere feilmeldinger.

  - Oppdatert LCD-visning for bedre brukeropplevelse:
    - Mer detaljert og nøyaktig visning av avstandsdata for både side- og midtsensorer på LCD-skjermen.
    - Justeringer i visningslogikken for å tydeliggjøre avstander og status.
    - Bedre animasjon ved aktivering av parkeringsmodus med tydeligere melding om PDC-status.

  - Diverse kodeopprydding og optimalisering:
    - Fjernet unødvendig duplikatdeklarering av pinner (`PARKING_LED_PIN`).
    - Generell forbedring av kodekommentarer og struktur for økt lesbarhet og enklere vedlikehold.

v2.0.17
- Ny funksjonalitet
  - Implementert støtte for sanntidsklokke (RTC) mottatt **via UART fra slaveenheten (S1)** i stedet for å bruke intern RTC i hovedenheten (M1).  
  - Ny parsing i `rawCommData()` for `RTC:`-tagg fra S1, som tolker full dato og tid som streng.  
  - Ny variabel `rtcSlave` brukes for å lagre RTC-strengen, som analyseres i `rtcTime()` for å hente ut `hh:mm`, dato og måned.  
  - Dette løser problemet med unøyaktig tid på Arduino UNO R4 WiFi, og gir nøyaktig visning på LCD.

- Forbedringer
  - `rtcTime()` er oppdatert for å bruke strengen `rtcSlave` mottatt fra S1, med sikring mot ufullstendig data.
  - Bedre kommentarlinjer og forklaringer rundt RTC-oppdatering og LCD-visning for lesbarhet og videre utvikling.

v2.0.16
- Forbedret LCD-visning i parkeringsmodus
  - Lagt til animert "PDC Enabled"-beskjed ved aktivering av parkeringsmodus første gang.
  - Utvidet visning med spesialtegn for rammer: venstre (`leftFrameBlock`), midt (`centreFrameBlock`), og høyre (`rightFrameBlock`) blokkrammer på LCD.
  - Bedre visuell fremstilling av statusbar for avstand (bruk av 4 tilpassede LCD-tegn).

- La til `parkingStartLcd`-logikk
  - Viser PDC-aktivering én gang når systemet først slår seg på i parkeringsmodus.

- Refaktorering av LCD-kode
  - Skiller mellom situasjoner der hindringer er innenfor rekkevidde og når ingen relevante målinger er mottatt.

- Diverse forbedringer
  - Bedre struktur på LCD-visningen og justert visningslogikk for høyre og venstre side individuelt.
  - Kommentarforklaringer forbedret for vedlikehold og forståelse.

v2.0.15
- Forbedret klokkeformat (RTC)
  Klokkeslett vises nå alltid i formatet `hh:mm` – også når time eller minutt er ensifret – ved å automatisk legge til `0` foran tall under 10. Dette gir en ren og konsekvent visning på LCD-en.

- INFO – Problemer med innebygd RTC på Arduino UNO R4 WiFi
  Den interne RTC-en bruker en lavpresisjons oscillator (LOCO) uten ekstern krystall, noe som kan føre til tidavvik på opptil 10 minutter per dag. Dette skyldes maskinvaren og ikke koden.  
  ---Løsning---
  Bruk en ekstern RTC-modul (f.eks. DS3231) for nøyaktig tidtaking i framtidige versjoner.

v2.0.14
- Ryddet og optimaliserte kodebasen.
- Fjernet dupliserte funksjonskall i lcdView() som tidligere kunne gi unødvendig belastning og forvirrende struktur.
- Forbedret generell ytelse ved å flytte funksjonskall som rtcTime(), communicationUART() og parkingButton() ut i loop() og bort fra visningsfunksjoner.

v2.0.13
- La til UART-overføring av parkeringsstatus (P:1/0) til slaveenheten (S1).
- Dette åpner for funksjonalitet som buzzer og temperatur kun når parkering er aktiv.
- Enkel debug-logg lagt til i communicationUART() for visuell bekreftelse.

v2.0.12
 - Forbedret kommunikasjon mellom hovedenhet og slaveenhet:
- Ryddet opp i UART-kommunikasjonen.
- Fjernet unødvendig Serial.print() fra debug for renere kjøring.
- Bedre håndtering av hengerstatus:
- Bruker nå trailerStatus til å blokkere både knappetrykk og automatisk reversaktivering hvis henger er tilkoblet.
- Generell kodeopprydding og mer robust parsing i rawCommData().

v2.0.11
- La til visning av klokkeslett og dato når parkering ikke er aktiv.
- La til støtte for hengerdeteksjon: 
  Dersom henger er koblet til, skal ikke parkeringen aktiveres – hverken via knapp eller reverssignal.

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

/*---------------------- Biblioteker ----------------------
Biblioteker som inkluderes for å muliggjøre:
- Grafisk håndtering på LED-matrise
- Tekst/kommando-styrt LCD-skjerm (4-bit)
- Tid og dato fra RTC-modul
- EEPROM lagring for permanente brukerinnstillinger
-----------------------------------------------------------*/
#include "ArduinoGraphics.h"                                                                         // Grafikkfunksjoner for LED-matrisen
#include "Arduino_LED_Matrix.h"                                                                      // Kontroll av LED-matrisen på Nano 33 BLE Sense
#include <LiquidCrystal.h>                                                                           // Bibliotek for 16x2 LCD via 4-bit datalinjer
#include "RTC.h"                                                                                     // Bibliotek for sanntidshåndtering (Real Time Clock)

/*---------------------- Tidskontroll ----------------------*/
unsigned long previousMillisBuzzer     = 0;                                                          // Siste tid buzzer ble aktivert
unsigned long previousMillisRtc        = 0;                                                          // Siste tid RTC-data ble oppdatert
const long intervalRtc                 = 50;                                                         // Oppdateringsintervall for RTC (50 ms)

unsigned long previousMillisError      = 0;                                                          // Sist feilmelding ble oppdatert
const long intervalError               = 1000;                                                       // Hvor lenge hver feilmelding vises (1 sekund)

unsigned long previousMillisLcd        = 0;                                                          // Sist feilmelding ble oppdatert
const long intervalLcd                 = 200;                                                       // Hvor lenge hver feilmelding vises (1 sekund)

unsigned long previousMillisErrorMenu  = 0;                                                          // Sist feilmelding ble oppdatert i meny
const long intervalErrorMenu           = 1000;                                                       // Hvor lenge feilmeldinger vises i meny (1 sekund)

unsigned long lastCommunicationTimeRX    = 0;                                                        // Når vi sist mottok en gyldig melding
const long communicationTimeoutRX        = 5000;                                                     // 5 sekunder timeout

unsigned long lastCommunicationTimeTX    = 0;                                                        // Når vi sist mottok en gyldig melding
const long communicationTimeoutTX        = 5000;                                                     // 5 sekunder timeout


/*---------------------- Kommunikasjon ----------------------*/
constexpr unsigned long BAUD_RATE_M    = 115600;                                                     // Baudrate for seriell monitor (debugging via USB)
constexpr unsigned long BAUD_RATE_C    = 9600;                                                       // Baudrate for UART-kommunikasjon med slave (S1)

/*---------------------- LED Matrix ----------------------*/
ArduinoLEDMatrix matrix;                                                                             // Objekt for LED-matrise
int br                        = BAUD_RATE_M;                                                         // Variabel brukt av grafikkbiblioteket
int dt                        = 100;                                                                 // Delay mellom oppdateringer (100 ms)
String unitId                 = "M1";                                                                // Enhets-ID som vises på matrisen

/*---------------------- Pinner ----------------------*/
// Buzzer
const int BUZZER_PIN           = 13;                                                                 // Digital utgang koblet til buzzer

// LED
const int PARKING_LED_PIN      = 9;                                                                  // Digital utgang for parkeringsmodus-indikator

// Knapper
const int MENU_BUTTON_PIN      = 12;                                                                 // Menyknapp (aktiv lav)
const int SELECT_BUTTON_PIN    = 11;                                                                 // Velg-knapp (aktiv lav)
const int PARKING_BUTTON_PIN   = 10;                                                                 // Parkeringsknapp (aktiv lav)
const int REVERS_BUTTON_PIN    = 8;                                                                  // Revers-signal (aktiv lav)

// LCD (4-bit-modus)
const int RS_PIN               = 7;                                                                  // Register Select pin for LCD
const int ENABLE_PIN           = 6;                                                                  // Enable pin for LCD
const int D4_PIN               = 5;                                                                  // Datapin 4 til LCD
const int D5_PIN               = 4;                                                                  // Datapin 5 til LCD
const int D6_PIN               = 3;                                                                  // Datapin 6 til LCD
const int D7_PIN               = 2;                                                                  // Datapin 7 til LCD

LiquidCrystal carLCD(RS_PIN, ENABLE_PIN, D4_PIN, D5_PIN, D6_PIN, D7_PIN);                            // Oppretter objekt for LCD-styring
bool parkingStartLcd          = false;                                                               // Flagg for å vise velkomstmelding bare én gang

/*---------------------- LCD Spesialtegn ----------------------*/
// Full blokk
byte fullBlock[8] = {
  B11111, B11111, B11111, B11111, B11111, B11111, B11111, B11111
};

// Venstre rammeblokk
byte leftFrameBlock[8] = {
  B11111, B10000, B10000, B10000, B10000, B10000, B10000, B11111
};

// Midtre rammeblokk
byte centreFrameBlock[8] = {
  B11111, B00000, B00000, B00000, B00000, B00000, B00000, B11111
};

// Høyre rammeblokk
byte rightFrameBlock[8] = {
  B11111, B00001, B00001, B00001, B00001, B00001, B00001, B11111
};

// Grade icon
byte degree[8] = {
  B01110, B01010, B01110, B00000, B00000, B00000, B00000, B00000
};



/*---------------------- Kommunikasjonsdata fra S1 ----------------------*/
String uart = "";                                                                                    // Midlertidig lagering for mottatt UART-data fra slave (linje for linje)

int rightDistanceInt = 0;                                                                            // Avrundet høyre avstand (for visning på LCD)
int rightDistance = 0;                                                                               // Rå (ikke avrundet) verdi fra høyre sensor
int centreRightDistanceInt = 0;                                                                      // Avrundet midtre høyre avstand
int centreRightDistance = 0;                                                                         // Rå midtre høyre avstand
int centreLeftDistanceInt = 0;                                                                       // Avrundet midtre venstre avstand
int centreLeftDistance = 0;                                                                          // Rå midtre venstre avstand
int leftDistanceInt = 0;                                                                             // Avrundet venstre avstand
int leftDistance = 0;                                                                                // Rå venstre avstand

int combinedRightDistance = 0;                                                                       // Kombinert høyre avstand (høyre + midtre høyre)
int combinedLeftDistance = 0;                                                                        // Kombinert venstre avstand (venstre + midtre venstre)

bool trailerStatus = false;                                                                          // Status på tilkoblet henger (true = henger koblet til)

/*---------------------------- PDC Status --------------------------*/
bool leftReady = false;                                                                              // True hvis venstre sensor er klar (god måling tilgjengelig)
bool leftReduced = false;                                                                            // True hvis venstre sensor er redusert (dårlig måling eller avstand begrenset)
bool rightReady = false;                                                                             // True hvis høyre sensor er klar (god måling tilgjengelig)
bool rightReduced = false;                                                                           // True hvis høyre sensor er redusert (dårlig måling eller avstand begrenset)
  
/*---------------------- Error Meny handtering ----------------------*/
bool errorMenuState = false;                                                                         // Om feilmeldingsmenyen er aktiv
int errorListPosition = 0;                                                                           // Nåværende posisjon i feillisten

int posisionE0 = 0;                                                                                  // Posisjon for feilkode E0 i menyen
int posisionE1 = 0;                                                                                  // Posisjon for feilkode E1 i menyen
int posisionE2 = 0;                                                                                  // Posisjon for feilkode E2 i menyen
int posisionE3 = 0;                                                                                  // Posisjon for feilkode E3 i menyen
int posisionE4 = 0;                                                                                  // Posisjon for feilkode E4 i menyen
int posisionE5 = 0;                                                                                  // Posisjon for feilkode E5 i menyen
int posisionE6 = 0;                                                                                  // Posisjon for feilkode E6 i menyen
int posisionE7 = 0;                                                                                  // Posisjon for feilkode E7 i menyen

/*---------------------- Error håndtering ----------------------*/
int activeErrors = 0;                                                                                // Antall aktive feil (for bruk i meny og visning)
String errorCodes = "";                                                                              // Samlet streng med aktive feilkoder ("E1,E2,E3,...")
String errorParkingCodes = "";

bool errorRight = false;                                                                             // Feilstatus for høyre sensor (true = feil oppdaget)
bool errorCentreRight = false;                                                                       // Feilstatus for midtre høyre sensor
bool errorLeft = false;                                                                              // Feilstatus for venstre sensor
bool errorCentreLeft = false;                                                                        // Feilstatus for midtre venstre sensor

bool errorCommunication = false;                                                                     // Feil på generell kommunikasjon (brukes hvis ønskelig)
bool errorCommunicationTX = false;                                                                   // Feil ved sending (TX) fra master til slave
bool errorCommunicationRX = false;                                                                   // Feil ved mottak (RX) fra slave til master

bool errorTemperature = false;                                                                       // Feil ved temperatur sensoren

/*---------------------- Tidsvisning ----------------------*/
String currentDisplayTime = "";                                                                      // Nåværende klokkeslett for visning (hh:mm)
String currentDisplayDay = "";                                                                       // Nåværende dag (dato) for visning
String currentDisplayMonth = "";                                                                     // Nåværende måned (forkortet navn)
String rtcSlave = "";                                                                                // Full tekststreng fra slave (hh:mm:ss dd/mm/yyyy)

/*---------------------- Temperatur visning ----------------------*/
int temperatureSlave = 0;                                                                            // Variabel som lagrer temperaturen fra slave-enheten (i °C)

/*---------------------- Parkeringslogikk ----------------------*/
bool parkingLcdState = false;                                                                        // Om parkeringssystemet er aktivt (på LCD)
bool parkingLedState = false;                                                                        // Om parkerings-LED er aktivert
bool lastParkingButtonState = false;                                                                 // For å oppdage trykk på parkeringsknappen
bool lastReversState = false;                                                                        // For å sjekke endringer i reversignal

/*---------------------- Menyknapp Logikk ----------------------*/
bool lastMenuButtonState = false;                                                                    // For å sjekke om menyknappen ble trykket
bool lastSelectButtonState = false;                                                                  // For å sjekke om velgeknappen ble trykket

/*---------------------- RTC ----------------------*/
RTCTime currentTime;                                                                                 // Variabel for å lagre sanntid fra RTC-modulen

/*---------------------- Månedsnavn-funksjon ----------------------*/
// Returnerer forkortet månedsnavn basert på tallverdi (1–12)
String getMonthShortName(int month) {
  switch (month) {
    case 1:  return "Jan";                                                                           // Januar
    case 2:  return "Feb";                                                                           // Februar
    case 3:  return "Mar";                                                                           // Mars
    case 4:  return "Apr";                                                                           // April
    case 5:  return "May";                                                                           // Mai
    case 6:  return "Jun";                                                                           // Juni
    case 7:  return "Jul";                                                                           // Juli
    case 8:  return "Aug";                                                                           // August
    case 9:  return "Sep";                                                                           // September
    case 10: return "Oct";                                                                           // Oktober
    case 11: return "Nov";                                                                           // November
    case 12: return "Dec";                                                                           // Desember
    default: return "   ";                                                                           // Returnerer blankt navn hvis månedstallet er ugyldig
  }
}


/*---------------------- setup() ----------------------
setup() kjøres én gang ved oppstart:
- Starter seriell kommunikasjon (USB og UART)
- Initialiserer LED-matrisen med enhets-ID
- Setter opp knappene og parkerings-LED
- Initialiserer LCD og lager spesialtegn
-------------------------------------------------------*/
void setup() {
  Serial.begin(BAUD_RATE_M);                                                                         // Starter seriell kommunikasjon med PC (via USB) for debugging
  Serial1.begin(BAUD_RATE_C);                                                                        // Starter UART-kommunikasjon med slaveenheten (S1)

  // ---------------------- Oppsett for LED Matrix ----------------------
  matrix.begin();                                                                                    // Initialiserer LED-matrisen (starter hardware-kommunikasjon)
  matrix.beginDraw();                                                                                // Starter en "tegningsfase" (for å kunne tegne grafikk/tekst)
  matrix.textFont(Font_5x7);                                                                         // Setter fonten til standard 5x7 piksler
  matrix.beginText(1, 1, 255, 0, 0);                                                                 // Starter tekstskriving på posisjon (x=1, y=1), farge-argumenter er irrelevante
  matrix.println(unitId);                                                                            // Skriver enhets-ID ("M1") til matrisen
  matrix.endText();                                                                                  // Avslutter tekstskriving
  matrix.endDraw();                                                                                  // Viser det som ble tegnet på skjermen

  // ---------------------- Oppsett for knappestyring ----------------------
  pinMode(PARKING_BUTTON_PIN, INPUT_PULLUP);                                                         // Konfigurerer parkeringsknapp som input med intern pull-up
  pinMode(REVERS_BUTTON_PIN, INPUT_PULLUP);                                                          // Konfigurerer reversbryter som input med intern pull-up
  pinMode(MENU_BUTTON_PIN, INPUT_PULLUP);                                                            // Konfigurerer menyknapp som input med intern pull-up
  pinMode(SELECT_BUTTON_PIN, INPUT_PULLUP);                                                          // Konfigurerer velgeknapp (Select) som input med intern pull-up

  // ---------------------- Oppsett for LED ----------------------
  pinMode(PARKING_LED_PIN, OUTPUT);                                                                  // Konfigurerer LED for parkeringsindikasjon som output

  // ---------------------- Oppsett for LCD ----------------------
  carLCD.begin(16, 2);                                                                               // Initialiserer LCD-skjermen med 16 kolonner og 2 rader
  carLCD.createChar(0, fullBlock);                                                                   // Lager spesialtegn: full blokk på posisjon 0 i LCD-minnet
  carLCD.createChar(1, leftFrameBlock);                                                              // Lager spesialtegn: venstre ramme på posisjon 1
  carLCD.createChar(2, centreFrameBlock);                                                            // Lager spesialtegn: midtre ramme på posisjon 2
  carLCD.createChar(3, rightFrameBlock);                                                             // Lager spesialtegn: høyre ramme på posisjon 3
  carLCD.createChar(4, degree);                                                                      // Lager spesialtegn: temperatur symbol på posisjon 4

}

/*---------------------- loop() ----------------------
loop() kjøres kontinuerlig etter at setup() er ferdig:
- Oppdaterer tid og visning
- Leser inn og håndterer brukerinput (knapper)
- Kommuniserer med slaveenheten (S1)
- Styrer buzzer for varsling
------------------------------------------------------*/
void loop() {
  communicationUART();                                                                               // Leser data fra slave (S1) og sender parkeringsstatus tilbake
  error();                                                                                           // Oppdaterer aktiv feilkodeliste basert på sensorstatus
  menuButton();                                                                                      // Leser status fra menyknappen (for å gå til/fra feilmeldingsmeny)
  selectButton();                                                                                    // Leser status fra velgeknappen (for å bytte aktiv feilvisning)

  if (errorMenuState == false) {                                                                     // Hvis ikke i feilmeldingsmeny
    lcdView();                                                                                       // Viser normalinformasjon på LCD-skjermen (klokke, avstand, osv.)
    parkingButton();                                                                                 // Leser status fra parkeringsknappen og bytter parkeringsmodus
    reversState();                                                                                   // Leser revers-signal og overstyrer parkeringsmodus automatisk
    rtcTime();                                                                                       // Henter ny tid og dato fra RTC og oppdaterer visning
    parkingBuzzer();                                                                                 // Styrer buzzer basert på nærhet til hinder (ved parkering)
  } 
  else {                                                                                             // Hvis i feilmeldingsmeny
    errorMenu();                                                                                     // Viser aktive feilkoder og beskrivelser på LCD-skjermen
  }
}


/*---------------------- communicationUART() ----------------------
communicationUART() håndterer all UART-kommunikasjon med slaveenheten (S1):
- Leser inn meldinger linje for linje (avsluttes med '\n')
- Detekterer feil i RX/TX-kommunikasjon basert på meldingsinnhold
- Sender testmelding tilbake til slave (TX TEST)
- Oppdaterer feilstater basert på timeout
- Skriver status til seriell monitor for debugging
-------------------------------------------------------------------*/
void communicationUART() {
  while (Serial1.available()) {                                                                      // Så lenge det er data tilgjengelig på UART
    char c = Serial1.read();                                                                         // Les ett tegn fra UART

    if (c == '\n') {                                                                                 // Når linjeskift oppdages (fullført melding)
      if (uart.indexOf("RX") != -1) {                                                                // Hvis meldingen inneholder "RX"
        lastCommunicationTimeRX = millis();                                                          // Oppdater siste tid for gyldig RX
        errorCommunicationRX = false;                                                                // Nullstill RX-feilstatus
      }
      if (uart.indexOf("RX GOOD") != -1) {                                                           // Hvis meldingen spesifikt sier "RX GOOD"
        lastCommunicationTimeTX = millis();                                                          // Oppdater siste tid for gyldig TX
        errorCommunicationTX = false;                                                                // Nullstill TX-feilstatus
      }

      rawCommData(uart);                                                                             // Send hele uart meldingen videre til rawCommData
      uart = "";                                                                                     // Nullstill uart minnet for neste melding
    } else {
      uart += c;                                                                                     // Hvis ikke linjeskift, legg til tegnet i uart minnet
    }
  }

  // ---------------------- Timeout overvåking ----------------------
  if (millis() - lastCommunicationTimeRX > communicationTimeoutRX) {                                 // Hvis ingen gyldig RX innen timeout
    errorCommunicationRX = true;                                                                     // Sett RX-feil aktiv
  }

  if (millis() - lastCommunicationTimeTX > communicationTimeoutTX) {                                 // Hvis ingen gyldig TX innen timeout
    errorCommunicationTX = true;                                                                     // Sett TX-feil aktiv
  }

  delay(10);                                                                                         // Kort pause for stabilitet på UART-forbindelsen

  // ---------------------- Send status tilbake til slave ----------------------
  Serial1.println("TX TEST");                                                                        // Sender testmelding til slave for å bekrefte egen TX

}

/*---------------------- rawCommData(String data) ----------------------
rawCommData() tolker rå meldinger fra slaveenheten (S1) sendt via UART:
- Parser sensorverdier basert på prefiks (R:, CR:, L:, CL:, T:, RTC: TEMP:)
- Konverterer tekstverdier til heltall og oppdaterer globale variabler
- Runder av verdier til nærmeste 10 cm for visning
- Håndterer sensorfeil basert på spesialverdi 600
------------------------------------------------------------------------*/
void rawCommData(String data) {
  int distanceSensor_R = data.indexOf("R:");                                                         // Finn posisjonen til "R:" (høyre sensorverdi)
  int distanceSensor_CR = data.indexOf("CR:");                                                       // Finn posisjonen til "CR:" (centre høyre sensorverdi)
  int distanceSensor_L = data.indexOf("L:");                                                         // Finn posisjonen til "L:" (venstre sensorverdi)
  int distanceSensor_CL = data.indexOf("CL:");                                                       // Finn posisjonen til "CL:" (centre venstre sensorverdi)
  int trailerSensor = data.indexOf("T:");                                                            // Finn posisjonen til "T:" (hengerstatusverdi)
  int rtcSlaveUnit = data.indexOf("RTC:");                                                           // Finn posisjonen til "RTC:" (RTC tidsstempel)
  int temp = data.indexOf("TEMP:");                                                                  // Finn posisjonen til "TEMP:" (Temperatur)

  Serial.print("Mottatt: ");                                                                         // Debug: skriv ut "Mottatt: "
  Serial.println(data);                                                                              // Debug: skriv ut hele mottatt melding fra slave



  String distanceSensor_RValue = data.substring(distanceSensor_R + 2, distanceSensor_CR);            // Ekstraher verdien mellom "R:" og "CR:"
  String distanceSensor_CRValue = data.substring(distanceSensor_CR + 3, distanceSensor_L);           // Ekstraher verdien mellom "CR:" og "L:"
  String distanceSensor_LValue = data.substring(distanceSensor_L + 2, distanceSensor_CL);            // Ekstraher verdien mellom "L:" og "CL:"
  String distanceSensor_CLValue = data.substring(distanceSensor_CL + 3, trailerSensor);              // Ekstraher verdien mellom "CL:" og "T:"
  String trailerSensorValue = data.substring(trailerSensor + 2, rtcSlaveUnit);                       // Ekstraher verdien mellom "T:" og "RTC:"
  String rtcSlaveUnitValue = data.substring(rtcSlaveUnit + 4, temp);                                 // Ekstraher verdien mellom "RTC:" og "TEMP:"
  String tempValue = data.substring(temp + 5);                                                       // Ekstraher verdien etter "TEMP:"

  rightDistance = distanceSensor_RValue.toInt();                                                     // Konverter høyre sensorverdi til heltall
  centreRightDistance = distanceSensor_CRValue.toInt();                                              // Konverter centre høyre sensorverdi til heltall
  leftDistance = distanceSensor_LValue.toInt();                                                      // Konverter venstre sensorverdi til heltall
  centreLeftDistance = distanceSensor_CLValue.toInt();                                               // Konverter centre venstre sensorverdi til heltall
  trailerStatus = trailerSensorValue.toInt();                                                        // Konverter hengerstatus til heltall
  rtcSlave = rtcSlaveUnitValue;                                                                      // Lagre RTC tidspunktet som streng
  temperatureSlave = tempValue.toInt();                                                              // Lagre TEMP tidspunktet som streng

  // ---------------------- Errorhåndtering ----------------------
  if (rightDistance == 600) errorRight = true;                                                       // Hvis høyre sensor returnerer 600 → aktiver feil
  if (rightDistance < 450 || rightDistance == 500) errorRight = false;                               // Hvis normal verdi igjen eller timeout status "500" → fjern feil

  if (centreRightDistance == 600) errorCentreRight = true;                                           // Hvis midtre høyre sensor returnerer 600 → aktiver feil
  if (centreRightDistance < 450 || centreRightDistance == 500) errorCentreRight = false;             // Hvis normal verdi igjen timeout status "500" → fjern feil

  if (leftDistance == 600) errorLeft = true;                                                         // Hvis venstre sensor returnerer 600 → aktiver feil
  if (leftDistance < 450 || leftDistance == 500) errorLeft = false;                                  // Hvis normal verdi igjen timeout status "500" → fjern feil

  if (centreLeftDistance == 600) errorCentreLeft = true;                                             // Hvis midtre venstre sensor returnerer 600 → aktiver feil
  if (centreLeftDistance < 450 || centreLeftDistance == 500) errorCentreLeft = false;                // Hvis normal verdi igjen timeout status "500" → fjern feil

  if (temperatureSlave == 600) errorTemperature = true;                                              // Hvis temperatur sensor returnerer 600 → aktiver feil
  if (temperatureSlave != 600) errorTemperature = false;                                             // Hvis normal verdi igjen timeout status "500" → fjern feil

  // ---------------------- Avrunding og begrensning ----------------------
  rightDistanceInt = (constrain((int)rightDistance, 2, 450) / 10) * 10;                              // Begrens og rund ned høyre sensorverdi
  centreRightDistanceInt = (constrain((int)centreRightDistance, 2, 450) / 10) * 10;                  // Begrens og rund ned centre høyre sensorverdi
  centreLeftDistanceInt = (constrain((int)centreLeftDistance, 2, 450) / 10) * 10;                    // Begrens og rund ned centre venstre sensorverdi
  leftDistanceInt = (constrain((int)leftDistance, 2, 450) / 10) * 10;                                // Begrens og rund ned venstre sensorverdi

}

/*---------------------- error() ----------------------
error() bygger opp en feilkodeliste basert på hvilke sensorer
og kommunikasjonslinjer som rapporterer feil:
- Oppdaterer 'errorCodes' (f.eks. "E0,E1,E2,E3,E4,E5,E6")
- Teller antall aktive feil ('activeErrors')
- Oppdaterer posisjon for hver feil i feilmenyen
- Håndterer "reduced" status for venstre/høyre hvis bare én sensor feiler
--------------------------------------------------------*/
void error() {
  errorCodes = "";                                                                                   // Nullstill strengen som samler alle feilkoder
  errorParkingCodes = "";                                                                            // Nullstill strengen som samler kun parkeringsrelaterte feilkoder
  activeErrors = 0;                                                                                  // Nullstill antall aktive feil

  // ---------------------- E0: Kommunikasjonsfeil ----------------------
  if (errorCommunication) {                                                                          // Hvis generell kommunikasjonsfeil oppdaget
    if (errorCodes > 0) errorCodes += ",";                                                           // Legg til komma hvis ikke første feil
    errorCodes += "E0";                                                                              // Legg til E0 i feilstrengen
    posisionE0 = activeErrors;                                                                       // Sett posisjonen i feilmeny
    activeErrors++;                                                                                  // Øk antall feil
  }

  // ---------------------- E1: Høyre sensorfeil ----------------------
  if (errorRight) {                                                                                  // Hvis høyre sensor rapporterer feil
    if (errorCodes > 0) {
      errorCodes += ","; 
      errorParkingCodes += ",";
    }
    errorCodes += "E1";                                                                              // Legg til E1 i generell feilstreng
    errorParkingCodes += "E1";                                                                       // Legg til E1 også i parkeringsfeilstreng
    posisionE1 = activeErrors;                                                                       // Registrer posisjon for feilmeny
    activeErrors++;
  }

  // ---------------------- E2: Midtre høyre sensorfeil ----------------------
  if (errorCentreRight) {                                                                            // Hvis midtre høyre sensor har feil
    if (errorCodes > 0) {
      errorCodes += ",";
      errorParkingCodes += ",";
    }
    errorCodes += "E2";
    errorParkingCodes += "E2";                                                                       
    posisionE2 = activeErrors;                                                                       // Registrer posisjon
    activeErrors++;
  }

  // ---------------------- E3: Venstre sensorfeil ----------------------
  if (errorLeft) {                                                                                   // Hvis venstre sensor feiler
    if (errorCodes > 0) {
      errorCodes += ",";
      errorParkingCodes += ",";
    }
    errorCodes += "E3";
    errorParkingCodes += "E3";
    posisionE3 = activeErrors;
    activeErrors++;
  }

  // ---------------------- E4: Midtre venstre sensorfeil ----------------------
  if (errorCentreLeft) {                                                                             // Hvis midtre venstre sensor feiler
    if (errorCodes > 0) {
      errorCodes += ",";
      errorParkingCodes += ",";
    }
    errorCodes += "E4";
    errorParkingCodes += "E4";
    posisionE4 = activeErrors;
    activeErrors++;
  }

  // ---------------------- E5: Kommunikasjon TX-feil ----------------------
  if (errorCommunicationTX) {                                                                       // Hvis feil ved sending (TX) oppdaget
    if (errorCodes > 0) errorCodes += ",";
    errorCodes += "E5";
    posisionE5 = activeErrors;
    activeErrors++;
  }

  // ---------------------- E6: Kommunikasjon RX-feil ----------------------
  if (errorCommunicationRX) {                                                                       // Hvis feil ved mottak (RX) oppdaget
    if (errorCodes > 0) errorCodes += ",";
    errorCodes += "E6";
    posisionE6 = activeErrors;
    activeErrors++;
  }

  // ---------------------- E7: Temperatur sensorfeil ----------------------
  if (errorTemperature) {                                                                           // Hvis temperatur sensor feiler
    if (errorCodes > 0) errorCodes += ",";
    errorCodes += "E7";
    posisionE6 = activeErrors;
    activeErrors++;
  }


  // ---------------------- Oppdater PDC Status (Høyre side) ----------------------
  if (errorRight == true && errorCentreRight == true) {                                              // Hvis begge høyresensorene er feil
    rightReady = false;                                                                              // Høyresiden ikke klar
  } else if (errorRight == true || errorCentreRight == true) {                                       // Hvis bare én av høyresensorene er feil
    rightReady = true;                                                                               // Høyresiden klar, men redusert
    rightReduced = true;
  } else {
    rightReady = true;                                                                               // Høyresiden er helt OK
    rightReduced = false;
  }

  // ---------------------- Oppdater PDC Status (Venstre side) ----------------------
  if (errorLeft == true && errorCentreLeft == true) {                                                // Hvis begge venstresensorene er feil
    leftReady = false;                                                                               // Venstresiden ikke klar
  } else if (errorLeft == true || errorCentreLeft == true) {                                         // Hvis bare én av venstresensorene er feil
    leftReady = true;                                                                                // Venstresiden klar, men redusert
    leftReduced = true;
  } else {
    leftReady = true;                                                                                // Venstresiden er helt OK
    leftReduced = false;
  }
}


/*---------------------- menuButton() ----------------------
menuButton() håndterer lesing av Menu-knappen:
- Brukes til å aktivere/deaktivere visning av feilmenyen
- Toggle-logikk: bytter mellom normalvisning og feilvisning
--------------------------------------------------------------*/
void menuButton() {
  bool currentMenuButtonState = !digitalRead(MENU_BUTTON_PIN);                                       // Leser knappens status (aktiv lav → true når trykket)

  if (currentMenuButtonState && !lastMenuButtonState) {                                              // Registrer kun trykk (fallende flanke)
    errorMenuState = !errorMenuState;                                                                // Bytt feilvisningsmodus: true → false eller false → true
  }

  lastMenuButtonState = currentMenuButtonState;                                                      // Husk nåværende knappestatus for å detektere neste trykk
}



/*---------------------- selectButton() ----------------------
selectButton() leser og tolker status fra Select-knappen:
- Brukes til å navigere mellom aktive feilkoder
- Inkrementerer 'errorListPosition' hver gang knappen trykkes
- Rullerer tilbake til start når siste feil er nådd
---------------------------------------------------------------*/
void selectButton() {
  bool currentSelectButtonState = !digitalRead(SELECT_BUTTON_PIN);                                   // Leser knappens status (aktiv lav → true når trykket)

  if (currentSelectButtonState && !lastSelectButtonState) {                                          // Kun registrer nytt trykk (fallende flankestyring)
    if (activeErrors > 1 && activeErrors <= 8) {                                                     // Hvis det er mer enn én aktiv feil
      errorListPosition++;                                                                           // Gå til neste feil i listen
      if (errorListPosition >= activeErrors) {                                                       // Hvis vi har passert siste feil
        errorListPosition = 0;                                                                       // Start på nytt (sirkulær navigasjon)
      }
    }
  }

  lastSelectButtonState = currentSelectButtonState;                                                  // Oppdater for å spore endringer til neste loop
}

/*---------------------- errorMenu() ----------------------
errorMenu() viser enkle feilmeldinger på LCD-skjermen:
- Tidsstyrt rullering av feil
- Viser antall aktive feil
- Viser feilkoder og beskrivelser basert på valgt feil
-----------------------------------------------------------*/
void errorMenu() {
  unsigned long currentMillis = millis();                                                            // Henter nåværende tid i millisekunder siden oppstart

  if (currentMillis - previousMillisErrorMenu >= intervalErrorMenu) {                                // Sjekk om det er på tide å oppdatere visningen
    previousMillisErrorMenu = currentMillis;                                                         // Lagre tidspunktet for siste oppdatering

    carLCD.clear();                                                                                  // Tøm LCD før ny visning
    carLCD.setCursor(0, 0);                                                                          // Sett markør på øverste linje
    carLCD.print("Error");                                                                           // Skriv "Error"
    carLCD.setCursor(6, 0);                                                                          // Flytt markøren til riktig sted for å vise antall feil
    carLCD.print("(" + String(activeErrors) + ")");                                                  // Vis antall aktive feil i parentes

    if (activeErrors == 0) {                                                                         // Hvis ingen feil er aktive
      carLCD.setCursor(0, 1);                                                                        // Sett markør på nederste linje
      carLCD.print("No Active Error");                                                               // Skriv "No Active Error"
      carLCD.setCursor(6, 0);                                                                        // Sett markør igjen for å vise (0)
      carLCD.print("(" + String(activeErrors) + ")");                                                
    }

    String errorLabel = "";                                                                          // Midlertidig lagring for feilkode (f.eks. "E1")
    String errorMessage = "";                                                                        // Midlertidig lagring for feilmelding (tekst)

    // ---------------------- Feilkodehåndtering ----------------------
    if (errorCommunication == true && errorListPosition == posisionE0) {                             // Hvis feilkode E0 (kommunikasjonsfeil) er valgt
      errorLabel = "E0";                                                                             // Sett feilkoden til "E0"
      errorMessage = "UART RX Fault";                                                                // Sett feilmeldingstekst til "UART RX Fault"
    } else if (errorRight == true && errorListPosition == posisionE1) {                              // Hvis feilkode E1 (høyre sensorfeil) er valgt
      errorLabel = "E1";                                                                             // Sett feilkoden til "E1"
      errorMessage = "Right Sensor";                                                                 // Sett feilmeldingstekst til "Right Sensor"
    } else if (errorCentreRight == true && errorListPosition == posisionE2) {                        // Hvis feilkode E2 (midtre høyre sensorfeil) er valgt
      errorLabel = "E2";                                                                             // Sett feilkoden til "E2"
      errorMessage = "Centre Right";                                                                 // Sett feilmeldingstekst til "Centre Right"
    } else if (errorLeft == true && errorListPosition == posisionE3) {                               // Hvis feilkode E3 (venstre sensorfeil) er valgt
      errorLabel = "E3";                                                                             // Sett feilkoden til "E3"
      errorMessage = "Left Sensor";                                                                  // Sett feilmeldingstekst til "Left Sensor"
    } else if (errorCentreLeft == true && errorListPosition == posisionE4) {                         // Hvis feilkode E4 (midtre venstre sensorfeil) er valgt
      errorLabel = "E4";                                                                             // Sett feilkoden til "E4"
      errorMessage = "Centre Left";                                                                  // Sett feilmeldingstekst til "Centre Left"
    } else if (errorCommunicationTX == true && errorListPosition == posisionE5) {                    // Hvis feilkode E5 (UART TX feil) er valgt
      errorLabel = "E5";                                                                             // Sett feilkoden til "E5"
      errorMessage = "UART TX Fail";                                                                 // Sett feilmeldingstekst til "UART TX Fail"
    } else if (errorCommunicationRX == true && errorListPosition == posisionE6) {                    // Hvis feilkode E6 (UART RX feil) er valgt
      errorLabel = "E6";                                                                             // Sett feilkoden til "E6"
      errorMessage = "UART RX Fail";                                                                 // Sett feilmeldingstekst til "UART RX Fail"
    } else if (errorTemperature == true && errorListPosition == posisionE7) {                        // Hvis feilkode E6 (temperatur feil) er valgt
      errorLabel = "E7";                                                                             // Sett feilkoden til "E7"
      errorMessage = "Temperature Fail";                                                             // Sett feilmeldingstekst til "Temperatur Fail"
    } 


    carLCD.setCursor(14, 0);                                                                         // Sett markør til høyre øverst
    carLCD.print(errorLabel);                                                                        // Skriv feilkoden (E1–E4)

    carLCD.setCursor(0, 1);                                                                          // Sett markør på nederste linje
    carLCD.print(errorMessage);                                                                      // Skriv feilmeldingen ("Right Sensor" osv.)
  }
}


/*---------------------- lcdView() ----------------------
lcdView() styrer hva som vises på LCD-skjermen:
- I parkeringsmodus: viser grafisk avstandsmåling på venstre og høyre side
- Utenfor parkeringsmodus: viser klokke og rullerende feilmeldinger
- Bruker millis() for å kontrollere hvor ofte feilmeldinger vises
---------------------------------------------------------*/
void lcdView() {
  unsigned long currentMillis = millis();                                                            // Henter nåværende systemtid i millisekunder

  if (errorCommunicationRX == true) {                                                                // Hvis kommunikasjon med slave feiler
    carLCD.clear();                                                                                  // Tøm LCD før ny visning
    carLCD.setCursor(0, 0);                                                                          // Sett markør på øverste linje
    carLCD.print("Comms Error");                                                                     // Skriv "Comms Error" (kommunikasjonsfeil)
    carLCD.setCursor(13, 0);                                                                         // Sett markør til kolonne 13
    carLCD.print("E0");                                                                              // Vis feilkode "E0" (kommunikasjonsfeil)

    carLCD.setCursor(0, 1);                                                                          // Flytt markøren til nederste linje
    carLCD.print("Check connection");                                                                // Vis tekst: "Check connection"

    delay(1500);                                                                                     // Hold meldingen i 1,5 sekunder

    carLCD.clear();                                                                                  // Tøm LCD igjen for neste melding
    carLCD.setCursor(0, 0);                                                                          // Sett markør på øverste linje
    carLCD.print("Comms Error");                                                                     // Skriv "Comms Error" på nytt
    carLCD.setCursor(13, 0);                                                                         // Sett markør til kolonne 13
    carLCD.print("E0");                                                                              // Vis feilkode "E0" igjen

    carLCD.setCursor(0, 1);                                                                          // Flytt markør til nederste linje
    carLCD.print("between Slave-");                                                                  // Vis tekst: "between Slave-"

    delay(1500);                                                                                     // Hold meldingen i 1,5 sekunder

    carLCD.clear();                                                                                  // Tøm LCD før siste del
    carLCD.setCursor(0, 0);                                                                          // Sett markør på øverste linje
    carLCD.print("Comms Error");                                                                     // Skriv "Comms Error" på nytt
    carLCD.setCursor(13, 0);                                                                         // Sett markør til kolonne 13
    carLCD.print("E0");                                                                              // Vis feilkode "E0" igjen

    carLCD.setCursor(0, 1);                                                                          // Flytt markør til nederste linje
    carLCD.print("and Master Unit");                                                                 // Vis tekst: "and Master Unit"

 delay(1500);                                                                                        // Hold feilmeldingen synlig i 1,5 sekunder
} 
else if (parkingLcdState == true && errorCommunicationRX == false && (rightReady == false || leftReady == false)) {  
    carLCD.clear();                                                                                  // Tøm LCD-skjermen
    carLCD.setCursor(0, 0);                                                                          // Sett markør på første linje
    carLCD.print("PDC Unavailable");                                                                 // Meld at parkeringssystemet ikke er klart
    carLCD.setCursor(0, 1);                                                                          // Sett markør på andre linje
    carLCD.print("Error " + errorParkingCodes);                                                      // Vis hvilke parkeringssensorer som feiler (E1–E4)
    delay(2000);                                                                                     // Hold meldingen i 2 sekunder

    parkingLcdState = false;                                                                         // Deaktiver parkeringsvisning
    parkingLedState = false;                                                                         // Deaktiver parkerings-LED
}
if (parkingLcdState == true && errorCommunicationRX == false && rightReady == true && leftReady == true) {  // Hvis parkeringssystemet er aktivt og alle sensorer er OK
    carLCD.clear();                                                                                  // Tøm skjermen

    if (!parkingStartLcd) {                                                                          // Hvis velkomstskjerm ikke allerede er vist
        carLCD.setCursor(0, 0);
        carLCD.print("PDC Enabled");                                                                 // Skriv "Parking Distance Control Enabled"

        if (rightReduced == true && leftReduced == true) {                                           // Hvis både høyre og venstre har redusert funksjon
            carLCD.setCursor(0, 1);
            carLCD.print("Both R/L Reduced");                                                        // Vis "Both Right/Left Reduced"
        } 
        else if (rightReduced == true) {                                                             // Hvis bare høyre side er redusert
            carLCD.setCursor(0, 1);
            carLCD.print("Right Reduced");                                                           // Vis "Right Reduced"
        } 
        else if (leftReduced == true) {                                                              // Hvis bare venstre side er redusert
            carLCD.setCursor(0, 1);
            carLCD.print("Left Reduced");                                                            // Vis "Left Reduced"
        }

        delay(1500);                                                                                 // Hold velkomstmeldingen i 1,5 sekunder
        parkingStartLcd = true;                                                                      // Sett flagg for at velkomst er vist
        carLCD.clear();                                                                              // Tøm skjermen etter velkomstvisning
    }
    // ------------------ VENSTRE SIDE ------------------
    
    combinedLeftDistance = min(leftDistanceInt, centreLeftDistanceInt);
    if (currentMillis - previousMillisLcd >= intervalLcd) {                                      // Kontrollert visningsoppdatering basert på intervall
      previousMillisLcd = currentMillis;     
    carLCD.setCursor(0, 0);                                                                          // Sett markør for venstresidegrafikk
    carLCD.write(byte(1));                                                                           // Venstre ramme
    for (int b = 1; b < 6; b++) {
      carLCD.setCursor(b, 0);
      carLCD.write(byte(2));                                                                         // Midtblokker
    }
    carLCD.setCursor(6, 0);
    carLCD.write(byte(3));                                                                           // Høyre ramme

    int numberOfV = (140 - combinedLeftDistance) / 20;                                               // Antall blokker basert på avstand
    for (int i = 0; i <= numberOfV && i < 7; i++) {
      carLCD.setCursor(i, 0);
      carLCD.write(byte(0));                                                                         // Tegn fylt blokk
    }

    carLCD.setCursor(0, 1);                                                                          // Gå til rad 1
    if (combinedLeftDistance <= 150) {                                                               // Innen 150 cm
      carLCD.print(combinedLeftDistance);                                                            // Vis avstand
      carLCD.setCursor(7, 1);
      carLCD.print("cm");                                                                            // Enhet
    } else {
      carLCD.print("Left");                                                                          // Lang avstand → vis tekst
    }

    // ------------------ HØYRE SIDE ------------------
    combinedRightDistance = min(rightDistanceInt, centreRightDistanceInt);

    carLCD.setCursor(9, 0);                                                                          // Start på kolonne 9
    carLCD.write(byte(1));                                                                           // Venstre ramme for høyreside
    for (int b = 1; b < 6; b++) {
      carLCD.setCursor(15 - b, 0);
      carLCD.write(byte(2));                                                                         // Midtblokker
    }
    carLCD.setCursor(15, 0);
    carLCD.write(byte(3));                                                                           // Høyre ramme

    int numberOfR = (140 - combinedRightDistance) / 20;                                              // Antall blokker basert på avstand
    for (int i = 0; i <= numberOfR && i < 7; i++) {
      carLCD.setCursor(15 - i, 0);
      carLCD.write(byte(0));
    }

    int rightCursor = (combinedRightDistance < 10) ? 15 : (combinedRightDistance < 100) ? 14 : 13;   // Dynamisk plassering basert på antall sifre
    carLCD.setCursor(rightCursor, 1);
    if (combinedRightDistance <= 150) {
      carLCD.print(combinedRightDistance);
      carLCD.setCursor(7, 1);
      carLCD.print("cm");
    } else {
      carLCD.setCursor(11, 1);
      carLCD.print("Right");
    }
    }

  } else if (parkingLcdState == false) {                                                             // === NORMALMODUS (IKKE PARKERING) ===
    if (currentMillis - previousMillisError >= intervalError) {                                      // Kontrollert visningsoppdatering basert på intervall
      previousMillisError = currentMillis;                                                           // Oppdater siste feilmeldingstid

      carLCD.clear();                                                                                // Tøm skjermen før ny visning

      carLCD.setCursor(0, 0);
      carLCD.print(currentDisplayTime);                                                              // Skriv klokkeslett (HH:MM)
      carLCD.setCursor(10, 0);
      carLCD.print(currentDisplayMonth);                                                             // Skriv måned (forkortet navn)
      carLCD.setCursor(14, 0);
      carLCD.print(currentDisplayDay);                                                               // Skriv dato (dag)

      if (activeErrors == 1 && !errorCommunicationRX) {                                              // Hvis nøyaktig én aktiv feil og kommunikasjon er OK
        carLCD.setCursor(0, 1);
        carLCD.print("Error");                                                                       // Skriv "Error" nederst
        carLCD.setCursor(13, 1);
        carLCD.print("(" + String(activeErrors) + ")");                                              // Vis antall feil i parentes
      } else if (activeErrors > 1) {                                                                 // Flere aktive feil
        carLCD.setCursor(0, 1);
        carLCD.print("Errors");                                                                      // Skriv "Errors"
        carLCD.setCursor(13, 1);
        carLCD.print("(" + String(activeErrors) + ")");                                              // Vis antall feil
      } else {                                                                                       // Ingen aktive feil
        carLCD.setCursor(0, 1);
        carLCD.print(String(temperatureSlave) + " ");                                                // Skriv temperaturverdi
        carLCD.write(byte(4));                                                                       // Tegn temperatur-ikon
        carLCD.print("C");                                                                           // Skriv "C" for Celsius
      }

      // --- Oppdater tid og dato igjen for å sikre korrekt skjermbilde ---
      carLCD.setCursor(0, 0);
      carLCD.print(currentDisplayTime);                                                              // Skriv klokkeslett (HH:MM)
      carLCD.setCursor(10, 0);
      carLCD.print(currentDisplayMonth);                                                             // Skriv måned (forkortet navn)
      carLCD.setCursor(14, 0);
      carLCD.print(currentDisplayDay);                                                               // Skriv dato (dag)
    }
  }

  if (errorCommunicationRX == false) {                                                               // Bare hvis kommunikasjon fungerer
    delay(200);                                                                                      // Kort pause for stabil skjermoppdatering
  }
}


/*---------------------- reversState() ----------------------
reversState() styrer automatisk aktivering/deaktivering av parkeringsmodus
basert på om bilen er i revers og om henger er tilkoblet:
- Aktiverer parkeringssystemet når revers aktiveres (hvis ingen henger)
- Deaktiverer det igjen når revers deaktiveres
-------------------------------------------------------------*/
void reversState() {
  bool isReversActive = !digitalRead(REVERS_BUTTON_PIN);                                             // Leser reversknappen: aktiv lav → true når trykket (INPUT_PULLUP)

  if (isReversActive && !lastReversState && !trailerStatus) {                                        // Hvis revers nå er aktiv og var inaktiv, og ingen henger er tilkoblet
    parkingLcdState = true;                                                                          // Aktiver LCD-visning for parkering
    parkingLedState = true;                                                                          // Aktiver LED-indikasjon for parkering
  }

  if (!isReversActive && lastReversState) {                                                          // Hvis revers nå er inaktiv, men var aktiv forrige syklus
    parkingLcdState = false;                                                                         // Deaktiver LCD-visning for parkering
    parkingLedState = false;                                                                         // Deaktiver LED for parkering
    parkingStartLcd = false;                                                                         // Nullstill velkomstmelding for parkering
  }

  digitalWrite(PARKING_LED_PIN, parkingLedState ? HIGH : LOW);                                       // Sett LED-pin til HIGH eller LOW basert på parkeringstilstand
  lastReversState = isReversActive;                                                                  // Lagre nåværende reversstatus for neste syklus
}

/*---------------------- parkingButton() ----------------------
parkingButton() leser og tolker status fra parkeringsknappen:
- Kan kun aktiveres manuelt når revers ikke er aktiv og ingen henger er koblet til
- Veksler parkeringsvisning og LED ved knappetrykk
---------------------------------------------------------------*/
void parkingButton() {
  bool currentParkingButtonState = !digitalRead(PARKING_BUTTON_PIN);                                 // Leser knappens status: aktiv lav → true når trykket

  if (!lastReversState && !trailerStatus) {                                                          // Bare når revers ikke er aktiv og ingen henger er tilkoblet
    if (currentParkingButtonState && !lastParkingButtonState) {                                      // Hvis knappen ble trykket ned (fallende flankestyring)
      parkingLcdState = !parkingLcdState;                                                            // Bytt LCD-visning: aktivér/deaktiver parkering
      parkingLedState = parkingLcdState;                                                             // Synkroniser LED med LCD-status

      if (parkingStartLcd == true && parkingLcdState == true) {                                      // Hvis parkering restartes, nullstill velkomstvisning
        parkingStartLcd = false;
      }

      digitalWrite(PARKING_LED_PIN, parkingLedState ? HIGH : LOW);                                   // Oppdater LED basert på ny parkeringsstatus
    }
  }

  lastParkingButtonState = currentParkingButtonState;                                                // Lagre knappens status til neste syklus
}


/*---------------------- rtcTime() ----------------------
rtcTime() parser og oppdaterer tid og dato basert på en
tekststreng mottatt via UART fra slave-enheten (S1):
- Strengen skal ha format: "HH:MM:SS DD/MM/YYYY"
- Oppdaterer visningsvariablene for klokkeslett og dato
- Bruker millis() for ikke-blokkerende intervallstyring
----------------------------------------------------------*/
void rtcTime() {
  unsigned long currentMillis = millis();                                                            // Henter gjeldende systemtid i millisekunder

  if (currentMillis - previousMillisRtc >= intervalRtc) {                                            // Utfør bare hvis nok tid har passert siden siste oppdatering
    previousMillisRtc = currentMillis;                                                               // Oppdater tidspunkt for siste utførte oppdatering

    if (rtcSlave.length() < 19) return;                                                              // Avslutt hvis strengen er for kort til å inneholde gyldig data

    // ---------------------- Parser tid og dato ----------------------
    int hourXX   = rtcSlave.substring(0, 2).toInt();                                                 // Henter timeverdi (HH) fra posisjon 0–1
    int minuteXX = rtcSlave.substring(3, 5).toInt();                                                 // Henter minuttverdi (MM) fra posisjon 3–4
    int secondXX = rtcSlave.substring(6, 8).toInt();                                                 // Henter sekundverdi (SS) fra posisjon 6–7
    int dayXX    = rtcSlave.substring(9, 11).toInt();                                                // Henter dagverdi (DD) fra posisjon 9–10
    int monthXX  = rtcSlave.substring(12, 14).toInt();                                               // Henter månedverdi (MM) fra posisjon 12–13
    int yearXX   = rtcSlave.substring(15, 19).toInt();                                               // Henter årsverdi (YYYY) fra posisjon 15–18

    // ---------------------- Formater klokke og dato ----------------------
    String hour   = (hourXX < 10 ? "0" : "") + String(hourXX);                                       // Legg til ledende null hvis time < 10
    String minute = (minuteXX < 10 ? "0" : "") + String(minuteXX);                                   // Legg til ledende null hvis minutt < 10

    currentDisplayTime = hour + ":" + minute;                                                        // Sett visningsstrengen for klokkeslett (HH:MM)
    currentDisplayDay   = String(dayXX);                                                             // Sett visningsstrengen for dag
    currentDisplayMonth = getMonthShortName(monthXX);                                                // Sett visningsstrengen for måned (forkortet navn)
  }
}


/*---------------------- parkingBuzzer() ----------------------
parkingBuzzer() aktiverer pipelyd basert på nærhet til hindringer:
- Sjekker både venstre og høyre sensorer
- Jo nærmere objektet er, jo hyppigere avgis pip
- Kun aktiv når parkeringsmodus (PDC) er aktivert
---------------------------------------------------------------*/
void parkingBuzzer() {
  unsigned long currentMillis = millis();                                                            // Henter nåværende systemtid i millisekunder

  if (parkingLcdState) {                                                                             // Kjør bare når parkeringsmodus er aktiv

    // ---------------------- FINN MINSTE AVSTAND ----------------------
    int minDistance = min(combinedLeftDistance, combinedRightDistance);                              // Velg den nærmeste avstanden (uansett side)

    // ---------------------- BUZZERLOGIKK ----------------------
    if (minDistance <= 50) {                                                                         // Kun når nærmeste objekt er innen 50 cm
      unsigned long intervalBuzzer = map(minDistance, 0, 50, 50, 1000);                              // Oversett avstand til pip-intervall (nær = raskere pip)

      if (currentMillis - previousMillisBuzzer >= intervalBuzzer) {                                  // Hvis tiden for nytt pip er nådd
        previousMillisBuzzer = currentMillis;                                                        // Oppdater for å spore neste pip

        tone(BUZZER_PIN, 2500);                                                                      // Start buzzer på 2500 Hz
        delay(50);                                                                                   // Spill pip i 50 ms
        noTone(BUZZER_PIN);                                                                          // Slå av buzzer etter pip
      }
    }
  }
}
