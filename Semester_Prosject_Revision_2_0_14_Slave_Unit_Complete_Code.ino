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
Aktiv Revisjon: 2.0.13
Revision Dato: 30. April 2025
Enhet ID: Slave Unit "S1"  

Kompanjongkode for masterenhet: Versjon 2.0.23

------------------- Version Endringer -------------------:

v2.0.13
- Endre avstandsformel slik at den bruker temperatur målt i beregning for mer korrect temperatur, med en if regle for handtering av feil ved sensor da bruker
  vi hastighet ved 20*C som er 0.0343 cm/micro sekunder. Hvis DHT returnere "nan" setter den verdien 600. 

v2.0.12
- Utvidet feilhåndtering av ultralydsensorer:
  - Bruker én felles Trigger/Echo-pin per sensor.  
  - Hvis `echo == 0` og **echoPin == LOW** (leses lav selv med pull-up), tolkes det som **brudd/frakoblet sensor** og feilkoden **600** sendes.  
  - Hvis `echo == 0` men **echoPin != LOW** (pull-up aktiv), tolkes det som **timeout/ingen retur** og feilkoden **500** sendes.  
  - Bruker eksterne **pull-up-motstander** mellom Trigger/Echo-signal og +5V for pålitelig feildeteksjon, uten behov for ekstra feilsignalpinne.

- Kommunikasjon forbedret:
  - Endret navn fra `buffer` til `uart` for klarere UART-mottakshåndtering.
  - Bedre timeout og tilstandshåndtering for kommunikasjon mellom S1 og M1.

- Forbedret målenøyaktighet: 
  - Økt antall ping-median målinger fra 5 til **10 per sensor** for bedre stabilitet.

- **Andre forbedringer:**  
  - Oppdatert `printRTC()` med bedre variabelnavn (fra `bufferTime` til `shortTime`).
  - Små forbedringer i kommentering og struktur i `setup()` og `loop()`.

v2.0.11
- Aktivert støtte for **temperaturmåling** via DHT11-sensor:
  - Lagt til funksjonen `readTemperature()` for å lese og lagre temperaturdata.
  - Initialisert DHT11-sensor i `setup()`.
  - Temperatur (`TEMP:`) sendes nå kontinuerlig over UART til hovedenheten (M1).

- Implementert **kommunikasjonsfeilovervåkning**:
  - Mottar og tolker "TX TEST"-meldinger fra hovedenheten.
  - Registrerer kommunikasjonstap etter **5 sekunder** uten mottak.
  - Sender tilstandsbeskjed `RX GOOD` eller `RX BAD` sammen med sensordata.

- Oppdatert `communicationUART()` for å inkludere temperatur og forbindelsesstatus i datastrømmen.
- Oppdatert kommentarer og variabelnavn for klarere funksjonsbeskrivelser.
- Endret feilkode til 500 for kompabilitet med SR05 som har 1-450 range, SR04 har 2-400. 

v2.0.10
- Implementert **feilkode (401)** for ultralydsensorer ved manglende retur (f.eks. hvis sensoren ikke mottar ekko eller er frakoblet). Dette gir bedre feilhåndtering og enklere diagnostikk i M1.
- Oppdatert funksjonen `distanceMeasurement()` til å sjekke returverdi fra `ping_median()` og logge en advarsel i seriell monitor ved feil.
- Debug-utskrift i UART-funksjonen (`communicationUART()`) viser nå også sensorfeil eksplisitt (401) for bedre oversikt under testing og drift.
- Ingen strukturelle endringer i logikkflyten, men forbedret robusthet i sensorhåndtering og feilsøkingsvennlighet.

v2.0.9 
- Utvidet avstandsmåling fra 2 til 4 ultralydsensorer**, nå inkludert "midtre venstre" (CL) og "midtre høyre" (CR), for å forbedre presisjonen og dekningen av sensorområdet bak bilen.
- Implementert biblioteket `NewPing.h` for ultralydsensorer for mer pålitelige og nøyaktige målinger med median-filtering (`ping_median()`).
- Endret RTC-pinnetilordning for å frigjøre og omdisponere pinner til nye ultralydsensorer.  
- Oppdatert UART-kommunikasjonsprotokollen med nye felter for å sende data fra de to nye sensorene, nå med prefiksene `CR:` og `CL:` for henholdsvis midtre høyre og midtre venstre sensor.
- Oppdatert debug-meldinger i UART-kommunikasjonen for å reflektere ny sensordata.
- Lagt til støtte for DHT temperatursensor via `DHT.h`-biblioteket for fremtidig temperaturmåling (funksjon ikke ferdigstilt).
- Fjernet ubrukt kode relatert til eldre målemetode med direkte trig- og echo-håndtering.

v2.0.8
- Lagt til RTC-støtte og sending av klokkeslett/dato til hovedenheten (M1)** via UART med prefiks `RTC:hh:mm:ss dd/mm/yyyy`.
- Oppdatert kommunikasjonssystemet** slik at både tid, avstander og hengerstatus sendes kontinuerlig til M1.
- Lagt til `printRTC()`-funksjon** som henter dato og klokkeslett fra RTC-modulen og formatterer det til en streng.
- Oppdatert `loop()`** til å inkludere `printRTC()` slik at tid blir oppdatert løpende.
- Refaktorert UART-data som sendes fra S1 til M1**, med `R:`, `L:`, `T:` og `RTC:` prefiks.
- Oppdatert initialisering av RTC** i `setup()` for å sikre at klokken starter og ikke er write-protected.
- Byttet ut `RTC.h` og `LiquidCrystal.h` med `RtcDS1302.h` og `ThreeWire.h`**, siden `LiquidCrystal` og `RTC.h` ikke ble brukt.
- Oppdatert versjonsinfo og kommentarblokk**, inkludert revisjonsdato og ny status for funksjoner.

v2.0.7
- Debuging av int trailerStatus = true dette forårsaket problemene. Dette er nå rettet opp i og jeg har fjerne debugging text på avstandsmålingen. 
- Har ikke løst buzzer problemet enda.

v2.0.6
- Lag til buzzer for piping ved 50 cm stigende lyd til 0 cm, men funger ikke helt debugging trengers her.
- Bug med restart av slave enhet hvor den sender ut T:0 alså trailerStatus true når knappen ikker er aktive trykked inn.

v2.0.5
- Lag til henger status signal og oppdatert kommunikasjonen for sending av status på henger.

v2.0.4
- Fjernet alle sketch fanen, settings.h og slo sammen hele koden i en sketch ".ino".

v2.0.3
- Lag en seperate kommunikasjons .ino for å ha bedre oversikt i koden.

v2.0.2
- Lag til Settings.h, slik at alle instilling kan endres fra en plass.

v2.0.1
- Endring i av koden ved bruk av moderne C++-best praktis på baud rate.

v1.0.0
- Første versjon

*/

/*---------------------- Biblioteker ----------------------
Legger til nødvendige biblioteker for grafikk, sensorer og RTC:
- ArduinoGraphics og Arduino_LED_Matrix for LED-matrisen
- NewPing for ultralydsensorer
- RtcDS1302 og ThreeWire for sanntidsklokke
- DHT inkludert for fremtidig temperaturmåling
-----------------------------------------------------------*/
#include "ArduinoGraphics.h"                                                                       // Bibliotek som gir grafiske funksjoner som tekst og former til LED Matrix
#include "Arduino_LED_Matrix.h"                                                                    // Bibliotek som brukes til å styre den innebygde LED-matrisen på Arduino Nano 33 BLE Sense
#include "RtcDS1302.h"                                                                             // Bibliotek for å hente tid og dato fra RTC-modulen (ikke brukt her, men tilgjengelig for fremtidig bruk)
#include "ThreeWire.h"                                                                             // Bibliotek for bruk med RTC DS1302
#include "NewPing.h"                                                                               // Bibliotek for bruk med trig/echo ultralydsensorer
#include "DHT.h"                                                                                   // Bibliotek for bruk med DHT-temperatursensor

/*---------------------- Kommunikasjon ----------------------*/
constexpr unsigned long BAUD_RATE_M = 115600;                                                      // Baudrate for seriell monitor (USB, til debugging)
constexpr unsigned long BAUD_RATE_C = 9600;                                                        // Baudrate for UART-kommunikasjon mellom hovedenhet (M1) og denne enheten (S1)

/*---------------------- LED Matrix ----------------------*/
ArduinoLEDMatrix matrix;                                                                           // Oppretter objekt for å vise tekst/grafikk på LED-matrisen
int br = BAUD_RATE_M;                                                                              // Baudrate-variabel brukt i grafikkbiblioteket (her satt lik seriell-monitor hastighet)
int dt = 100;                                                                                      // Delay-tid (ms) mellom visningsoppdateringer
String unitId = "S1";                                                                              // Enhets-ID som vises på LED-matrisen (brukes til visuell identifikasjon)

/*---------------------- Pinnekonfigurasjon ----------------------*/
// RTC-pinner
const int RST_PIN              = 4;                                                                // Reset-pin for RTC-modulen (DS1302)
const int DAT_PIN              = 5;                                                                // Data-pin for RTC
const int CLK_PIN              = 6;                                                                // Klokke-pin for RTC

// Temperatur-sensor
const int SIGNAL_DHT_PIN       = 3;                                                                // Digital inngang for temperaturføler

// Brytere og status
const int TRAILER_PIN          = 2;                                                                // Digital inngang for hengerdeteksjon (aktiv lav)

/*---------------------- Ultralydsensor Pinner ----------------------*/
const int rightTrigPin         = 13;                                                               // Trig/Echo pin for høyre sensor (HC-SR05)
const int rightEchoPin         = 13;                                                               // (samme pin brukt til både trigger og echo)

const int centreRightTrigPin   = 12;                                                               // Trig/Echo pin for midtre høyre sensor
const int centreRightEchoPin   = 12;                                                               // (samme pin for trigger og echo)

const int centreLeftTrigPin    = 11;                                                               // Trig/Echo pin for midtre venstre sensor
const int centreLeftEchoPin    = 11;                                                               // (samme pin for trigger og echo)

const int leftTrigPin          = 10;                                                               // Trig/Echo pin for venstre sensor
const int leftEchoPin          = 10;                                                               // (samme pin for trigger og echo)


/*---------------------- Ultralydoppsett ----------------------*/
const int maxDistance          = 450;                                                              // Maks måleavstand for sensorene (i cm)
int interaksjoner              = 5;                                                                // Antall målinger per sensor for å beregne gjennomsnitt

NewPing rightSensor(rightTrigPin, rightEchoPin, maxDistance);                                      // Høyre ultralydsensor
NewPing centreRightSensor(centreRightTrigPin, centreRightEchoPin, maxDistance);                    // Midtre høyre sensor
NewPing centreLeftSensor(centreLeftTrigPin, centreLeftEchoPin, maxDistance);                       // Midtre venstre sensor
NewPing leftSensor(leftTrigPin, leftEchoPin, maxDistance);                                         // Venstre sensor

/*---------------------- Globale variabler ----------------------*/
// Rå målinger (flyttall)
float rightDuration, rightDistance;                                                                // Tid og avstand fra høyre sensor
float centreRightDuration, centreRightDistance;                                                    // Tid og avstand fra midtre høyre sensor
float centreLeftDuration, centreLeftDistance;                                                      // Tid og avstand fra midtre venstre sensor
float leftDuration, leftDistance;                                                                  // Tid og avstand fra venstre sensor

// Avrundede avstander (0–400 cm) for enklere bruk på LCD
unsigned int rightDistanceInt        = 0;                                                          // Avrundet høyre avstand
unsigned int centreRightDistanceInt  = 0;                                                          // Avrundet midtre høyre avstand
unsigned int centreLeftDistanceInt   = 0;                                                          // Avrundet midtre venstre avstand
unsigned int leftDistanceInt         = 0;                                                          // Avrundet venstre avstand


// Kommunikasjon
String uart = "";                                                                                  // Midlertidig lagering for mottatt UART-data fra master (linje for linje)
bool errorCommunicationRX = false; 
unsigned long lastCommunicationTime    = 0;                                                        // Når vi sist mottok en gyldig melding
unsigned long communicationTimeout     = 5000;                                                     // 5 sekunder timeout

// Hengerstatus
bool trailerStateInt        = false;                                                               // true = henger tilkoblet
bool lastTrailerState       = false;                                                               // Brukt til å oppdage endring i hengerstatus

// Parkeringsstatus
bool parkingStatus          = false;                                                               // true = parkeringsmodus aktiv
bool lastParkingStatus      = false;                                                               // Brukt til å oppdage endring i status

// RTC
ThreeWire extRTC(DAT_PIN, CLK_PIN, RST_PIN);                                                       // Oppretter ThreeWire-grensesnitt for RTC
RtcDS1302<ThreeWire> Rtc(extRTC);                                                                  // Instans av RTC-objektet
String rtcTime              = "";                                                                  // Strengversjon av tid/dato fra RTC

/*---------------------- Temperaturmåling ----------------------*/
DHT tempSensor(SIGNAL_DHT_PIN, DHT11);                                                             // Initialiserer DHT11 på angitt pinne
float temperature = 0;                                                                             // Målt temperatur i grader Celsius


/*---------------------- setup() ----------------------
setup() kjøres én gang ved oppstart:
- Starter serielle grensesnitt (USB og UART)
- Initialiserer LED-matrisen og viser enhets-ID
- Setter opp I/O for hengerdeteksjon
- Initialiserer og klargjør RTC-modulen
-------------------------------------------------------*/
void setup() {
  Serial.begin(BAUD_RATE_M);                                                                       // Starter seriell kommunikasjon med PC via USB for debugging og logging
  Serial1.begin(BAUD_RATE_C);                                                                      // Starter UART-seriell kommunikasjon med hovedenheten (M1)

  // ---------------------- Oppsett for LED Matrix ----------------------
  matrix.begin();                                                                                  // Initialiserer LED-matrisen
  matrix.beginDraw();                                                                              // Starter en tegningsfase – nødvendig for å sende grafikk til matrisen
  matrix.textFont(Font_5x7);                                                                       // Velger en 5x7 font for tydelig visning på liten skjerm
  matrix.beginText(1, 1, 255, 0, 0);                                                               // Starter tekstskriving fra posisjon (1,1) med rød farge (uten effekt på monokrom LED)
  matrix.println(unitId);                                                                          // Skriver enhets-ID (f.eks. "S1") på LED-matrisen for visuell identifikasjon
  matrix.endText();                                                                                // Avslutter tekstskrivingen
  matrix.endDraw();                                                                                // Fullfører tegningsfasen og oppdaterer skjermen

  // ---------------------- Oppsett for innganger ----------------------
  pinMode(TRAILER_PIN, INPUT_PULLUP);                                                              // Setter opp inngang for hengerstatus-sensor (aktiv lav med intern pull-up)

  // ---------------------- Oppsett for RTC ----------------------
  Rtc.Begin();                                                                                     // Initialiserer RTC-modulen
  //Rtc.SetDateTime(RtcDateTime(__DATE__, __TIME__));                                              // Setter klokkeslett til tidspunkt for kompilering (kommentert ut)
  if (Rtc.GetIsWriteProtected()) Rtc.SetIsWriteProtected(false);                                   // Deaktiverer skrivebeskyttelse hvis aktiv
  if (!Rtc.GetIsRunning()) Rtc.SetIsRunning(true);                                                 // Starter klokken hvis den er stoppet

  tempSensor.begin();                                                                              // Initialiserer DHT11-temperatursensor
}


/*---------------------- loop() ----------------------
loop() kjøres kontinuerlig etter at setup() er ferdig:
- Håndterer all kommunikasjon, sensoravlesning og tilstandsstyring
- Kaller de sentrale funksjonene i riktig rekkefølge hver syklus
------------------------------------------------------*/
void loop() {
  communicationUART();                                                                             // Kommuniserer med Master-enheten (M1) via UART:
                                                                                                   // - Leser inn styringskommandoer
                                                                                                   // - Sender sensordata (avstand + trailerstatus)

  distanceMeasurement();                                                                           // Utfører måling med alle fire ultralydsensorer:
                                                                                                   // - Sender triggsignal
                                                                                                   // - Leser ekko
                                                                                                   // - Konverterer til avstand i centimeter

  trailerState();                                                                                  // Leser status fra tilhengerdeteksjon:
                                                                                                   // - 1 = henger tilkoblet
                                                                                                   // - 0 = ingen henger

  printRTC();                                                                                      // Leser sanntidsklokke (RTC):
                                                                                                   // - Formatterer tid og dato til streng
                                                                                                   // - Klargjort for sending over UART

  readTemperature();                                                                               // Leser temperatur fra DHT11-sensor:
                                                                                                   // - Brukes i systemets normalvisning
}

/*---------------------- communicationUART() ----------------------
communicationUART() håndterer all UART-kommunikasjon mellom S1 og hovedenheten (M1):
- Leser innkommende meldinger fra M1 ("TX TEST") for å overvåke forbindelsen
- Sender kontinuerlig avstandsmålingsdata, hengerstatus, temperatur og klokkeslett tilbake til M1
- Oppdaterer feilstater basert på kommunikasjonens kvalitet
------------------------------------------------------------------*/
void communicationUART() {

  // ---------------------- Mottar innkommende data fra M1 ----------------------
  while (Serial1.available()) {                                                                    // Så lenge data er tilgjengelig fra M1
    char c = Serial1.read();                                                                       // Les neste tegn fra UART

    if (c == '\n') {                                                                               // Når en hel melding er mottatt
      if (uart.indexOf("TX TEST") != -1) {                                                         // Sjekk om meldingen inneholder "TX TEST"
        lastCommunicationTime = millis();                                                          // Oppdater siste tid for vellykket kommunikasjon
        errorCommunicationRX = false;                                                              // Nullstill kommunikasjonsfeil
      }
      uart = "";                                                                                   // Tøm uart minnet etter behandling
    } else {
      uart += c;                                                                                   // Legg til tegnet i uart minne
    }
  }

  if (millis() - lastCommunicationTime > communicationTimeout) {                                   // Hvis det går for lang tid uten gyldig melding
    errorCommunicationRX = true;                                                                   // Sett kommunikasjonsfeil
  }

  delay(10);                                                                                       // Kort forsinkelse for å stabilisere UART

  // ---------------------- Sender måledata til hovedenheten ----------------------
  Serial1.print("R:");                                                                             // Høyre sensoravstand
  Serial1.print(rightDistanceInt);
  Serial1.print(" CR:");                                                                           // Midtre høyre sensoravstand
  Serial1.print(centreRightDistanceInt);
  Serial1.print(" L:");                                                                            // Venstre sensoravstand
  Serial1.print(leftDistanceInt);
  Serial1.print(" CL:");                                                                           // Midtre venstre sensoravstand
  Serial1.print(centreLeftDistanceInt);
  Serial1.print(" T:");                                                                            // Tilhengerstatus
  Serial1.print(trailerStateInt);
  Serial1.print(" RTC:");                                                                          // Tid og dato
  Serial1.print(rtcTime);
  Serial1.print(" TEMP:");                                                                         // Temperatur
  Serial1.print(temperature);

  if (errorCommunicationRX == false) {
    Serial1.println(" RX GOOD");                                                                   // Send "RX GOOD" hvis kommunikasjon er OK
  } else {
    Serial1.println(" RX BAD");                                                                    // Send "RX BAD" hvis feil
  }

  // ---------------------- Debug-utskrift til seriellmonitor ----------------------
  Serial.print("Sendt: R:");
  Serial.print(rightDistanceInt);
  Serial.print(" CR:");
  Serial.print(centreRightDistanceInt);
  Serial.print(" L:");
  Serial.print(leftDistanceInt);
  Serial.print(" CL:");
  Serial.print(centreLeftDistanceInt);
  Serial.print(" T:");
  Serial.print(trailerStateInt);
  Serial.print(" RTC:");
  Serial.print(rtcTime);
  Serial.print(" TEMP:");
  Serial.println(temperature);
}

/*---------------------- distanceMeasurement() ----------------------
distanceMeasurement() måler avstand fra fire ultralydsensorer:
- Bruker NewPing for presise målinger (median av flere)
- Tar hensyn til temperatur for å justere lydens hastighet
- Setter feilkode 600 hvis sensor er fysisk frakoblet (ingen respons og lavt pin-signal)
- Setter feilkode 500 hvis det ikke kommer noen retur (timeout)
----------------------------------------------------------------------*/
void distanceMeasurement() {
  float speedOfSound = ((331.4 + 0.6 * temperature) * 100.0 / 1000000.0);                           // Beregner lydens hastighet i cm/us basert på aktuell temperatur

  // ---------------------- Høyre sensor ----------------------
  rightDuration = rightSensor.ping_median(interaksjoner);                                           // Utfører måling med høyre sensor og lagrer ekko-tiden
  if (rightDuration == 0) {                                                                         // Hvis ingen retur fra objekt
    if (digitalRead(rightEchoPin) == LOW) {                                                         // Hvis ECHO-pinnen er lav – sensor anses som fysisk frakoblet
      rightDistanceInt = 600;                                                                       // Feilkode 600 → sensor mangler
    } else {
      rightDistanceInt = 500;                                                                       // Feilkode 500 → ingen retur fra objekt
    }
  } else if (temperature != 600) {                                                                  // Hvis temperaturmåling er gyldig
    rightDistance = (rightDuration * speedOfSound) / 2;                                             // Beregner distanse med temperaturjustert lydhastighet
    rightDistanceInt = constrain((int)rightDistance, 2, 450);                                       // Avstanden begrenses til mellom 2 og 400 cm
  } else {                                                                                          // Hvis temperaturmåling feiler
    rightDistance = (rightDuration * 0.0343) / 2;                                                   // Bruker standard lydhastighet ved 20*C (343 m/s) uten temperaturjustering
    rightDistanceInt = constrain((int)rightDistance, 2, 450);                                       // Begrenset til gyldig intervall
  }

  delay(10);                                                                                        // Kort pause mellom målinger for stabilitet

  // ---------------------- Midtre høyre sensor ----------------------
  centreRightDuration = centreRightSensor.ping_median(interaksjoner);                               // Måling fra midtre høyre sensor
  if (centreRightDuration == 0) {
    if (digitalRead(centreRightEchoPin) == LOW) {
      centreRightDistanceInt = 600;                                                                 // Sensor frakoblet
    } else {
      centreRightDistanceInt = 500;                                                                 // Ingen retur
    }
  } else if (temperature != 600) {
    centreRightDistance = (centreRightDuration * speedOfSound) / 2;                                 // Bruk temperaturbasert lydfart
    centreRightDistanceInt = constrain((int)centreRightDistance, 2, 450);                           // Begrens avstand
  } else {
    centreRightDistance = (centreRightDuration * 0.0343) / 2;                                       // Bruker standard lydhastighet ved 20*C (343 m/s) uten temperaturjustering
    centreRightDistanceInt = constrain((int)centreRightDistance, 2, 450);                           // Begrens til gyldig intervall
  }

  delay(10);                                                                                        // Pause før neste sensor

  // ---------------------- Midtre venstre sensor ----------------------
  centreLeftDuration = centreLeftSensor.ping_median(interaksjoner);                                 // Måling fra midtre venstre sensor
  if (centreLeftDuration == 0) {
    if (digitalRead(centreLeftEchoPin) == LOW) {
      centreLeftDistanceInt = 600;                                                                  // Feil: sensor frakoblet
    } else {
      centreLeftDistanceInt = 500;                                                                  // Feil: ingen retur
    }
  } else if (temperature != 600) {
    centreLeftDistance = (centreLeftDuration * speedOfSound) / 2;                                   // Bruk temperaturbasert lydfart
    centreLeftDistanceInt = constrain((int)centreLeftDistance, 2, 450);                             // Begrens verdien til 2–400 cm
  } else {
    centreLeftDistance = (centreLeftDuration * 0.0343) / 2;                                         // Bruker standard lydhastighet ved 20*C (343 m/s) uten temperaturjustering
    centreLeftDistanceInt = constrain((int)centreLeftDistance, 2, 450);                             // Begrens verdien
  }

  delay(10);                                                                                        // Pause før siste sensor

  // ---------------------- Venstre sensor ----------------------
  leftDuration = leftSensor.ping_median(interaksjoner);                                             // Måling fra venstre sensor
  if (leftDuration == 0) {
    if (digitalRead(leftEchoPin) == LOW) {
      leftDistanceInt = 600;                                                                        // Feil: sensor borte
    } else {
      leftDistanceInt = 500;                                                                        // Feil: ingen retur
    }
  } else if (temperature != 600) {
    leftDistance = (leftDuration * speedOfSound) / 2;                                               // Bruk temperaturjustert fart
    leftDistanceInt = constrain((int)leftDistance, 2, 450);                                         // Begrens avstand
  } else {
    leftDistance = (leftDuration * 0.0343) / 2;                                                     // Bruker standard lydhastighet ved 20*C (343 m/s) uten temperaturjustering
    leftDistanceInt = constrain((int)leftDistance, 2, 450);                                         // Begrens verdien
  }

  delay(10);                                                                                        // Siste pause før funksjonen avsluttes
}


/*---------------------- trailerState() ----------------------
trailerState() registrerer endringer i hengerstatus og oppdaterer statusverdi for sending:
- Leser inn digitalt signal fra hengerdetektor (aktiv lav)
- Oppdaterer statusvariabel kun når det skjer en faktisk endring
- Statusen lagres for neste sammenligning
--------------------------------------------------------------*/
void trailerState() {
  bool isTrailerActive = !digitalRead(TRAILER_PIN);                                                 // Leser inn signal fra hengerdeteksjon – aktiv lav (true når henger er tilkoblet)

  if (isTrailerActive && !lastTrailerState) {                                                       // Hvis henger nå er tilkoblet og var frakoblet forrige syklus
    trailerStateInt = true;                                                                         // Oppdater statusvariabelen til true (henger tilkoblet)
  }

  if (!isTrailerActive && lastTrailerState) {                                                       // Hvis henger nå er frakoblet og var tilkoblet forrige syklus
    trailerStateInt = false;                                                                        // Oppdater statusvariabelen til false (henger frakoblet)
  }

  lastTrailerState     = isTrailerActive;                                                           // Lagrer nåværende status for sammenligning i neste syklus
}


/*---------------------- printRTC() ----------------------
printRTC() leser gjeldende tid og dato fra RTC-modulen og lagrer det som streng:
- Henter tidspunktet fra RTC som RtcDateTime-objekt
- Formatterer tiden som "HH:MM:SS DD/MM/YYYY"
- Lagres som String i rtcTime for videre bruk eller visning
----------------------------------------------------------*/
void printRTC() {
  RtcDateTime now = Rtc.GetDateTime();                                                              // Leser tid og dato fra RTC
  char shortTime[20];                                                                               // Midlertidig buffer for formatert tekst

  snprintf(shortTime, sizeof(shortTime), "%02u:%02u:%02u %02u/%02u/%04u",                           // Formaterer som "HH:MM:SS DD/MM/YYYY"
           now.Hour(), now.Minute(), now.Second(),                                                  // Henter time, minutt, sekund
           now.Day(), now.Month(), now.Year());                                                     // Henter dag, måned, år

  rtcTime = String(shortTime);                                                                      // Konverterer til String og lagrer i global variabel
}

/*---------------------- readTemperature() ----------------------
readTemperature() henter temperatur fra sensoren og lagrer som float:
- Leser temperatur via bibliotek (tempSensor)
- Ved ugyldig verdi lagres feilverdi 600
- Brukes for både visning og kommunikasjon
---------------------------------------------------------------*/
void readTemperature() {
  float temperatureValue = tempSensor.readTemperature();                                            // Leser temperatur fra sensor i °C

  if (isnan(temperatureValue)) {                                                                    // Hvis verdien er ugyldig (NaN)
    temperature = 600;                                                                              // Lagre feilverdi for temperatur (600)
  } else {
    temperature = temperatureValue;                                                                 // Ellers: lagre den gyldige målingen
  }
}
