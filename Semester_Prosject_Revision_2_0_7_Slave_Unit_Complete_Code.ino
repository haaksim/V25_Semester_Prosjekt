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
Aktiv Revisjon: 2.0.7
Revision Dato: 18. April 2025

------------------- Version Endringer -------------------:

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

// Legger til de ulike bibliotekene vi trenger for at kode skal virke
#include "ArduinoGraphics.h"                                                                        // Bibliotek som gir grafiske funksjoner som tekst og former til LED Matrix
#include "Arduino_LED_Matrix.h"                                                                     // Bibliotek som brukes til å styre den innebygde LED-matrisen på Arduino Nano 33 BLE Sense
#include <LiquidCrystal.h>                                                                          // Bibliotek for å styre et 16x2 LCD-display via 4-bit datalinjer (ikke i bruk på S1, men kan være nyttig ved fremtidig utvidelse)
#include "RTC.h"                                                                                    // Bibliotek for å hente tid og dato fra RTC-modulen (ikke brukt her, men tilgjengelig for fremtidig bruk)

// Sette opp seriell & UART kommunikasjon
constexpr unsigned long BAUD_RATE_M = 115600;                                                       // Baudrate for kommunikasjon med seriell monitor (for debugging)
constexpr unsigned long BAUD_RATE_C = 9600;                                                         // Baudrate for UART-kommunikasjon mellom hovedenhet og slave

// Lager objekt for LED-matrisen
ArduinoLEDMatrix matrix;                                                                            // Oppretter et objekt for å kunne vise tekst eller grafikk på LED-matrisen

// Innstillinger for bruk med ArduinoGraphics biblioteket
int br = BAUD_RATE_M;                                                                               // Baudrate-variabel brukt i grafikkbiblioteket (her satt lik seriell-monitor hastighet)
int dt = 100;                                                                                       // Delay-tid (100 ms) mellom visningsoppdateringer på LED-matrisen

// Navn som skal vises på LED Matrixen for å lettere identifisere Arduino-enheten visuelt
String unitId = "S1";                                                                               // Enhets-ID (f.eks. "S1" for Slave) som kan vises på LED-matrisen

// Analog-pinner
// (Ingen brukt i denne enheten foreløpig, men seksjonen er forberedt for evt. analoge sensorer)

// Temperature-pinner (digital input)
const int SIGNAL_THD_PIN = 8;                                                                       // Digital inngang for temperaturføler (f.eks. DHT11 eller THD-sensor) – ikke implementert ennå

// Buzzer-pinner (digital output)
const int BUZZER_PIN = 7;                                                                           // Digital utgang koblet til buzzer – for varsling med lyd (TO DO)

// Bryter-pinner (digital input)
const int TRAILER_PIN = 6;                                                                          // Digital inngang koblet til bryter eller signal for hengerdeteksjon

// Definer pinneoppsett for ultralyd sensorer
const int rightTrigPin  = 5;                                                                        // Trig-pin for høyre HC-SR04 sensor (sender ultralydpuls)
const int rightEchoPin  = 4;                                                                        // Echo-pin for høyre HC-SR04 sensor (mottar reflektert puls)
const int leftTrigPin = 3;                                                                          // Trig-pin for venstre HC-SR04 sensor (sender ultralydpuls)
const int leftEchoPin = 2;                                                                          // Echo-pin for venstre HC-SR04 sensor (mottar reflektert puls)

// Globale variabler for avstand
float rightDuration, rightDistance;                                                                 // Variabler for tid og beregnet avstand (høyre sensor)
float leftDuration, leftDistance;                                                                   // Variabler for tid og beregnet avstand (venstre sensor)

uint16_t rightDistanceInt = 0;                                                                      // Høyre avstand, avrundet og begrenset til 0–400 cm
uint16_t leftDistanceInt  = 0;                                                                      // Venstre avstand, avrundet og begrenset til 0–400 cm

// Kommunikasjon UART fra Master
String buffer = "";                                                                                 // Midlertidig tekstbuffer for mottatt data fra UART (f.eks. "P:1")

// Gobale variable for henger status
int trailerStateInt = false;                                                                        // Integer som representerer hengerstatus true = tilkoblet, false = ikke tilkoblet
bool lastTrailerState = false;                                                                      // Brukes til å oppdage endringer i hengerstatus

// Brytervariabler fra Master enhet
bool parkingStatus = false;                                                                         // Lagrer om parkeringsmodus er aktivert (mottas via UART fra M1)
bool lastParkingStatus = false;                                                                     // Brukes til å oppdage endringer i parkeringsstatus

// Setup kjøres én gang ved oppstart
void setup() {
  Serial.begin(BAUD_RATE_M);                                                                        // Starter seriell kommunikasjon med PC via USB for debugging og logging
  Serial1.begin(BAUD_RATE_C);                                                                       // Starter UART-seriell kommunikasjon med hovedenheten (M1)

  // ---------------------- Oppsett for LED Matrix ----------------------
  matrix.begin();                                                                                   // Initialiserer LED-matrisen
  matrix.beginDraw();                                                                               // Starter en tegningsfase – nødvendig for å sende grafikk til matrisen
  matrix.textFont(Font_5x7);                                                                        // Velger en 5x7 font for tydelig visning på liten skjerm
  matrix.beginText(1, 1, 255, 0, 0);                                                                // Starter tekstskriving fra posisjon (1,1) med rød farge (uten effekt på monokrom LED)
  matrix.println(unitId);                                                                           // Skriver enhets-ID (f.eks. "S1") på LED-matrisen for visuell identifikasjon
  matrix.endText();                                                                                 // Avslutter tekstskrivingen
  matrix.endDraw();                                                                                 // Fullfører tegningsfasen og oppdaterer skjermen

  // ---------------------- Oppsett for innganger ----------------------
  pinMode(TRAILER_PIN, INPUT_PULLUP);                                                               // Setter opp inngang for hengerstatus-sensor (aktiv lav med intern pull-up)

  // ---------------------- Oppsett for ultralydsensorer ----------------------
  pinMode(rightTrigPin, OUTPUT);                                                                    // Trig-pin for høyre sensor – må settes som utgang for å sende signal
  pinMode(rightEchoPin, INPUT);                                                                     // Echo-pin for høyre sensor – må settes som inngang for å motta signal
  pinMode(leftTrigPin, OUTPUT);                                                                     // Trig-pin for venstre sensor – utgang
  pinMode(leftEchoPin, INPUT);                                                                      // Echo-pin for venstre sensor – inngang
}

// loop() kjøres kontinuerlig etter at setup() er ferdig
void loop() {
  communicationUART();                                                                              // Utfører seriell kommunikasjon via UART med hovedenheten (M1):
                                                                                                    // - Sender avstandene (venstre og høyre) og hengerstatus
                                                                                                    // - Mottar eventuell styringsinformasjon (f.eks. parkeringsstatus)

  distanceMeasurement();                                                                            // Utfører målinger med begge HC-SR04 ultralydsensorene:
                                                                                                    // - Triggere sensorer
                                                                                                    // - Leser tilbake ekko-tid
                                                                                                    // - Konverterer til avstand i cm

  trailerState();                                                                                   // Leser status fra henger-sensoren:
                                                                                                    // - Oppdaterer trailerStateInt til 1 hvis henger er tilkoblet
                                                                                                    // - Setter til 0 hvis henger er frakoblet
                                                                                                    // - Brukes til å deaktivere systemet automatisk
  
  parkingBuzzer();                                                                                  // Styrer buzzerlyd basert på nærhet og aktiv parkeringsmodus
}

// communicationUART() håndterer all UART-kommunikasjon mellom S1 og hovedenheten (M1)
void communicationUART() {

  // Lese innkommende data fra M1 via UART
  while (Serial1.available()) {                                                                     // Sjekker om det er tilgjengelige tegn i mottaksbufferen fra UART
    char c = Serial1.read();                                                                        // Leser ett og ett tegn fra UART (f.eks. "P:1\n") som kommer fra hovedenheten

    if (c == '\n') {                                                                                // Når et linjeskift er mottatt, anses meldingen som komplett og klar for behandling
      rawCommData(buffer);                                                                          // Sender den komplette meldingen til rawCommData() for å tolke f.eks. parkeringsstatus
      buffer = "";                                                                                  // Nullstiller bufferen slik at ny melding kan bygges fra bunnen av
    } else {
      buffer += c;                                                                                  // Legger mottatt tegn til bufferen for å bygge hele meldingen over tid
      Serial.print("Mottatt: ");                                                                     // Debug: skriver ut nåværende innhold i bufferen til seriellmonitor
      Serial.println(buffer);                                                                       // Fullfører debug-linjen med linjeskift
    }
  }

  delay(10);                                                                                        // Liten forsinkelse for å sikre stabil overføring og behandling av data

  // Sender oppdatert data til M1 i formatet: "R:<høyre> L:<venstre> T:<hengerstatus>"
  Serial1.print("R:");                                                                              // Starter meldingen med høyre avstandsinformasjon (prefiks R:)
  Serial1.print(rightDistanceInt);                                                                  // Sender høyre avstand målt i centimeter (avrundet til heltall)

  Serial1.print(" L:");                                                                             // Legger til mellomrom og L: som prefiks for venstre avstand
  Serial1.print(leftDistanceInt);                                                                   // Sender venstre avstand målt i centimeter

  Serial1.print(" T:");                                                                             // Legger til mellomrom og T: som prefiks for hengerstatus
  Serial1.println(trailerStateInt);                                                                 // Sender hengerstatus (1 = tilkoblet, 0 = ikke tilkoblet), etterfulgt av linjeskift

  // Debug-utskrift til seriellmonitor for visuell kontroll under utvikling
  Serial.print("Sendt: R:");                                                                        // Starter debug-linje med R: og høyre avstand
  Serial.print(rightDistanceInt);                                                                   // Viser høyre avstand
  Serial.print(" L:");                                                                              // Fortsetter med L:
  Serial.print(leftDistanceInt);                                                                    // Viser venstre avstand
  Serial.print(" T:");                                                                              // Fortsetter med T:
  Serial.println(trailerStateInt);                                                                  // Viser trailer status og avslutter debug-linjen
}


// rawCommData() tolker mottatt UART-data fra M1 og oppdaterer statusvariabler
void rawCommData(String data) {                                                                     // Mottar hele meldingen som en streng (eks: "P:1") fra buffer
  int parkingStatusIndex = data.indexOf("P:");                                                      // Søker etter startposisjonen til "P:" i strengen (som angir parkeringsstatus)

  if (parkingStatusIndex != -1) {                                                                   // Hvis "P:" faktisk finnes i meldingen
    String parkingStatusVal = data.substring(parkingStatusIndex + 2);                               // Henter verdien etter "P:" – hopper over de to første tegnene
    parkingStatusVal.trim();                                                                        // Fjerner mellomrom eller linjeskift fra verdien (for å sikre korrekt parsing)
    parkingStatus = parkingStatusVal.toInt();                                                       // Konverterer verdien til heltall (0 eller 1) og lagrer som parkeringsstatus
  } else {
    Serial.println("Feil i datamottak, sjekk kommunikasjonen");                                     // Feilmelding i Serial Monitor hvis "P:" ikke ble funnet i strengen
  }
}

// distanceMeasurement() måler avstand fra begge ultralydsensorer og lagrer resultatene
void distanceMeasurement() {
  // --- Mål HØYRE ---
  digitalWrite(rightTrigPin, LOW);                                                                  // Setter trig-pinnen lav i minst 2 µs for å nullstille sensoren før ny måling
  delayMicroseconds(2);                                                                             // Kort forsinkelse (2 mikrosekunder)
  digitalWrite(rightTrigPin, HIGH);                                                                 // Setter trig høy i 10 µs for å sende ultralydpuls
  delayMicroseconds(10);                                                                            // Holder signalet høy i 10 mikrosekunder
  digitalWrite(rightTrigPin, LOW);                                                                  // Setter trig lav igjen for å avslutte puls

  rightDuration = pulseIn(rightEchoPin, HIGH);                                                      // Måler hvor lang tid det tar før ekko returnerer (tiden mellom utsendt og mottatt puls)
  rightDistance = (rightDuration * 0.0343) / 2;                                                     // Konverterer tiden til avstand i centimeter (lydens hastighet: 343 m/s)
  rightDistanceInt = constrain((int)rightDistance, 0, 400);                                         // Avrunder og begrenser avstanden til 0–400 cm for å unngå ekstremverdier

  delay(50);                                                                                        // Liten pause før neste måling for å sikre stabil drift

  // --- Mål VENSTRE ---
  digitalWrite(leftTrigPin, LOW);                                                                   // Nullstiller venstre sensor med lavt signal
  delayMicroseconds(2);                                                                             // Kort forsinkelse
  digitalWrite(leftTrigPin, HIGH);                                                                  // Sender ultralydpuls fra venstre sensor
  delayMicroseconds(10);                                                                            // Holder signalet høy i 10 mikrosekunder
  digitalWrite(leftTrigPin, LOW);                                                                   // Slår av trig-signalet

  leftDuration = pulseIn(leftEchoPin, HIGH);                                                        // Måler tiden det tar før ekko returneres
  leftDistance = (leftDuration * 0.0343) / 2;                                                       // Regner om tid til avstand i cm
  leftDistanceInt = constrain((int)leftDistance, 0, 400);                                           // Begrens verdien til 0–400 cm for stabilitet

  delay(50);                                                                                        // Liten pause før neste målesyklus for å unngå forstyrrelser mellom målingene
}

// trailerState() registrerer endringer i hengerstatus og oppdaterer statusverdi for sending
void trailerState() {
  bool isTrailerActive = !digitalRead(TRAILER_PIN);                                                 // Leser inn signal fra hengerdeteksjon – aktiv lav (true når henger er tilkoblet)

  if (isTrailerActive && !lastTrailerState) {                                                       // Hvis henger nå er tilkoblet og var frakoblet forrige syklus
    trailerStateInt = true;                                                                         // Oppdater statusvariabelen til true (henger tilkoblet)
  }

  if (!isTrailerActive && lastTrailerState) {                                                       // Hvis henger nå er frakoblet og var tilkoblet forrige syklus
    trailerStateInt = false;                                                                        // Oppdater statusvariabelen til false (henger frakoblet)
  }

  lastTrailerState = isTrailerActive;                                                               // Lagrer nåværende status for sammenligning i neste syklu
}

// Styrer buzzerlyd basert på nærhet når parkering er aktiv
void parkingBuzzer() {
  if (parkingStatus) {                                                                              // Kun aktiver buzzer når parkering er aktivert
    int minDistance = min(rightDistanceInt, leftDistanceInt);                                       // Finn korteste avstand fra de to sensorene
    int delayTime = map(minDistance, 0, 50, 100, 1000);                                             // Mapper avstand til delay: jo nærmere, jo raskere lyd (100ms–1000ms)

    if (minDistance <= 50) {                                                                        // Hvis avstand er under eller lik 50 cm
      tone(BUZZER_PIN, 3000);                                                                       // Spiller en tone på buzzer (3000 Hz)
      delay(delayTime);                                                                             // Venter basert på nærhet
      noTone(BUZZER_PIN);                                                                           // Stopper lyd
    }
  }
}
