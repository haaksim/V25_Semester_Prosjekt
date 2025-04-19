# 🚗 Semesterprosjekt – Ryggesensor Prototype

Semesterprosjektet består av utviklingen av en Arduino-basert prototype som simulerer en ryggesensor for bil. Systemet er bygget opp av to Arduino-enheter: en hovedenhet (M1) og en slaveenhet (S1), som kommuniserer med hverandre via UART.

Slaveenheten er ansvarlig for å måle avstand til hindringer på venstre og høyre side ved hjelp av ultralydsensorer. Disse målingene sendes til hovedenheten, som tolker dataene og viser relevant informasjon på en LCD-skjerm. I tillegg benyttes LED-indikatorer for å gi sjåføren visuelle varsler om avstanden til hindringer.

Systemet aktiveres enten automatisk når bilen settes i revers, eller manuelt ved hjelp av en knapp. Hvis en tilhenger er tilkoblet, deaktiveres systemet for å unngå feilaktige varsler.

For å gi føreren mer kontekstuell informasjon, inkluderer systemet også en sanntidsklokke (RTC) for visning av klokkeslett, samt funksjonalitet for temperaturmåling. Dette gir en mer komplett og brukervennlig opplevelse, og demonstrerer hvordan mikrokontrollerbaserte systemer kan brukes til å forbedre sikkerheten og informasjonstilgangen i kjøretøy.

---

## 📦 Innhold

- `M1_Master/` – Kode for hovedenheten (M1)
- `S1_Slave/` – Kode for slaveenheten (S1)
- `images/` – Illustrasjoner og skjemaer
- `doc/` – Teknisk dokumentasjon
- `README.md` – Denne filen
- `LICENSE` – Valgfri lisens (f.eks. MIT eller GPL)

---

## 🔧 Systemoversikt

**Hovedenhet (M1):**
- Mottar avstandsinformasjon via UART
- Viser info på LCD (klokke, dato, eller avstand)
- Styrer LED-varslinger basert på avstand:
  - Rød: < 50 cm
  - Gul: 50–99 cm
  - Grønn: 100–149 cm
- Aktiveres via parkeringsknapp eller reverssignal
- Deaktiveres automatisk hvis henger er tilkoblet

**Slaveenhet (S1):**
- Leser avstand med to HC-SR04 ultralydsensorer
- Sender avstander + hengerstatus til M1
- Detekterer tilkoblet henger via inngangspin
- Planlagt støtte: Temperaturmåler + buzzer (TO DO)

---

## 🛠️ Maskinvarekrav

- 2x Arduino uno r4 Wi-Fi
- 2x HC-SR04 ultralydsensorer
- 16x2 LCD (koblet til M1)
- 4x LED (rød, gul, 2 x grønn)
- 1x Buzzer (koblet til S1)
- Revers-signal (bryter eller sensor)
- Parkeringsknapp (bryter eller sensor)
- Hengerdeteksjon (bryter eller sensor)
- UART (TX/RX) mellom M1 og S1

---

## ⚙️ Oppsett og tilkobling

| Komponent        | M1 Pin 	| S1 Pin 		|
|------------------|----------|----------------------|
| LCD RS           | 7      	| –      		|
| LCD E            | 6      	| –      		|
| LCD D4-D7        | 5–2    	| –      		|
| LED Rød/Gul/Grønn| 13/12/11	| –      		|
| Parking LED      | 10     	| –      		|
| Parking Button   | 9      	| –      		|
| Revers Signal    | 8      	| –      		|
| UART RX/TX       | 0/1    	| 0/1    		|
| HC-SR04 (venstre)| –      	| Trig: 3 / Echo: 2 	|
| HC-SR04 (høyre)  | –      	| Trig: 5 / Echo: 4 	|
| Henger-signal    | –      	| 6      		|
| Buzzer           | –      	| 7      		|

---

## 🧪 Eksempel på UART-data

```txt
R:121 L:98 T:0 P:0
