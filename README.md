# 🚗 Semesterprosjekt – Ryggesensor Prototype

Et Arduino-basert system utviklet som semesterprosjekt for å simulere en ryggesensor for bil, bestående av to Arduino-enheter: en hovedenhet (M1) og en slaveenhet (S1). Systemet benytter ultralydsensorer, LCD-skjerm, LED-indikatorer, UART-kommunikasjon og sanntidsklokke for å gi sjåføren informasjon om avstander, klokkeslett og hengerstatus.

Semesterprosjektet omhandler utviklingen av en ryggesensor-prototype for bil, bestående av to Arduino-enheter: en hovedenhet (M1) og en slaveenhet (S1). 
Slaveenheten måler avstand til hindringer på høyre og venstre side med ultralydsensorer og sender data via UART til hovedenheten, 
som viser informasjonen på en LCD-skjerm og gir visuelle varsler med LED-lys. Systemet aktiveres automatisk ved reversgir eller manuelt via en knapp, og 
deaktiveres hvis henger er tilkoblet. Brukeren får informasjon om både avstand og klokkeslett, 
og systemet benytter sanntidsklokke og temperaturmåling for utvidet funksjonalitet.

---

## 📦 Innhold

- `M1_Master/` – Kode for hovedenheten (M1)
- `S1_Slave/` – Kode for slaveenheten (S1)
- `images/` – Illustrasjoner og skjemaer (valgfritt)
- `doc/` – Teknisk dokumentasjon (valgfritt)
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

- 2x Arduino (f.eks. Nano 33 BLE Sense)
- 2x HC-SR04 ultralydsensorer
- 16x2 LCD (koblet til M1)
- 3x LED (rød, gul, grønn)
- 1x Buzzer (koblet til S1)
- Revers-signal (bryter eller sensor)
- Parkeringsknapp (PTC eller vanlig knapp)
- Hengerdeteksjon (bryter eller kabel)
- UART (TX/RX) mellom M1 og S1

---

## ⚙️ Oppsett og tilkobling

| Komponent        | M1 Pin 	| S1 Pin 		|
|------------------|------------|-----------------------|
| LCD RS           | 7      	| –      		|
| LCD E            | 6      	| –      		|
| LCD D4-D7        | 5–2    	| –      		|
| LED Rød/Gul/Grønn| 13/12/11	| –      		|
| Parking LED      | 10     	| –      		|
| Parking Button   | 9      	| –      		|
| Revers Signal    | 8      	| –      		|
| UART RX/TX       | Serial1	| Serial1		|
| HC-SR04 (venstre)| –      	| Trig: 3 / Echo: 2 	|
| HC-SR04 (høyre)  | –      	| Trig: 5 / Echo: 4 	|
| Henger-signal    | –      	| 6      		|
| Buzzer           | –      	| 7      		|

---

## 🧪 Eksempel på UART-data

```txt
R:121 L:98 T:0 P:0
