// Nodo Classic project
// Config:
#define SKETCH_PATH S:\Software\Nodo\source\Nodo R787\Nodo
#define CONFIG_FILE Config_01.c
#include <SD.h>                                 // Deze include noodzakelijk voor een Nodo-Mega. Niet gebruiken voor een Small!
#include <EthernetNodo.h>                       // Deze include optioneel als Ethernet wordt gebruikt. Niet gebruiken voor een Small!

//****************************************************************************************************************************
// Based on last Nodo 3.7 edition, build R787, github commit on March 10th 2015
// (This is before the drastic changes to variables and communication)
// One fix was needed to revert a change to On/Off values:  Reverted on/off values from 9/10 back to 74/75

// Other changes:
// Beta stuff introduced in R755 is now formal, removed conditional compile rules
// Autoreset feature using special serial char (upload without DTR pulse), needs custom hardware, can be ignored
// Enabled "Status Unit" command for Nodo small, using the nodotool
// Changed eventscanning:  if (Fetched!=1) RepeatingTimer=millis()+SIGNAL_REPEAT_TIME;
// Added "Delayms" command for delays in milliseconds

// Additional plugins:
// Plugin 092  Nodo Communication with local UDP/W5100, remote ESP Easy as bridge
// Plugin 093  Nodo Communication with local and remote ESP Easy as bridge
// Plugin 094  Limited IR receive/send for Samsumg, Sony, Philips, works on Nodo Small!
// Plugin 095  SleepExternal
// Plugin 096  Xiron temp/hum (must change SIGNAL_TIMEOUT to 3 to get this working)
// Plugin 098  DomoticzHTTP support
// Plugin 099  Nodotool for Nodo Small debugging/programming
//****************************************************************************************************************************


/****************************************************************************************************************************\
* Arduino project "Nodo" © Copyright 2010..2015 Paul Tonkes 
* 
* This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License 
* as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
* This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty 
* of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
* You received a copy of the GNU General Public License along with this program in file 'COPYING.TXT'.
*
* Voor toelichting op de licentievoorwaarden zie    : http://www.gnu.org/licenses
* Uitgebreide documentatie is te vinden op          : http://www.nodo-domotica.nl
* bugs kunnen worden gelogd op                      : https://code.google.com/p/arduino-nodo/
* Compiler voor deze programmacode te downloaden op : http://arduino.cc
* Voor vragen of suggesties, mail naar              : p.k.tonkes@gmail.com
* Compiler                                          : Arduino Compiler met minimaal versie 1.0.5
* Libraties                                         : EthernetNodo library vervangt de standaard Ethernet library!
\*************************************************************************************************************************/

// ============================================================================================================================
// We kennen twee type Nodo's:
//
// Nodo-Mega:   Een Nodo op basis van een Arduino Mega met een ATMega1280 of 2560 processor. Deze Nodo heeft o.a. de mogelijkheid
//              tot ethernet communicatie.
//
// Nodo-Small:  Dit is een kleine Nodo die wordt vooral wordt gebruikt als satelliet in combinatie met een centrale Nodo-Mega.
//              Een Nodo-Small maakt gebruik van een Arduino met een ATMega328 processor. (Nano, Pro,Duemillanove, Uno, etc)                                                                                                                
//
// Bij gebruik van meerdere Nodo's kan voor iedere Nodo een eigen configutatie file worden aangemaakt. In deze configuratie files
// kan worden aangegeven welke plugins worden gebruikt en kunnen eventueel speciale instellingen worden opgegeven die mee
// gaan met het compileren van de code.
// Configuratie bestanden bevinden zich in de directory ../Config. In deze configuratiefiles kunnen settings worden opgegeven 
// die worden meegecompileerd of kunnen devices worden opgegeven waar de Nodo mee moet kunnen communiceren.
// Default zijn de volgende configuratie files gemaakt:
//
// Config_01.c => Deze is default bestemd voor een Nodo Mega met unitnummer 1.
// Config_15.c => Deze is default bestemd voor een Nodo Small met unitnummer 15.
//
// Alle regels gemarkeerd met een '//' worden niet meegecompilileerd.
// 
// LET OP:
//  
// -  Het unitnummer van de Nodo zal pas veranderen nadat de Nodo software voor het eerst wordt geinstalleerd of de Nodo het 
//    commando [Reset] uitvoert. Dus niet altijd na een compilatie!
// -  Indien gewenst kunnen de config files ook voor andere unitnummers worden aangemaakt (1..31)
//
// ============================================================================================================================

