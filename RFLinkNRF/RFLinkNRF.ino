// Custom version RFLink R35 with support for Nodo Small units using the NRF24L01 radio
// Supports Nodo 3.7 only
// Only supports receiving variables from Nodo slaves
// For further description, check Nodo slave plugin 090
// Modifications:
//   tab NRF was added, file NRF.ino
//   this call added to the end of setup():   NRF_Radio_Init();
//   this call added to the main loop()   :   NRF_Radio_Check();

// *********************************************************************************************************************************
// * Arduino project "Nodo RadioFrequencyLink aka Nodo RFLink Version 1.1" 
// * © Copyright 2015 StuntTeam - NodoRFLink 
// * Portions © Copyright 2010..2015 Paul Tonkes (original Nodo 3.7 code)
// *
// *                                       Nodo RadioFrequencyLink aka Nodo RFLink Version 1.1
// *                                                      
// ********************************************************************************************************************************
// * This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License 
// * as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty 
// * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
// * You received a copy of the GNU General Public License along with this program in file 'COPYING.TXT'.
// * For more information on GPL licensing: http://www.gnu.org/licenses
// ********************************************************************************************************************************

// ================================================================================================================================
// Supply the full path to the RFLink files in the define below 
//
// Geef in onderstaande "define" regel het volledige pad op waar de .ino bestanden zich bevinden die je nu geopend hebt.
// ================================================================================================================================

#define SKETCH_PATH S:\Software\Nodo\source\RFLink R35 NRF\RFLinkNRF

// ================================================================================================================================
// IMPORTANT NOTE: This code only runs on an Arduino MEGA !!!!!!!!!!!!!!!!!!!!!!  It was designed that way for various reasons.
// ================================================================================================================================

