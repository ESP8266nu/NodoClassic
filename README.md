# NodoClassic

Nodo Project release for historic reasons.

I've cleaned up my dropbox account and this repository provides access to code and tools that may still be in use somewhere.

I'm still using several Nodo units, connected to ESP/Domoticz and the Small units are still programmed with the Nodotool that is also used for much easier debugging on Nodo Small units.
A feature that is lacking in the original codebase.

This repository is based on Nodo 3.7, R765 since this has been the most stable Nodo edition in my own home. Some patches have been applied, just before the major changes on Nodo 3.8.
So patchlevel is now R787.

Some additional plugins in this repository:

* Plugin 092  Nodo Communication with local UDP/W5100, remote ESP Easy as bridge
* Plugin 093  Nodo Communication with local and remote ESP Easy as bridge
* Plugin 094  Limited IR receive/send for Samsumg, Sony, Philips, works on Nodo Small!
* Plugin 095  SleepExternal
* Plugin 096  Xiron temp/hum (must change SIGNAL_TIMEOUT to 3 to get this working)
* Plugin 098  DomoticzHTTP support
* Plugin 099  Nodotool for Nodo Small debugging/programming

# NodoTiny
An ATTiny85 based Nodo Compatible sensor that can run on batteries for more then a year...

# TinyKakuReceiver
An ATTiny85 based device that emulates a Kaku dimmer.

# RFLinkNRF
A customized RFlink that can receive data from Nodo NRF slaves using the NRF24L01 radio, in addition to the standard 433 MHz reception.


