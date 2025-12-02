# SDTR---Distributed-Thermal-Monitoring-and-Control-System-for-IT-Rooms
Proiect Sisteme distribuite și de timp real - Felicia Dobrin - SECI RCD

Proiectul propune dezvoltarea unui sistem distribuit de monitorizare și control termic destinat camerelor IT, bazat pe microcontroller-ul STM32WB55, care integrează un nucleu ARM și un modul de comunicație Bluetooth Low Energy (BLE). Fiecare nod al rețelei include senzori de temperatură și umiditate și elemente de execuție (ventilator, LED/alarmă), controlate în timp real prin FreeRTOS. Nodurile comunică între ele prin Bluetooth, partajând valorile măsurate și starea sistemului, astfel încât să se poată regla automat ventilația atunci când pragurile de temperatură sunt depășite.
