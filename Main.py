#__________________________________________IMPORT_________________________________________________

import RPi.GPIO as GPIO
import time
import math

import cv2
import numpy as np

from threading import Thread, Lock

#__________________________________________CLASSES_________________________________________________


class WebcamVideoStream :
    def __init__(self) :
        self.stream = cv2.VideoCapture(0)
        (self.grabbed, self.frame) = self.stream.read()
        self.started = False
        self.read_lock = Lock()

    def start(self) :
        if self.started :
            print ("Warning: Webcam Thread already running.")
            return None
        self.started = True
        self.thread = Thread(target=self.update, args=())
        self.thread.start()
        return self

    def update(self) :
        while self.started :
            (grabbed, frame) = self.stream.read()
            self.read_lock.acquire()
            self.grabbed, self.frame = grabbed, frame
            self.read_lock.release()

    def read(self) :
        self.read_lock.acquire()
        frame = self.frame.copy()
        self.read_lock.release()
        return frame

    def stop(self) :
        self.started = False
        self.thread.join()

    def __exit__(self, exc_type, exc_value, traceback) :
        self.stream.release()
       
     
#__________________________________________INIT_________________________________________________

#BCM umschalten
GPIO.setmode(GPIO.BCM)

#Webcam initialisieren
cap = WebcamVideoStream().start()

#Motorseite Rechts
PIN_IN1_R = 23
PIN_IN2_R = 24
PIN_EN_R = 18

#Motorseite Links
PIN_IN1_L = 27
PIN_IN2_L = 22
PIN_EN_L = 17

#Globale
g_stop = 0                 #Stoppt Programm
ending = 0                 #Leitet Endphase ein
ending_counter = 0         #Zeititervall Endphase
imagecounter = 0           #Zähler zum Speichern von Frames
lost_blink = 0             #Status-LED blinken
lost_blink_counter = 0     #Zeitintervall Blinken

soll_speed = 70            #(40)    Durchschnittliche Geschwidnigkeit (Von 0 - 100)
stellgewicht = 6           #(6)     Wie stark soll die Auslenkung sein (Je größer desto weniger)
begrenzungsfaktor = 1.7    #(1.7)   Wie stark darf die maximale Auslenkung sein (Je größer desto kleiner)
potenzgewicht = 1.15       #(1.15)  Mit welcher Potenz wird die Auslenkungsfunktion gewichtet

lost_counter_aktiv = False #Linie verloren
lost_counter = 0           #Zeitintervall für Korrektur
ampelmodus = 1             #Mit der Ampelfunktion starten
lost_right = 0             #Letzte bekannte Linienposition

#LED 
PIN_LED1 = 5

PIN_LED_A_1 = 19
PIN_LED_A_2 = 21
PIN_LED_A_3 = 20
PIN_LED_A_4 = 13
PIN_LED_A_5 = 26

#Arduino
PIN_MODUS_1 = 6
PIN_MODUS_2 = 25
PIN_MODUS_3 = 12

#Modus Output
GPIO.setup(PIN_MODUS_1, GPIO.OUT)
GPIO.setup(PIN_MODUS_2, GPIO.OUT)
GPIO.setup(PIN_MODUS_3, GPIO.OUT)

#Motoren Starten
GPIO.setup(PIN_IN1_R, GPIO.OUT)
GPIO.setup(PIN_IN2_R, GPIO.OUT)
GPIO.setup(PIN_EN_R, GPIO.OUT)
Motor_R = GPIO.PWM(PIN_EN_R, 70)
Motor_R.start(0)

GPIO.setup(PIN_IN1_L, GPIO.OUT)
GPIO.setup(PIN_IN2_L, GPIO.OUT)
GPIO.setup(PIN_EN_L, GPIO.OUT)
Motor_L = GPIO.PWM(PIN_EN_L, 70)
Motor_L.start(0)

#LEDs starten
GPIO.setup(PIN_LED1, GPIO.OUT)
LED1 = GPIO.PWM(PIN_LED1, 100)
LED1.start(0)

GPIO.setup(PIN_LED_A_1, GPIO.OUT)
LED_A_1 = GPIO.PWM(PIN_LED_A_1, 100)
LED_A_1.start(0)

GPIO.setup(PIN_LED_A_2, GPIO.OUT)
LED_A_2 = GPIO.PWM(PIN_LED_A_2, 100)
LED_A_2.start(0)

GPIO.setup(PIN_LED_A_3, GPIO.OUT)
LED_A_3 = GPIO.PWM(PIN_LED_A_3, 100)
LED_A_3.start(0)

GPIO.setup(PIN_LED_A_4, GPIO.OUT)
LED_A_4 = GPIO.PWM(PIN_LED_A_4, 100)
LED_A_4.start(0)

GPIO.setup(PIN_LED_A_5, GPIO.OUT)
LED_A_5 = GPIO.PWM(PIN_LED_A_5, 100)
LED_A_5.start(0)


#__________________________________________LINIENFINDUNG_________________________________________________


def distanz_linie(img_debug):
    #Auszulesende Zeile (0 = Oben, 480 = Unten)
    zeilen_nr = 380

    #Webcam Frame Laden
    img_frame = cap.read()
    
    #Bilder abspeichern
    img_speichern = img_debug

    if img_speichern == 1:
        global imagecounter
        
        name = "frames/frame%d.png" % imagecounter
        cv2.imwrite(name, img_frame)
        imagecounter = imagecounter+1
    
    #Farbkanäle extrahieren
    r = img_frame[zeilen_nr, :, 2].astype('int16')
    g = img_frame[zeilen_nr, :, 1].astype('int16')
    b = img_frame[zeilen_nr, :, 0].astype('int16')
    
    #Frame an Ampelerkennung weiterleiten
    ampel_check_stop(img_frame)
        
    #Rotkanal extrahieren
    messzeile = r-(g+b)/2
    
    #Threshold
    messzeile_thresh = messzeile > 45
    
    #Schwerpunkt finden
    masse = messzeile_thresh.sum()
    x = list(range(640))
    
    masse_gewichtet = x * messzeile_thresh
    masse_gewichtet_summe = masse_gewichtet.sum()
    
    #Keine Linie
    if masse == 0:
        return 0
        
    schwerpunkt = masse_gewichtet_summe / masse
    
    return schwerpunkt

#__________________________________________AMPELERKENNUNG_________________________________________________

def ampel_check_stop(frame):
    #Zeilen für Ampelfindung
    zeile_ampel = [340, 240, 140]
    
    #Stopfunktion
    global ending
    
    #Ampelfindung (Stop)
    masse_ampel = 0
    
    #Genauigkeit der Ampelfindung durch Verwenden mehrerer Zeilen erhöhen
    for p in range(3):
        ra = frame[zeile_ampel[p], :, 2].astype('int16')
        ga = frame[zeile_ampel[p], :, 1].astype('int16')
        ba = frame[zeile_ampel[p], :, 0].astype('int16')
        
        ampelzeile = ba-(ga+ra)/2
        
        ampelzeile_thresh = ampelzeile > 65
        
        masse_ampel = masse_ampel + ampelzeile_thresh.sum()
    
    #Bei Blau Ende einleiten
    if masse_ampel > 15:
        ending = 1
        print("AMPEL ERKANNT - AUTO ANHALTEN")

def ampel_check_start(): 
    #Webcam Frame Laden
    img_frame = cap.read()   
    
    #Farbkanäle extrahieren
    r = img_frame[:, :, 2].astype('int16')
    g = img_frame[:, :, 1].astype('int16')
    b = img_frame[:, :, 0].astype('int16')
    
    #Grünkanal extrahieren
    img_green = g-(r+b)/2
    
    #Bild auslesen
    messbild = img_green[:, :]
    
    #Threshold
    messbild_thresh = messbild > 45
    
    #Schwerpunkt finden
    masse = messbild_thresh.sum()
    
    #Losfahren
    if masse > 1200:
        return 1
    else:
        return 0
   
         
#__________________________________________LENKUNG_________________________________________________


def geschwindigkeit_berechnen( messdistanz, soll_speed, stellgewicht, potenzgewicht ):
    #Abweichung
    abweichung = 320 - messdistanz
    
    #Geschwindikeitskurven Erzeugen
    speed_modifier_L = soll_speed + math.fabs(abweichung/20)
    speed_modifier_R = soll_speed - math.fabs(abweichung/20)
    
    #Gewichtung
    if abweichung >= 0:
        abweichung = math.pow(math.fabs(abweichung), potenzgewicht)
        abweichung_gewicht = int(abweichung/stellgewicht)
    else:
        abweichung = math.pow(math.fabs(abweichung), potenzgewicht)
        abweichung_gewicht = -int(abweichung/stellgewicht)

    #Geschwindigkeit einstellen
    motor_speed_L = speed_modifier_L+abweichung_gewicht
    motor_speed_R = speed_modifier_R-abweichung_gewicht
    
    #Geschwindigkeit bei hoher Abweichung reduzieren
    motor_speed_L = motor_speed_L - math.fabs(abweichung_gewicht/1.4) #1.4
    motor_speed_R = motor_speed_R - math.fabs(abweichung_gewicht/1.4) #1.4
    
    #Kompensation für schlechte Motoren
    motor_speed_R = motor_speed_R + 35
    
    #Maximalgeschwindigkeit begrenzen
    if motor_speed_R > 100:
        motor_speed_R = 100
    elif motor_speed_R < 10:
        motor_speed_R = 10

    if motor_speed_L > 80:
        motor_speed_L = 80
    elif motor_speed_L < 10:
        motor_speed_L = 10
        
    #print("R: ", motor_speed_L)
    #print("L: ", motor_speed_R)

    #Tupel zurückgeben
    motor_speed = (motor_speed_L, motor_speed_R)
    return motor_speed


#__________________________________________MOTORENSTEUERUNG_________________________________________________


def motorensteuerung( geschwindigkeit_L, geschwindigkeit_R ):
    global PIN_IN1_L
    global PIN_IN1_R
    global Motor_L
    global Motor_R
    global LED1
    
    #Richtung einstellen Links
    if geschwindigkeit_L < 0:
        GPIO.output(PIN_IN1_L, GPIO.LOW)
        GPIO.output(PIN_IN2_L, GPIO.HIGH)
    else:
        GPIO.output(PIN_IN1_L, GPIO.HIGH)
        GPIO.output(PIN_IN2_L, GPIO.LOW)
        
    #Richtugn eistellen Rechts
    if geschwindigkeit_R < 0:
        GPIO.output(PIN_IN1_R, GPIO.LOW)
        GPIO.output(PIN_IN2_R, GPIO.HIGH)
    else:
        GPIO.output(PIN_IN1_R, GPIO.HIGH)
        GPIO.output(PIN_IN2_R, GPIO.LOW)

    #Motoren einstellenmko
    Motor_L.ChangeDutyCycle(math.fabs(geschwindigkeit_L))
    Motor_R.ChangeDutyCycle(math.fabs(geschwindigkeit_R))
    
    #Geschwindigkeits-LED einstellen
    LED_duty = (60 - math.fabs(geschwindigkeit_L-geschwindigkeit_R))
    
    #Fehlerschutz
    if LED_duty < 0:
        LED_duty = 0
    elif LED_duty > 100:
        LED_duty = 100
     
    LED1.ChangeDutyCycle(LED_duty)  
        
        
#__________________________________________LED STEUERUNG_________________________________________________


def distanz_leds( distanz ):
    global LED_A_1
    global LED_A_2
    global LED_A_3
    global LED_A_4
    global LED_A_5
    
    global lost_blink
    global lost_blink_counter
    
    #Distanzsektoren festlegen und Status LEDs einstellen
    if distanz != -100:
        if -100 < distanz < -70:
            LED_A_1.ChangeDutyCycle( 100 )
            LED_A_2.ChangeDutyCycle( 0 )
            LED_A_3.ChangeDutyCycle( 0 )
            LED_A_4.ChangeDutyCycle( 0 )
            LED_A_5.ChangeDutyCycle( 0 )
        elif -70 <= distanz < -30:
            LED_A_1.ChangeDutyCycle( 0 )
            LED_A_2.ChangeDutyCycle( 100 )
            LED_A_3.ChangeDutyCycle( 0 )
            LED_A_4.ChangeDutyCycle( 0 )
            LED_A_5.ChangeDutyCycle( 0 )
        elif -30 <= distanz < 30:
            LED_A_1.ChangeDutyCycle( 0 )
            LED_A_2.ChangeDutyCycle( 0 )
            LED_A_3.ChangeDutyCycle( 100 )
            LED_A_4.ChangeDutyCycle( 0 )
            LED_A_5.ChangeDutyCycle( 0 )
        elif 30 <= distanz < 70:
            LED_A_1.ChangeDutyCycle( 0 )
            LED_A_2.ChangeDutyCycle( 0 )
            LED_A_3.ChangeDutyCycle( 0 )
            LED_A_4.ChangeDutyCycle( 100 )
            LED_A_5.ChangeDutyCycle( 0 )
        elif 70 <= distanz <= 100:
            LED_A_1.ChangeDutyCycle( 0 )
            LED_A_2.ChangeDutyCycle( 0 )
            LED_A_3.ChangeDutyCycle( 0 )
            LED_A_4.ChangeDutyCycle( 0 )
            LED_A_5.ChangeDutyCycle( 100 )
    else:
        #Bei verlorener Linie alle Status-LEDs blinken lassen
        lost_blink_counter = lost_blink_counter + 1
        
        #Zähler alternieren
        if lost_blink_counter > 40:
            lost_blink_counter = 0
            lost_blink ^= 1
            
        #Blinken übernehmen
        LED_A_1.ChangeDutyCycle( 100 * lost_blink )
        LED_A_2.ChangeDutyCycle( 100 * lost_blink )
        LED_A_3.ChangeDutyCycle( 100 * lost_blink )
        LED_A_4.ChangeDutyCycle( 100 * lost_blink )
        LED_A_5.ChangeDutyCycle( 100 * lost_blink )
    
def neopixel_modus(modus):
    #Schnittstelle mit Arduino
    global PIN_MODUS_1
    global PIN_MODUS_2
    global PIN_MODUS_3
    
    #Alle Outputs auf Low setzen = keine Veränderung
    GPIO.output(PIN_MODUS_1, GPIO.LOW)
    GPIO.output(PIN_MODUS_2, GPIO.LOW)
    GPIO.output(PIN_MODUS_3, GPIO.LOW)
    
    if modus == 1:
        #Lauflicht vor Rennstart
        GPIO.output(PIN_MODUS_1, GPIO.HIGH)
    elif modus == 2:
        #Lichtempfindliche Frontbeleuchtung
        GPIO.output(PIN_MODUS_2, GPIO.HIGH)
    elif modus == 3:
        #Doppelring nach Beendigung des Rennens
        GPIO.output(PIN_MODUS_3, GPIO.HIGH)
        
#______________________________________________DEBUG_________________________________________________

        
def print_debug(modus, messdistanz):
    if modus == 1:
        #Debuglinie erzeugen
        debug_genau = 100
        debug_leerzeichen = int(messdistanz/640*debug_genau)
        debug_line = ' '*debug_leerzeichen
        debug_line2= ' '*(debug_genau-debug_leerzeichen)

        #Ausgeben
        if -10 <= messdistanz < 10:
            print('Left|', debug_line, '-X-', debug_line2,'|Right')
        else:
            print('Left|', debug_line, '-#-', debug_line2,'|Right')
        
#________________________________________________________________________________________________________#
#__________________________________________MAINLOOP______________________________________________________#    
#________________________________________________________________________________________________________#


while g_stop == 0: 

    #_______________________________________________Auf Ampel warten_________________________________
    
    if ampelmodus == 1:
        
        #Beleuchtung einstellen
        neopixel_modus(1)
        
        #Ampel prüfen
        go = ampel_check_start()
        
        print('WAITING')
        
        #Nach kurzem Intervall starten
        if go == 1:
            time.sleep(0.05)
            ampelmodus = 0
            print('START')
            
    else:  #___________________________________________Linienfindung__________________________________
       
        #Anhalten sobald das Ende eingeleitet wurde
        if ending == 1:
            
            #Fahrezug noch über die Ziellinie fahren lassen
            ending_counter = ending_counter + 1
            
            #End-Beleuchtung
            neopixel_modus(3)
            
            #Zähler
            if ending_counter > 300:
                #Programm beenden
                g_stop = 1
        else:        
            #Streckenbeleuchtung
            neopixel_modus(2)

        #Distanz zur Linien finden
        messdistanz = distanz_linie(0)
        
        #Letzte bekannte Linienposition zwischenspeichern
        if messdistanz != 0:
            if messdistanz >= 320:
                lost_right = 1
            elif messdistanz < 320:
                lost_right = 0

        #Status-LEDs einstellen
        distanz_leds( (messdistanz - 320)/3.2 )      


        if messdistanz == 0:  #__________________________Linie verloren_______________________________

            #Korrekturablauf durch Zähler bestimmen
            lost_counter = lost_counter + 1
                       
            if lost_counter > 50:
                
                #Wagen nach links bzw. rechts drehen lassen um Linie wiederzufinden
                if lost_right == 1:
                    motorensteuerung( -40, 65 )
                else:
                    motorensteuerung( 40, -65 )

                print('LINE LOST - CORRECTING')
                
            elif lost_counter > 1:
                
                #Wagen zur Stabilisierung anhalten
                motorensteuerung( 0, 0)
                
                print('LINE LOST - WAITING')
                                
            else:
                
                #Wagen weiterfahren lassen bevor Korrektur eingeleitet wird
                print('LINE LOST - SEARCHING')

        else: #_________________________________________Linie verfolgen_______________________________
            
            #Wenn der Wagen aus Korrekturmodus kam, alles zurücksetzen
            if lost_counter > 0:
                
                #Wagen zur Stabilisierung kurz anhalten lassen
                motorensteuerung( 0, 0)
                time.sleep(.015)
                
                #Zurücksetzen
                lost_counter = 0
                print('CORRECTION FINISHED - RESTARTING')

            else:

                #Geschwindigkeit berechnen
                motor_speed = geschwindigkeit_berechnen( messdistanz, soll_speed, stellgewicht, potenzgewicht )
                
                #Motoren anpassen
                motorensteuerung( motor_speed[0], motor_speed[1] )

                #Debug Ausgabe
                print_debug(1, messdistanz)
            

#_____________________________________________CLEANUP_________________________________________________
    
#Cleanup
cap.stop()

Motor_R.stop()
Motor_L.stop()

GPIO.cleanup()