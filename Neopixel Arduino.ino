//Includes
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif


//Neopixel Data PIN
#define PIN            2

//Anzahl der LEDs im Ring
#define NUMPIXELS      12

// Neopixel Setup
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

 //Werte
int lux = 0;
int delayval = 1;
int modus = 0;
int r = 0;
int b = 0;
int g = 0;

//-----------------------------SETUP-----------------

void setup() {

	//Neopixel Init
	pixels.begin();
	
	//PI Kommunikation
	pinMode(9, INPUT);
	pinMode(10, INPUT);
	pinMode(12, INPUT);

}

//------------------------------MAIN-----------------

void loop() {

	//Modi mit Pi Kommandos einstellen
	if((digitalRead(9) == HIGH) && (digitalRead(10) == LOW) && (digitalRead(12) == LOW)){
		modus = 0;
	}

	if((digitalRead(9) == LOW) && (digitalRead(10) == HIGH) && (digitalRead(12) == LOW)){
		modus = 1;
	}

	if((digitalRead(9) == LOW) && (digitalRead(10) == LOW) && (digitalRead(12) == HIGH)){
		modus = 2;
	}

	//Modus 1
	if( modus == 0){

		for(int i = 0; i < NUMPIXELS; i++){
		
			//Lauflicht einstellen
			pixels.setPixelColor(i, pixels.Color(10,0,0));
			pixels.setPixelColor(i-1, pixels.Color(0,0,10));
			
			//Wrap Around
			if(i == 0){
				pixels.setPixelColor(11, pixels.Color(0,0,10)); 
			}
			
			//Anzeigen
			pixels.show();
			delay(100);
		}
	}
  
	//Modus 2
	if( modus == 1){
		
		//Fotowiderstand auslesen
		lux = analogRead(0);
		lux = map(lux,0,512,0,32);

		//Umrechnen
		r = 15-lux;
		g = 15-lux;
		b = 15-lux;

		//Fehlerschutz
		if(r < 0){
			r = 0;
		}
		
		if(g < 0){
			g = 0;
		}
		
		if(b < 0){
			b = 0;
		}
			
		//Alle Pixel mit entsprechender Helligkeit aufleuchten
		for(int i = -1; i < NUMPIXELS; i++){
			pixels.setPixelColor(i, pixels.Color((r),(g),(b)));
		}
		
		//Anzeigen
		pixels.show();
	}

	//Modus 3
   	if( modus == 2){
    
		//Jeden zweiten Neopixel in einer bestimmten Farbe aufleuchten
		for(int i = 0; i < NUMPIXELS;i ++){
			if(i % 2){
				pixels.setPixelColor(i,pixels.Color(0,0,20));
			} else {
				pixels.setPixelColor(i,pixels.Color(10,10,0));
			}
		}

		//Anzeigen            
		pixels.show();
		delay(500);

		//Plaetze tauschen
		for(int i = 0; i < NUMPIXELS;i ++){
			if(i % 2){
				pixels.setPixelColor(i,pixels.Color(10,10,10));
			} else {
				pixels.setPixelColor(i,pixels.Color(0,0,20));
			}
		}

		//Anzeigen  
		pixels.show();
		delay(500);       
	}
}
