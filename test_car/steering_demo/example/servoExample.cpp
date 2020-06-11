#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <JHPWMPCA9685.h>


// Calibrated for a TRAXXAS RUSTLER Servo

int servoMin = 220 ;
int servoMax = 440 ;

int getkey() {
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}

// Map an integer from one coordinate system to another
// This is used to map the servo values to degrees
// e.g. map(90,0,180,servoMin, servoMax)
// Maps 90 degrees to the servo value

int map ( int x, int in_min, int in_max, int out_min, int out_max) {
    int toReturn =  (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min ;
    // For debugging:
    // printf("MAPPED %d to: %d\n", x, toReturn);
    return toReturn ;
}

int main() {
    PCA9685 *pca9685 = new PCA9685() ;
    int err = pca9685->openPCA9685();
    if (err < 0){
        printf("Error: %d", pca9685->error);
    } else {
        printf("PCA9685 Device Address: 0x%02X\n",pca9685->kI2CAddress) ;
        pca9685->setAllPWM(0,0) ;
        pca9685->reset() ;
        pca9685->setPWMFrequency(50) ;
        // 27 is the ESC key
        printf("Hit ESC key to exit\n");
        while(pca9685->error >= 0){
		//this part just oscillates the wheels automatically
		/*
            pca9685->setPWM(0,0,servoMin) ;
            pca9685->setPWM(1,0,servoMin) ;

            sleep(2) ;
            pca9685->setPWM(0,0,servoMax) ;
            pca9685->setPWM(1,0,map(90,0,180,servoMin, servoMax)) ;
            sleep(2) ;
		*/

		//this part lets you control from keyboard, way cooler
	    int key = getkey();
	    if(key == 67)
	    {
		pca9685->setPWM(0,0,servoMin) ;
            	pca9685->setPWM(1,0,servoMin);
	    }
	    else if(key == 68)
	    {
		pca9685->setPWM(0,0,servoMax) ;
            	pca9685->setPWM(1,0,map(90,0,180,servoMin, servoMax)) ;
	    }
        }
        pca9685->setPWM(1,0,map(0,0,180,servoMin, servoMax));
        pca9685->setPWM(0,0,map(90,0,180,servoMin, servoMax));
        sleep(1);
    }
    pca9685->closePCA9685();
}
