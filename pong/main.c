//#include "pre_emptive_os/api/osapi.h"
//#include "pre_emptive_os/api/general.h"
#include "lpc2xxx.h"
#include "config.h"
#include "general.h"
#include "lcd.h"
#include <printf_P.h>
#include <ea_init.h>
#include <consol.h>
#include <stdint.h>
#include "adc.h"
#include <math.h>
#include "i2c.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 80
#define PADDLE_WIDTH 30
#define PADDLE_HEIGHT 5
#define BALL_SIZE 6

#define JOY_CENTER (1 << 8)
#define JOY_LEFT   (1 << 9)
#define JOY_UP     (1 << 10)
#define JOY_RIGHT  (1 << 11)
#define JOY_DOWN   (1 << 12)

#define ACCEL_CHANNEL 1
#define SHAKE_THRESHOLD 200

#define EEPROM_ADDR 0xA0
#define I2C_EEPROM_SND (EEPROM_ADDR + (0 << 1))
#define I2C_EEPROM_RCV (EEPROM_ADDR + (0 << 1) + 0x01)

#define HIGHSCORE_ADDR 0x0

#define SAMPLE_RATE 22050
#define BUFFER_SIZE 1000

int paddlePos = SCREEN_WIDTH/2 - PADDLE_WIDTH/2;
int ballX = SCREEN_WIDTH/2;
int ballY = 10;
int ballSpeedX = 2;
int ballSpeedY = 2;
int score = 0;
int highscore = 0;
char scoreX[] = "0";
int game = 1;
int waitForRestart = 0;
int prevBallX = 0, prevBallY = 0;
int prevPaddlePos = 0;
int gameOverDisplayed = 0;
int min=0, sec=0, msec = 0;
char timer[8];
static tU16 audioBuffer[BUFFER_SIZE];
static tU16 audioCounter = 0;
static tU8 audioEnabled = 0;

tU16 prevAccelValue = 0;

void initAdc();

void initJoy() {
    IODIR0 &= ~(JOY_CENTER | JOY_LEFT | JOY_UP | JOY_RIGHT | JOY_DOWN);
}

void initI2C() {
    i2cInit();
}

tU16 getAnalogueInput(tU8 channel);

tS8 eepromPoll(void);
tS8 eepromWrite(tU16 addr, tU8* pData, tU16 len);
tS8 eepromPageRead(tU16 address, tU8* pBuf, tU16 len);

void initAudio() {
    PINSEL1 &= ~0x000C0000;
    PINSEL1 |= 0x00080000;
    audioEnabled = 1;
}

const tU16 sinTable16[] = {
    512, 768, 896, 960, 992, 1008, 1016, 1020,
    1023, 1020, 1016, 1008, 992, 960, 896, 768
};

void playTone(tU16 frequency, tU16 durationMs) {
    if (!audioEnabled) return;

    tU32 samples = (SAMPLE_RATE * durationMs) / 1000;
    tU32 phase = 0;
    tU32 phaseIncrement = (frequency * 16 * 65536) / SAMPLE_RATE;
    tU32 i;

    for (i = 0; i < samples; i++) {

        tU16 sample = sinTable16[(phase >> 16) & 0x0F];

        DACR = (sample << 6) | (1 << 16); 


        phase += phaseIncrement;

        udelay(1000000 / SAMPLE_RATE);
        }
}

void delayMs(tU16 delayInMs) {
    T1TCR = 0x02;          
    T1PR = 0x00;         
    T1MR0 = delayInMs * (CORE_FREQ / (PBSD * 1000));
    T1IR = 0xff;
    T1MCR = 0x04;          
    T1TCR = 0x01;          

    while (T1TCR & 0x01);  
}

void udelay(unsigned int delayInUs) {
    T1TCR = 0x02;
    T1PR = 0x00;
    T1MR0 = delayInUs * (CORE_FREQ / (PBSD * 1000000));
    T1IR = 0xff;
    T1MCR = 0x04;
    T1TCR = 0x01;

    while (T1TCR & 0x01);
}

int isJoyPressed(tU32 direction) {
    return !(IOPIN0 & direction);
}

void drawPaddle(int position) {
    int y = SCREEN_HEIGHT - PADDLE_HEIGHT;
    lcdRect(position, y, PADDLE_WIDTH, PADDLE_HEIGHT, 1);
}

void drawBall(int x, int y) {
    lcdRect(x, y, BALL_SIZE, BALL_SIZE, 1);
}

void drawScore(int x, int y) {
	sprintf(scoreX, "%d", score);
	lcdGotoxy(x, y);
	lcdPuts("SCORE: ");
	lcdGotoxy(x+62, y);
	lcdPuts(scoreX);
}

void drawHighscore(int x, int y) {
    char highscoreStr[10];
    sprintf(highscoreStr, "%d", highscore);
    lcdGotoxy(x, y);
    lcdPuts("HIGH SCORE:      ");
    lcdGotoxy(x+100, y);
    lcdPuts(highscoreStr);
}

void drawTime(int x, int y) {
	lcdGotoxy(x, y);
	lcdPuts("TIME:");
	lcdGotoxy(x+62, y);
	sprintf(timer, "%02d:%02d", min, sec);
	lcdPuts(timer);
}

void clearBall(int x, int y) {
	lcdRect(x, y, BALL_SIZE, BALL_SIZE, 0xff);
}

void clearPaddle(int position) {
	int y = SCREEN_HEIGHT - PADDLE_HEIGHT;
	lcdRect(position, y, PADDLE_WIDTH, PADDLE_HEIGHT, 0xff);
}

void updateTimer() {
	msec++;
	if (msec >= 12) {
		msec = 0;
		sec++;
	}
	if (sec >= 60) {
		sec = 0;
		min++;
	}
}

uint32_t getMillis() {
	return T0TC;
}

void handleInput() {
    if(isJoyPressed(JOY_LEFT) && paddlePos > 0)
        paddlePos -= 5;
    if(isJoyPressed(JOY_RIGHT) && paddlePos < SCREEN_WIDTH - PADDLE_WIDTH)
        paddlePos += 5;
}

void checkAccelerometer() {
    tU16 currentAccelValue = getAnalogueInput(ACCEL_CHANNEL);
    int diff = abs(currentAccelValue - prevAccelValue);
    if (diff > SHAKE_THRESHOLD) {
        highscore = 0;
        writeHighscoreToEEPROM(highscore);
        lcdGotoxy(3,61);
        lcdPuts("          ");
        drawHighscore(3,61);
    }
    prevAccelValue = currentAccelValue;
}

void writeHighscoreToEEPROM(int highscore) {
    tU8 data[2];
    data[0] = (highscore >> 8) & 0xFF; 
    data[1] = highscore & 0xFF;        
    if (eepromWrite(HIGHSCORE_ADDR, data, 2) == I2C_CODE_OK) {
        eepromPoll();
    } else {
        
        lcdGotoxy(3, 80);
        lcdPuts("Write Error");
        delayMs(500);
    }
}

int readHighscoreFromEEPROM() {
    tU8 data[2] = {0, 0};
    if (eepromPageRead(HIGHSCORE_ADDR, data, 2) == I2C_CODE_OK) {
        int hs = (data[0] << 8) | data[1];
        return hs;
    }

    return 0; 
}

void updateGame() {
	if (game == 0) {
		return;
	}

    ballX += ballSpeedX;
    ballY += ballSpeedY;

    if(ballX <= 0 || ballX >= SCREEN_WIDTH - BALL_SIZE)
        ballSpeedX = -ballSpeedX;

    if(ballY <= 0)
        ballSpeedY = -ballSpeedY;

//		Bug Dla szybkiego nabicia punktow
//    if(ballY >= SCREEN_HEIGHT - PADDLE_HEIGHT - BALL_SIZE &&
//       ballX >= paddlePos && ballX <= paddlePos + PADDLE_WIDTH )
    	//&& ballSpeedY > 0{
    if(ballY + BALL_SIZE >= SCREEN_HEIGHT - PADDLE_HEIGHT &&
        ballY < SCREEN_HEIGHT - PADDLE_HEIGHT &&
        ballX  + BALL_SIZE >= paddlePos &&
        ballX <= paddlePos + PADDLE_WIDTH && ballSpeedY > 0){
        ballSpeedY = -ballSpeedY;
        score++;
        playTone(1000, 100);

        if (score > highscore)
        {
            highscore = score;
            writeHighscoreToEEPROM(highscore);
        }
    }

    if(ballY >= SCREEN_HEIGHT) {
    	game = 0;
    	waitForRestart = 1;
    }
}

void resetGame() {
	game = 1;
	waitForRestart = 0;
	ballX = SCREEN_WIDTH / 2;
	ballY = 10;
	paddlePos = SCREEN_WIDTH / 2 - PADDLE_WIDTH / 2;
	score = 0;

	prevBallX = ballX;
	prevBallY = ballY;
	prevPaddlePos = paddlePos;
}

void play() {

	if (game == 1) {

		int oldBallX = ballX;
		int oldBallY = ballY;
		int oldPaddlePos = paddlePos;
		static int lastScore = -1;

		handleInput();
		updateGame();

		clearBall(oldBallX, oldBallY);
		clearPaddle(oldPaddlePos);

		drawPaddle(paddlePos);
		drawBall(ballX, ballY);
		drawTime(3, 90);
		drawScore(3, 110);
	    updateTimer();

	} else {
		if (waitForRestart == 1) {
			if (gameOverDisplayed == 0) {
				lcdClrscr();
				lcdGotoxy(3, 3);
				lcdPuts("GAME OVER");
				lcdGotoxy(3, 85);
				lcdPuts("Press UP");
				lcdGotoxy(3, 110);
				lcdPuts("to restart");
				drawTime(3, 21);
				drawScore(3, 41);
			    drawHighscore(3, 61);
				gameOverDisplayed = 1;
			}
			checkAccelerometer();

			if (isJoyPressed(JOY_UP)) {
				resetGame();
				lcdClrscr();
				gameOverDisplayed = 0;
				min = 0;
				sec = 0;
			}
		}
	}

	delayMs(75);
}

int main(void) {

    initJoy();
    lcdInit();
    lcdColor(0xff, 0x00);
    initAudio();
    initAdc();
    initI2C();

    highscore = readHighscoreFromEEPROM();

    lcdClrscr();
    lcdGotoxy(3, 3);
    lcdPuts("PONG GAME");
    lcdGotoxy(3, 21);
    lcdPuts("Use LEFT/RIGHT");
    delayMs(2000);
    lcdClrscr();

    while(1){
		play();
	}
}
