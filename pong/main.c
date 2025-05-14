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

/*!
 *  @brief    Inicjalizuje piny joysticka jako wejscia
 *
 *  Funkcja konfiguruje piny P0.8 do P0.12 (srodek, lewo, gora, prawo, dol) jako wejscia cyfrowe
 *  poprzez wyczyszczenie odpowiednich bitow w rejestrze IODIR0.
 *
 *  @param    Brak
 *
 *  @returns  Brak
 *
 *  @side effects:
 *            Modyfikuje rejestr IODIR0
 */
void initJoy() {
    IODIR0 &= ~(JOY_CENTER | JOY_LEFT | JOY_UP | JOY_RIGHT | JOY_DOWN);
}

/*!
 *  @brief    Inicjalizuje interfejs I2C
 *
 *  Funkcja wywoluje i2cInit() z biblioteki i2c.h, aby skonfigurowac magistrale I2C
 *  (piny P0.2 jako SCL i P0.3 jako SDA) oraz ustawic parametry transmisji.
 *
 *  @param    Brak
 *
 *  @returns  Brak
 *
 *  @side effects:
 *            Modyfikuje rejestry zwiazane z I2C (np. PINSEL0, I2C_CONSET)
 */
void initI2C() {
    i2cInit();
}

tU16 getAnalogueInput(tU8 channel);
tS8 eepromPoll(void);
tS8 eepromWrite(tU16 addr, tU8* pData, tU16 len);
tS8 eepromPageRead(tU16 address, tU8* pBuf, tU16 len);

/*!
 *  @brief    Inicjalizuje wyjscie DAC do odtwarzania dzwieku
 *
 *  Funkcja konfiguruje pin P0.25 mikrokontrolera jako wyjscie analogowe
 *  polaczone z wewnetrznym przetwornikiem cyfrowo-analogowym (DAC).
 *  Modyfikuje rejestr PINSEL1 w celu ustawienia funkcji DAC dla odpowiedniego pinu
 *  oraz wlacza mozliwosc odtwarzania dzwieku przez ustawienie flagi audioEnabled.
 *
 *  @param    Brak
 *
 *  @returns  Brak
 *
 *  @side effects:
 *            Modyfikuje rejestr PINSEL1, aktywuje wyjscie DAC
 */
void initAudio() {
    PINSEL1 &= ~0x000C0000;
    PINSEL1 |= 0x00080000;
    audioEnabled = 1;
}

const tU16 sinTable16[] = {
    512, 768, 896, 960, 992, 1008, 1016, 1020,
    1023, 1020, 1016, 1008, 992, 960, 896, 768
};

/*!
 *  @brief    Odtwarza ton o okreslonej czestotliwosci i czasie trwania
 *
 *  @param    frequency
 *            Czestotliwosc tonu w Hz
 *  @param    durationMs
 *            Czas trwania tonu w milisekundach
 *
 *  @returns  Brak
 *
 *  @side effects:
 *            Modyfikuje rejestr DACR, generuje dzwiek, wprowadza opoznienie
 */
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

/*!
 *  @brief    Inicjuje i wykorzystuje TIMER1 do realizacji opoznienia w milisekundach
 *
 *  Funkcja konfiguruje TIMER1, ustawiajac rejestr T1TCR na reset, T1PR na 0,
 *  T1MR0 na wartosc odpowiadajaca opoznieniu w milisekundach, oraz wlacza timer.
 *
 *  @param    delayInMs
 *            Liczba milisekund do odczekania
 *
 *  @returns  Brak
 *
 *  @side effects:
 *            Zatrzymuje dzialanie programu na czas okreslony przez parametr.
 *            Modyfikuje rejestry T1TCR, T1PR, T1MR0, T1IR, T1MCR
 */
void delayMs(tU16 delayInMs) {
    T1TCR = 0x02;
    T1PR = 0x00;
    T1MR0 = delayInMs * (CORE_FREQ / (PBSD * 1000));
    T1IR = 0xff;
    T1MCR = 0x04;
    T1TCR = 0x01;

    while (T1TCR & 0x01);
}

/*!
 *  @brief    Inicjuje i wykorzystuje TIMER0 do realizacji opoznienia w mikrosekundach
 *
 *  Funkcja konfiguruje TIMER0, ustawiajac rejestr T0TCR na reset, T0PR na 0,
 *  T0MR0 na wartosc odpowiadajaca opoznieniu w mikrosekundach, oraz wlacza timer.
 *
 *  @param    delayInUs
 *            Liczba mikrosekund do odczekania
 *
 *  @returns  Brak
 *
 *  @side effects:
 *            Zatrzymuje dzialanie programu na czas okreslony przez parametr.
 *            Modyfikuje rejestry T0TCR, T0PR, T0MR0, T0IR, T0MCR
 */
void udelay(unsigned int delayInUs) {
    T0TCR = 0x02;
    T0PR = 0x00;
    T0MR0 = delayInUs * (CORE_FREQ / (PBSD * 1000000));
    T0IR = 0xff;
    T0MCR = 0x04;
    T0TCR = 0x01;

    while (T0TCR & 0x01);
}

/*!
 *  @brief    Sprawdza, czy dany kierunek joysticka jest wcisniety
 *
 *  Funkcja odczytuje stan pinu GPIO w rejestrze IOPIN0 i sprawdza, czy bit odpowiadajacy
 *  danemu kierunkowi joysticka (np. P0.9 dla lewo) jest w stanie niskim (wcisniety).
 *
 *  @param    direction
 *            Maska bitowa odpowiadajaca pinowi joysticka (np. JOY_LEFT)
 *
 *  @returns  1, jesli kierunek jest wcisniety, 0 w przeciwnym razie
 *
 *  @side effects:
 *            Brak
 */
int isJoyPressed(tU32 direction) {
    return !(IOPIN0 & direction);
}

/*!
 *  @brief    Rysuje paletke na ekranie LCD
 *
 *  Funkcja rysuje prostokat reprezentujacy paletke na ekranie LCD w zadanej pozycji poziomej
 *  i stalym polozeniu pionowym (dol ekranu). Uzywa funkcji lcdRect() do rysowania.
 *
 *  @param    position
 *            Wspolrzedna X (pozioma) pozycji startowej paletki
 *
 *  @returns  Brak
 *
 *  @side effects:
 *            Modyfikuje zawartosc ekranu LCD
 */
void drawPaddle(int position) {
    int y = SCREEN_HEIGHT - PADDLE_HEIGHT;
    lcdRect(position, y, PADDLE_WIDTH, PADDLE_HEIGHT, 1);
}

/*!
 *  @brief    Rysuje pileczke na ekranie LCD
 *
 *  Funkcja rysuje kwadrat reprezentujacy pileczke na ekranie LCD w zadanych wspolrzednych.
 *  Uzywa funkcji lcdRect() do rysowania.
 *
 *  @param    x
 *            Wspolrzedna X (pozioma) pozycji pileczki
 *  @param    y
 *            Wspolrzedna Y (pionowa) pozycji pileczki
 *
 *  @returns  Brak
 *
 *  @side effects:
 *            Modyfikuje zawartosc ekranu LCD
 */
void drawBall(int x, int y) {
    lcdRect(x, y, BALL_SIZE, BALL_SIZE, 1);
}

/*!
 *  @brief    Wyswietla aktualny wynik na ekranie LCD
 *
 *  Funkcja wyswietla etykiete "SCORE:" oraz aktualny wynik w zadanej pozycji na ekranie LCD.
 *  Wynik jest formatowany jako ciag znakow przy uzyciu sprintf().
 *
 *  @param    x
 *            Wspolrzedna X (pozioma) pozycji startowej
 *  @param    y
 *            Wspolrzedna Y (pionowa) pozycji startowej
 *
 *  @returns  Brak
 *
 *  @side effects:
 *            Modyfikuje zawartosc ekranu LCD, uzywa bufora scoreX
 */
void drawScore(int x, int y) {
    sprintf(scoreX, "%d", score);
    lcdGotoxy(x, y);
    lcdPuts("SCORE: ");
    lcdGotoxy(x+62, y);
    lcdPuts(scoreX);
}

/*!
 *  @brief    Wyswietla najlepszy wynik (high score) na ekranie LCD
 *
 *  Funkcja wyswietla etykiete "HIGH SCORE:" oraz najlepszy wynik w zadanej pozycji
 *  na ekranie LCD. Wynik jest formatowany jako ciag znakow przy uzyciu sprintf().
 *
 *  @param    x
 *            Wspolrzedna X (pozioma) pozycji startowej
 *  @param    y
 *            Wspolrzedna Y (pionowa) pozycji startowej
 *
 *  @returns  Brak
 *
 *  @side effects:
 *            Modyfikuje zawartosc ekranu LCD, uzywa lokalnego bufora highscoreStr
 */
void drawHighscore(int x, int y) {
    char highscoreStr[10];
    sprintf(highscoreStr, "%d", highscore);
    lcdGotoxy(x, y);
    lcdPuts("HIGH SCORE:      ");
    lcdGotoxy(x+100, y);
    lcdPuts(highscoreStr);
}

/*!
 *  @brief    Wyswietla aktualny czas na ekranie LCD
 *
 *  Funkcja wyswietla etykiete "TIME:" oraz aktualny czas w formacie MM:SS
 *  w zadanej pozycji na ekranie LCD, uzywa funkcji lcdGotoxy() do ustawienia
 *  kursora oraz lcdPuts() do wyswietlania tekstu, liczba minut i sekund pobierana
 *  jest z globalnych zmiennych min i sec. Formatowanie tekstu odbywa sie przy
 *  uzyciu funkcji sprintf().
 *
 *  @param    x
 *            Wspolrzedna X (pozioma) pozycji startowej na ekranie LCD
 *  @param    y
 *            Wspolrzedna Y (pionowa) pozycji startowej na ekranie LCD
 *
 *  @returns  Brak
 *
 *  @side effects:
 *            Modyfikuje zawartosc ekranu LCD, uzywa bufora tekstowego timer
 */
void drawTime(int x, int y) {
    lcdGotoxy(x, y);
    lcdPuts("TIME:");
    lcdGotoxy(x+62, y);
    sprintf(timer, "%02d:%02d", min, sec);
    lcdPuts(timer);
}

/*!
 *  @brief    Usuwa pileczke z ekranu LCD
 *
 *  Funkcja usuwa kwadrat reprezentujacy pileczke z ekranu LCD w zadanych wspolrzednych
 *  poprzez wypelnienie obszaru kolorem tla przy uzyciu funkcji lcdRect().
 *
 *  @param    x
 *            Wspolrzedna X (pozioma) pozycji pileczki
 *  @param    y
 *            Wspolrzedna Y (pionowa) pozycji pileczki
 *
 *  @returns  Brak
 *
 *  @side effects:
 *            Modyfikuje zawartosc ekranu LCD
 */
void clearBall(int x, int y) {
    lcdRect(x, y, BALL_SIZE, BALL_SIZE, 0xff);
}

/*!
 *  @brief    Usuwa paletke z ekranu LCD
 *
 *  Funkcja usuwa prostokat reprezentujacy paletke z ekranu LCD w zadanej pozycji poziomej
 *  i stalym polozeniu pionowym (dol ekranu) poprzez wypelnienie obszaru kolorem tla.
 *
 *  @param    position
 *            Wspolrzedna X (pozioma) pozycji startowej paletki
 *
 *  @returns  Brak
 *
 *  @side effects:
 *            Modyfikuje zawartosc ekranu LCD
 */
void clearPaddle(int position) {
    int y = SCREEN_HEIGHT - PADDLE_HEIGHT;
    lcdRect(position, y, PADDLE_WIDTH, PADDLE_HEIGHT, 0xff);
}

/*!
 *  @brief    Aktualizuje licznik czasu
 *
 *  Funkcja inkrementuje licznik milisekund i przelicza go na sekundy i minuty.
 *  Gdy licznik milisekund osiaga wartosc 12, zeruje go i zwieksza licznik sekund.
 *  Gdy licznik sekund osiaga 60, zeruje go i zwieksza licznik minut.
 *
 *  @param    Brak
 *
 *  @returns  Brak
 *
 *  @side effects:
 *            Modyfikuje globalne zmienne: msec, sec, min
 */
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

/*!
 *  @brief    Odczytuje aktualny czas z timera
 *
 *  Funkcja zwraca aktualna wartosc licznika timera T0TC, reprezentujaca czas
 *  w milisekundach od uruchomienia timera.
 *
 *  @param    Brak
 *
 *  @returns  Wartosc rejestru T0TC (czas w milisekundach)
 *
 *  @side effects:
 *            Brak
 */
uint32_t getMillis() {
    return T0TC;
}

/*!
 *  @brief    Obsluguje wejscie gracza z joysticka
 *
 *  Funkcja sprawdza stan joysticka (piny P0.9 dla lewo, P0.11 dla prawo) i aktualizuje
 *  pozycje paletki, przesuwajac ja w lewo lub prawo, jesli nie wychodzi poza granice ekranu.
 *
 *  @param    Brak
 *
 *  @returns  Brak
 *
 *  @side effects:
 *            Modyfikuje globalna zmienna paddlePos
 */
void handleInput() {
    if(isJoyPressed(JOY_LEFT) && paddlePos > 0)
        paddlePos -= 5;
    if(isJoyPressed(JOY_RIGHT) && paddlePos < SCREEN_WIDTH - PADDLE_WIDTH)
        paddlePos += 5;
}

/*!
 *  @brief    Sprawdza akcelerometr w celu zresetowania najlepszego wyniku
 *
 *  Funkcja odczytuje wartosc z akcelerometru, porownuje ja z poprzednia wartoscia
 *  i resetuje najlepszy wynik (highscore), jesli roznica przekracza prog (SHAKE_THRESHOLD).
 *  Po resecie aktualizuje wyswietlanie wyniku na ekranie LCD.
 *
 *  @param    Brak
 *
 *  @returns  Brak
 *
 *  @side effects:
 *            Modyfikuje globalne zmienne: highscore, prevAccelValue
 *            Modyfikuje zawartosc ekranu LCD
 *            Wywoluje zapis do EEPROM
 */
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

/*!
 *  @brief    Zapisuje najlepszy wynik do pamieci EEPROM
 *
 *  Funkcja zapisuje 16-bitowy wynik (highscore) do pamieci EEPROM pod adresem HIGHSCORE_ADDR.
 *  Wynik jest dzielony na dwa bajty i zapisywany za pomoca funkcji eepromWrite().
 *  W przypadku bledu zapisu wyswietla komunikat na ekranie LCD.
 *
 *  @param    highscore
 *            Najlepszy wynik do zapisania
 *
 *  @returns  Brak
 *
 *  @side effects:
 *            Modyfikuje pamiec EEPROM
 *            Modyfikuje zawartosc ekranu LCD w przypadku bledu
 *            Wywoluje opoznienie w przypadku bledu
 */
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

/*!
 *  @brief    Odczytuje najlepszy wynik z pamieci EEPROM
 *
 *  Funkcja odczytuje 16-bitowy wynik (highscore) z pamieci EEPROM pod adresem HIGHSCORE_ADDR.
 *  Odczytane dwa bajty sa laczone w jedna wartosc. W przypadku bledu odczytu zwraca 0.
 *
 *  @param    Brak
 *
 *  @returns  Odczytany najlepszy wynik lub 0 w przypadku bledu
 *
 *  @side effects:
 *            Brak
 */
int readHighscoreFromEEPROM() {
    tU8 data[2] = {0, 0};
    if (eepromPageRead(HIGHSCORE_ADDR, data, 2) == I2C_CODE_OK) {
        int hs = (data[0] << 8) | data[1];
        return hs;
    }
    return 0;
}

/*!
 *  @brief    Aktualizuje stan gry
 *
 *  Funkcja aktualizuje pozycje pileczki, sprawdza kolizje z krawedziami ekranu i paletka,
 *  zwieksza wynik przy odbiciu, odtwarza dzwiek i zapisuje nowy najlepszy wynik do EEPROM,
 *  jesli osiagnieto wyzszy wynik. Jesli pileczka wypadnie poza ekran, konczy gre.
 *
 *  @param    Brak
 *
 *  @returns  Brak
 *
 *  @side effects:
 *            Modyfikuje globalne zmienne: ballX, ballY, ballSpeedX, ballSpeedY, score,
 *            highscore, game, waitForRestart
 *            Modyfikuje pamiec EEPROM przy zapisie nowego wyniku
 *            Wywoluje odtwarzanie dzwieku
 */
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

    if(ballY + BALL_SIZE >= SCREEN_HEIGHT - PADDLE_HEIGHT &&
       ballY < SCREEN_HEIGHT - PADDLE_HEIGHT &&
       ballX + BALL_SIZE >= paddlePos &&
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

/*!
 *  @brief    Resetuje stan gry do wartosci poczatkowych
 *
 *  Funkcja ustawia gre w stan poczatkowy przygotowujac ja do nowego uruchomienia.
 *  Ustawia flage gry na aktywna, resetuje flage oczekiwania na restart,
 *  ustawia pileczke na srodku szerokosci ekranu i na pozycji poczatkowej w pionie.
 *  Paletka zostaje ustawiona na srodku ekranu. Wynik zostaje zresetowany do zera.
 *
 *  @param    Brak
 *
 *  @returns  Brak
 *
 *  @side effects:
 *            Modyfikuje globalne zmienne: game, waitForRestart, ballX, ballY, paddlePos, score,
 *            prevBallX, prevBallY, prevPaddlePos
 */
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

/*!
 *  @brief    Obsluguje glowna logike gry
 *
 *  Funkcja zarzadza rozgrywka: aktualizuje stan gry, rysuje elementy graficzne,
 *  obsluguje wejscie gracza i wyswietla ekran konca gry. Jesli gra jest aktywna,
 *  aktualizuje pozycje pileczki i paletki, wyswietla czas i wynik. Po przegranej
 *  wyswietla ekran "GAME OVER" i pozwala na restart lub reset wyniku.
 *
 *  @param    Brak
 *
 *  @returns  Brak
 *
 *  @side effects:
 *            Modyfikuje globalne zmienne: ballX, ballY, paddlePos, score, min, sec,
 *            gameOverDisplayed, highscore
 *            Modyfikuje zawartosc ekranu LCD
 *            Wywoluje opoznienie (75 ms)
 */
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

/*!
 *  @brief    Glowna funkcja programu
 *
 *  Funkcja inicjalizuje peryferia (joystick, LCD, dzwiek, ADC, I2C), odczytuje
 *  najlepszy wynik z EEPROM, wyswietla ekran powitalny i wchodzi w nieskonczona petle,
 *  w ktorej wywoluje funkcje play() do obslugi gry.
 *
 *  @param    Brak
 *
 *  @returns  Brak (funkcja nigdy nie konczy dzialania)
 *
 *  @side effects:
 *            Inicjalizuje wszystkie peryferia
 *            Modyfikuje zawartosc ekranu LCD
 *            Odczytuje i modyfikuje pamiec EEPROM
 *            Wywoluje opoznienie (2000 ms na ekranie powitalnym)
 */
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