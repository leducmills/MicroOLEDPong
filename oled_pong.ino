#include "application.h"
#include "SFE_MicroOLED.h"  // Include the SFE_MicroOLED library
#include "math.h"


//////////////////////////
// MicroOLED Definition //
//////////////////////////

// // Also connect pin 13 to SCK and pin 11 to MOSI
#define PIN_RESET D7  // Connect RST to pin 9 (req. for SPI and I2C)
#define PIN_DC    D3  // Connect DC to pin 8 (required for SPI)
#define PIN_CS    A2 // Connect CS to pin 10 (required for SPI)
MicroOLED oled(MODE_SPI, PIN_RESET, PIN_DC, PIN_CS);

//#define SINGLE_PLAYER

const int player1Pin = A1;
#ifndef SINGLE_PLAYER
const int player2Pin = A0;
#endif

const float sensorMaxValue = 1024.0;
int sensor1Calibration = LCDHEIGHT*2; //photocell w/330
int sensor2Calibration = LCDHEIGHT/4; //potentiometer

const int renderDelay = 16;
const int startDelay = 4000;
const int gameOverDelay = 6000;

const int scoreToWin = 3;

int player1Score = 0;
int player2Score = 0;

const float paddleWidth = LCDWIDTH / 16.0;
const float paddleHeight = LCDHEIGHT / 3.0;
const float halfPaddleWidth = paddleWidth / 2.0;
const float halfPaddleHeight = paddleHeight / 2.0;

float player1PosX = 1.0 + halfPaddleWidth;
float player1PosY = 0.0;
float player2PosX = LCDWIDTH - 1.0 - halfPaddleWidth;
float player2PosY = 0.0;

// This is only used in SINGLE_PLAYER mode:
#ifdef SINGLE_PLAYER
float enemyVelY = 0.5;
#endif

const float ballRadius = 2.0;
const float ballSpeedX = 1.0;
float ballPosX = LCDWIDTH / 2.0;
float ballPosY = LCDHEIGHT / 2.0;
float ballVelX = -1.0 * ballSpeedX;
float ballVelY = 0;

void setup()
{
  initializeGraphics();
  initializeInput();
  displayGameStart();
}


void loop()
{
  updateGame();
  renderGame();

  if (player1Score >= scoreToWin)
  {
    gameOver(true);
  }
  else if (player2Score >= scoreToWin)
  {
    gameOver(false);
  }
}

void initializeGraphics()
{
  oled.begin();
  oled.setFontType(1);
}

void initializeInput()
{
  pinMode(player1Pin, INPUT);
  #ifndef SINGLE_PLAYER
  pinMode(player2Pin, INPUT);
  #endif
}

void displayGameStart()
{
  oled.clear(PAGE);
  renderString(20, 10, "Get");
  renderString(10, 30, "Ready!");
  oled.display();
  delay(startDelay);
}

void updateGame()
{
  updatePlayer1();
  updatePlayer2();
  updateBall();
}



void resetGame()
{
  player2Score = 0;
  player1Score = 0;
  player2PosY = 0.0;
  ballPosX = LCDWIDTH / 2.0;
  ballPosY = LCDHEIGHT / 2.0;
  ballVelX = -1.0 * ballSpeedX;
  ballVelY = 0.0;
}


float clampPaddlePosY(float paddlePosY)
{
  float newPaddlePosY = paddlePosY;

  if (paddlePosY - halfPaddleHeight < 0)
  {
    newPaddlePosY = halfPaddleHeight;
  }
  else if (paddlePosY + halfPaddleHeight > LCDHEIGHT)
  {
    newPaddlePosY = LCDHEIGHT - halfPaddleHeight;
  }

  return newPaddlePosY;
}

void updatePlayer1()
{
  int potVal = analogRead(player1Pin);
  player1PosY = map(potVal, 0, 1023, 0, sensor1Calibration);
}

void updatePlayer2()
{
  // If it's a single player game, update the AI's position
#ifdef SINGLE_PLAYER
  // Follow the ball at a set speed
  if (player2PosY < ballPosY)
  {
    player2PosY += enemyVelY;
  }
  else if (player2PosY > ballPosY)
  {
    player2PosY -= enemyVelY;
  }

  player2PosY = clampPaddlePosY(player2PosY);
#else  // Else if this is multiplayer, get player 2's position
  int lightVal = analogRead(player2Pin);
  player2PosY = map(lightVal, 0, 1023, 0, sensor2Calibration);
#endif
}

void updateBall()
{
  ballPosY += ballVelY;
  ballPosX += ballVelX;

  // Top and bottom wall collisions
  if (ballPosY < ballRadius)
  {
    ballPosY = ballRadius;
    ballVelY *= -1.0;
  }
  else if (ballPosY > LCDHEIGHT - ballRadius)
  {
    ballPosY = LCDHEIGHT - ballRadius;
    ballVelY *= -1.0;
  }

  // Left and right wall collisions
  if (ballPosX < ballRadius)
  {
    ballPosX = ballRadius;
    ballVelX = ballSpeedX;
    player2Score++;
  }
  else if (ballPosX > LCDWIDTH - ballRadius)
  {
    ballPosX = LCDWIDTH - ballRadius;
    ballVelX *= -1.0 * ballSpeedX;
    player1Score++;
  }

  // Paddle collisions
  if (ballPosX < player1PosX + ballRadius + halfPaddleWidth)
  {
    if (ballPosY > player1PosY - halfPaddleHeight - ballRadius &&
        ballPosY < player1PosY + halfPaddleHeight + ballRadius)
    {
      ballVelX = ballSpeedX;
      ballVelY = 2.0 * (ballPosY - player1PosY) / halfPaddleHeight;
    }
  }
  else if (ballPosX > player2PosX - ballRadius - halfPaddleWidth)
  {
    if (ballPosY > player2PosY - halfPaddleHeight - ballRadius &&
        ballPosY < player2PosY + halfPaddleHeight + ballRadius)
    {
      ballVelX = -1.0 * ballSpeedX;
      ballVelY = 2.0 * (ballPosY - player2PosY) / halfPaddleHeight;
    }
  }
}


void renderGame()
{
  oled.clear(PAGE);

  renderScores(player1Score, player2Score);
  renderPaddle(player1PosX, player1PosY);
  renderPaddle(player2PosX, player2PosY);
  renderBall(ballPosX, ballPosY);

  oled.display();
  delay(renderDelay);
}

void renderString(int x, int y, String string)
{
  oled.setCursor(x, y);
  oled.print(string);
}

void renderPaddle(int x, int y)
{
  oled.rect(
    x - halfPaddleWidth,
    y - halfPaddleHeight,
    paddleWidth,
    paddleHeight);
}

void renderBall(int x, int y)
{
  oled.circle(x, y, 2);
}

void renderScores(int firstScore, int secondScore)
{
  renderString(10, 0, String(firstScore));
  renderString(LCDWIDTH - 14, 0, String(secondScore));
}

void gameOver(bool didWin)
{
  if (didWin)
  {
#ifdef SINGLE_PLAYER
    renderString(20, 10, "You");
    renderString(20, 30, "Win!");
#else
    renderString(0, 10, "Playr 1");
    renderString(15, 30, "Wins");
#endif
  }
  else
  {
#ifdef SINGLE_PLAYER
    renderString(20, 10, "You");
    renderString(15, 30, "Lose!");
#else
    renderString(0, 10, "Playr 2");
    renderString(15, 30, "Wins");
#endif
  }

  oled.display();
  delay(gameOverDelay);

  // Get ready to start the game again.
  resetGame();
  displayGameStart();
}
