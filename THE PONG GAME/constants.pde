// Display Size
int displayWidth=500, displayHeight=500;

// Velocity Cnstants
float BALL_VELOCITY = 7; //This is the MAX speed of the ball.
float PADDLE_VELOCITY = 10;

// Background Color
color bgColor = color(13, 23, 26);

// Ball parameters
float ballX, ballY;
float ballVx, ballVy; // ballVx is always BALL_VELOCTY or -BALL_VELOCITY; ballVy varies.
float ballRadius = 10;
color ballColor = color(255,100,0);

// Y - position of Left and Right paddles
float leftPaddle=displayHeight/2, rightPaddle=displayHeight/2;
float halflp=61;
float halfrp=61;
// Paddle Dimensions
float paddleLength = 126, paddleWidth = 15;

// Score Variables
int leftScore, rightScore;

// Controls for the Left Paddle
char LEFT_UP = 'q', LEFT_DOWN = 'a';

// Controls for the Right Paddle
char RIGHT_UP = 'o', RIGHT_DOWN = 'l';

// Game Controls
char RESET = 'r', PAUSE = 'p', START = 's';

// Keyoard Handling Booleans
boolean left_up, right_up, left_down, right_down;
boolean reset, pause, start;

// Colours
color paddlecolor1=color(0, 100, 255);
color paddlecolor2=color(0, 255, 100);
float coeff=1;

float buffer=paddleLength/2;

int leftscore=0;
int rightscore=0;

int gamestate=0;

color textcolor=color(243,234,21);