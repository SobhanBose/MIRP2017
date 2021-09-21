void setup()
{
  size(displayWidth, displayHeight);
  resetGame();
  textFont(createFont("Arial Bold", 50));
  ballX=width/2;
  ballY=height/2;
  ballVx=BALL_VELOCITY;
  ballVy=1;
}

void draw()
{
  if(gamestate==0)
      startscreen();
  if(gamestate==1)
      drawGameScreen();
  if(gamestate==2)
      pausegame();
}

void drawGameScreen()
{
  background(bgColor);
  updateBallVelocity();
  Bounce();
  updateBallPosition();
  updatePaddlePositions();
  drawBall();
  drawPaddles();
  ballpaddlebounce();
  leftLose();
  rightLose();
  fill(paddlecolor1);
  textSize(50);
  text(leftscore, 150, 123);
  fill(paddlecolor2);
  textSize(50);
  text(rightscore, 300, 123);  
}

void drawBall() 
{

  fill(ballColor);
  ellipse(ballX, ballY, 2*ballRadius, 2*ballRadius);
}

void drawPaddles() 
{
  fill(paddlecolor1);
  rectMode(CENTER);
  rect(10, leftPaddle, paddleWidth, paddleLength, 0, 30, 30, 0);
  fill(paddlecolor2);
  rectMode(CENTER);
  rect(displayWidth-10, rightPaddle, paddleWidth, paddleLength, 30, 0, 0, 30);
}

void pausegame()
{
  fill(textcolor);
  textSize(50);
  text("PAUSED", width/2-100, height/2);
  textSize(18);
  text("Press 's' to start", width/2-70, height/2+50);
}

void startscreen()
{
  background(bgColor);
  fill(textcolor);
  textSize(72);
  text("PONG", width/2-100, height/2);
  textSize(12);
  text("Press 's' to start" , width/2-45, height/2+50);
  leftscore=0;
  rightscore=0;
  ballX=width/2;
  ballY=height/2;
  leftPaddle=displayHeight/2;
  rightPaddle=displayWidth/2;
  ballVx=BALL_VELOCITY;
  ballVy=1;
}

void resetGame()
{
  
}