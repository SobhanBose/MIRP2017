void updateBallVelocity() {
  ballVx+=0.001;
}

void updateBallPosition() {
  ballX += ballVx;
  ballY += ballVy;
}

void updatePaddlePositions()
{
  if(left_up==true)
  leftPaddle+=-10;
  if(left_down==true)
  leftPaddle+=10;
  if(right_up==true)
  rightPaddle+=-10;
  if(right_down==true)
  rightPaddle+=10;
  if(leftPaddle>displayHeight-halflp)
  leftPaddle=displayHeight-halflp;
  if(rightPaddle>displayHeight-halfrp)
  rightPaddle=displayHeight-halfrp;
  if(leftPaddle-paddleLength/2<=0)
  leftPaddle=0+paddleLength/2;
  if(rightPaddle-paddleLength/2<=0)
  rightPaddle=0+paddleLength/2;  // Based on the keys pressedd, move the paddles (update Y position)
  // Make sure the paddles don't leave the screen
}

void Bounce()
{
if(ballY>displayHeight-ballRadius)
{
  ballY=displayHeight-ballRadius;
  ballVy*=-coeff;
}
if(ballX>displayWidth-ballRadius)
{
  ballX=displayWidth-ballRadius;
  ballVx*=-coeff;
}
if(ballY<=0)
{
  ballVy*=-coeff;
  ballY+=ballRadius;
}
if(ballX<=0)
{
  ballVx*=-coeff;
  ballX+=ballRadius;
}
}

void ballpaddlebounce()
{
if(ballX>displayWidth-(paddleWidth+ballRadius) && (ballY>=(rightPaddle-paddleLength/2)) && (ballY<=rightPaddle+paddleLength/2)) 
{
ballX=displayWidth-(paddleWidth+ballRadius);
ballVx *= -coeff;
ballVy=BALL_VELOCITY*(ballY-rightPaddle)/buffer;
}
if(ballX<ballRadius+paddleWidth && (ballY>=(leftPaddle-paddleLength/2)) && ballY<=leftPaddle+paddleLength/2) 
{
ballX=ballRadius+paddleWidth;
ballVx *= -coeff;
ballVy=BALL_VELOCITY*(ballY-leftPaddle)/buffer;
}
}