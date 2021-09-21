void rightLose() 
{
   if(ballX+ballRadius>=displayWidth)
   leftscore++;
}

void leftLose() 
{
   if(ballX<=0)
   rightscore++;
}