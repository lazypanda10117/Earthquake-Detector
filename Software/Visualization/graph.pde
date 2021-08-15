import processing.serial.*;
Serial myPort;  
//PShape square;  
String prev0 = "";
double prev1 = 0;
double prev2 = 0;
double prev3 = 0;
double prev4 = 0;
double prev5 = 0;
double prev6 = 0;
String p0 = "";
double p1 = 0;
double p2 = 0;
double p3 = 0;
double p4 = 0;
double p5 = 0;
double p6 = 0;
double segment = 150;
int widthL = 1280;
int lengthL = 720;
int count = 0;

int midA = (lengthL/4);
int maxA = 10;
int scaleA = midA/maxA;

int midG = 3*(lengthL/4);
int maxG = 8;
int scaleG = midA/maxA;

void setup(){
  //printArray(Serial.list());
  myPort = new Serial(this, Serial.list()[7], 115200);
  size(1280,720);
  background(255);
  stroke(150);
  strokeWeight(3);
  line(0,lengthL/2, widthL, lengthL/2);
  strokeWeight(1);
  line(0,lengthL/4, widthL, lengthL/4);  
  line(0,3*(lengthL/4), widthL, 3*(lengthL/4));  
  fill(0);
  textSize(12);
  text("Unix Time (s): ", 0, 20);
}

void draw(){
  while (myPort.available() > 0) {
    String inBuffer = "";
    char c = myPort.readChar();
    inBuffer += c;
    while(c!='}' && myPort.available() > 0){
      c = myPort.readChar();
      inBuffer += c;
    }
      try{
        if(inBuffer.charAt(0) == '{'){
          String tempS = inBuffer.substring(1,inBuffer.length()-1);
          String sa[] = split(tempS,",");
          p0 = sa[0];
          p1 = Double.parseDouble(sa[1]);
          p2 = Double.parseDouble(sa[2]);
          p3 = Double.parseDouble(sa[3]);
          p4 = Double.parseDouble(sa[4]);
          p5 = Double.parseDouble(sa[5]);
          p6 = Double.parseDouble(sa[6]);
          plotGraph(p0, p1,p2,p3,p4,p5,p6,count);
          prev0 = p0;
          prev1 = p1;
          prev2 = p2;
          prev3 = p3;
          prev4 = p4;
          prev5 = p5;
          prev6 = p6;
        }
      }catch(Exception e){
        plotGraph(prev0, prev1,prev2,prev3,prev4,prev5,prev6,count);
      }
      
      println(inBuffer);
    }
  if(count%2 == 0){
    noStroke();
    fill(255);
    rect(0 + textWidth("Unix Time (s): "),10,textWidth("1231231231231.14"),14);
  }
  if(count > segment){
    background(255);
    stroke(150);
    strokeWeight(3);
    line(0,lengthL/2, widthL, lengthL/2);
    strokeWeight(1);
    line(0,lengthL/4, widthL, lengthL/4);  
    line(0,3*(lengthL/4), widthL, 3*(lengthL/4));
    fill(0);
    textSize(12);
    text("Unix Time (s): ", 0, 20);
    count = 0;
  }
}

void plotGraph(String t, double a, double b, double c, double d, double e, double f, int g){

  fill(0);
  textSize(12);
  text(t,0 + textWidth("Unix Time (s): "), 20);
  //println(t);
  double x1 = (g-1)*((double)widthL/segment);
  double x2 = g*((double)widthL/segment);

  strokeWeight(1.2);
  //gyroscope
  stroke(204, 102, 0); //red
  line((float)x1,(float)(midA-scaleA*prev1),(float)x2,(float)(midA-scaleA*a));
  stroke(150, 230, 128); //green 
  line((float)x1,(float)(midA-scaleA*prev2),(float)x2,(float)(midA-scaleA*b));
  stroke(0, 143, 134); //blue
  line((float)x1,(float)(midA-scaleA*prev3),(float)x2,(float)(midA-scaleA*c));
  //accelerometer
  stroke(204, 102, 0); //red
  line((float)x1,(float)(midG-scaleG*prev4),(float)x2,(float)(midG-scaleG*d));
  stroke(150, 230, 128); //green
  line((float)x1,(float)(midG-scaleG*prev5),(float)x2,(float)(midG-scaleG*e));
  stroke(0, 143, 134); //blue
  line((float)x1,(float)(midG-scaleG*prev6),(float)x2,(float)(midG-scaleG*f));
  count++;
}