import processing.serial.*;
Serial myPort;  
double p1 = 0;
double p2 = 0;
double p3 = 0;
double segment = 100;
int widthL = 800;
int lengthL = 600;
int count = 0;
void setup(){
  //printArray(Serial.list());
  myPort = new Serial(this, Serial.list()[7], 115200);
  size(800,600);
  background(255);
  line(0,lengthL/2, widthL, lengthL/2);
}

void draw(){
  //println("a");
  while (myPort.available() > 0) {
    String inBuffer = myPort.readString();   
    if (inBuffer != null) {
      try{
        String tempS = inBuffer.substring(1,inBuffer.length()-1);
        String sa[] = split(tempS,",");
        plotGraph(Double.parseDouble(sa[1]),Double.parseDouble(sa[2]), Double.parseDouble(sa[3]), count);
        p1 = Double.parseDouble(sa[1]);
        p2 = Double.parseDouble(sa[2]);
        p3 = Double.parseDouble(sa[3]);
      }catch(Exception e){
        plotGraph(p1, p2, p3, count);
      }
      println(inBuffer);
    }
  }
  if(count > segment){
    background(255); 
    line(0,lengthL/2, widthL, lengthL/2);
    count = 0;
  }
  count ++; 
}

void plotGraph(double a, double b, double c, int d){
  double x1 = (d-1)*((double)widthL/segment);
  double x2 = d*((double)widthL/segment);
  line((float)x1,(float)(lengthL-30*p1),(float)x2,(float)(lengthL-30*a));
  line((float)x1,(float)(lengthL-30*p2),(float)x2,(float)(lengthL-30*b));
  line((float)x1,(float)(lengthL-30*p3),(float)x2,(float)(lengthL-30*c));
}