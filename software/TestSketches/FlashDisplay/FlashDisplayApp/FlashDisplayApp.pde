import processing.serial.*;

Serial myPort;
int pos = 0;
int blockNum = 0;
int scale = 768;
int sizeFactor = scale/256;
int blockSize = 256*256*sizeFactor;
int startAddr = 0;

void setup() {
  size(768, 768);
  noStroke();
  background(32);
  frameRate(1000);
  myPort = new Serial(this, "COM6", 230400);
  myPort.write(1);
}

void draw() {
  if(myPort.available() > 0) {
    int raw = myPort.read();
    float r = map(((raw & 0xE0) >> 5), 0, 7, 0, 255);
    float g = map(((raw & 0x1C) >> 2), 0, 7, 0, 255);
    float b = map((raw & 0x02), 0, 3, 0, 255);
    fill(color(r, g, b));
    rect(pos % scale, sizeFactor * (pos / scale), sizeFactor, sizeFactor);
    pos+=sizeFactor;
    
    if(pos == blockSize) {
      pos = 0;
      int blockAddr = startAddr + blockNum*0x00010000;
      String name = String.format("%08X", blockAddr);
      println(name);
      save(name + ".png");
      blockNum++;
      background(32);
    }
  }
  
}
