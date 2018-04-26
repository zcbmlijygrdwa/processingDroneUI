import peasy.*;
import peasy.org.apache.commons.math.*;
import peasy.org.apache.commons.math.geometry.*;

import java.io.*;
import java.util.Iterator; 
import javax.imageio.ImageIO;

//import java.util.Base64;
import ros.Publisher;
import ros.RosBridge;
import ros.RosListenDelegate;
import ros.SubscriptionRequestMsg;
import ros.msgs.std_msgs.PrimitiveMsg;
import ros.tools.MessageUnpacker;

import com.fasterxml.jackson.databind.JsonNode;

import java.awt.image.BufferedImage;


/**
 * imports for unfolding map
 * An application with a basic interactive map. You can zoom and pan the map.
 */

import de.fhpotsdam.unfolding.*;
import de.fhpotsdam.unfolding.geo.*;
import de.fhpotsdam.unfolding.utils.*;
import de.fhpotsdam.unfolding.providers.*;



//split screen
PGraphics skeletonModel;
PGraphics videoStream;
PGraphics map;

int skeletonModel_w = 640;
int skeletonModel_h = 800;
int videoStream_w = 640;
int videoStream_h = 400;
int map_w = 640;
int map_h = 400;




PVector gridPos = new PVector(0, 0, 0);//the position of the grid (it moves with the camera to make it look infinite)

PeasyCam cam;
float globalPtich = 0;
float globalYaw = 0;

float drawingScale = 50;

int a = 1;

PImage img;
ArrayList<Float> dataArray = new ArrayList<Float>();
void setup() {

  // need to run: roslaunch rosbridge_server rosbridge_websocket.launch 

  size(1280, 800, OPENGL);


  skeletonModel = createGraphics(skeletonModel_w, skeletonModel_h, OPENGL);
  videoStream = createGraphics(videoStream_w, videoStream_h, P2D);
  map = createGraphics(map_w, map_h, P2D);


  img = loadImage("map.png");



  cam = new PeasyCam(this, skeletonModel, 800);
  cam.setMinimumDistance(360);
  cam.setLeftDragHandler(new PeasyDragHandler() {
    public void handleDrag(final double dx, final double dy) {
      if (globalPtich+dy*0.01>=0&&globalPtich+dy*0.01<=PI/2.0) {
        globalPtich+=dy*0.008;
      }
      globalYaw+=dx*0.008;
    }
  }
  );


  int trajectoryPlanningSampleRate = 100; //Hz

  println(a); 

  Publisher p;

  PrimitiveMsg pp;


  String URI = "ws://localhost:9090";

  RosBridge bridge = new RosBridge();
  bridge.connect(URI, true);

  bridge.subscribe(SubscriptionRequestMsg.generate("/xyzr")
    .setType("std_msgs/Float32MultiArray")
    .setFragmentSize(50000)
    .setThrottleRate(1)
    .setQueueLength(1), 
  new RosListenDelegate() {

    public void receive(JsonNode data, String stringRep) {
      ArrayNode slaidsNode = (ArrayNode)  data.get("msg").get("data");
      Iterator<JsonNode> slaidsIterator = slaidsNode.elements();
      dataArray.clear();
      while (slaidsIterator.hasNext ()) {
        JsonNode slaidNode = slaidsIterator.next();
        dataArray.add(slaidNode.floatValue());

      }
      System.out.println("dataArray.size() = "+dataArray.size());
    }
  }
  );
}


void draw() {
  //skeleton model
  skeletonModel.beginDraw();
  skeletonModel.background(0);
  skeletonModel.rotateX(PI-globalPtich);
  skeletonModel.rotateY(PI-globalYaw);

  skeletonModel.strokeWeight(1);
  skeletonModel.pushMatrix();
  skeletonModel.translate(gridPos.x, gridPos.y, gridPos.z);//everything that apears to be folowing the the center of the grid, goes after here
  rectGrid(25, (int)(1*drawingScale), 0);//a rectangle grid can be a lot bigger than a boxgrid, without caursing lag


  skeletonModel.popMatrix();
  skeletonModel.scale(drawingScale);
  skeletonModel.stroke(0);
  skeletonModel.strokeWeight(1/drawingScale);
  skeletonModel.fill(255);
  skeletonModel.box(1);
  if (dataArray!=null&&dataArray.size()!=0) {
    for (int i = 4; i < dataArray.size (); i+=4) {
      skeletonModel.stroke(255);

      skeletonModel.line(dataArray.get(i-4), dataArray.get(i+2-4), dataArray.get(i+1-4), dataArray.get(i), dataArray.get(i+2), dataArray.get(i+1));
    }

    skeletonModel.noStroke();

    int index = ((frameCount*10)%dataArray.size())/4;
    if (index*4+2<dataArray.size()) {
      skeletonModel.pushMatrix();
      //println("index = "+index);
      //System.out.println(dataArray.get(index*4)+","+ dataArray.get(index*4+1)+","+ dataArray.get(index*4+2));

      skeletonModel.translate(dataArray.get(index*4), dataArray.get(index*4+2), dataArray.get(index*4+1));
      float yaw = dataArray.get(index+3);
      skeletonModel.rotateY(yaw);
      skeletonModel.fill(map(index*4, 0, dataArray.size(), 255, 50), 255, map(index*4, 0, dataArray.size(), 50, 255));
      skeletonModel.box(0.1);
      skeletonModel.popMatrix();
    }
  }

  skeletonModel.endDraw();





  //video stream

  videoStream.beginDraw();
  videoStream.image(img, 0, 0, videoStream_w, videoStream_h);
  videoStream.endDraw();





  //map
  map.beginDraw();
  map.image(img, 0, 0, map_w, map_h);
  map.endDraw();



  image(skeletonModel, 0, 0);
  image(videoStream, skeletonModel_w, 0);
  image(map, skeletonModel_w, videoStream_h);
}

void rectGrid(int size, int tilesize, float y) {
  skeletonModel.noFill();//i only want the outline of the rectangles
  for (float x = -size/2; x <= size/2; x++) {
    for (float z = -size/2; z <= size/2; z++) {
      //run two for loops, cycling through 10 different positions of rectangles
      skeletonModel.pushMatrix();

      skeletonModel.stroke(0, 255, 0, map(dist(-gridPos.x, -gridPos.z, x*tilesize, z*tilesize), 0, size/2*tilesize, 255, 0));//the rectangles close to you, are clear, while the ones farther from you, are much fainter
      //uncomment the next line:
      //stroke(0,255,0);
      // to see how the infinity thing works

      skeletonModel.translate(x*tilesize, y, z*tilesize);//move the rectangles to where they shall be
      skeletonModel.rotateX(HALF_PI);
      skeletonModel.rect(0, 0, tilesize, tilesize);
      skeletonModel.popMatrix();
    }
  }
}
