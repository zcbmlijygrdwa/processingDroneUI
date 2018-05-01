import java.util.Random;

class DroneFlightEmulator {
  private static DroneFlightEmulator instance = null;

  float yaw;
  float locX;
  float locY;
  
  PIDController pid_locX;
  PIDController pid_locY;
  PIDController pid_yaw;
  
  
  Random rand;
  
  boolean isRandom;

  public static DroneFlightEmulator getInstance() {
    if (instance==null) {
      instance = new DroneFlightEmulator();
    } 
    return instance;
  }


public void setRandom(boolean b){
 isRandom = b; 
}

public void setTarget(float tX, float tY,float tYaw){
  pid_locX.setReference(tX);
  pid_locX.setReference(tY);
  pid_yaw.setReference(tYaw);
}

public void setTargetLocation(float tX, float tY){
  pid_locX.setReference(tX);
  pid_locX.setReference(tY);
}

public void setTarget(float tYaw){
  pid_yaw.setReference(tYaw);
}


  private DroneFlightEmulator() {
    yaw = 0.0f;
    locX = 0.0f;
    locY = 0.0f;
    isRandom = true;
    rand = new Random();
    
    pid_locX = new PIDController(1,0,0);
    pid_locY = new PIDController(1,0,0);
    pid_yaw = new PIDController(1,0,0);
    
  }
  
  
  public float nextRotation(float currentYaw){
    if(isRandom){
      yaw+=(rand.nextFloat()-0.5f)*0.05f;
    }
    else{
      if(pid_yaw.isOn()){
        yaw = pid_yaw.process(currentYaw);
      }
      else{
       yaw = currentYaw; 
      }
    }
    return yaw;
  }
  
    public float[] nextTranslation(float currentX, float currentY){
      if(isRandom){
        locX+=(rand.nextFloat()-0.5f)*1f;
        locY+=(rand.nextFloat()-0.5f)*1f;
      }
      
      else{
        if(pid_locX.isOn()){
          locX = pid_locX.process(currentX);
        }
        else{
          locX = currentX;
        }
        
        if(pid_locY.isOn()){
          locY = pid_locY.process(currentY);
        }
        else{
          locY = currentY;
        }
      }
    
    return new float[]{locX, locY};
  }
  
}
