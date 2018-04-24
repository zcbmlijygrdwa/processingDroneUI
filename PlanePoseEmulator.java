import java.util.Random;

class PlanePoseEmulator {
  private static PlanePoseEmulator instance = null;

  float yaw;
  float offSetX;
  float offSetY;
Random rand;

  public static PlanePoseEmulator getInstance() {
    if (instance==null) {
      instance = new PlanePoseEmulator();
    } 
    return instance;
  }



  private PlanePoseEmulator() {
    yaw = 0.0f;
    offSetX = 0.0f;
    offSetY = 0.0f;
    rand = new Random();
  }
  
  
  public float nextRotation(){
    yaw+=(rand.nextFloat()-0.5f)*0.001f;
    return yaw;
  }
  
}

