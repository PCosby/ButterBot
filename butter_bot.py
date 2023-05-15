from butter_bot_setup import *
  
def Startup():
  PlayWhatIsMyPurpose()
  Catapult_Stop()
  Wheels_Stop()
  Blade_Retract()
  Push_Retract()
  time.sleep(5)
  PlayOmg()
  time.sleep(5)
  
  
  
def Cleanup():
  PlayNoFriendship()
  cv2.destroyAllWindows()
  vs.stop()
  
def main():
    
  for i in range(2):
    WaitUntilInPicture()
    print("found ya !!")
    PlayOmg()
    OrientRotate()
    MoveForwardUS()
    
    print(i, "cutting!")
    CutButter()
    PlayButter()
    
    print(i, "FIRE!!!")
    PlayButter()
    LaunchCatapult()
    PushButter()
    
    
    
    
  print("last 1")
  ResetPush()
  WaitUntilInPicture()
  print("found ya !!")
  PlayOmg()
  OrientRotate()
  MoveForwardUS()
  time.sleep(2)
  
  PlayButter()
  LaunchCatapult()
  print("FIRE")
  
  print(":)")
  
  
Startup()
main()
Cleanup()
    
  
  