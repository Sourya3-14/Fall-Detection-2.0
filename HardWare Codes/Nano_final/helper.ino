int decision2(float hrv, float spo2, float acc,float angle){
  hrv = (hrv - 95.657002) / 17.576499;
  spo2 = (spo2 - 83.563649) / 11.111592;
  acc = (acc - 0.661599) / 0.473282;
  if (spo2 <= -0.323044 ) 
    return 2;
  else{
    if (acc <= -0.341527)
      return 0;
    else if(angle == 1)
      return 2;
    else 
      return 1;
  }
}