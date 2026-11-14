PROGRAM main
VAR
  Start_PB      AT %IX0.0 : BOOL;
  Reset_PB      AT %IX0.1 : BOOL;
  EStop_OK      AT %IX0.2 : BOOL;
  Box_Sensor    AT %IX0.3 : BOOL;
  Robot_Done_In AT %QX1.0 : BOOL;
  Conveyor      AT %QX0.0 : BOOL;
  Robot_Pick    AT %QX0.1 : BOOL;
  Alarm_Lamp    AT %QX0.2 : BOOL;
  Full_Lamp     AT %QX0.3 : BOOL;
  Run_Lamp      AT %QX0.4 : BOOL;
  State      : INT := 0;
  BoxCount   : INT := 0;
  FaultCode  : INT := 0;
  Use_Sim    : BOOL := FALSE;
  Robot_Done : BOOL;
  StartEdge  : R_TRIG;
  ResetEdge  : R_TRIG;
  tNoBox     : TON;
  tRobot     : TON;
  tSim       : TON;
END_VAR

StartEdge(CLK := Start_PB);
ResetEdge(CLK := Reset_PB);

tNoBox(IN := (State = 1), PT := T#10s);
tRobot(IN := (State = 2), PT := T#40s);
tSim(IN := (State = 2) AND Use_Sim, PT := T#3s);

Robot_Done := (Use_Sim AND tSim.Q) OR ((NOT Use_Sim) AND Robot_Done_In);

(* E-stop overrides everything *)
IF NOT EStop_OK THEN
  IF State <> 9 THEN
    FaultCode := 1;
  END_IF;
  State := 9;
END_IF;

CASE State OF
    0: IF StartEdge.Q AND EStop_OK THEN BoxCount := 0; State := 1; END_IF;
  1: IF Box_Sensor THEN
       State := 2;
     ELSIF tNoBox.Q THEN
       FaultCode := 2; State := 9;
     END_IF;

  2: IF Robot_Done THEN
       State := 3;
     ELSIF tRobot.Q THEN
       FaultCode := 3; State := 9;
     END_IF;

  3: IF NOT Robot_Done_In THEN
       BoxCount := BoxCount + 1;
       IF BoxCount >= 12 THEN State := 4; ELSE State := 1; END_IF;
     END_IF;

  4: IF ResetEdge.Q THEN BoxCount := 0; State := 0; END_IF;

  9: IF ResetEdge.Q AND EStop_OK THEN FaultCode := 0; State := 0; END_IF;
END_CASE;

Conveyor   := (State = 1);
Robot_Pick := (State = 2);
Alarm_Lamp := (State = 9);
Full_Lamp  := (State = 4);
Run_Lamp   := (State = 1) OR (State = 2) OR (State = 3);

END_PROGRAM
