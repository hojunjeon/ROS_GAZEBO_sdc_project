#신호등, 좌회전 모드 조향 문제 해결
#좌회전 모드 -> 차선주행 모드로 돌아오면 좌회전 모드 속도가 적용되는 문제 발생

from .Detection.Lanes.Lane_Detection import detect_Lane
from .Detection.Signs.SignDetectionApi import detect_Signs
from .Detection.TrafficLights.TrafficLights_Detection import detect_TrafficLights
import cv2
from numpy import interp
from .config import config
# 4 Improvements that will be done in (Original) SDC control algorithm
# a) lane assist had iregular steering predictions
#    Solution : use rolling average filter
# b) Considering road is barely 1.5 car wide. A quarter of Image width for distance from the road mid 
#                                             from the predicted road center seems bit too harsh
#    Solution:  Increase to half of image width
# c) Car was drifting offroad in sharper turns causing it to lose track of road
#    Solution: Increase weightage of distance (road_center <=> car front) from 50% to 65% 
#              So steers more in case it drift offroad
# d) Car not utilizing its full steering range causing it to drift offroad in sharp turns
#    Solution: Increase car max turn capability

# 2 additons to Drive_Bot.py
# a) 1 control block added for enable/disable Sat_Nav feature
# b) Track Traffic Light and Road Speed Limits (State)  ==> Essential for priority control mechanism 
#                                                           That we will create for integrating Sat_Nav 
#                                                           ability to the SDC


from collections import deque
class Debugging:

    def __init__(self):
        self.TL_Created = False
        self.Lan_Created = False

    def nothing(self,x):
        pass

    cv2.namedWindow('CONFIG')

    enable_SatNav = 'Sat-Nav'
    cv2.createTrackbar(enable_SatNav, 'CONFIG',False,True,nothing)

    # creating (Engine) on/off trackbar 
    Motors = 'Engine'
    cv2.createTrackbar(Motors, 'CONFIG',False,True,nothing)

    # create switch for ON/OFF functionality
    debugging_SW = 'Debug'
    cv2.createTrackbar(debugging_SW, 'CONFIG',False,True,nothing)
    # create switch for ON/OFF functionality
    debuggingLane_SW = 'Debug Lane'
    cv2.createTrackbar(debuggingLane_SW, 'CONFIG',False,True,nothing)
    # create switch for ON/OFF functionality
    debuggingSigns_SW = 'Debug Sign'
    cv2.createTrackbar(debuggingSigns_SW, 'CONFIG',False,True,nothing)
    # create switch for ON/OFF functionality
    debuggingTL_SW = 'Debug TL'
    cv2.createTrackbar(debuggingTL_SW, 'CONFIG',False,True,nothing)


    def setDebugParameters(self):
        # get current positions of four trackbars
        # get current positions of trackbar
        # get current positions of four trackbars
        enable_SatNav = cv2.getTrackbarPos(self.enable_SatNav,'CONFIG')
        Motors = cv2.getTrackbarPos(self.Motors,'CONFIG')

        debug = cv2.getTrackbarPos(self.debugging_SW,'CONFIG')
        debugLane = cv2.getTrackbarPos(self.debuggingLane_SW,'CONFIG')
        debugSign = cv2.getTrackbarPos(self.debuggingSigns_SW,'CONFIG')
        debugTrafficLights = cv2.getTrackbarPos(self.debuggingTL_SW,'CONFIG')


        if enable_SatNav:
            config.enable_SatNav = True
        else:
            config.enable_SatNav = False

        # If trackbar changed modify engines_on config parameter
        if Motors:
            config.engines_on = True
        else:
            config.engines_on = False
            
        if debug:
            config.debugging = True
        else:
            config.debugging = False            
        if debugLane:
            config.debugging_Lane = True
        else:
            config.debugging_Lane = False    
        if debugSign:
            config.debugging_Signs = True
        else:
            config.debugging_Signs = False           
        if debugTrafficLights:
            config.debugging_TrafficLights = True
        else:
            config.debugging_TrafficLights = False

        if config.debugging_TrafficLights:
            
            debuggingTLConfig_SW = 'Debug Config'
            if not self.TL_Created:
                self.TL_Created = True
                cv2.namedWindow('CONFIG_TL')
                cv2.createTrackbar(debuggingTLConfig_SW, 'CONFIG_TL',False,True,self.nothing)

            debugTL_Config = cv2.getTrackbarPos(debuggingTLConfig_SW,'CONFIG_TL')

            if debugTL_Config:
                config.debugging_TL_Config = True
            else:
                config.debugging_TL_Config = False

        else:
            self.TL_Created = False
            cv2.destroyWindow('CONFIG_TL')

        
        if config.debugging_Lane:
            
            debuggingLANEConfig_SW = 'Debug (Stage)'
            if not self.Lan_Created:
                self.Lan_Created = True
                cv2.namedWindow('CONFIG_LANE')
                cv2.createTrackbar(debuggingLANEConfig_SW, 'CONFIG_LANE',0,3,self.nothing)

            debugLane_Config = cv2.getTrackbarPos(debuggingLANEConfig_SW,'CONFIG_LANE')

            if debugLane_Config == 0:
                config.debugging_L_ColorSeg = True
                config.debugging_L_Est = config.debugging_L_Cleaning = config.debugging_L_LaneInfoExtraction = False                    
            elif debugLane_Config == 1:
                config.debugging_L_Est = True
                config.debugging_L_ColorSeg = config.debugging_L_Cleaning = config.debugging_L_LaneInfoExtraction = False   
            elif debugLane_Config == 2:
                config.debugging_L_Cleaning = True
                config.debugging_L_ColorSeg = config.debugging_L_Est = config.debugging_L_LaneInfoExtraction = False   
            elif debugLane_Config == 3:
                config.debugging_L_LaneInfoExtraction = True
                config.debugging_L_ColorSeg = config.debugging_L_Est = config.debugging_L_Cleaning = False

        else:
            self.Lan_Created = False
            cv2.destroyWindow('CONFIG_LANE')        


class Control:
    def keep_distance(self, Angle, Speed, front_distance, target_distance, now, prev_front_distance, prev_time):
        # 초기화 및 예외 처리
        if front_distance is None:
            return Angle, Speed, prev_front_distance, prev_time
        if front_distance < 0:
            Speed = 90.0
            prev_front_distance = None
            prev_time = None
        elif front_distance <= target_distance - 2:
            Speed = 0.0
            prev_front_distance = None
            prev_time = None
        else:
            if prev_front_distance is None or prev_time is None:
                Speed = 90.0
                prev_front_distance = front_distance
                prev_time = now
            else:
                delta = front_distance - prev_front_distance
                if delta < 0:
                    Speed = 60.0
                else:
                    Speed = 90.0
                prev_front_distance = front_distance
                prev_time = now
        return Angle, Speed, prev_front_distance, prev_time

    def __init__(self):
        self.prev_Mode = "Detection"
        self.prev_Mode_LT = "Detection"
        self.car_speed = 50
        self.angle_of_car = 0
        self.Left_turn_iterations = 0
        self.Frozen_Angle = 0
        self.Detected_LeftTurn = False
        self.Activat_LeftTurn = False        
        self.TrafficLight_iterations = 0
        self.GO_MODE_ACTIVATED = False
        self.STOP_MODE_ACTIVATED = False
        self.lane_follow_speed = 50
        self.angle_queue = deque(maxlen=10)

    def follow_Lane(self,Max_Sane_dist,distance,curvature , Mode , Tracked_class):

        # [NEW]: Turning at normal speed is not much of a problem in simulation
        IncreaseTireSpeedInTurns = False

        if((Tracked_class!=0) and (self.prev_Mode == "Tracking") and (Mode == "Detection")):
            if  (Tracked_class =="speed_sign_30"):
                self.car_speed = 30
            elif(Tracked_class =="speed_sign_60"):
                self.car_speed = 60
            elif(Tracked_class =="speed_sign_90"):
                self.car_speed = 90
            elif(Tracked_class =="stop"):
                self.car_speed = 0
            
        self.prev_Mode = Mode # Set prevMode to current Mode
        
        Max_turn_angle_neg = -90
        Max_turn_angle = 90

        CarTurn_angle = 0

        if( (distance > Max_Sane_dist) or (distance < (-1 * Max_Sane_dist) ) ):
            # Max sane distance reached ---> Max penalize (Max turn Tires)
            if(distance > Max_Sane_dist):
                #Car offseted left --> Turn full wheels right
                CarTurn_angle = Max_turn_angle + curvature
            else:
                #Car Offseted right--> Turn full wheels left
                CarTurn_angle = Max_turn_angle_neg + curvature
        else:
            # Within allowed distance limits for car and lane
            # Interpolate distance to Angle Range
            Turn_angle_interpolated = interp(distance,[-Max_Sane_dist,Max_Sane_dist],[-90,90])
            #[NEW]: Modified to calculate carturn_angle based on following criteria
            #             65% turn suggested by distance to the lane center + 35 % how much the lane is turning
            CarTurn_angle = (0.65*Turn_angle_interpolated) + (0.35*curvature)

        # Handle Max Limit [if (greater then either limits) --> set to max limit]
        if( (CarTurn_angle > Max_turn_angle) or (CarTurn_angle < (-1 *Max_turn_angle) ) ):
            if(CarTurn_angle > Max_turn_angle):
                CarTurn_angle = Max_turn_angle
            else:
                CarTurn_angle = -Max_turn_angle

        #angle = CarTurn_angle
        # [NEW]: Increase car turning capability by 30 % to accomodate sharper turns
        angle = interp(CarTurn_angle,[-90,90],[-60,60])

        curr_speed = self.car_speed
        
        if (IncreaseTireSpeedInTurns and (Tracked_class !="left_turn")):
            if(angle>30):
                car_speed_turn = interp(angle,[30,45],[80,100])
                curr_speed = car_speed_turn
            elif(angle<-30):
                car_speed_turn = interp(angle,[-45,-30],[100,80])
                curr_speed = car_speed_turn

        
        return angle , curr_speed


    def Obey_LeftTurn(self,Angle,Speed,Mode,Tracked_class):
        # base_speed: 차선주행 등에서 계산된 원래 속도
        if (Tracked_class == "left_turn"):
            Speed = 30

            if ( (self.prev_Mode_LT =="Detection") and (Mode=="Tracking")):
                self.prev_Mode_LT = "Tracking"
                self.Detected_LeftTurn = True

            elif ( (self.prev_Mode_LT =="Tracking") and (Mode=="Detection")):
                self.Detected_LeftTurn = False
                self.Activat_LeftTurn = True

                if ( ((self.Left_turn_iterations % 10 ) ==0) and (self.Left_turn_iterations>10)and (self.Left_turn_iterations<70)):
                    self.Frozen_Angle = self.Frozen_Angle -6.5 # Move left by 1 degree 

                if(self.Left_turn_iterations==90):
                    self.prev_Mode_LT = "Detection"
                    self.Activat_LeftTurn = False
                    self.Left_turn_iterations = 0

                self.Left_turn_iterations = self.Left_turn_iterations + 1
                if (self.Activat_LeftTurn or self.Detected_LeftTurn):
                    #Follow previously Saved Route
                    Angle = self.Frozen_Angle

        return Angle,Speed,self.Detected_LeftTurn,self.Activat_LeftTurn


    def OBEY_TrafficLights(self,a,b,Traffic_State,CloseProximity):

        if((Traffic_State == "Stop") and CloseProximity):
            b = 0 # Noob luqman
            self.STOP_MODE_ACTIVATED = True
        else:
            if (self.STOP_MODE_ACTIVATED or self.GO_MODE_ACTIVATED):

                if (self.STOP_MODE_ACTIVATED and (Traffic_State=="Go")):
                    self.STOP_MODE_ACTIVATED = False
                    self.GO_MODE_ACTIVATED = True

                elif(self.STOP_MODE_ACTIVATED):
                    b = 0

                elif(self.GO_MODE_ACTIVATED):
                    a = 0.0                    
                    if(self.TrafficLight_iterations==50):
                        self.GO_MODE_ACTIVATED = False
                        print("Interchange Crossed !!!")
                        self.TrafficLight_iterations = 0 #Reset

                    self.TrafficLight_iterations = self.TrafficLight_iterations + 1
        return a,b




    def drive_car(self, Current_State, Inc_TL, Inc_LT, front_distance=None, target_distance=10.0, prev_front_distance=None, prev_time=None, now=None):

        [Distance, Curvature, frame_disp, Mode, Tracked_class, Traffic_State, CloseProximity] = Current_State
        current_speed = 0
        # 1. 차선 추종
        if (Distance != -1000) and (Curvature != -1000):
            self.angle_of_car, current_speed = self.follow_Lane(int(frame_disp.shape[1]/2), Distance, Curvature, Mode, Tracked_class)
            if abs(self.angle_of_car)>5 :
                current_speed = 30
        config.angle_orig = self.angle_of_car
        self.angle_queue.append(self.angle_of_car)
        self.angle_of_car = (sum(self.angle_queue) / len(self.angle_queue))
        config.angle = self.angle_of_car

        # 2. 좌회전 표지판 처리
        if Inc_LT:
            self.angle_of_car, current_speed, Detected_LeftTurn, Activat_LeftTurn = self.Obey_LeftTurn(
                self.angle_of_car, current_speed, Mode, Tracked_class)
        else:
            Detected_LeftTurn = False
            Activat_LeftTurn = False


        # 3. 신호등 처리
        if Inc_TL:
            self.angle_of_car, current_speed = self.OBEY_TrafficLights(self.angle_of_car, current_speed, Traffic_State, CloseProximity)

        # 4. 차간거리 유지(lead car distance keeping) 처리
        # (Car에서 전달받은 front_distance, target_distance, prev_front_distance, prev_time, now 사용)
        if front_distance is not None and now is not None:
            self.angle_of_car, current_speed, prev_front_distance, prev_time = self.keep_distance(
                self.angle_of_car, current_speed, front_distance, target_distance, now, prev_front_distance, prev_time)
        
            if abs(self.angle_of_car)>5 :
                current_speed = 30            


        return self.angle_of_car, current_speed, Detected_LeftTurn, Activat_LeftTurn, prev_front_distance, prev_time


class Car:

    def __init__(self, Inc_TL=True, Inc_LT=True):
        self.Control_ = Control()
        self.Inc_TL = Inc_TL
        self.Inc_LT = Inc_LT
        self.Tracked_class = "Unknown"
        self.Traffic_State = "Unknown"
        # 외부에서 차간거리만 받아 저장 (front_vehicle_detector 연동)
        self.front_distance = None
        self.target_distance = 10.0
        self.min_speed = 30.0
        self.max_speed = 90.0
        self.k_p = 5.0
        # lead_car 속도 추종 관련 변수 초기화
        self.last_lead_speed = None
        self.lead_speed_buffer = []
        self.lead_speed_buffer_time = None
        self.lane_follow_speed = 60

    def set_front_distance(self, distance):
        self.front_distance = distance

    def display_state(self,frame_disp,angle_of_car,current_speed,Tracked_class,Traffic_State,Detected_LeftTurn, Activat_LeftTurn):
    
        ###################################################  Displaying CONTROL STATE ####################################

        if (angle_of_car <-10):
            direction_string="[ Left ]"
            color_direction=(120,0,255)
        elif (angle_of_car >10):
            direction_string="[ Right ]"
            color_direction=(120,0,255)
        else:
            direction_string="[ Straight ]"
            color_direction=(0,255,0)

        current_speed = 0 if current_speed is None else current_speed

        if(current_speed>0 ):
            direction_string = "Moving --> "+ direction_string
        else:
            color_direction=(0,0,255)


        cv2.putText(frame_disp,str(direction_string),(20,40),cv2.FONT_HERSHEY_DUPLEX,0.4,color_direction,1)

        angle_speed_str = "[ Angle ,Speed ] = [ " + str(int(angle_of_car)) + "deg ," + str(int(current_speed)) + "mph ]"
        cv2.putText(frame_disp,str(angle_speed_str),(20,20),cv2.FONT_HERSHEY_DUPLEX,0.4,(0,0,255),1)

        cv2.putText(frame_disp,"Traffic Light State = [ "+Traffic_State+" ] ",(20,60),cv2.FONT_HERSHEY_COMPLEX,0.35,255)
        
        if (Tracked_class=="left_turn"):
            font_Scale = 0.32
            if (Detected_LeftTurn):
                Tracked_class = Tracked_class + " : Detected { True } "
            else:
                Tracked_class = Tracked_class + " : Activated { "+ str(Activat_LeftTurn) + " } "
        else:
            font_Scale = 0.37
        cv2.putText(frame_disp,"Sign Detected ==> "+str(Tracked_class),(20,80),cv2.FONT_HERSHEY_COMPLEX,font_Scale,(0,255,255),1)

    def driveCar(self, frame, debug_print=True):
        import time
        Detected_LeftTurn = False
        Activat_LeftTurn = False

        img = frame[0:640,238:1042]
        img = cv2.resize(img,(320,240))
        img_orig = img.copy()

        distance, Curvature = detect_Lane(img)

        if self.Inc_TL:
            Traffic_State, CloseProximity = detect_TrafficLights(img_orig.copy(),img)
        else:
            Traffic_State = "Unknown"
            CloseProximity = False

        Mode , Tracked_class = detect_Signs(img_orig,img)

        Current_State = [distance, Curvature, img, Mode, Tracked_class, Traffic_State, CloseProximity]

        now = time.time()
        # another_Drive_Bot.py 스타일: 각 이벤트를 함수로 분리하여 순차적으로 처리
        Angle, Speed, Detected_LeftTurn, Activat_LeftTurn, self.prev_front_distance, self.prev_time = self.Control_.drive_car(
            Current_State, self.Inc_TL, self.Inc_LT,
            front_distance=self.front_distance,
            target_distance=self.target_distance,
            prev_front_distance=getattr(self, 'prev_front_distance', None),
            prev_time=getattr(self, 'prev_time', None),
            now=now
        )
        
        '''if not (Detected_LeftTurn or Activat_LeftTurn) and Tracked_class == "left_turn":
            Tracked_class = "Unknown"'''


        

        # 디버그 메시지
        if Tracked_class in ["speed_sign_30", "speed_sign_60", "speed_sign_90"]:
            limit = Tracked_class.split('_')[-1]
            debug_msg = f"표지판 인식 : 속도제한 = {limit} / 속도 = {Speed:.2f} / 조향각 = {Angle:.2f}"
        elif Tracked_class == "left_turn" and (Detected_LeftTurn or Activat_LeftTurn):
            debug_msg = f"표지판 인식 : LeftTurn / 속도 = {Speed:.2f} / 조향각 = {Angle:.2f}"
        elif Traffic_State == "Stop" and CloseProximity:
            debug_msg = f"신호등 인식 : 빨간불 / 속도 = {Speed:.2f} / 조향각 = {Angle:.2f}"
        elif Traffic_State == "Go":
            debug_msg = f"신호등 인식 : 초록불 / 속도 = {Speed:.2f} / 조향각 = {Angle:.2f}"
        elif self.front_distance is not None:
            debug_msg = (
                f"전방 차량 감지 : 차간 거리 = {self.front_distance:.2f} / 속도 = {Speed:.2f} / 조향각 = {Angle:.2f}"
                if self.front_distance is not None and Speed is not None and Angle is not None
                else f"전방 차량 감지 : 차간 거리 = {self.front_distance or 'N/A'} / 속도 = {Speed or 'N/A'} / 조향각 = {Angle or 'N/A'}"
            )
        else:
            debug_msg = f"차선 인식 주행 : 속도 = {Speed:.2f} / 조향각 = {Angle:.2f}"

        if debug_print:
            print(debug_msg)

        self.Tracked_class = Tracked_class
        self.Traffic_State = Traffic_State

        self.display_state(img, Angle, Speed, Tracked_class, Traffic_State, Detected_LeftTurn, Activat_LeftTurn)

        Angle = interp(Angle, [-60, 60], [0.8, -0.8])
        if Speed != 0:
            Speed = interp(Speed, [20, 90], [1, 4])

        if abs(Angle) < 0.03:
            Angle = 0.0
        elif abs(Angle) < 0.1:
            Angle *= 0.5

        Speed = float(Speed)

        return Angle, Speed, img