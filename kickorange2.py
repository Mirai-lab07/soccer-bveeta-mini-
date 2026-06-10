#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

# --- CRASH-PROOF HARDWARE IMPORT ---
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except (ImportError, RuntimeError):
    try:
        import Jetson.GPIO as GPIO
        GPIO_AVAILABLE = True
    except Exception:
        GPIO_AVAILABLE = False

# ============================================================================
# STATES
# ============================================================================
STATE_MATCH_STOPPED  = 0   
STATE_KICK_OFF_PASS  = 1   
STATE_SEARCH_BALL    = 2   
STATE_APPROACH_BALL  = 3   
STATE_PUSH_TO_GOAL   = 5   
STATE_RETREAT        = 6   
STATE_STALEMATE_WAIT = 7   

class BveetaRoboSotSoccer:
    def __init__(self):
        rospy.init_node("Bveeta_RoboSot_Soccer_Pro")
        
        # --- PUBLISHERS & SUBSCRIBERS ---
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.kick_pub = rospy.Publisher("/kick", Bool, queue_size=1)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        self.ref_sub = rospy.Subscriber("/game_status", String, self.referee_callback)
        
        # --- KICKER HARDWARE SETUP ---
        self.relay_pin = 18
        self.kick_duration = 0.12      
        self.cooldown_duration = 1.5   
        
        if GPIO_AVAILABLE:
            GPIO.setmode(GPIO.BCM)
            # SENYAPKAN WARNING GPIO
            GPIO.setwarnings(False) 
            GPIO.setup(self.relay_pin, GPIO.OUT)
            GPIO.output(self.relay_pin, GPIO.LOW)
            rospy.loginfo(f"GPIO Relay Kicker sedia pada Pin {self.relay_pin}")
        else:
            rospy.logwarn("GPIO tidak diaktifkan. Berjalan dalam SAFE SIMULATION MODE.")
            
        # --- STRATEGI PERLAWANAN & WARNA GOL ---
        self.opponent_goal_color = "YELLOW"  
        
        # Nilai Asal HSV (Default) untuk Bola Oren
        self.orangeLower = [5, 135, 140]     
        self.orangeUpper = [23, 255, 255]
        
        self.blueLower   = [90, 50, 50]       
        self.blueUpper   = [130, 255, 255]
        self.yellowLower = [20, 100, 100]     
        self.yellowUpper = [35, 255, 255]
        
        # --- STATUS PERMAINAN AUTONOMOUS ---
        self.current_state = STATE_MATCH_STOPPED  
        self.last_seen_direction = 1
        self.last_goal_direction = 1
        self.init_time = rospy.Time.now() 
        self.state_start_time = rospy.Time.now()
        
        # Memori Kedudukan Terakhir Bola (Penyelesaian isu Blind Spot)
        self.last_ball_x = 0
        self.last_ball_y = 0
        
        # Pemasa & Kebuntuan
        self.last_kick_time = rospy.Time.now()
        self.is_kicking = False
        self.stalemate_start_time = rospy.Time.now()
        self.ball_is_stationary = False  
        
        # Lidar Obstacle
        self.obstacle_detected = False
        self.obstacle_side = 0
        self.min_dist_threshold = 0.45  

    def referee_callback(self, msg):
        command = msg.data.upper().strip()
        rospy.loginfo(f"Menerima Arahan Pengadil: {command}")
        
        if command == "START" or command == "KICKOFF":
            if self.current_state == STATE_MATCH_STOPPED:
                rospy.loginfo("ISYARAT KICK-OFF DITERIMA! Robot mula bergerak...")
                self.current_state = STATE_KICK_OFF_PASS
                self.state_start_time = rospy.Time.now()
        elif command == "STOP" or command == "HALT":
            rospy.logwarn("ISYARAT STOP DITERIMA! Robot dihentikan.")
            self.current_state = STATE_MATCH_STOPPED

    def set_solenoid(self, status):
        if GPIO_AVAILABLE:
            GPIO.output(self.relay_pin, GPIO.HIGH if status else GPIO.LOW)

    def lidar_callback(self, data):
        num_points = len(data.ranges)
        if num_points == 0: return
        def get_index(angle_deg): return int((angle_deg % 360) * (num_points / 360.0))
        
        front_indices = [get_index(a) for a in range(-25, 26)]
        left_indices = [get_index(a) for a in range(26, 75)]
        right_indices = [get_index(a) for a in range(285, 334)]

        def get_min_dist(indices):
            valid = [data.ranges[i] for i in indices if i < num_points and np.isfinite(data.ranges[i]) and data.ranges[i] > 0.05]
            return min(valid) if valid else 10.0

        if get_min_dist(front_indices) < self.min_dist_threshold:
            self.obstacle_detected = True
            self.obstacle_side = 1 if get_min_dist(left_indices) > get_min_dist(right_indices) else -1
        else:
            self.obstacle_detected = False

    def move_robot(self, vision_error, linear_speed, p_gain):
        twist = Twist()
        deadzone = 0.05
        steering = 0.0 if abs(vision_error) < deadzone else vision_error * p_gain
        
        if self.obstacle_detected and self.current_state != STATE_RETREAT:
            twist.linear.x = linear_speed * 0.3
            twist.angular.z = steering + (0.7 * self.obstacle_side)
        else:
            twist.linear.x = linear_speed
            twist.angular.z = steering
        self.cmd_vel_pub.publish(twist)

    def on_trackbar_change(self, val, trackbar_idx):
        if trackbar_idx == 0: self.orangeLower[0] = val
        elif trackbar_idx == 1: self.orangeUpper[0] = val
        elif trackbar_idx == 2: self.orangeLower[1] = val
        elif trackbar_idx == 3: self.orangeUpper[1] = val
        elif trackbar_idx == 4: self.orangeLower[2] = val
        elif trackbar_idx == 5: self.orangeUpper[2] = val

    def run(self):
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        rate = rospy.Rate(20) 
        
        interface_window = "Interface Slider Warna Live"
        cv2.namedWindow(interface_window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(interface_window, 450, 350)
        
        cv2.createTrackbar("H Lower", interface_window, self.orangeLower[0], 179, lambda v: self.on_trackbar_change(v, 0))
        cv2.createTrackbar("H Upper", interface_window, self.orangeUpper[0], 179, lambda v: self.on_trackbar_change(v, 1))
        cv2.createTrackbar("S Lower", interface_window, self.orangeLower[1], 255, lambda v: self.on_trackbar_change(v, 2))
        cv2.createTrackbar("S Upper", interface_window, self.orangeUpper[1], 255, lambda v: self.on_trackbar_change(v, 3))
        cv2.createTrackbar("V Lower", interface_window, self.orangeLower[2], 255, lambda v: self.on_trackbar_change(v, 4))
        cv2.createTrackbar("V Upper", interface_window, self.orangeUpper[2], 255, lambda v: self.on_trackbar_change(v, 5))
        
        rospy.loginfo("Sistem Visi Berkelajuan Tinggi Berjaya Dilancarkan.")

        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret: 
                rospy.logwarn_throttle(2, "Gagal membaca data kamera!")
                continue
            
            # --- PENYELESAIAN DI SINI: Ditambah .copy() supaya layout memori bersesuaian dengan OpenCV ---
            frame = np.flip(frame, axis=(0, 1)).copy() 
            
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            frame_w = frame.shape[1]
            frame_h = frame.shape[0]
            
            roi_y = int(frame_h * (2.4 / 3)) 
            cv2.line(frame, (0, roi_y), (frame_w, roi_y), (0, 255, 255), 2)

            now = rospy.Time.now()
            time_since_last_kick = (now - self.last_kick_time).to_sec()
            
            if self.current_state == STATE_MATCH_STOPPED:
                time_waiting = (now - self.init_time).to_sec()
                if time_waiting > 5.0:
                    self.current_state = STATE_SEARCH_BALL
                    self.state_start_time = rospy.Time.now()

            if self.is_kicking:
                if time_since_last_kick > self.kick_duration:
                    self.set_solenoid(False)
                    self.is_kicking = False
                status_kicker = "STRIKING/PASSING!!!"
                kicker_color = (0, 255, 255)
            elif time_since_last_kick < self.cooldown_duration:
                status_kicker = f"COOLDOWN ({self.cooldown_duration - time_since_last_kick:.1f}s)"
                kicker_color = (0, 0, 255)
            else:
                status_kicker = "READY TO SHOOT"
                kicker_color = (0, 255, 0)

            mask_ball = cv2.inRange(hsv, np.array(self.orangeLower), np.array(self.orangeUpper))
            
            if self.opponent_goal_color == "YELLOW":
                mask_goal = cv2.inRange(hsv, np.array(self.yellowLower), np.array(self.yellowUpper))
            else:
                mask_goal = cv2.inRange(hsv, np.array(self.blueLower), np.array(self.blueUpper))

            cnts_ball, _ = cv2.findContours(mask_ball, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts_goal, _ = cv2.findContours(mask_goal, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            ball_target = None
            ball_detected = False
            bx, by = 0, 0
            
            if cnts_ball:
                c = max(cnts_ball, key=cv2.contourArea)
                if cv2.contourArea(c) > 200:
                    x, y, w, h = cv2.boundingRect(c)
                    bx = x + w//2
                    by = y + h//2
                    ball_target = (bx, by)
                    ball_detected = True
                    self.last_ball_x = bx  
                    self.last_ball_y = by  
                    cv2.rectangle(frame, (x,y), (x+w, y+h), (0,165,255), 2)
                    cv2.circle(frame, (bx, by), 5, (0, 255, 0), -1)

            # ============================================================================
            # FSM LOGIC
            # ============================================================================
            if self.current_state == STATE_MATCH_STOPPED:
                self.cmd_vel_pub.publish(Twist())

            elif self.current_state == STATE_KICK_OFF_PASS:
                elapsed = (rospy.Time.now() - self.state_start_time).to_sec()
                if elapsed < 0.15:
                    if not self.is_kicking and time_since_last_kick >= self.cooldown_duration:
                        self.set_solenoid(True)
                        self.kick_pub.publish(True)
                        self.is_kicking = True
                        self.last_kick_time = now
                if elapsed < 0.8:
                    twist = Twist()
                    twist.linear.x = -0.3 
                    self.cmd_vel_pub.publish(twist)
                else:
                    self.current_state = STATE_SEARCH_BALL
                    self.state_start_time = rospy.Time.now()

            elif self.current_state == STATE_RETREAT:
                elapsed = (rospy.Time.now() - self.state_start_time).to_sec()
                if elapsed < 1.5:
                    self.move_robot(0, -0.25, 0) 
                else:
                    self.current_state = STATE_SEARCH_BALL
                    self.state_start_time = rospy.Time.now()
                    
            elif self.current_state == STATE_STALEMATE_WAIT:
                self.cmd_vel_pub.publish(Twist()) 
                elapsed = (rospy.Time.now() - self.state_start_time).to_sec()
                if elapsed >= 3.0: 
                    self.current_state = STATE_SEARCH_BALL
                    self.stalemate_start_time = rospy.Time.now()

            else:
                if self.ball_is_stationary and self.current_state == STATE_PUSH_TO_GOAL:
                    if (rospy.Time.now() - self.stalemate_start_time).to_sec() > 5.0:
                        self.current_state = STATE_STALEMATE_WAIT
                        self.state_start_time = rospy.Time.now()
                        continue

                # --- SEARCH BALL & APPROACH BALL ---
                if self.current_state in [STATE_SEARCH_BALL, STATE_APPROACH_BALL]:
                    if ball_target:
                        bx_val, by_val = ball_target
                        error = (frame_w//2 - bx_val) / float(frame_w//2)
                        
                        self.last_seen_direction = 1 if error > 0 else -1
                        
                        if by_val > roi_y: 
                            self.current_state = STATE_PUSH_TO_GOAL
                            self.stalemate_start_time = rospy.Time.now()
                        else:
                            self.current_state = STATE_APPROACH_BALL
                            self.move_robot(error, 0.35, 1.2) 
                    else:
                        # --- 360 DEGREE SPIN SEARCH ---
                        self.current_state = STATE_SEARCH_BALL
                        cv2.putText(frame, "SEARCHING: 360 SPIN ACTIVE", (20, 130), 0, 0.5, (0, 0, 255), 2)
                        
                        twist = Twist()
                        twist.linear.x = 0.0
                        twist.angular.z = 0.85 * self.last_seen_direction 
                        self.cmd_vel_pub.publish(twist)

                elif self.current_state == STATE_PUSH_TO_GOAL:
                    # --- BOLA BLIND SPOT DI BAWAH KAMERA ---
                    if not ball_target:
                        if self.last_ball_y > (roi_y - 40):
                            rospy.loginfo("BOLA HILANG DI BLIND SPOT (BAWAH KAMERA)! KICKER AKTIF!")
                            if not self.is_kicking and time_since_last_kick >= self.cooldown_duration:
                                self.set_solenoid(True)
                                self.kick_pub.publish(True)
                                self.is_kicking = True
                                self.last_kick_time = now
                                self.current_state = STATE_RETREAT
                                self.state_start_time = rospy.Time.now()
                                self.last_ball_y = 0 
                                continue
                        else:
                            self.current_state = STATE_SEARCH_BALL
                            self.state_start_time = rospy.Time.now()
                            continue

                    if cnts_goal:
                        c = max(cnts_goal, key=cv2.contourArea)
                        if cv2.contourArea(c) > 120:
                            x, y, w, h = cv2.boundingRect(c)
                            gx = x + w//2
                            error = (frame_w//2 - gx) / float(frame_w//2)
                            self.last_goal_direction = 1 if error > 0 else -1
                            cv2.rectangle(frame, (x,y), (x+w, y+h), (255, 255, 0) if self.opponent_goal_color == "YELLOW" else (255, 0, 0), 2)
                            
                            is_aligned = abs(error) < 0.25  
                            ball_in_position = ball_detected and by > (roi_y - 30)

                            if (is_aligned and ball_in_position) and not self.is_kicking and time_since_last_kick >= self.cooldown_duration:
                                self.set_solenoid(True)
                                self.kick_pub.publish(True)
                                self.is_kicking = True
                                self.last_kick_time = now
                                self.current_state = STATE_RETREAT
                                self.state_start_time = rospy.Time.now()
                                continue
                            
                            self.move_robot(error, 0.70, 0.9) 
                        else:
                            self.move_to_last_goal_dir()
                    else:
                        if ball_detected and by > (frame_h - 40):
                            if not self.is_kicking and time_since_last_kick >= self.cooldown_duration:
                                self.set_solenoid(True)
                                self.kick_pub.publish(True)
                                self.is_kicking = True
                                self.last_kick_time = now
                                self.current_state = STATE_RETREAT
                                self.state_start_time = rospy.Time.now()
                                continue
                        self.move_to_last_goal_dir()

            # --- HUD & DISPLAY ---
            state_labels = {STATE_MATCH_STOPPED: "MATCH STOPPED", STATE_KICK_OFF_PASS: "KICK-OFF PROTOCOL",
                            STATE_SEARCH_BALL: "SEARCH BALL", STATE_APPROACH_BALL: "APPROACH BALL",
                            STATE_PUSH_TO_GOAL: "PUSH TO GOAL (FAST ATTACK)",
                            STATE_RETREAT: "RETREAT GOAL", STATE_STALEMATE_WAIT: "STALEMATE RULE ACTIVE"}
            
            cv2.putText(frame, f"STATE: {state_labels.get(self.current_state, 'UNKNOWN')}", (10, 30), 0, 0.6, (255, 255, 255), 2)
            cv2.putText(frame, f"KICKER STATUS: {status_kicker}", (10, 60), 0, 0.6, kicker_color, 2)
            
            cv2.imshow("Bveeta RoboSot Pro HUD", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'): break
            rate.sleep()

        cap.release(); cv2.destroyAllWindows()
        if GPIO_AVAILABLE:
            GPIO.output(self.relay_pin, GPIO.LOW)
            GPIO.cleanup()

    def move_to_last_goal_dir(self):
        twist = Twist()
        twist.linear.x = 0.15
        twist.angular.z = 0.65 * self.last_goal_direction
        self.cmd_vel_pub.publish(twist)

if __name__ == "__main__":
    try: BveetaRoboSotSoccer().run()
    except rospy.ROSInterruptException: pass