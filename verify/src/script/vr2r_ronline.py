# run in wsl
import os,sys
sys.path.append(os.getcwd())

import rospy
import verify.include.PX4MavCtrlV4 as PX4MavCtrler
import verify.include.RflyDB as RflyDB
import verify.include.RflyCtrl as RflyCtrl
import verify.include.RflySoftwarm as RflySW
import verify.include.RflyDtrain as RflyDtrain
from geometry_msgs.msg import TwistStamped, PoseStamped
from transforms3d.euler import quat2euler
from transforms3d.euler import quat2euler
import re
import math
import queue
import numpy as np
import collections
import time
import threading
import torch
import shutil
import subprocess
from sklearn.metrics import confusion_matrix
from tkinter.messagebox import *
from collections import deque


class MavConfig:
    def generate_trajectory(shape, total_time=10, time_step=0.1, repetitions=1.0, **kwargs):
        total_duration = total_time * repetitions
        time_points = np.arange(0, total_duration, time_step)
        trajectory = []

        if shape == 'circle':
            radius = kwargs.get('radius', 1)
            center_x = kwargs.get('center_x', 0)
            center_y = kwargs.get('center_y', 0)
            center_z = kwargs.get('center_z', 0)
            for t in time_points:
                relative_t = t % total_time
                angle = 2 * np.pi * (t / total_time)  
                x = center_x + radius * np.cos(angle)
                y = center_y + radius * np.sin(angle)
                z = center_z
                trajectory.append([x, y, z])

        elif shape == 'rectangle':
            width = kwargs.get('width', 2)
            height = kwargs.get('height', 2)
            center_x = kwargs.get('center_x', 0)
            center_y = kwargs.get('center_y', 0)
            center_z = kwargs.get('center_z', 0)
            perimeter = 2 * (width + height)
            speed = perimeter / total_time  
            for t in time_points:
                total_distance = speed * t
                distance = total_distance % perimeter
                if distance < width:
                    x = center_x - width / 2 + distance
                    y = center_y - height / 2
                elif distance < width + height:
                    x = center_x + width / 2
                    y = center_y - height / 2 + (distance - width)
                elif distance < 2 * width + height:
                    x = center_x + width / 2 - (distance - width - height)
                    y = center_y + height / 2
                else:
                    x = center_x - width / 2
                    y = center_y + height / 2 - (distance - 2 * width - height)
                z = center_z
                trajectory.append([x, y, z])

        elif shape == 'figure8':
            radius = kwargs.get('radius', 1)
            center_x = kwargs.get('center_x', 0)
            center_y = kwargs.get('center_y', 0)
            center_z = kwargs.get('center_z', 0)
            for t in time_points:
                cycle_ratio = t / total_time
                x = center_x + radius * np.sin(2 * np.pi * cycle_ratio)
                y = center_y + radius * np.sin(4 * np.pi * cycle_ratio)
                z = center_z
                trajectory.append([x, y, z])

        elif shape == 'sin':
            amplitude = kwargs.get('amplitude', 1)
            frequency = kwargs.get('frequency', 1)
            center_x = kwargs.get('center_x', 0)
            center_y = kwargs.get('center_y', 0)
            center_z = kwargs.get('center_z', 0)
            for t in time_points:
                cycle_ratio = t / total_time 
                x = center_x + t  
                y = center_y + amplitude * np.sin(2 * np.pi * frequency * cycle_ratio)
                z = center_z
                trajectory.append([x, y, z])

        return trajectory
    
class DataPool:
    def __init__(self, pool_size = 120):
        self.max_size = pool_size
        self.pool = collections.deque(maxlen=pool_size)

    def add_data(self, timestamp, data, flag=None):
        timestamp = time.time() 
        self.pool.append({'timestamp': timestamp, 'data': data, 'flag': flag})

    def is_pool_full(self):
        return len(self.pool) == self.max_size

    def __iter__(self):
        return iter(self.pool)

class IMUFilter:
    def __init__(self, 
                 ma_window_size=20,   
                 lowpass_alpha=0.2): 
        self.acc_ma_buff = {
            'x': deque(maxlen=ma_window_size),
            'y': deque(maxlen=ma_window_size),
            'z': deque(maxlen=ma_window_size)
        }
        self.gyro_ma_buff = {
            'x': deque(maxlen=ma_window_size),
            'y': deque(maxlen=ma_window_size),
            'z': deque(maxlen=ma_window_size)
        }
        
        self.acc_lowpass_prev = None
        self.gyro_lowpass_prev = None
        self.lowpass_alpha = lowpass_alpha

    def _moving_average(self, data, buff):
        buff.append(data)
        return np.mean(buff) if len(buff) > 0 else data

    def _first_order_lowpass(self, current, prev):
        if prev is None:  
            return current
        return self.lowpass_alpha * current + (1 - self.lowpass_alpha) * prev

    def filter_acc(self, acc_data):
        acc_ma_x = self._moving_average(acc_data[0], self.acc_ma_buff['x'])
        acc_ma_y = self._moving_average(acc_data[1], self.acc_ma_buff['y'])
        acc_ma_z = self._moving_average(acc_data[2], self.acc_ma_buff['z'])
        acc_ma = [acc_ma_x, acc_ma_y, acc_ma_z]

        if self.acc_lowpass_prev is None:
            self.acc_lowpass_prev = acc_ma
        acc_filtered = [
            self._first_order_lowpass(acc_ma[0], self.acc_lowpass_prev[0]),
            self._first_order_lowpass(acc_ma[1], self.acc_lowpass_prev[1]),
            self._first_order_lowpass(acc_ma[2], self.acc_lowpass_prev[2])
        ]
        self.acc_lowpass_prev = acc_filtered
        
        return acc_filtered

    def filter_gyro(self, gyro_data):
        gyro_ma_x = self._moving_average(gyro_data[0], self.gyro_ma_buff['x'])
        gyro_ma_y = self._moving_average(gyro_data[1], self.gyro_ma_buff['y'])
        gyro_ma_z = self._moving_average(gyro_data[2], self.gyro_ma_buff['z'])
        gyro_ma = [gyro_ma_x, gyro_ma_y, gyro_ma_z]

        if self.gyro_lowpass_prev is None:
            self.gyro_lowpass_prev = gyro_ma
        gyro_filtered = [
            self._first_order_lowpass(gyro_ma[0], self.gyro_lowpass_prev[0]),
            self._first_order_lowpass(gyro_ma[1], self.gyro_lowpass_prev[1]),
            self._first_order_lowpass(gyro_ma[2], self.gyro_lowpass_prev[2])
        ]
        self.gyro_lowpass_prev = gyro_filtered
        
        return gyro_filtered

class FDMav:
    def __init__(self, mav, model, device):
        self.mav = mav
        self.model = model
        self.device = device
        self.fd_lock = threading.Lock()

        self.lastTime = time.time()
        self.fd_hz = 10
        
        self.label = self.get_label(mav)
        print('self.label',self.label)

        self.num = 0
        self.right_cnt = 0
        self.fault_cnt = 0
        self.fault_info = []
        self.fault_log = ""
        self.fd_id = 0

    def record_accuracy(self, id, phase, max_index, info):
        if phase in self.label:
            self.num += 1
            if self.label[phase][max_index] == 1:
                self.right_cnt += 1
            else:
                self.fault_cnt += 1
                self.fault_log += f"LOG_ID [{id}], Phase [{phase}], Fault Type: {info}\n"
                finfo = f"LOG_ID [{id}], Phase [{phase}], Fault Type: {info}"
                self.fault_info.append(finfo)
    
    def get_label(self, mavs):
        global model_name  
        ctrlSeq = mavs[0].ctrlSeq 
        Allfinal_result = None
        
        for sub_seq in ctrlSeq:
            if sub_seq.startswith("2,9"):
                raw_result = sub_seq[4:].replace(";", "").split(",")
                Allfinal_result = [item.strip() for item in raw_result if item.strip()]
                break 
        
        model_name_lower = model_name.lower()
        fault_label = None
        fault_label_normal = None
        
        if "c127" in model_name_lower:
            self.ori_lab = np.array([0,0,0])
            self.out_lab = np.array([0,0,0])
            self.category = ['normal', 'af3', 'm1f3']
            fault_label = [1, 0, 0]  
            fault_label_normal = [1, 0, 0]
            if Allfinal_result is not None:
                # 【12,3,1,-1,-1,0,3】→ [0,1,0]
                if Allfinal_result[:6] == ['12', '3', '1', '-1', '-1', '0']:
                    fault_label = [0, 1, 0]
                # 【11,3,-1,-1,1,-1,1000,200】→ [0,0,1]
                elif Allfinal_result[:7] == ['11', '3', '-1', '-1', '1', '-1', '1000']:
                    fault_label = [0, 0, 1]
        elif "c1247" in model_name_lower:
            self.ori_lab = np.array([0,0,0,0])
            self.out_lab = np.array([0,0,0,0])
            self.category = ['normal', 'af3', 'm1f4', 'm1f3']
            fault_label = [1, 0, 0, 0]  
            fault_label_normal = [1, 0, 0, 0]
            if Allfinal_result is not None:
                # 【12,3,1,-1,-1,0,3】→ [0,1,0,0]
                if Allfinal_result[:7] == ['12', '3', '1', '-1', '-1', '0', '3']:
                    fault_label = [0, 1, 0, 0]
                # 【11,4,1,1,1,0.85】→ [0,0,1,0]
                elif Allfinal_result[:6] == ['11', '4', '1', '1', '1', '0.85']:
                    fault_label = [0, 0, 1, 0]
                # 【11,3,-1,-1,1,-1,1000,200】→ [0,0,0,1]
                elif Allfinal_result[:8] == ['11', '3', '-1', '-1', '1', '-1', '1000', '200']:
                    fault_label = [0, 0, 0, 1]
        
        label = {
            '3': fault_label_normal,
            '6': fault_label_normal,
            '8': fault_label_normal,
            '9': fault_label,
        }
        return label
    
    def ShowWin(self):        
        update_thread = threading.Thread(target=self.update_console)
        update_thread.start()
        print(f"{self.fd_hz}Hz")
        print("-" * 80)

    def update_console(self):
        global all_messages, FD_LOG, folder_name, log_cnt

        while True:
            if not FD_LOG.empty():
                latest_result = FD_LOG.get()
                if isinstance(latest_result, str):
                    all_messages += latest_result
                else:
                    all_messages += f"LOG_ID [{latest_result[0]}], Phase [{latest_result[1]}], Fault Type: {latest_result[2]}\n"
                    print(f"LOG_ID [{latest_result[0]}], Phase [{latest_result[1]}], Fault Type: {latest_result[2]}")  


                if isinstance(latest_result, str):
                    max_index = np.argmax(self.label['9'])
                    file_path = os.path.join(sys.path[0], 'info')

                    if not os.path.exists(file_path):
                        os.makedirs(file_path)

                    path = os.path.join(file_path, f"LOG_{folder_name}_{self.category[max_index]}_{log_cnt}.txt")
                    if os.path.exists(path):
                        os.remove(path)

                    with open(path, "w") as file: 
                        file.write(all_messages)
                    print(f"- {path}")
                    print("-" * 80)

                    return
            
            time.sleep(1.0 / self.fd_hz)

    def get_data(self, mav_data):
        data = np.array([entry['data'] for entry in mav_data])
        return data

    def FD(self, data, LP):
        self.model.eval()
        orign = torch.tensor(data, dtype=torch.float32)
        datain = orign.unsqueeze(0).to(self.device)
        out = self.model(datain)
        out = out.cpu().detach().numpy()
        self.out_lab = np.vstack((self.out_lab, out))

        predicted_classes = np.argmax(out)

        self.ori_lab = np.vstack((self.ori_lab, self.label[LP]))        

        max_index = predicted_classes
        self.fd_id += 1
        info = [self.fd_id, LP, self.category[max_index]]
        self.record_accuracy(self.fd_id, LP, max_index, self.category[max_index])

        return out, info
    
    def caculate_cm(self, ori_lab, out_lab):
        train_cm = confusion_matrix(np.argmax(ori_lab, axis=1), np.argmax(out_lab, axis=1))

        TP = np.diag(train_cm)
        FP = np.sum(train_cm, axis=0) - TP
        FN = np.sum(train_cm, axis=1) - TP
        TN = np.sum(train_cm) - (TP + FP + FN)
        
        precision = np.mean(TP / (TP + FP))
        recall = np.mean(TP / (TP + FN))
        f1_score = 2 * (precision * recall) / (precision + recall)
        accuracy = np.sum(TP) / np.sum(train_cm)
        
        return TP, FP, FN, TN, precision, recall, f1_score, accuracy

    def FauluDiagnosis(self): 
        global mavs, breakflag, stop_flag, FD_LOG
        
        self.tShow = threading.Thread(target=self.ShowWin, args=())
        self.tShow.start()
        
        start_fd = True
        full = False
        while True:
            self.lastTime = self.lastTime + (1.0/self.fd_hz)
            sleepTime = self.lastTime - time.time()
            if sleepTime > 0:
                time.sleep(sleepTime)
            else:
                self.lastTime = time.time()

            if not full:
                if all(mav.mavAccB.is_pool_full() for mav in mavs):
                    full = True
            
            if full:
                source_acc, source_gyro = self.get_data(mavs[0].mavAccB), self.get_data(mavs[0].mavGyro)
                data_2_model = np.hstack((source_acc, source_gyro))
                if RflyCtrl.MAVREG.FD_LOG_PHASE in self.label and start_fd:
                    LP = RflyCtrl.MAVREG.FD_LOG_PHASE
                    with self.fd_lock: 
                        _, info = self.FD(data_2_model, LP)  
                    FD_LOG.put(info)
                    
            if breakflag:
                acc_log = f"\nTrue/False Number of diagnosis results: [{self.right_cnt} / {self.fault_cnt}] \nAccuracy: {(self.right_cnt / self.num) * 100}%%\n"
                
                TP, FP, FN, TN, precision, recall, f1_score, accuracy = self.caculate_cm(self.ori_lab[1:], self.out_lab[1:])
                print("TP: {}".format(TP))
                print("FP: {}".format(FP))
                print("FN: {}".format(FN))
                print("TN: {}".format(TN))
                print("Precision: {:.4f}  Recall: {:.4f}  F1 Score: {:.4f}  Accuracy: {:.4f}".format(precision, recall, f1_score, accuracy))
                index_log = f"\nTP: {TP} \nFP: {FP} \nFN: {FN} \nTN: {TN} \nPrecision: {precision} \nRecall: {recall} \nF1 Score: {f1_score} \nAccuracy: {accuracy}"

                info = acc_log + self.fault_log + index_log
                FD_LOG.put(info)
                stop_flag = True

                break

class Sycn_Reg:
    phase_container = {}

    def get_step(phase_container, ID):

        key_min, min_val = min(phase_container.items(), key=lambda x: x[1])
        key_max, max_val = max(phase_container.items(), key=lambda x: x[1])

        if ID == key_max:
            step = max_val // min_val
        elif ID == key_min:
            step = 1
        
        return step

class RflyMav:
    def __init__(self, ID, IP, Port, Connect='sim', LocatSources='Dtcap') -> None:

        self.ID = ID 
        self.IP = IP
        self.Port = Port
        self.Connect = Connect
        self.LocatSources = LocatSources


        self.hz = 500
        self.is_alive = False
        self.round_over = False
        self.stage = ['none','none']

        if self.Connect == 'real':
            self.mavID = int(self.Port) - 15500
            self.mav = PX4MavCtrler.PX4MavCtrler(self.mavID, self.IP, 'Direct', self.Port)
        elif self.Connect == 'sim':
            self.mav = PX4MavCtrler.PX4MavCtrler(self.ID, self.IP)

        self.caseIndex = 0
        self.caseID, self.caseNum, self.caseInfo, self.ctrlSeq = self.get_cmd()
        print(f'mav{self.ID} case {self.caseID} ctrlSeq: ',self.ctrlSeq)

        if self.LocatSources == 'Mocap':
            self.vrpn_pos = np.array([0.0, 0.0, 0.0])
            self.vrpn_vel = np.array([0.0, 0.0, 0.0])
            self.yaw = 0
            self.px4_yaw = 0
            self.offset_yaw = 0
            self.init_px4_yaw = 0
            self.vrpn_quat = np.array([1.0, 0.0, 0.0, 0.0])
            self.ini_yaw_success = False
            self.yaw_err = 0

            self.pose_rec_task = None
            self.twist_rec_task = None 

            self.pose_topic = "/vrpn_client_node/" + "droneyee0" + f"{self.mavID}" + "/pose"
            self.twist_topic = "/vrpn_client_node/" + "droneyee0" + f"{self.mavID}" + "/twist"

            self.check_ok = self.check_status()

        # Init control sequence class object
        self.CFID = RflyCtrl.CmdCtrl(self.mav,self.ID,self.Connect) 
        self.CID1OBJ = self.CFID.CID1
        self.CID2OBJ = self.CFID.CID2

        # Init flight index variable
        self.EXITFLAG = False
        self.hz = 250
        self.is_alive = False
        self.round_over = False

    def check_status(self):
        def get_command_output(cmd):
            result = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            return result.stdout.decode('utf-8')
        def ping_website(website):   
            cmd = ["ping", "-c", "1", website]
            try:  
                process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)  
                stdout, stderr = process.communicate()  
                return process.returncode == 0  
            except Exception as e:  
                return False

        res = get_command_output('rostopic list')
        if self.pose_topic not in res:
            print("Error, no pose_topic")
            return False
        
        
        if not ping_website(self.IP):
            print('{} unreachable'.format(self.IP))
            return False
        return True
    
    def init_yaw(self):    
        self.offset_yaw = self.yaw - self.px4_yaw
        self.px4_yaw_d = self.px4_yaw
        self.init_px4_yaw = self.px4_yaw
        self.ini_yaw_success = True

    def update_pose(self):
        while True:
            if self.round_over:
                break

            msg = rospy.wait_for_message(self.pose_topic, PoseStamped, timeout=None)    
            self.vrpn_pos[0] = msg.pose.position.x
            self.vrpn_pos[1] = msg.pose.position.y
            self.vrpn_pos[2] = msg.pose.position.z
            self.vrpn_quat[0] = msg.pose.orientation.w
            self.vrpn_quat[1] = msg.pose.orientation.x
            self.vrpn_quat[2] = msg.pose.orientation.y
            self.vrpn_quat[3] = msg.pose.orientation.z
            [roll, pitch, self.yaw] = quat2euler(self.vrpn_quat)
            self.px4_yaw = self.mav.uavAngEular[2]
            self.battery = self.mav.batInfo[0]
    
            self.send_vision_capture()

    def update_twist(self):
        while True:
            if self.round_over:
                break

            msg = rospy.wait_for_message(self.twist_topic, TwistStamped, timeout=None)
            
            self.vrpn_vel[0] = msg.twist.linear.x
            self.vrpn_vel[1] = msg.twist.linear.y
            self.vrpn_vel[2] = msg.twist.linear.z

    def send_vision_capture(self):
        def limit_yaw(yaw):
            res = yaw
            if math.fabs(res) > math.pi:
                if res > 0:
                    res = res - 2 * math.pi
                else:
                    res = 2 * math.pi + res
            return res
        
        if not self.ini_yaw_success:
            return
        yaw = -self.offset_yaw - self.yaw
        yaw = limit_yaw(yaw)
        yaw_err =  self.offset_yaw
        x = self.vrpn_pos[0] * math.cos(yaw_err) - self.vrpn_pos[1] * math.sin(yaw_err)
        y = -self.vrpn_pos[1] * math.cos(yaw_err) - self.vrpn_pos[0] * math.sin(yaw_err)
        z = -self.vrpn_pos[2]
        self.mav.send_vision_position(x, y, z, yaw)
    
    def init_params(self):
        # Init test case
        self.lastTime = time.time()
        self.MavCmdInd = 0
        self.MavCmdNum = len(self.ctrlSeq)
        self.frame = 1

        # Init control sequence class object
        self.CFID = RflyCtrl.CmdCtrl(self.mav,self.frame,self.ID) 
        self.CID1OBJ = self.CFID.CID1
        self.CID2OBJ = self.CFID.CID2

        # Init data pool which received from px4 
        self.mavPosNED = DataPool()      # Estimated Local Pos (related to takeoff position) from PX4 in NED frame
        self.mavVelNED = DataPool()      # Estimated local velocity from PX4 in NED frame
        self.mavAccB = DataPool()        # Estimated acc from PX4
        self.mavGyro = DataPool()        # Estimated Gyro from PX4
        self.mavMag = DataPool()         # Estimated Mag from PX4
        self.mavVibr = DataPool()        # Estimated vibration xyz from PX4
        self.mavAngEular = DataPool()    # Estimated Eular angles from PX4
        self.mavAngRate = DataPool()     # Estimated angular rate from PX4
        self.mavAngQuatern = DataPool()  # Estimated AngQuatern from PX4
        self.imu_filter = IMUFilter(ma_window_size=20, lowpass_alpha=0.1)

        self.EXITFLAG = False
    
    def init_connection(self):
        if self.Connect == 'real' and self.LocatSources == 'Mocap': 
            if not self.check_ok:
                print("No check! Stop init")
                return
            
            self.pose_rec_task = threading.Thread(target=self.update_pose, args=())
            self.twist_rec_task = threading.Thread(target=self.update_twist, args=())
            self.pose_rec_task.start()
            self.twist_rec_task.start()

            print("Start init Mavloop mode!")
            self.mav.InitMavLoop(UDPMode=2)
            time.sleep(3)

            self.init_yaw()
            time.sleep(5)

            print("Start init Offboard mode!")
            self.mav.initOffboard()
            time.sleep(3)

        elif self.Connect == 'sim' and self.LocatSources == 'None':
            print("Start init DtSim Mavloop mode!")
            self.mav.InitMavLoop()
            time.sleep(2.5)

            self.mav.InitTrueDataLoop()
            time.sleep(0.5)

            time.sleep(5)
            self.mav.initOffboard()
            time.sleep(3)

    def End_connection(self):
        self.mav.endMavLoop() 
        time.sleep(1)
    
    def end_connection(self):
        self.mav.EndTrueDataLoop()
        time.sleep(0.5)
        self.mav.endMavLoop() 
        time.sleep(1)

    def get_cmd(self):
        db = RflyDB.RflyDB(json_path)
        caseNum = len(db.GET_CASEID()[self.ID - 1])
        caseID = db.GET_CASEID()[self.ID - 1][self.caseIndex]
        caseInfo = db.GET_CASEINFO(caseID)
        ctrlSeq = caseInfo.get('ControlSequence')
        case = re.split(';',ctrlSeq)
        cmd = np.array([])
        for i in range(len(case)):
            cmd = np.append(cmd,case[i])
        
        return caseID, caseNum, caseInfo, cmd
    
    def trigger(self,ctrlseq):
        global mission
        cmdseq = ctrlseq # '2,3,0,0,-20'
        cmdseq = re.findall(r'-?\d+\.?[0-9]*',cmdseq) # ['2', '3', '0', '0', '-20']
        cmdCID = cmdseq[0]
        if  cmdCID in self.CFID.CID:
            FID = self.CFID.FIDPro(cmdCID)
            # if has param
            if len(cmdseq) > 2:
                # get param
                param = cmdseq[2:len(cmdseq)]
                param = [float(val) for val in param]
                FID[cmdseq[1]](param)

            else:
                '''mission mode'''
                if cmdseq[1] == '8':
                    FID[cmdseq[1]](mission)
                else:
                    FID[cmdseq[1]]()
        else:
            print(f'mav{self.ID} Command input error, please re-enter')
    
    def Run(self):
        # Start time and end time (unlock after startup to prevent the ground station from not starting timing)
        print(f'mav{self.ID} Sim start')
        while True: 
            if self.caseIndex >= self.caseNum:
                print(f'mav{self.ID} all case test finish!')
                break

            # step1: init_connection
            self.init_params()
            self.init_connection()
            self.is_alive = True

            # step2: run
            while True:
                # 250HZ receiving data
                self.lastTime = self.lastTime + (1.0/self.hz)
                sleepTime = self.lastTime - time.time()
                if sleepTime > 0:
                    time.sleep(sleepTime)
                else:
                    self.lastTime = time.time()
                
                # Starting receiving data at 250HZ using fault diagnoise
                mavTimestamp = self.mav.uavTimeStmp
                mavAccB = self.mav.uavAccB   
                mavGyro = self.mav.uavGyro

                mavAccB_filtered = self.imu_filter.filter_acc(mavAccB)
                mavGyro_filtered = self.imu_filter.filter_gyro(mavGyro)

                self.mavAccB.add_data(timestamp=mavTimestamp, data=mavAccB_filtered)
                self.mavGyro.add_data(timestamp=mavTimestamp, data=mavGyro_filtered)      

                # Processing instruction sequence
                self.stage = self.ctrlSeq[self.MavCmdInd]
                self.trigger(self.ctrlSeq[self.MavCmdInd])
                if re.findall(r'-?\d+\.?[0-9]*',self.ctrlSeq[self.MavCmdInd])[0] == '1' and self.CID1OBJ.isDone == 1 or re.findall(r'-?\d+\.?[0-9]*',self.ctrlSeq[self.MavCmdInd])[0] == '2' and self.CID2OBJ.isDone == 1:
                    self.MavCmdInd = self.MavCmdInd + 1
                    # print(f'mav{self.ID}: Process: [{self.MavCmdInd} / {self.MavCmdNum}]')
                
                if self.MavCmdInd >= self.MavCmdNum and self.EXITFLAG == False:
                    self.EXITFLAG = True
                    print(f'mav{self.ID}: CaseID {self.caseID} test completed')
                    break
                
            self.caseIndex += 1
            self.end_connection()
            self.round_over = True

if __name__ == "__main__":

    json_path = os.path.join(sys.path[0], 'db_FD.json')
    rospy.init_node('rflydt', anonymous=False)

    '''[ID, IP, Port, is_Mocap]'''
    drones = [
        # {
        #     "ID": 1,
        #     "IP": '127.0.0.1',
        #     "Port": '20100',
        #     "Connect": 'sim',
        #     "LocatSources": 'None'
        # },
        {
            "ID": 1,
            "IP": '192.168.151.108',
            "Port": '15508',
            "Connect": 'real',
            "LocatSources": 'Mocap'
        }
    ]
    mav_num = len(drones)
    sw = RflySW.RflySW(mav_num)

    mavs = []
    threads = []
    for drone in drones:
        ID, IP, Port, Connect, LocatSource = drone['ID'], drone['IP'], drone['Port'], drone['Connect'], drone['LocatSources']
        mav = RflyMav(ID, IP, Port, Connect, LocatSource)
        thread = threading.Thread(target=mav.Run)
        mavs.append(mav)
        threads.append(thread)
    
    for thread in threads:
        thread.start()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model_name = 'diagModel_with_real_data_c127.pth' # change to your model
    folder_name = model_name.split('.')[0]
    path = os.path.join(sys.path[0], '..', '..', '..', 'model', 'real2real', folder_name, model_name) # change to your path
    from utilits.FD_CNN_LSTM import *
    model = torch.load(path, map_location=torch.device('cpu'))
    model.to(device)

    all_messages = ""
    breakflag = False
    stop_flag = False
    FD_process_start = False
    FD_process_end = False
    log_cnt = 2
    FD_LOG = queue.Queue(maxsize=50)

    FDIns = FDMav(mavs, model, device)
    fdt = threading.Thread(target=FDIns.FauluDiagnosis)

    lastTime = time.time()
    hz = 500
    while True:
        lastTime = lastTime + (1.0/hz)
        sleepTime = lastTime - time.time()
        if sleepTime > 0:
            time.sleep(sleepTime)
        else:
            lastTime = time.time()

        if RflyCtrl.MAVREG.FD_LOG_PHASE == '6' and FD_process_start == False:
            FD_process_start = True
            fdt.start()
        
        if RflyCtrl.MAVREG.FD_LOG_PHASE == '7' and FD_process_end == False:
            FD_process_end = True
            breakflag = True

        if all(mav.round_over for mav in mavs):
            break

    
    


    