#!/usr/bin/env python3
import mujoco as mj
import numpy as np
from mujoco_base import MuJoCoBase
from mujoco.glfw import glfw
import rospy
import rospkg
from std_msgs.msg import Float32MultiArray,Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import time
from scipy.spatial.transform import Rotation as R
import math as m
# from sympy import symbols, Matrix, Function, diff
# import sympy as sp 

init_joint_pos_mujoco = np.array([-0.157, 0.247, 0.157, -0.247, -0.157, 0.247, 0.157, -0.247, 0.00, 0.913, -1.85, 0.00, 0.913, -1.85])
init_joint_pos_control = np.array([0, 0.07, 0, 0.07, 0.00, 0.913, -1.85, 0.00, 0.913, -1.85 ])
init_base_pos = np.array([-0.08, 0, 0])
init_base_eular_zyx = np.array([3.14, -0., 0.0])
imu_eular_bias = np.array([0.0, 0.0, 0.0])
l_1 = 0.13
l_2 = 0.15
l_3 = 0.27
l_4 = 0.27
l_5 = 0.15
alpha_A_offset = np.pi - 0.597
alpha_B_offset = 0.597
L_virtual_offset = 0.12
phi_virtual_offset = 0.0
# Lf = 0.15
# Lt = 0.27
# Lcro = 0.12
DEFAULT_MOTOR_POS = np.array([0.0, 0.0]) # Default motor angles [alpha_A, alpha_B]
DEFAULT_MOTOR_VEL = np.array([0.0, 0.0]) # Default motor velocities
DEFAULT_MOTOR_TORQUE = np.array([0.0, 0.0]) # Default motor torques
DEFAULT_JACOBIAN = np.eye(2) # Default Jacobian (identity)
DEFAULT_JACOBIAN_INV = np.eye(2) # Default Jacobian inverse

class WheelLeggedSim(MuJoCoBase):
  def __init__(self, xml_path):
    super().__init__(xml_path)
    self.simend = 1000.0
    self.sim_rate = 1000.0
    # print('Total number of DoFs in the model:', self.model.nv)
    # print('Generalized positions:', self.data.qpos)  
    # print('Generalized velocities:', self.data.qvel)
    # print('Actuator forces:', self.data.qfrc_actuator)
    # print('Actoator controls:', self.data.ctrl)
    # mj.set_mjcb_control(self.controller)
    # * Set subscriber and publisher

    # initialize target joint position, velocity, and torque
    self.targetPos = init_joint_pos_control
    self.targetVel = np.zeros(10)
    self.targetTorque = np.zeros(10)
    self.targetKp = np.zeros(10)
    self.targetKd = np.zeros(10)

    self.pubJoints = rospy.Publisher('/jointsPosVel', Float32MultiArray, queue_size=10)
    self.pubOdom = rospy.Publisher('/ground_truth/state', Odometry, queue_size=10)
    self.pubImu = rospy.Publisher('/base_imu', Imu, queue_size=10)
    self.pubRealTorque = rospy.Publisher('/realTorque', Float32MultiArray, queue_size=10)

    rospy.Subscriber("/targetTorque", Float32MultiArray, self.targetTorqueCallback) 
    rospy.Subscriber("/targetPos", Float32MultiArray, self.targetPosCallback) 
    rospy.Subscriber("/targetVel", Float32MultiArray, self.targetVelCallback)
    rospy.Subscriber("/targetKp", Float32MultiArray, self.targetKpCallback)
    rospy.Subscriber("/targetKd", Float32MultiArray, self.targetKdCallback)
    #set the initial joint position
    self.data.qpos[:3] = init_base_pos
    # init rpy to init quaternion
    self.data.qpos[3:7] = R.from_euler('xyz', init_base_eular_zyx).as_quat()
    self.data.qpos[-14:] = init_joint_pos_mujoco

    self.data.qvel[:3] = np.array([0, 0, 0])
    self.data.qvel[-14:] = np.zeros(14)

    # * show the model
    mj.mj_step(self.model, self.data)
    # enable contact force visualization
    self.opt.flags[mj.mjtVisFlag.mjVIS_CONTACTFORCE] = True

    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(
        self.window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    # Update scene and render
    mj.mjv_updateScene(self.model, self.data, self.opt, None, self.cam,
                        mj.mjtCatBit.mjCAT_ALL.value, self.scene)
    mj.mjr_render(viewport, self.scene, self.context)

  def _five_bar_fk_cartesian(self, alpha_A, alpha_B):
    P_A_x = l_2 * m.cos(alpha_A)
    P_A_y = l_2 * m.sin(alpha_A)
    P_C_x = l_1 + l_5 * m.cos(alpha_B)
    P_C_y = l_5 * m.sin(alpha_B)
    
    a = 2 * (P_A_x - P_C_x) * l_3
    b = 2 * (P_A_y - P_C_y) * l_3
    l_sq = (P_A_x - P_C_x)**2 + (P_A_y - P_C_y)**2
    c = l_4**2 - l_3**2 - l_sq
    
    sqrt_term = a**2 + b**2 - c**2
    if sqrt_term < 0:
        return None
        
    theta1 = m.atan2(b + m.sqrt(sqrt_term), a + c) * 2
    theta2 = m.atan2(b - m.sqrt(sqrt_term), a + c) * 2
    
    theta1 = theta1 if theta1 >= 0 else theta1 + 2 * m.pi
    theta2 = theta2 if theta2 >= 0 else theta2 + 2 * m.pi

    selected_theta = theta2 if theta1 >= m.pi / 2 else theta1
    
    x_f = P_A_x + l_3 * m.cos(selected_theta)
    y_f = P_A_y + l_3 * m.sin(selected_theta)

    return x_f, y_f


  # 1. 正向运动学 (五连杆电机角度 -> 虚拟腿参数)
  def five_bar_to_virtual_leg_fk(self, alpha_A, alpha_B):
    """
    Forward Kinematics: Calculates virtual leg parameters from five-bar motor angles.
    Based on Section "1. 正向运动学 (Forward Kinematics)" from the formulas.

    Args:
        alpha_A, alpha_B: Motor angles (radians).
        l_1, l_2, l_3, l_4, l_5: Link lengths.
        fk_config_choice: Configuration choice for Cartesian FK.

    Returns:
        (L_virtual, phi_virtual, x_f, y_f) or None if unreachable.
        L_virtual: Length of the virtual leg (L in formulas).
        phi_virtual: Angle of the virtual leg (φ in formulas).
        x_f, y_f: Cartesian coordinates of the foot.
    """
    alpha_A = alpha_A + np.pi - 0.597
    alpha_B = - alpha_B + 0.597 
    # print("alpha_A, alpha_B fk: ", alpha_A, alpha_B)
    foot_cartesian = self._five_bar_fk_cartesian(alpha_A, alpha_B)

    if foot_cartesian is None:
        return None

    x_f, y_f = foot_cartesian
    x_f = x_f - l_1 / 2
    # Step 4: Calculate virtual leg parameters L and φ
    L_virtual = np.sqrt(x_f**2 + y_f**2) - 0.32
    phi_virtual = np.arctan2(y_f, x_f) - np.pi/2
    # print("L_virtual, phi_virtual: ", L_virtual, phi_virtual)

    return L_virtual, phi_virtual, x_f, y_f

  # 2. 逆向运动学 (虚拟腿参数 -> 五连杆电机角度)
  def virtual_leg_to_five_bar_ik(self, L_virtual, phi_virtual):
    """
    Inverse Kinematics: Calculates five-bar motor angles from virtual leg parameters.
    Based on Section "2. 逆向运动学 (Inverse Kinematics)" from the formulas.
    ik_config_choice (0-3) selects one of up to four possible IK solutions.

    Args:
        L_virtual: Desired length of the virtual leg (L in formulas).
        phi_virtual: Desired angle of the virtual leg (φ in formulas).
        l_1, l_2, l_3, l_4, l_5: Link lengths.
        ik_config_choice: Integer (0 to 3) to select among possible IK solutions.
                           Choice for P1: (ik_config_choice // 2) % 2
                           Choice for P2: ik_config_choice % 2

    Returns:
        (alpha_A, alpha_B) motor angles in radians, or None if unreachable/invalid.
    """
    # Step 1: Calculate target foot coordinates (x_f, y_f)
    L_virtual = L_virtual + 0.32
    phi_virtual = np.pi/2 + phi_virtual
    x_f = L_virtual * np.cos(phi_virtual) + l_1/2
    y_f = L_virtual * np.sin(phi_virtual)

    a = 2*l_2*x_f
    b = 2*l_2*y_f
    c = x_f**2 + y_f**2 + l_2**2 - l_3**2
    d = 2*l_5*(x_f - l_1)
    e = 2*l_5*y_f
    f = (x_f - l_1)**2 + l_5**2 + y_f**2 - l_4**2

    sqrt_term_alpha_r = a**2 + b**2 - c**2
    if sqrt_term_alpha_r < 0:
        print("错误: 右腿目标不可达 (alpha 计算)。")
        return None
    sqrt_term_beta_r = d**2 + e**2 - f**2
    if sqrt_term_beta_r < 0:
        print("错误: 右腿目标不可达 (beta 计算)。")
        return None
    
    alpha1_r = m.atan2(b + m.sqrt(sqrt_term_alpha_r), a + c) * 2
    alpha2_r = m.atan2(b - m.sqrt(sqrt_term_alpha_r), a + c) * 2
    beta1_r = m.atan2(e + m.sqrt(sqrt_term_beta_r), d + f) * 2
    beta2_r = m.atan2(e - m.sqrt(sqrt_term_beta_r), d + f) * 2

    if alpha1_r >= 0:
        alpha1_r = alpha1_r
    else:
        alpha1_r = alpha1_r + 2 * m.pi

    if alpha2_r >= 0:
        alpha2_r = alpha2_r
    else:
        alpha2_r = alpha2_r + 2 * m.pi

    if alpha1_r >= m.pi / 2:
        alpha_A = alpha1_r
    else:
        alpha_A = alpha2_r

    # if 0 <= beta1_r <= m.pi / 2:
    alpha_B = np.pi - beta1_r
    # else:
    #     alpha_B = beta2_r

    # alpha1_r = alpha1_r if alpha1_r >= 0 else alpha1_r + 2 * m.pi
    # alpha2_r = alpha2_r if alpha2_r >= 0 else alpha2_r + 2 * m.pi
    # alpha_A = alpha1_r if alpha1_r >= m.pi / 2 else alpha2_r
    # alpha_B = beta1_r if 0 <= beta1_r <= m.pi / 2 else beta2_r

    alpha_A = alpha_A - np.pi + 0.597
    
    print("alpha_A, alpha_B ik: ", alpha_A, alpha_B)

    return  alpha_A, alpha_B

  # 3. 雅可比矩阵 (Jacobian Matrix)
  def calculate_five_bar_virtual_leg_jacobian(self, alpha_A, alpha_B):
    """
    Calculates the Jacobian matrix mapping motor velocities to virtual leg parameter velocities.
    J_virtual_leg: [dL_virtual/dt, dphi_virtual/dt]^T = J_virtual_leg * [d_alpha_A/dt, d_alpha_B/dt]^T
    Based on Section "3. 雅可比矩阵 (Jacobian Matrix)" from the formulas.

    Args:
        alpha_A, alpha_B: Current motor angles (radians).
        l_1, l_2, l_3, l_4, l_5: Link lengths.
        fk_config_choice: Configuration choice for Cartesian FK.

    Returns:
        A 2x2 numpy array for J_virtual_leg, or None if singular or foot unreachable.
    """
    # Get current foot position (x_f, y_f) and virtual leg parameters (L, phi) using FK
    fk_result = self.five_bar_to_virtual_leg_fk(alpha_A, alpha_B)
    if fk_result is None:
        return None
    L_virtual, phi_virtual, x_f, y_f = fk_result
    x_f = x_f + l_1 / 2
    if L_virtual < 1e-9 : # Avoid division by zero if foot is at origin
        return None

    alpha_A = alpha_A + np.pi - 0.597
    alpha_B = - alpha_B + 0.597
    L_virtual = L_virtual + 0.32
    phi_virtual = np.pi/2 + phi_virtual
    # Calculate intermediate points P1 and P2
    P1_x = l_2 * np.cos(alpha_A)
    P1_y = l_2 * np.sin(alpha_A)
    P2_x = l_1 + l_5 * np.cos(alpha_B) # Pivot B is at (l_1, 0)
    P2_y = l_5 * np.sin(alpha_B)

    # A. Cartesian Jacobian J_cartesian = M_A_inv @ M_B
    # Form M_A matrix
    M_A = np.array([
        [x_f - P1_x, y_f - P1_y],
        [x_f - P2_x, y_f - P2_y]
    ])

    # Form M_B matrix
    sA, cA = np.sin(alpha_A), np.cos(alpha_A)
    sB, cB = np.sin(alpha_B), np.cos(alpha_B)

    term_B11 = (x_f - P1_x) * (-l_2 * sA) + (y_f - P1_y) * (l_2 * cA)
    term_B22 = (x_f - P2_x) * (-l_5 * sB) + (y_f - P2_y) * (l_5 * cB)
    
    M_B = np.array([
        [term_B11, 0],
        [0, term_B22]
    ])

    try:
        M_A_inv = np.linalg.inv(M_A)
    except np.linalg.LinAlgError: # M_A is singular
        return None
        
    J_cartesian = M_A_inv @ M_B
    # J_cartesian maps [d_alpha_A, d_alpha_B]^T to [dx_f, dy_f]^T

    # B. Coordinate Transformation Jacobian J_transform_xy_to_Lphi
    J_transform_xy_to_Lphi = np.array([
        [np.cos(phi_virtual), np.sin(phi_virtual)],
        [-np.sin(phi_virtual) / L_virtual, np.cos(phi_virtual) / L_virtual]
    ])

    # C. Final Jacobian for virtual leg parameters J_virtual_leg
    J_virtual_leg = J_transform_xy_to_Lphi @ J_cartesian
    
    return J_virtual_leg

  def targetPosCallback(self, data):
    self.targetPos = data.data

  def targetTorqueCallback(self, data):
    self.targetTorque = data.data

  def targetVelCallback(self, data):
    self.targetVel = data.data

  def targetKpCallback(self, data):
    self.targetKp = data.data

  def targetKdCallback(self, data):
    self.targetKd = data.data

  def reset(self):
    # Set camera configuration
    self.cam.azimuth = 89.608063
    self.cam.elevation = -11.588379
    self.cam.distance = 5.0
    self.cam.lookat = np.array([0.0, 0.0, 1.5])

  # def controller(self, model, data):
  #   self.data.ctrl[0] = 100
  #   pass

  def simulate(self):
    publish_time = self.data.time
    torque_publish_time = self.data.time
    sim_epoch_start = time.time()
    while not glfw.window_should_close(self.window):
      simstart = self.data.time
    #   print("========================================")
      phi_des_L = -self.targetPos[0]
      L_des_L = self.targetPos[1]
      # targetPos_motorL will be a tuple (motor_A_des, motor_B_des) or None
      targetPos_motorL_tuple = self.virtual_leg_to_five_bar_ik(L_des_L, phi_des_L)
      # Initialize defaults for this iteration
      Jac_L = DEFAULT_JACOBIAN 
      targetVel_motorL = DEFAULT_MOTOR_VEL
      targetTor_motorL = DEFAULT_MOTOR_TORQUE 
      Jac_L_inv = DEFAULT_JACOBIAN_INV
      # Use a mutable numpy array for targetPos_motorL for PD control if IK fails
      # current_qpos_motor_L = np.array([self.data.qpos[7], self.data.qpos[9]])
      # targetPos_motorL_for_pd = current_qpos_motor_L.copy() # Default to current if IK fails

      if targetPos_motorL_tuple is not None:
          targetPos_motorL_for_pd = np.array(targetPos_motorL_tuple) # IK success, use target

          # Calculate Jacobian AT THE TARGET motor positions (if this is the intent)
          # Or use current motor positions if Jacobian should reflect current state            
          # For feedforward velocity and torque based on desired virtual state,
          # Jacobian at target motor positions can be appropriate.
          # The user's error line suggests using targetPos_motorL for Jacobian args.
          temp_Jac_L = self.calculate_five_bar_virtual_leg_jacobian(
              targetPos_motorL_tuple[0], # Desired motor_A_L from IK
              targetPos_motorL_tuple[1]  # Desired motor_B_L from IK
          )
          if temp_Jac_L is not None:
              Jac_L = temp_Jac_L.copy() # Safe to call .copy()
              # else Jac_L remains DEFAULT_JACOBIAN

          if not np.array_equal(Jac_L, DEFAULT_JACOBIAN):
              try:
                  Jac_L_inv = np.linalg.inv(Jac_L)
                  phi_dot_des_L = -self.targetVel[0]
                  L_dot_des_L = self.targetVel[1]
                  target_virtual_vel_L_ordered = np.array([L_dot_des_L, phi_dot_des_L]) # Order for Jacobian [L, phi]
                  targetVel_motorL = Jac_L_inv @ target_virtual_vel_L_ordered
              except np.linalg.LinAlgError:
                  rospy.logwarn_throttle(1.0, "Left Leg: Jacobian (at target) singular for velocity calculation.")
                  Jac_L_inv = DEFAULT_JACOBIAN_INV 
                  targetVel_motorL = DEFAULT_MOTOR_VEL
          # else targetVel_motorL remains DEFAULT_MOTOR_VEL

          # Torque calculation using Jac_L (at target configuration)
          F_phi_des_L = -self.targetTorque[0] 
          F_L_des_L = self.targetTorque[1]   
          if not np.array_equal(Jac_L, DEFAULT_JACOBIAN):
              F_virtual_vector_L = np.array([F_L_des_L, F_phi_des_L]) # Order for Jacobian [F_L, F_phi]
              calculated_torques_L = Jac_L.T @ F_virtual_vector_L
              targetTor_motorL = np.array(calculated_torques_L)
              # else targetTor_motorL remains DEFAULT_MOTOR_TORQUE
      else: 
          rospy.logwarn_throttle(1.0, "Left Leg: IK failed. Using default motor targets (hold current pos for PD).")
          # targetPos_motorL_for_pd is already set to current
          # Jac_L, Jac_L_inv, targetVel_motorL, targetTor_motorL remain their initialized defaults.

      # --- Right Leg Calculations (mirror Left Leg logic) ---
      phi_des_R = -self.targetPos[2]
      L_des_R = self.targetPos[3]
      targetPos_motorR_tuple = self.virtual_leg_to_five_bar_ik(L_des_R, phi_des_R)
      
      Jac_R = DEFAULT_JACOBIAN
      targetVel_motorR = DEFAULT_MOTOR_VEL
      targetTor_motorR = DEFAULT_MOTOR_TORQUE
      Jac_R_inv = DEFAULT_JACOBIAN_INV
      # current_qpos_motor_R = np.array([self.data.qpos[11], self.data.qpos[13]])
      # targetPos_motorR_for_pd = current_qpos_motor_R.copy()
    #   print("========================================")
      if targetPos_motorR_tuple is not None:
          targetPos_motorR_for_pd = np.array(targetPos_motorR_tuple)
          temp_Jac_R = self.calculate_five_bar_virtual_leg_jacobian(
              targetPos_motorR_tuple[0], 
              targetPos_motorR_tuple[1]
          )
          if temp_Jac_R is not None:
              Jac_R = temp_Jac_R.copy()

          if not np.array_equal(Jac_R, DEFAULT_JACOBIAN):
              try:
                  Jac_R_inv = np.linalg.inv(Jac_R)
                  phi_dot_des_R = -self.targetVel[2]
                  L_dot_des_R = self.targetVel[3]
                  target_virtual_vel_R_ordered = np.array([L_dot_des_R, phi_dot_des_R])
                  targetVel_motorR = Jac_R_inv @ target_virtual_vel_R_ordered
              except np.linalg.LinAlgError:
                  rospy.logwarn_throttle(1.0, "Right Leg: Jacobian (at target) singular for velocity calculation.")
                  Jac_R_inv = DEFAULT_JACOBIAN_INV
                  targetVel_motorR = DEFAULT_MOTOR_VEL
                
          F_phi_des_R = -self.targetTorque[2]
          F_L_des_R = self.targetTorque[3]
          if not np.array_equal(Jac_R, DEFAULT_JACOBIAN):
              F_virtual_vector_R = np.array([F_L_des_R, F_phi_des_R])
              calculated_torques_R = Jac_R.T @ F_virtual_vector_R
              targetTor_motorR = np.array(calculated_torques_R)
      else:
          rospy.logwarn_throttle(1.0, "Right Leg: IK failed. Using default motor targets (hold current pos for PD).")

      while (self.data.time - simstart <= 1.0/60.0 and not self.pause_flag):

        if (time.time() - sim_epoch_start >= 1.0 / self.sim_rate):
          # MIT control-
          self.data.ctrl[0] = (targetTor_motorL[0] + self.targetKp[0] * (targetPos_motorL_for_pd[0] - self.data.qpos[7]) + self.targetKd[0] * (targetVel_motorL[0] - self.data.qvel[6]))
          self.data.ctrl[1] = (targetTor_motorL[1] + self.targetKp[1] * (targetPos_motorL_for_pd[1] - self.data.qpos[9]) + self.targetKd[1] * (targetVel_motorL[1] - self.data.qvel[8]))
          # self.data.ctrl[2] = self.targetTorque[2] + self.targetKp[2] * (self.targetPos[2] - self.data.qpos[15]) + self.targetKd[2] * (self.targetVel[2] - self.data.qvel[14])
          self.data.ctrl[2] = (targetTor_motorR[0] + self.targetKp[2] * (targetPos_motorR_for_pd[0] - self.data.qpos[11]) + self.targetKd[2] * (targetVel_motorR[0] - self.data.qvel[10]))
          self.data.ctrl[3] = (targetTor_motorR[1] + self.targetKp[3] * (targetPos_motorR_for_pd[1] - self.data.qpos[13]) + self.targetKd[3] * (targetVel_motorR[1] - self.data.qvel[12]))
          # self.data.ctrl[5] = self.targetTorque[5] + self.targetKp[5] * (self.targetPos[5] - self.data.qpos[20]) + self.targetKd[5] * (self.targetVel[5] - self.data.qvel[19])
          self.data.ctrl[4:10] = self.targetTorque[4:10] + self.targetKp[4:10] * (self.targetPos[4:10] - self.data.qpos[15:21]) + self.targetKd[4:10] * (self.targetVel[4:10] - self.data.qvel[14:20])
          # Step simulation environment
        #   print("ctrl:", self.data.ctrl[0:4])
          mj.mj_step(self.model, self.data)
          sim_epoch_start = time.time()

        
        if (self.data.time - publish_time >= 1.0 / 500.0):
          # * Publish joint positions and velocities
          jointsPosVel = Float32MultiArray()
          # get last 12 element of qpos and qvel
          qp_leg = self.data.qpos[15:21].copy()
          qp_Lvmc = self.five_bar_to_virtual_leg_fk(self.data.qpos[7],self.data.qpos[9])
          L_act_L, phi_act_L = qp_Lvmc[0], qp_Lvmc[1]
          qp_Rvmc = self.five_bar_to_virtual_leg_fk(self.data.qpos[11],self.data.qpos[13])
          L_act_R, phi_act_R = qp_Rvmc[0], qp_Rvmc[1]

          qp_L_ordered = np.array([0.0, 0.0])
          qp_R_ordered = np.array([0.0, 0.0])

          qp_L_ordered = np.array([phi_act_L, L_act_L])
          qp_R_ordered = np.array([phi_act_R, L_act_R])
          # qp_leg = np.atleast_1d(qp_leg)
          
          qp = np.concatenate((qp_L_ordered, qp_R_ordered, qp_leg))
          # print("qp:", qp[0:4])

          qv_leg = self.data.qvel[14:20].copy()
          Jac_L_current = self.calculate_five_bar_virtual_leg_jacobian(self.data.qpos[7], self.data.qpos[9])
          Jac_R_current = self.calculate_five_bar_virtual_leg_jacobian(self.data.qpos[11], self.data.qpos[13])
          # Jac_L_inv = np.linalg.inv(Jac_L)
          qv_L_ordered = DEFAULT_MOTOR_VEL
          if isinstance(Jac_L_current, np.ndarray) and Jac_L_current.shape == (2, 2):
              current_motor_vel_L = np.array([self.data.qvel[6], self.data.qvel[8]])
              try:
                  actual_virtual_vel_L_raw = Jac_L_current @ current_motor_vel_L 
                  qv_L_ordered = np.array([actual_virtual_vel_L_raw[1], actual_virtual_vel_L_raw[0]]) 
              except ValueError as e: 
                  rospy.logwarn_throttle(1.0, f"Publish (active): Left Leg matmul for velocity failed: {e}. Jac_L_current was {Jac_L_current}. Using default qv_L_ordered.")
          else: # MODIFIED: 如果 Jac_L_current 无效或为 None，则记录警告
              rospy.logwarn_throttle(1.0, f"Publish (active): Left Leg Jacobian invalid or None for velocity. Jac_L_current type: {type(Jac_L_current)}, value: {Jac_L_current}. Using default qv_L_ordered.")

          qv_R_ordered = DEFAULT_MOTOR_VEL
          if isinstance(Jac_R_current, np.ndarray) and Jac_R_current.shape == (2, 2):
              current_motor_vel_R = np.array([self.data.qvel[10], self.data.qvel[12]])
              try:
                  actual_virtual_vel_R_raw = Jac_R_current @ current_motor_vel_R 
                  qv_R_ordered = np.array([actual_virtual_vel_R_raw[1], actual_virtual_vel_R_raw[0]])
              except ValueError as e:
                  rospy.logwarn_throttle(1.0, f"Publish (active): Right Leg matmul for velocity failed: {e}. Jac_R_current was {Jac_R_current}. Using default qv_R_ordered.")
          else: # MODIFIED: 如果 Jac_R_current 无效或为 None，则记录警告
              rospy.logwarn_throttle(1.0, f"Publish (active): Right Leg Jacobian invalid or None for velocity. Jac_R_current type: {type(Jac_R_current)}, value: {Jac_R_current}. Using default qv_R_ordered.")

          qv = np.concatenate((qv_L_ordered, qv_R_ordered, qv_leg))

          jointsPosVel.data = np.concatenate((qp, qv))

          self.pubJoints.publish(jointsPosVel)
          # * Publish body pose
          bodyOdom = Odometry()
          pos = self.data.sensor('BodyPos').data.copy()

          #add imu bias
          ori = self.data.sensor('BodyQuat').data.copy()
          ori = R.from_quat(ori).as_euler('xyz')
          ori += imu_eular_bias
          ori = R.from_euler('xyz', ori).as_quat()

          vel = self.data.qvel[:3].copy()
          angVel = self.data.sensor('BodyGyro').data.copy()

          bodyOdom.header.stamp = rospy.Time.now()
          bodyOdom.pose.pose.position.x = pos[0]
          bodyOdom.pose.pose.position.y = pos[1]
          bodyOdom.pose.pose.position.z = pos[2]
          bodyOdom.pose.pose.orientation.x = ori[1]
          bodyOdom.pose.pose.orientation.y = ori[2]
          bodyOdom.pose.pose.orientation.z = ori[3]
          bodyOdom.pose.pose.orientation.w = ori[0]
          bodyOdom.twist.twist.linear.x = vel[0]
          bodyOdom.twist.twist.linear.y = vel[1]
          bodyOdom.twist.twist.linear.z = vel[2]
          bodyOdom.twist.twist.angular.x = angVel[0]
          bodyOdom.twist.twist.angular.y = angVel[1]
          bodyOdom.twist.twist.angular.z = angVel[2]
          self.pubOdom.publish(bodyOdom)

          bodyImu = Imu()
          acc = self.data.sensor('BodyAcc').data.copy()
          bodyImu.header.stamp = rospy.Time.now()
          bodyImu.angular_velocity.x = angVel[0]
          bodyImu.angular_velocity.y = angVel[1]
          bodyImu.angular_velocity.z = angVel[2]
          bodyImu.linear_acceleration.x = acc[0]
          bodyImu.linear_acceleration.y = acc[1]
          bodyImu.linear_acceleration.z = acc[2]
          bodyImu.orientation.x = ori[1]
          bodyImu.orientation.y = ori[2]
          bodyImu.orientation.z = ori[3]
          bodyImu.orientation.w = ori[0]
          bodyImu.orientation_covariance = [0.0, 0, 0, 0, 0.0, 0, 0, 0, 0.0]
          bodyImu.angular_velocity_covariance = [0.0, 0, 0, 0, 0.0, 0, 0, 0, 0.0]
          bodyImu.linear_acceleration_covariance = [0.0, 0, 0, 0, 0.0, 0, 0, 0, 0.0]
          self.pubImu.publish(bodyImu)

          publish_time = self.data.time

      if (self.data.time - torque_publish_time >= 1.0 / 40.0):
        targetTorque = Float32MultiArray()
        
        published_virtual_forces_L = DEFAULT_MOTOR_VEL
        current_motor_torques_L = np.array([self.data.ctrl[0], self.data.ctrl[1]])
        F_gen_L_raw = Jac_L_inv.T @ current_motor_torques_L # J maps to d(L,phi), so F_gen = [F_L, F_phi]
        published_virtual_forces_L = np.array([-F_gen_L_raw[1], F_gen_L_raw[0]]) 

        published_virtual_forces_R = DEFAULT_MOTOR_VEL
        current_motor_torques_R = np.array([self.data.ctrl[2], self.data.ctrl[3]])
        F_gen_R_raw = Jac_R_inv.T @ current_motor_torques_R
        published_virtual_forces_R = np.array([-F_gen_R_raw[1], F_gen_R_raw[0]])

        actual_wheel_torques = self.data.ctrl[4:10].copy()
        targetTorque.data = np.concatenate((published_virtual_forces_L, published_virtual_forces_R, actual_wheel_torques))
        # print("Published Torque: ", targetTorque.data[0:4])
        self.pubRealTorque.publish(targetTorque)
        torque_publish_time = self.data.time

      if self.data.time >= self.simend:
          break
      if self.pause_flag:
        # publish the state even if the simulation is paused
        # * Publish joint positions and velocities
        # jointsPosVel = Float32MultiArray()
        # get last 12 element of qpos and qvel
        jointsPosVel = Float32MultiArray() 
        qp_wheels_pause = self.data.qpos[15:21].copy()

        phi_act_L_pause, L_act_L_pause = 0.0, 0.0
        phi_act_R_pause, L_act_R_pause = 0.0, 0.0

        fk_L_output_pause = self.five_bar_to_virtual_leg_fk(self.data.qpos[7], self.data.qpos[9]) 
        if fk_L_output_pause is not None:
            L_act_L_pause, phi_act_L_pause = fk_L_output_pause[0], fk_L_output_pause[1] 
        else:
            rospy.logwarn_throttle(1.0, "Pause: Left Leg FK failed. Using default values for publishing.")
                
        fk_R_output_pause = self.five_bar_to_virtual_leg_fk(self.data.qpos[11], self.data.qpos[13]) 
        if fk_R_output_pause is not None:
            L_act_R_pause, phi_act_R_pause = fk_R_output_pause[0], fk_R_output_pause[1] 
        else:
            rospy.logwarn_throttle(1.0, "Pause: Right Leg FK failed. Using default values for publishing.")

        qp_L_ordered_pause = np.array([phi_act_L_pause, L_act_L_pause]) 
        qp_R_ordered_pause = np.array([phi_act_R_pause, L_act_R_pause]) 
                
        qp_pause = np.concatenate((qp_L_ordered_pause, qp_R_ordered_pause, qp_wheels_pause))
        # print("qp_pause: ", qp_pause[0:4])
        qv_pause = np.zeros(10) 

        jointsPosVel.data = np.concatenate((qp_pause, qv_pause))

        self.pubJoints.publish(jointsPosVel)
        # * Publish body pose
        bodyOdom = Odometry()
        pos = self.data.sensor('BodyPos').data.copy()

        #add imu bias
        ori = self.data.sensor('BodyQuat').data.copy()
        ori = R.from_quat(ori).as_euler('xyz')
        ori += imu_eular_bias
        ori = R.from_euler('xyz', ori).as_quat()

        vel = self.data.qvel[:3].copy()
        angVel = self.data.sensor('BodyGyro').data.copy()
        bodyOdom.header.stamp = rospy.Time.now()
        bodyOdom.pose.pose.position.x = pos[0]
        bodyOdom.pose.pose.position.y = pos[1]
        bodyOdom.pose.pose.position.z = pos[2]
        bodyOdom.pose.pose.orientation.x = ori[1]
        bodyOdom.pose.pose.orientation.y = ori[2]
        bodyOdom.pose.pose.orientation.z = ori[3]
        bodyOdom.pose.pose.orientation.w = ori[0]
        bodyOdom.twist.twist.linear.x = 0
        bodyOdom.twist.twist.linear.y = 0
        bodyOdom.twist.twist.linear.z = 0
        bodyOdom.twist.twist.angular.x = 0
        bodyOdom.twist.twist.angular.y = 0
        bodyOdom.twist.twist.angular.z = 0
        self.pubOdom.publish(bodyOdom)

        bodyImu = Imu()
        bodyImu.header.stamp = rospy.Time.now()
        bodyImu.angular_velocity.x = 0
        bodyImu.angular_velocity.y = 0
        bodyImu.angular_velocity.z = 0
        bodyImu.linear_acceleration.x = 0
        bodyImu.linear_acceleration.y = 0
        bodyImu.linear_acceleration.z = 9.81
        bodyImu.orientation.x = ori[1]
        bodyImu.orientation.y = ori[2]
        bodyImu.orientation.z = ori[3]
        bodyImu.orientation.w = ori[0]
        self.pubImu.publish(bodyImu)

      # get framebuffer viewport
      viewport_width, viewport_height = glfw.get_framebuffer_size(
          self.window)
      viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

      # Update scene and render
      mj.mjv_updateScene(self.model, self.data, self.opt, None, self.cam,
                          mj.mjtCatBit.mjCAT_ALL.value, self.scene)
      mj.mjr_render(viewport, self.scene, self.context)

      # swap OpenGL buffers (blocking call due to v-sync)
      glfw.swap_buffers(self.window)

      # process pending GUI events, call GLFW callbacks
      glfw.poll_events()

    glfw.terminate()

def main():
    # ros init
    rospy.init_node('hector_sim', anonymous=True)

    # get xml path
    rospack = rospkg.RosPack()
    rospack.list()
    hector_desc_path = rospack.get_path('wheel_legged_description')
    xml_path = hector_desc_path + "/urdf/Wheel_legged_display.xml"
    sim = WheelLeggedSim(xml_path)
    sim.reset()
    sim.simulate()

if __name__ == "__main__":
    main()
