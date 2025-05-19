"""
목적 : 차량·지형 모델 (서스펜션 + 조향 안정성 강화)
작성자 : 박진석
수정자 : 박진석
최종 수정일자 : 2025‑05‑05
"""

import mujoco

XML = """
<mujoco model="racecar">
  <compiler inertiafromgeom="true"/>
  <option timestep="0.001" gravity="0 0 -9.81"/>

  <!-- ====== asset ====== -->
  <asset>
    <texture name="grid" type="2d" builtin="checker"
             rgb1="0.2 0.3 0.4" rgb2="0.8 0.8 0.8"
             width="500" height="500"/>
    <material name="grid_mat" texture="grid" texrepeat="5 5"
              specular="0.3" shininess="0.3"/>
    <texture name="wheel_pattern" type="2d" builtin="checker"
             rgb1="1 1 1" rgb2="0 0 0"
             width="64" height="64"/>
    <material name="wheel_mat" texture="wheel_pattern" texrepeat="8 1"
              specular="0.5" shininess="0.5"/>
  </asset>

  <!-- ====== 월드 ====== -->
  <worldbody>
    <geom name="floor" type="plane" size="500 500 0.1"
          material="grid_mat" friction="1 0.5 0.5"
          contype="1" conaffinity="1" condim="3"/>

    <!-- ====== 차체 ====== -->
    <body name="chassis" pos="0 0 1.5">
      <joint name="root" type="free"/>
      <geom name="chassis" type="box" size="1.875 1.25 0.375"
            rgba="1 0 0 1" mass="3000" friction="1 0.5 0.5"/>
      <geom name="front"  type="box" size="0.625 0.375 0.125"
            pos="1.25 0 0.375" rgba="0 1 0 1" friction="1 0.5 0.5"/>
      <geom name="rear"   type="box" size="0.625 0.375 0.125"
            pos="-1.25 0 0.375" rgba="0 0 1 1" friction="1 0.5 0.5"/>

      <!-- ===== 서스펜션 / 바퀴 4개 ===== -->
      <!-- 앞왼쪽 -->
      <body name="front_left_susp" pos="1.25 1.5 0.7">
        <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
        <joint name="fl_susp" type="slide" axis="0 0 1"
               range="-0.15 0.05" ref="0.05"
               stiffness="20000" damping="3000"/>
        <body name="front_left_wheel" pos="0 0 -1">
          <joint name="fl_steer" type="hinge" axis="0 0 1"
                 range="-0.6 0.6" damping="50" stiffness="1000"/>
          <joint name="fl_wheel" type="hinge" axis="0 1 0" damping="0.1"/>
          <geom name="fl_geom" type="cylinder" size="0.35 0.175"
                euler="90 0 0" material="wheel_mat" mass="7"
                friction="1 0.5 0.5"
                solimp="0.9 0.95 0.001 0.1 2"
                contype="1" conaffinity="1" condim="3"/>
          <site name="fl_site" pos="0 0 -0.7" size="0.01" group="3"/>
        </body>
      </body>

      <!-- 앞오른쪽 -->
      <body name="front_right_susp" pos="1.25 -1.5 0.7">
        <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
        <joint name="fr_susp" type="slide" axis="0 0 1"
               range="-0.15 0.05" ref="0.05"
               stiffness="20000" damping="3000"/>
        <body name="front_right_wheel" pos="0 0 -1">
          <joint name="fr_steer" type="hinge" axis="0 0 1"
                 range="-0.6 0.6" damping="50" stiffness="1000"/>
          <joint name="fr_wheel" type="hinge" axis="0 1 0" damping="0.1"/>
          <geom name="fr_geom" type="cylinder" size="0.35 0.175"
                euler="90 0 0" material="wheel_mat" mass="7"
                friction="1 0.5 0.5"
                solimp="0.9 0.95 0.001 0.1 2"
                contype="1" conaffinity="1" condim="3"/>
          <site name="fr_site" pos="0 0 -0.7" size="0.01" group="3"/>
        </body>
      </body>

      <!-- 뒤왼쪽 -->
      <body name="rear_left_susp" pos="-1.25 1.5 0.7">
        <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
        <joint name="rl_susp" type="slide" axis="0 0 1"
               range="-0.15 0.05" ref="0.05"
               stiffness="20000" damping="3000"/>
        <body name="rear_left_wheel" pos="0 0 -1">
          <joint name="rl_wheel" type="hinge" axis="0 1 0" damping="0.1"/>
          <geom name="rl_geom" type="cylinder" size="0.35 0.175"
                euler="90 0 0" material="wheel_mat" mass="7"
                friction="1 0.5 0.5"
                solimp="0.9 0.95 0.001 0.1 2"
                contype="1" conaffinity="1" condim="3"/>
          <site name="rl_site" pos="0 0 -0.7" size="0.01" group="3"/>
        </body>
      </body>

      <!-- 뒤오른쪽 -->
      <body name="rear_right_susp" pos="-1.25 -1.5 0.7">
        <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
        <joint name="rr_susp" type="slide" axis="0 0 1"
               range="-0.15 0.05" ref="0.05"
               stiffness="20000" damping="3000"/>
        <body name="rear_right_wheel" pos="0 0 -1">
          <joint name="rr_wheel" type="hinge" axis="0 1 0" damping="0.1"/>
          <geom name="rr_geom" type="cylinder" size="0.35 0.175"
                euler="90 0 0" material="wheel_mat" mass="7"
                friction="1 0.5 0.5"
                solimp="0.9 0.95 0.001 0.1 2"
                contype="1" conaffinity="1" condim="3"/>
          <site name="rr_site" pos="0 0 -0.7" size="0.01" group="3"/>
        </body>
      </body>
    </body>
  </worldbody>

  <!-- ====== 액추에이터 ====== -->
  <actuator>
    <!-- 구동 -->
    <motor name="rl_drive" joint="rl_wheel" ctrlrange="-2000 2000" gear="1"/>
    <motor name="rr_drive" joint="rr_wheel" ctrlrange="-2000 2000" gear="1"/>

    <!-- 브레이크 -->
    <motor name="rl_brake" joint="rl_wheel" ctrlrange="-3000 3000" gear="1"/>
    <motor name="rr_brake" joint="rr_wheel" ctrlrange="-3000 3000" gear="1"/>
    <motor name="fl_brake" joint="fl_wheel" ctrlrange="-3000 3000" gear="1"/>
    <motor name="fr_brake" joint="fr_wheel" ctrlrange="-3000 3000" gear="1"/>


    <!-- 조향 -->
    <motor name="fl_steer_motor" joint="fl_steer" ctrlrange="-200 200" gear="10"/>
    <motor name="fr_steer_motor" joint="fr_steer" ctrlrange="-200 200" gear="10"/>
  </actuator>

  <!-- ====== 센서 ====== -->
  <sensor>
    <force name="fl_force" site="fl_site"/>
    <force name="fr_force" site="fr_site"/>
    <force name="rl_force" site="rl_site"/>
    <force name="rr_force" site="rr_site"/>
  </sensor>
</mujoco>
"""

def load_model():
    model = mujoco.MjModel.from_xml_string(XML)
    data  = mujoco.MjData(model)
    return model, data
import mujoco
import mujoco.viewer
import time

XML = """ ... """  # 기존 XML 그대로 유지

def load_model():
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)
    return model, data

# 입력 상태 저장
key_state = {
    "w": False, "s": False,
    "a": False, "d": False,
    "space": False
}

# 모드 및 티어
mode = "manual"      # manual / auto
speed_tier = 1       # 0: 저속, 1: 일반, 2: 고속
hud_visible = True

# 속도 티어별 구동력 설정
speed_force_table = [400, 1000, 2000]

def on_keydown(keycode):
    global mode, speed_tier, hud_visible

    if keycode == ord('w'):
        key_state["w"] = True
    elif keycode == ord('s'):
        key_state["s"] = True
    elif keycode == ord('a'):
        key_state["a"] = True
    elif keycode == ord('d'):
        key_state["d"] = True
    elif keycode == ord(' '):
        key_state["space"] = True
    elif keycode == ord('m'):
        mode = "auto" if mode == "manual" else "manual"
        print(f"[모드 전환] 현재 모드: {mode}")
    elif keycode == ord('h'):
        hud_visible = not hud_visible
        print(f"[HUD] 표시 여부: {hud_visible}")
    elif keycode == ord('r'):
        reset_vehicle()
        print("[Reset] 차량 위치 초기화됨")
    elif keycode == mujoco.viewer.KEY_SHIFT:
        speed_tier = (speed_tier + 1) % 3
        print(f"[속도 변경] 티어: {speed_tier} / 힘: {speed_force_table[speed_tier]}")

def on_keyup(keycode):
    if keycode == ord('w'):
        key_state["w"] = False
    elif keycode == ord('s'):
        key_state["s"] = False
    elif keycode == ord('a'):
        key_state["a"] = False
    elif keycode == ord('d'):
        key_state["d"] = False
    elif keycode == ord(' '):
        key_state["space"] = False

def reset_vehicle():
    data.qpos[:] = 0
    data.qvel[:] = 0
    data.qpos[2] = 1.5  # 지면 위로 띄우기
    mujoco.mj_forward(model, data)

def my_control(model, data):
    if mode == "manual":
        drive_force = speed_force_table[speed_tier]
        steer_force = 100 if speed_tier == 0 else 200

        # 후륜 구동
        if key_state["w"]:
            data.ctrl[0] = drive_force   # rl_drive
            data.ctrl[1] = drive_force   # rr_drive
        elif key_state["s"]:
            data.ctrl[0] = -drive_force
            data.ctrl[1] = -drive_force
        else:
            data.ctrl[0] = 0
            data.ctrl[1] = 0

        # 조향 (fl_steer_motor, fr_steer_motor)
        if key_state["a"]:
            data.ctrl[6] = steer_force   # 왼쪽
            data.ctrl[7] = steer_force
        elif key_state["d"]:
            data.ctrl[6] = -steer_force  # 오른쪽
            data.ctrl[7] = -steer_force
        else:
            data.ctrl[6] = 0
            data.ctrl[7] = 0

        # 브레이크 (space 키)
        brake_force = 2000 if key_state["space"] else 0
        data.ctrl[2] = brake_force  # rl_brake
        data.ctrl[3] = brake_force  # rr_brake
        data.ctrl[4] = brake_force  # fl_brake
        data.ctrl[5] = brake_force  # fr_brake

    elif mode == "auto":
        # 자율 주행 간단 예시 (직진)
        drive_force = 500
        data.ctrl[0] = drive_force
        data.ctrl[1] = drive_force
        data.ctrl[6] = 0  # 조향
        data.ctrl[7] = 0
        data.ctrl[2:6] = 0  # 브레이크 없음

if __name__ == "__main__":
    model, data = load_model()

    with mujoco.viewer.launch_control(
        model, data,
        my_control,
        keydown_callback=on_keydown,
        keyup_callback=on_keyup
    ) as viewer:
        while viewer.is_running():
            if hud_visible:
                viewer.user_scn.text.append(
                    f"MODE: {mode.upper()} | SPEED TIER: {speed_tier} ({speed_force_table[speed_tier]})"
                )
            mujoco.mj_step(model, data)
            time.sleep(model.opt.timestep)

