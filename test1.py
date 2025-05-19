import mujoco
import glfw
import numpy as np
import time

# 握拳阶段枚举
from enum import Enum
class GraspStage(Enum):
    CLOSING = 0
    HOLD_CLOSED = 1
    OPENING = 2
    HOLD_OPEN = 3

def main():
    # 加载模型
    model = mujoco.MjModel.from_xml_path('scene_combined.xml')
    data = mujoco.MjData(model)

    # 初始化GLFW窗口
    glfw.init()
    window = glfw.create_window(2560, 1440, "da wangwang", None, None)
    glfw.make_context_current(window)
    glfw.swap_interval(1)

    # 可视化组件
    scene = mujoco.MjvScene(model, maxgeom=10000)
    context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
    camera = mujoco.MjvCamera()
    option = mujoco.MjvOption()

    # 相机初始设置
    camera.distance = 2.0
    camera.azimuth = 135
    camera.elevation = -20
    camera.lookat = np.array([0, 0, 0.5])

    # 鼠标控制变量
    mouse_sensitivity = 0.5
    prev_mouse = np.zeros(2)
    is_dragging = False

    # 滚轮缩放回调
    def scroll_callback(window, xoffset, yoffset):
        camera.distance *= 0.9**yoffset
    glfw.set_scroll_callback(window, scroll_callback)

    # 获取执行器索引
    def get_actuator_indices(side):
        return {f"{finger}_{joint}": model.actuator(f"{side}_{finger}_{joint}_actuator").id
                for finger in ['thumb', 'index', 'middle', 'ring', 'pinky']
                for joint in ['mcp', 'pip']}
    
    left_act = get_actuator_indices('left')
    right_act = get_actuator_indices('right')

    # 握拳控制参数
    grasp_config = {
        'close_duration': 1.0,  # 握拳耗时
        'hold_closed': 1.0,     # 握拳保持
        'open_duration': 1.0,   # 张开耗时
        'hold_open': 1.0,       # 张开保持
        'hand_sync': 0.0        # 双手同步
    }

    # 状态跟踪
    grasp_stage = GraspStage.CLOSING
    stage_start_time = time.time()
    cycle_progress = 0.0

    # 主循环
    while not glfw.window_should_close(window):
        glfw.poll_events()

        # === 鼠标视角控制 ===
        button_pressed = None
        if glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT):
            button_pressed = 'left'
        elif glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE):
            button_pressed = 'middle'

        if button_pressed is not None:
            curr_mouse = np.array(glfw.get_cursor_pos(window))
            if is_dragging:
                delta = curr_mouse - prev_mouse
                if button_pressed == 'left':
                    # 旋转处理
                    camera.azimuth -= delta[0] * mouse_sensitivity
                    camera.elevation = np.clip(camera.elevation + delta[1] * mouse_sensitivity, -90, 90)
                elif button_pressed == 'middle':
                    # 平移处理
                    azimuth_rad = np.radians(camera.azimuth)
                    elevation_rad = np.radians(camera.elevation)
                    # 计算前向向量
                    forward = np.array([
                        np.sin(azimuth_rad) * np.cos(elevation_rad),
                        np.cos(azimuth_rad) * np.cos(elevation_rad),
                        np.sin(elevation_rad)
                    ])
                    # 右向量：前向与世界Z轴的叉积
                    right = np.cross(forward, np.array([0, 0, 1]))
                    norm = np.linalg.norm(right)
                    if norm < 1e-6:
                        # 处理前向与Z轴平行的情况
                        right = np.array([1.0, 0.0, 0.0])
                    else:
                        right /= norm
                    # 上向量：右与前向的叉积
                    up = np.cross(right, forward)
                    up /= np.linalg.norm(up)
                    # 计算平移量，考虑distance的影响
                    sensitivity_pan = 0.005 * camera.distance
                    delta_x = delta[0] * sensitivity_pan
                    delta_y = -delta[1] * sensitivity_pan  # 屏幕Y轴向下，取反
                    # 更新lookat点
                    camera.lookat += right * delta_x + up * delta_y
            prev_mouse = curr_mouse
            is_dragging = True
        else:
            is_dragging = False

        # === 分阶段握拳控制 ===
        elapsed = time.time() - stage_start_time
        stage_durations = {
            GraspStage.CLOSING: grasp_config['close_duration'],
            GraspStage.HOLD_CLOSED: grasp_config['hold_closed'],
            GraspStage.OPENING: grasp_config['open_duration'],
            GraspStage.HOLD_OPEN: grasp_config['hold_open']
        }

        # 阶段过渡检测
        if elapsed > stage_durations[grasp_stage]:
            grasp_stage = GraspStage((grasp_stage.value + 1) % 4)
            stage_start_time = time.time()
            elapsed = 0.0

        # 计算握力系数
        if grasp_stage == GraspStage.CLOSING:
            grip = min(elapsed / grasp_config['close_duration'], 1.0)
        elif grasp_stage == GraspStage.HOLD_CLOSED:
            grip = 1.0
        elif grasp_stage == GraspStage.OPENING:
            grip = 1.0 - min(elapsed / grasp_config['open_duration'], 1.0)
        else:
            grip = 0.0

        # 应用控制信号
        def apply_grip(actuators, strength):
            for key in actuators:
                if 'thumb' in key:  # 拇指单独处理
                    data.ctrl[actuators[key]] = strength * 0.8 if 'mcp' in key else strength * 1.0
                else:
                    data.ctrl[actuators[key]] = strength

        apply_grip(left_act, grip)
        apply_grip(right_act, grip)

        # 物理模拟
        mujoco.mj_step(model, data)

        # 渲染场景
        viewport = mujoco.MjrRect(0, 0, *glfw.get_framebuffer_size(window))
        mujoco.mjv_updateScene(model, data, option, None, camera, mujoco.mjtCatBit.mjCAT_ALL, scene)
        mujoco.mjr_render(viewport, scene, context)
        
        # 显示状态信息
        stage_names = ["Closing", "Holding Closed", "Opening", "Holding Open"]
        mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, 
                          mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                          viewport,
                          f"Current Stage: {stage_names[grasp_stage.value]}\n"
                          f"Stage Progress: {elapsed:.1f}/{stage_durations[grasp_stage]:.1f}s\n"
                          f"Grip Strength: {grip:.2f}",
                          None, context)

        glfw.swap_buffers(window)

    glfw.terminate()

if __name__ == "__main__":
    main()