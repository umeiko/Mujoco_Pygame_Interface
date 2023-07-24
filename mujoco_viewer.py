import pygame
import pygame.image
import pygame.display
import pygame.event
import pygame.time
import pygame.font
import time
import pygame.locals as Keys

from typing import Callable, Optional, NewType

# The basic mujoco wrapper.
from dm_control import mujoco
from dm_control import mjcf



SimTime = NewType("SimTime", float)

class MujocoPhysics(mujoco.Physics):
    """为方便调整摄像机而重载的Physics类"""
    def __init__(self, data):
        super().__init__(data)
        
    def my_render(self,
      height=240,
      width=320,
      overlays=(),
      pos= None,
      depth=False,
      segmentation=False,
      scene_option=None,
      render_flag_overrides=None,
      scene_callback: Optional[Callable[['MujocoPhysics', mujoco.MjvScene],
                                        None]] = None,
    ):
        """重载的渲染函数"""
        move_cam = mujoco.MovableCamera(
            physics=self,
            height=height,
            width = width,
            scene_callback=scene_callback)
        if pos == None:
            pass
        else:
            move_cam.set_pose(*pos)
        image = move_cam.render(overlays=overlays, depth=depth, segmentation=segmentation,
        scene_option=scene_option, render_flag_overrides=render_flag_overrides)
        return image,move_cam

class MjcfPhysics(mjcf.Physics):
    """为方便调整摄像机而重载的Physics类"""
    def __init__(self, data):
        super().__init__(data)
        
    def my_render(self,
      height=240,
      width=320,
      overlays=(),
      pos= None,
      depth=False,
      segmentation=False,
      scene_option=None,
      render_flag_overrides=None,
      scene_callback: Optional[Callable[['MjcfPhysics', mujoco.MjvScene],
                                        None]] = None,
    ):
        """重载的渲染函数"""
        move_cam = mujoco.MovableCamera(
            physics=self,
            height=height,
            width = width,
            scene_callback=scene_callback)
        if pos == None:
            pass
        else:
            move_cam.set_pose(*pos)
        image = move_cam.render(overlays=overlays, depth=depth, segmentation=segmentation,
        scene_option=scene_option, render_flag_overrides=render_flag_overrides)
        return image, move_cam

def diff(pos_0:tuple, pos_1:tuple)->list:
    return [pos_0[i] - pos_1[i] for i in range(len(pos_0))]


class MjViewer():
    def __init__(self, scene_option=None) -> None:
        pygame.init()
        pygame.font.init()
        self.font = pygame.font.SysFont(name="arial", size=16)
        self.clock = pygame.time.Clock()
        self.size = [320,240]
        self.scale = 1
        self.display = pygame.display.set_mode(self.size,pygame.RESIZABLE)
        
        if not scene_option:
            self.scene_option = mujoco.wrapper.core.MjvOption()
            # self.scene_option.flags[enums.mjtVisFlag.mjVIS_JOINT] = True
        else:
            self.scene_option = scene_option

        self.isfull_screen = False
        self.move_cam = None
        self.cam_pose = None
        self.is_dragging = False
        self.is_moving = False
        self.mouse_lookat_start_move = (0,0)
        self.mouse_pos_start_drag = (0,0)
        self.cam_pose_start_drag = None
        self.cam_lookat_start_move = None
        self.physics = None
        self.start_time = 0.
        self._event_groups = {}
        self._custom_funcs = []

    def render(self,img_array, flip=True):
        """
        渲染图像
        """
        img = pygame.surfarray.make_surface(img_array)
        img = pygame.transform.flip(img,True,False)
        img = pygame.transform.rotate(img,90)
        self.display.blit(img, (0,0))
        self.clock.tick()
        if flip:
            pygame.display.flip()
        pygame.display.set_caption(f"FPS: {self.clock.get_fps():.4}")
    
    def _custom_key_event_trigger(self, event:pygame.event.Event):
        if event.type in self._event_groups.keys():
            if event.key in self._event_groups[event.type].keys():
               self._event_groups[event.type][event.key]()

    def add_key_event(self, event_type:int, key:int, event_func:Callable) -> None:
        """增添自定义的用户按键行为"""
        if event_type not in self._event_groups.keys():
            self._event_groups[event_type] = {}
        self._event_groups[event_type][key] = event_func

    def add_function_to_simulation(self, custom_func:Callable[[SimTime], None]) -> None:
        """增添自定义的仿真期间行为"""
        self._custom_funcs.append(custom_func)

    def _event_handler(self):
        """
        事件处理器
        """
        for event in pygame.event.get():
            self._custom_key_event_trigger(event)
            if event.type == pygame.QUIT:
                exit()

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_F11:
                    if not self.isfull_screen:
                        self.display = pygame.display.set_mode(self.size, pygame.SCALED)
                        pygame.display.toggle_fullscreen() 
                    else:
                        pygame.display.toggle_fullscreen() 
                        self.display = pygame.display.set_mode(self.size, pygame.RESIZABLE)
                    self.isfull_screen = not self.isfull_screen
            elif event.type == pygame.KEYUP:
                ... 
                
            elif event.type == pygame.WINDOWSIZECHANGED:
                self.size = [event.x,event.y]
            elif event.type == pygame.MOUSEWHEEL:#滚轮实现缩放
                if self.cam_pose is None:
                    self.cam_pose = list(self.move_cam.get_pose())
                if event.y == -1 :
                    self.cam_pose[1] += 0.1 * self.cam_pose[1]
                elif event.y == 1:
                    self.cam_pose[1] -= 0.1 * self.cam_pose[1]
            elif event.type == pygame.MOUSEBUTTONDOWN:#左键视角旋转，右键平移
                self.cam_pose = list(self.move_cam.get_pose())
                if event.button == 1:#左键  
                    self.is_dragging = True
                    self.mouse_pos_start_drag = event.pos
                    self.cam_pose_start_drag = list(self.move_cam.get_pose())
                elif event.button == 3:#右键
                    self.is_moving = True
                    self.mouse_lookat_start_move = event.pos
                    self.cam_lookat_start_move = list(self.move_cam.get_pose())                

            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:#左键
                    self.is_dragging = False
                elif event.button == 3:#右键
                    self.is_moving = False
            
            elif event.type == pygame.MOUSEMOTION:
                if self.is_dragging:
                    diff_ = diff(event.pos, self.mouse_pos_start_drag)
                    self.cam_pose[2] = self.cam_pose_start_drag[2] - diff_[0] / 4
                    self.cam_pose[3] = self.cam_pose_start_drag[3] - diff_[1] / 4
                elif self.is_moving:
                    ...

    def screen_print(self,texts:str):
        """使用该函数在屏幕上打印文字"""
        y = 0
        for text in texts.split("\n"):
            text_surface = self.font.render(text, True, (255, 0, 0))
            self.display.blit(text_surface, (0, y))
            y += self.font.get_height()

    def _run_user_function(self, simulation_time:SimTime, *args):
        for func in self._custom_funcs:
            func(simulation_time, *args)

    def reset(self, *kargs):
        self.physics.reset(*kargs)
        self.start_time = time.time()

    def realtime_render(self, physics:MujocoPhysics, framerate, sim_rate=1.0, min_framerate=10):
        """
        实时渲染
        """
        self.physics = physics
        self.physics.reset()
        min_time = 1 / min_framerate
        self.start_time = time.time()
        time_since_last_render = time.time()
        clock = pygame.time.Clock()

        while True:
            now_time = time.time()
            if self.physics.data.time < sim_rate * (now_time - self.start_time) and \
            (now_time - time_since_last_render) < min_time:
                self._run_user_function(self.physics.data.time)
                self.physics.step()
                continue

            height, width = self.size[1],self.size[0]
            pixels, self.move_cam = self.physics.my_render(int(self.scale * height), int(self.scale * width),
                                                        pos=self.cam_pose, 
                                                        scene_option=self.scene_option)
            clock.tick(framerate)
            self._event_handler()
            self.render(pixels, False)
            self.screen_print(f"Simulate_time: {self.physics.data.time:.3f}\nRealistic_time: {now_time - self.start_time:.3f}")
            pygame.display.flip()
            time_since_last_render = now_time
    
    def simulate(self, physics:MujocoPhysics):
        """不渲染图形界面的情况下执行仿真"""
        self.physics = physics
        self.physics.reset()
        while True:
            self._run_user_function(self.physics.data.time)
            self.physics.step()



if __name__ == "__main__":
    framerate = 120  # (Hz)
    with open("./models/pure_vessel.xml","r") as f :
        swinging_body = f.read()
    # swinging_body = """
    #     <mujoco>
    #     <visual>
    #         <global offwidth="1920" offheight="1080"/>
    #     </visual>
    #     <worldbody>
    #         <light name="top" pos="0 0 1"/>
    #         <body name="box_and_sphere" euler="0 0 -30">  
    #         <joint name="swing" type="hinge" axis="1 -1 0" pos="-.2 -.2 -.2"/>
    #         <geom name="red_box" type="box" size=".2 .2 .2" rgba="1 0 0 1"/>
    #         <geom name="green_sphere" pos=".2 .2 .2" size=".1" rgba="0 1 0 1"/>
    #         </body>
    #     </worldbody>
    #     </mujoco>
    #     """
    physics = MujocoPhysics.from_xml_string(swinging_body)
    env_render = MjViewer()
    env_render.realtime_render(physics,framerate)
    

    