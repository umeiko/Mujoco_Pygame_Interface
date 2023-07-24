from dm_control import mjcf
import mujoco_viewer
from mujoco_viewer import MjViewer, MjcfPhysics, Keys
import os

class Env():
   def __init__(self, timestep=0.002):
        stl_path = "./models/tube"
        xml_string = """
        <mujoco>
            <visual>
                <global offwidth="1920" offheight="1080"/>
            </visual>

            <option timestep=""" +f"\"{timestep}\""+ """/>

            <worldbody>
                <body name="ground">
                    <geom name="ground" friction="0.8" type="plane" pos="0 0 0" size="100 100 0.05" rgba="0.4 0.4 0.4 1"/>
                </body>
            </worldbody>
        </mujoco>
        """
        self.mjcf_model = mjcf.from_xml_string(xml_string)
        self.mjcf_model.worldbody.add(
            "light", directional="true", pos="-0.5 0.5 3", dir="0 0 -1")
        vessel = self.mjcf_model.worldbody.add(
                "body", name="vessel", pos="0 0 -0.03",)
        for f_name in os.listdir(stl_path):
            if f_name.endswith(".stl"):
                self.mjcf_model.asset.add(
                    'mesh', name=f_name, file=os.path.join(stl_path, f_name), scale=[0.3, 0.3, 0.3])
                vessel.add(
                    "geom", name=f'{f_name}_', type="mesh", pos="0 0 0", mesh=f_name, rgba ="0.9 0.1 0.1 0.6",)
            # vessel.add(
            #     "inertial", pos="0 0 0", mass="0", diaginertia="0 0 0")
        # self.boxes = []
        self.box = self.mjcf_model.worldbody.add(
            "body", name="0", pos = "0 0 0",)
        self.box.add("geom", name="0", friction="0.9", type="box", size=".07 .07 .07", rgba="0 1 1 0.8")
        self.box.add("joint", name="ball_joint", type="free")
        
        # self.boxes.append(self.box)
        tempbox = self.box
        for i in range(15):
            tempbox2 = tempbox.add(
            "body", name=f"box_{i}", pos = f"{0.15} 0 0",)
            tempbox2.add("geom", name=f"box_{i}", friction="0.4", type="capsule", euler="0 90 0",size=".07 .07 .07", rgba="0 0 1 0.8")
            tempbox2.add("joint", name=f"ball_joint_{i}", stiffness="5e3", damping="5e1",type="ball")
            tempbox = tempbox2

if __name__ == "__main__":
    framerate = 120  # (Hz)

    timestep = 0.0005
    sim_rate = 1
    force = 600

    move_spd = .7 * timestep
    jump_spd = 2 * timestep

    body = Env(timestep)
    physics = MjcfPhysics.from_mjcf_model(body.mjcf_model)

    viewver = MjViewer()


    spd = [0, 0, 0]
    start_time = 0.
    def test_key_down_event(axis, value):
        global spd
        spd[axis] += value
    
    def test_pulse_force(axis, value):
        # print(physics.named.data.qfrc_applied)
        physics.named.data.qfrc_applied['ball_joint'][axis] += value
        print(physics.named.data.qfrc_applied['ball_joint'])

    def test_key_up_event(axis, value):
        global spd
        spd[axis] -= value
    
    def test_time_event(sim_time:mujoco_viewer.SimTime):
        ...
        # print(sim_time)
        for axis, value in enumerate(spd):
            physics.named.data.qpos['ball_joint'][axis] += value
        # global start_time
        # if sim_time > start_time + 2:
        #     print(physics.named.data.qpos['ball_joint'][:3], '\n')
        #     start_time = sim_time
    
    def log():
        print(physics.named.data.qpos)
    
    # viewver.add_key_event(Keys.KEYDOWN, Keys.K_w, lambda: test_key_down_event(1,move_spd))
    # viewver.add_key_event(Keys.KEYDOWN, Keys.K_s, lambda: test_key_down_event(1,-move_spd))
    # viewver.add_key_event(Keys.KEYDOWN, Keys.K_a, lambda: test_key_down_event(0,-move_spd))
    # viewver.add_key_event(Keys.KEYDOWN, Keys.K_d, lambda: test_key_down_event(0,move_spd))
    # viewver.add_key_event(Keys.KEYDOWN, Keys.K_SPACE, lambda: test_key_down_event(2,jump_spd))
    
    # viewver.add_key_event(Keys.KEYUP, Keys.K_w, lambda: test_key_up_event(1,move_spd))
    # viewver.add_key_event(Keys.KEYUP, Keys.K_s, lambda: test_key_up_event(1,-move_spd))
    # viewver.add_key_event(Keys.KEYUP, Keys.K_a, lambda: test_key_up_event(0,-move_spd))
    # viewver.add_key_event(Keys.KEYUP, Keys.K_d, lambda: test_key_up_event(0,move_spd))
    # viewver.add_key_event(Keys.KEYUP, Keys.K_SPACE, lambda: test_key_up_event(2,jump_spd))

    viewver.add_key_event(Keys.KEYDOWN, Keys.K_UP, lambda: test_pulse_force(1,force))
    viewver.add_key_event(Keys.KEYDOWN, Keys.K_DOWN, lambda: test_pulse_force(1,-force))
    viewver.add_key_event(Keys.KEYDOWN, Keys.K_LEFT, lambda: test_pulse_force(0,-force))
    viewver.add_key_event(Keys.KEYDOWN, Keys.K_RIGHT, lambda: test_pulse_force(0,force))
    viewver.add_key_event(Keys.KEYDOWN, Keys.K_SPACE, lambda: test_pulse_force(2,jump_spd))

    viewver.add_key_event(Keys.KEYUP, Keys.K_UP, lambda: test_pulse_force(1,-force))
    viewver.add_key_event(Keys.KEYUP, Keys.K_DOWN, lambda: test_pulse_force(1,force))
    viewver.add_key_event(Keys.KEYUP, Keys.K_LEFT, lambda: test_pulse_force(0,force))
    viewver.add_key_event(Keys.KEYUP, Keys.K_RIGHT, lambda: test_pulse_force(0,-force))
    viewver.add_key_event(Keys.KEYUP, Keys.K_SPACE, lambda: test_pulse_force(2,jump_spd))
    
    viewver.add_key_event(Keys.KEYDOWN, Keys.K_r, viewver.reset)
    viewver.add_key_event(Keys.KEYDOWN, Keys.K_l, log)

    # viewver.add_function_to_simulation(test_time_event)

    viewver.realtime_render(physics, framerate, sim_rate)

    