from dm_control import mjcf
import mujoco_viewer
from mujoco_viewer import MjViewer, MjcfPhysics, Keys


class Env():
   def __init__(self):
        xml_string = """
        <mujoco>
            
            <visual>
                <global offwidth="1920" offheight="1080"/>
            </visual>

            <worldbody>
                <body name="ground">
                    <geom name="ground" friction="0.8" type="plane" pos="0 0 0" size="100 100 0.05" rgba="0.4 0.4 0.4 1"/>
                </body>
            </worldbody>
        </mujoco>
        """
        self.mjcf_model = mjcf.from_xml_string(xml_string)
        self.mjcf_model.asset.add(
            'mesh', name="vessel_mesh", file="./models/vessel.obj", scale=[0.1, 0.1, 0.1])
        self.mjcf_model.worldbody.add(
            "light", directional="true", pos="-0.5 0.5 3", dir="0 0 -1")
        
        vessel = self.mjcf_model.worldbody.add(
            "body", name="vessel", pos="0 0 0.1",)
        vessel.add(
            "geom", name="vessel", type="mesh", pos="0 0 0.1", mesh="vessel_mesh", rgba ="0.9 0.1 0.1 0.6",)
        vessel.add(
            "inertial", pos="0 0 0", mass="0", diaginertia="0 0 0")
        
        self.box = self.mjcf_model.worldbody.add(
            "body", name="box", pos = "0 0 .5",)
        self.box.add("geom", name="box", friction="0.9", type="box", size=".1 .1 .1", rgba="0 0 1 0.6")
        self.box.add("joint", name="ball_joint", type="free")
        


if __name__ == "__main__":
    framerate = 120  # (Hz)
    body = Env()
    physics = MjcfPhysics.from_mjcf_model(body.mjcf_model)
    viewver = MjViewer()
    
    spd = [0, 0, 0]
    start_time = 0.
    
    def test_key_down_event(axis, value):
        global spd
        spd[axis] += value

    def test_key_up_event(axis, value):
        global spd
        spd[axis] -= value
    
    def test_time_event(sim_time:mujoco_viewer.SimTime):
        for axis, value in enumerate(spd):
            physics.named.data.qpos['ball_joint'][axis] += .1 * value
        global start_time
        if sim_time > start_time + 2:
            print(physics.named.data.qpos['ball_joint'][:3], '\n')
            start_time = sim_time
    
    viewver.add_key_event(Keys.KEYDOWN, Keys.K_w, lambda: test_key_down_event(1,0.1))
    viewver.add_key_event(Keys.KEYDOWN, Keys.K_s, lambda: test_key_down_event(1,-0.1))
    viewver.add_key_event(Keys.KEYDOWN, Keys.K_a, lambda: test_key_down_event(0,-0.1))
    viewver.add_key_event(Keys.KEYDOWN, Keys.K_d, lambda: test_key_down_event(0,0.1))
    viewver.add_key_event(Keys.KEYDOWN, Keys.K_SPACE, lambda: test_key_down_event(2,0.1))
    
    viewver.add_key_event(Keys.KEYUP, Keys.K_w, lambda: test_key_up_event(1,0.1))
    viewver.add_key_event(Keys.KEYUP, Keys.K_s, lambda: test_key_up_event(1,-0.1))
    viewver.add_key_event(Keys.KEYUP, Keys.K_a, lambda: test_key_up_event(0,-0.1))
    viewver.add_key_event(Keys.KEYUP, Keys.K_d, lambda: test_key_up_event(0,0.1))
    viewver.add_key_event(Keys.KEYUP, Keys.K_SPACE, lambda: test_key_up_event(2,0.1))
    
    viewver.add_key_event(Keys.KEYDOWN, Keys.K_r, viewver.reset)

    viewver.add_function_to_simulation(test_time_event)

    viewver.realtime_render(physics, framerate)

    