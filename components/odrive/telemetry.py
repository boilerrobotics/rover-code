
import utils
import asyncio
import time
import pandas as pd

def get_vel(odrvs):
    info = []
    x = 0
    initial_time = time.time()
    while(x < 100):
        for odrv in odrvs:
            # info.append((" ".join((odrv.section, str(time.time() - initial_time))), odrv.axis0.motor.get_power(), odrv.axis0.motor.get_current()))
            info.append({
                "timestamp": time.time() - initial_time,
                "velocity": odrv.axis0.encoder.get_vel(),
            })
            time.sleep(.1)
        x += 1
    odrvs[0].axis0.controller.set_speed(0)
    # with open("curr_2.txt", "w") as fp:
    #     fp.writelines("".join((" : ".join((x[0], str(x[1]), str(x[2]))), "\n")) for x in info)
    pd.DataFrame(info).to_csv("vel.csv", index=False)

def get_current(odrvs):
    info = []
    x = 0
    initial_time = time.time()
    while(x < 100):
        for odrv in odrvs:
            # info.append((" ".join((odrv.section, str(time.time() - initial_time))), odrv.axis0.motor.get_power(), odrv.axis0.motor.get_current()))
            info.append({
                "timestamp": time.time() - initial_time,
                "power": odrv.axis0.motor.get_power(),
                "current": odrv.axis0.motor.get_current()
            })
            time.sleep(.1)
        x += 1
    odrvs[0].axis0.controller.set_speed(0)
    # with open("curr_2.txt", "w") as fp:
    #     fp.writelines("".join((" : ".join((x[0], str(x[1]), str(x[2]))), "\n")) for x in info)
    pd.DataFrame(info).to_csv("curr.csv", index=False)



async def main():
    odrvs = await utils.find_odrvs_async()
    odrvs[0].axis0.request_close_loop_control()
    odrvs[0].axis0.controller.set_speed(10)
    #get_vel(odrvs)
    get_current(odrvs)
    


if __name__ == "__main__":
    asyncio.run(main())