import utils as m
import asyncio
import odrive.utils as u

def beans(odrvs):
    def bean():
        beans = True
        while(beans):
            vel_gain = int(input("gimme a gain"))
            odrvs[0].axis0.controller.config.vel_gain = vel_gain
            odrvs[0].axis0.controller.set

async def main():
    odrvs = await m.find_odrvs_async()
    b = beans(odrvs)
    graph = asyncio.create_task(u.start_liveplotter(lambda:[odrvs[0].axis0.encoder.return_vel(), odrvs[0].axis0.controller.return_commanded_vel()]))
    tune = asyncio.create_task(b)
