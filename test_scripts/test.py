from bosdyn.client.command_line import RobotModel
import bosdyn.client
import argparse

class Options:
    
    outdir = "Model_Files"


options = Options()


sdk = bosdyn.client.create_standard_sdk('ArmObjectGraspClient')
robot = sdk.create_robot("192.168.37.157")
bosdyn.client.util.authenticate(robot)
robot.time_sync.wait_for_sync()

parser = argparse.ArgumentParser(description="Robot CLI")
subparsers = parser.add_subparsers(title="commands", dest="command")
command_dict = {}

model = RobotModel(subparsers, command_dict)

model._run(robot, options)

print(vars(model))