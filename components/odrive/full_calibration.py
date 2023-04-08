import calibration
import one_time_calibration
import test_run
import utils


odrvs = utils.find_odrvs()

print('Running Calibration ...')
calibration.run_calib(odrvs)
print('Test run ...')
test_run.test_run(odrvs, running_time=5, running_speed=3)
print('Setting Configuration ...')
one_time_calibration.one_time(odrvs)
print('Test run ...')
test_run.test_run(odrvs, running_time=5, running_speed=3)