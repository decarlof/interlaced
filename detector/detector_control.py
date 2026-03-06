import time
import logging

import numpy as np
from epics import PV

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')


def init_epics_PVs(detector_prefix):
    epics_PVs = {}

    # detector pv's
    camera_prefix = detector_prefix + 'cam1:' 

    epics_PVs['CamManufacturer_RBV']       = PV(camera_prefix + 'Manufacturer_RBV')
    epics_PVs['CamModel']                  = PV(camera_prefix + 'Model_RBV')
    epics_PVs['Cam1SerialNumber']          = PV(camera_prefix + 'SerialNumber_RBV')
    epics_PVs['Cam1ImageMode']             = PV(camera_prefix + 'ImageMode')
    epics_PVs['Cam1ArrayCallbacks']        = PV(camera_prefix + 'ArrayCallbacks')
    epics_PVs['Cam1AcquirePeriod']         = PV(camera_prefix + 'AcquirePeriod')
    epics_PVs['Cam1SoftwareTrigger']       = PV(camera_prefix + 'SoftwareTrigger') 
    epics_PVs['Cam1AcquireTime']           = PV(camera_prefix + 'AcquireTime')
    epics_PVs['Cam1AcquireTime_RBV']       = PV(camera_prefix + 'AcquireTime_RBV')
    epics_PVs['Cam1FrameType']             = PV(camera_prefix + 'FrameType')
    epics_PVs['Cam1AttributeFile']         = PV(camera_prefix + 'NDAttributesFile')
    epics_PVs['Cam1SizeX']                 = PV(camera_prefix + 'SizeX')
    epics_PVs['Cam1SizeY']                 = PV(camera_prefix + 'SizeY')
    epics_PVs['Cam1NumImages']             = PV(camera_prefix + 'NumImages')
    epics_PVs['Cam1TriggerMode']           = PV(camera_prefix + 'TriggerMode')
    epics_PVs['Cam1Acquire']               = PV(camera_prefix + 'Acquire')
    epics_PVs['Cam1SizeX_RBV']             = PV(camera_prefix + 'SizeX_RBV')
    epics_PVs['Cam1SizeY_RBV']             = PV(camera_prefix + 'SizeY_RBV')
    epics_PVs['Cam1MaxSizeX_RBV']          = PV(camera_prefix + 'MaxSizeX_RBV')
    epics_PVs['Cam1MaxSizeY_RBV']          = PV(camera_prefix + 'MaxSizeY_RBV')
    epics_PVs['Cam1PixelFormat_RBV']       = PV(camera_prefix + 'PixelFormat_RBV')
    epics_PVs['ArrayRate_RBV']             = PV(camera_prefix + 'ArrayRate_RBV')

    image_prefix = detector_prefix + 'image1:'
    epics_PVs['Image']                     = PV(image_prefix + 'ArrayData')
    epics_PVs['Cam1Display']               = PV(image_prefix + 'EnableCallbacks')

    manufacturer = epics_PVs['CamManufacturer_RBV'].get(as_string=True)
    model = epics_PVs['CamModel'].get(as_string=True)

    if model in ('Oryx ORX-10G-51S5M', 'Oryx ORX-10G-310S9M'):
        logging.info('Detector %s model %s detected', manufacturer, model)
        epics_PVs['Cam1AcquireTimeAuto']   = PV(detector_prefix + 'AcquireTimeAuto')
        epics_PVs['Cam1FrameRateOnOff']    = PV(detector_prefix + 'FrameRateEnable')
        epics_PVs['Cam1TriggerSource']     = PV(detector_prefix + 'TriggerSource')
        epics_PVs['Cam1TriggerOverlap']    = PV(detector_prefix + 'TriggerOverlap')
        epics_PVs['Cam1ExposureMode']      = PV(detector_prefix + 'ExposureMode')
        epics_PVs['Cam1TriggerSelector']   = PV(detector_prefix + 'TriggerSelector')
        epics_PVs['Cam1TriggerActivation'] = PV(detector_prefix + 'TriggerActivation')
    else:
        logging.error('Detector %s model %s is not supported', manufacturer, model)
        return None        

    # Aerotech Ensemble PSO
    tomoscan_prefix = '2bmb:TomoScan:'

    epics_PVs['PSOControllerModel'] = PV(tomoscan_prefix + 'PSOControllerModel')
    epics_PVs['PSOCountsPerRotation'] = PV(tomoscan_prefix + 'PSOCountsPerRotation')

    return epics_PVs


def frame_rate():
    detector_prefix = '2bmSP1:'
    epics_PVs = init_epics_PVs(detector_prefix)

    if epics_PVs is None:
        logging.error('Failed to initialize PVs for %s', detector_prefix)
        return None

    detector_sn = epics_PVs['Cam1SerialNumber'].get()
    if detector_sn in (None, 'Unknown'):
        logging.error('Detector with EPICS IOC prefix %s is down', detector_prefix)
        return None
    else:
        logging.info('Detector with EPICS IOC prefix %s and serial number %s is on', detector_prefix, detector_sn)

        epics_PVs['Cam1ImageMode'].put(2, wait=True)  # set Continuous
        logging.info('ImageMode set to %s', epics_PVs['Cam1ImageMode'].get(as_string=True))

        epics_PVs['Cam1Acquire'].put(1)
        time.sleep(3)

        fr = epics_PVs['ArrayRate_RBV'].get()
        logging.info('Measured frame rate: %.2f Hz', fr)

        epics_PVs['Cam1Acquire'].put(0)

    return fr


def compute_frame_time():
    """Computes the time to collect and readout an image from the camera.

    This method is used to compute the time between triggers to the camera.
    This can be used, for example, to configure a trigger generator or to compute
    the speed of the rotation stage.

    The calculation is camera specific.  The result can depend on the actual exposure time
    of the camera, and on a variety of camera configuration settings (pixel binning,
    pixel bit depth, video mode, etc.)


    Returns
    -------
    float
        The frame time, which is the minimum time allowed between triggers for the value of the
        ``ExposureTime`` PV.
    """
    detector_prefix = '2bmSP1:'
    epics_PVs = init_epics_PVs(detector_prefix)
    # The readout time of the camera depends on the model, and things like the
    # PixelFormat, VideoMode, etc.
    # The measured times in ms with 100 microsecond exposure time and 1000 frames
    # without dropping
    camera_model = epics_PVs['CamModel'].get(as_string=True)
    readout = None
    video_mode = None
    # Adding 1% read out margin to the exposure time, and at least 1 ms seems to work for FLIR cameras
    # This is empirical and if needed should adjusted for each camera
    readout_margin = 1.01
    if camera_model == 'Grasshopper3 GS3-U3-51S5M':
        pixel_format = epics_PVs['Cam1PixelFormat_RBV'].get(as_string=True) 
        readout_times = {
            'Mono8': 6.18,
            'Mono12Packed': 8.20,
            'Mono12p': 8.20,
            'Mono16': 12.34
        }
        readout = readout_times[pixel_format]/1000.            
    if camera_model == 'Oryx ORX-10G-51S5M':
        pixel_format = epics_PVs['Cam1PixelFormat_RBV'].get(as_string=True) 
        readout_margin = 1.05
        readout_times = {
            'Mono8': 6.18,
            'Mono12Packed': 8.20,
            'Mono16': 12.34
        }
        readout = readout_times[pixel_format]/1000.
    if camera_model == 'Oryx ORX-10G-310S9M':
        pixel_format = epics_PVs['Cam1PixelFormat_RBV'].get(as_string=True) 
        readout_times = {
            'Mono8': 30.0,
            'Mono12Packed': 30.0,
            'Mono16': 30.0
        }
        readout_margin = 1.2
        readout = readout_times[pixel_format]/1000.

    if readout is None:
        log.error('Unsupported combination of camera model, pixel format and video mode: %s %s %s',
                      camera_model, pixel_format, video_mode)            
        return 0

    # We need to use the actual exposure time that the camera is using, not the requested time
    exposure = epics_PVs['Cam1AcquireTime_RBV'].value
    # Add some extra time to exposure time for margin.

    frame_time = exposure * readout_margin   
    # If the time is less than the readout time then use the readout time plus 1 ms.
    if frame_time < readout:
        frame_time = readout + .001

    return frame_time


def rotary_stage_velocity(rotation_start, rotation_step, num_angles):

    detector_prefix = '2bmSP1:'
    epics_PVs = init_epics_PVs(detector_prefix)

    # Computes several parameters describing the fly scan motion.
    # Computes the spacing between points, ensuring it is an integer number
    # of encoder counts.
    # Uses this spacing to recalculate the end of the scan, if necessary.
    # Computes the taxi distance at the beginning and end of scan to allow
    # the stage to accelerate to speed.
    # Assign the fly scan angular position to theta[]
    # Compute the actual delta to keep each interval an integer number of encoder counts
    encoder_multiply = float(epics_PVs['PSOCountsPerRotation'].get()) / 360.
    raw_delta_encoder_counts = rotation_step * encoder_multiply
    delta_encoder_counts = round(raw_delta_encoder_counts)
    if abs(raw_delta_encoder_counts - delta_encoder_counts) > 1e-4:
        logging.warning('  *** *** *** Requested scan would have used a non-integer number of encoder counts.')
        logging.warning('  *** *** *** Calculated # of encoder counts per step = {0:9.4f}'.format(raw_delta_encoder_counts))
        logging.warning('  *** *** *** Instead, using {0:d}'.format(delta_encoder_counts))
        new_rotation_step = delta_encoder_counts / encoder_multiply

        logging.warning('  *** *** *** new rotation_step = %.7f° instead of %.7f°' % (new_rotation_step, rotation_step))
        rotation_stop     = rotation_start + num_angles * rotation_step
        new_rotation_stop = rotation_start + num_angles * new_rotation_step
        logging.warning('  *** *** *** new rotation_step = %.7f° instead of %.7f°' % (new_rotation_stop, rotation_stop))

    # In the regular fly scan we compute the time to collect each frame which is the exposure time plus the readout time plus a margin
    # then we use this time as the time to travel a rotation step
    time_per_angle_step = compute_frame_time()
    logging.info('Time per angular step %f s', time_per_angle_step)

    motor_speed = np.abs(new_rotation_step) / time_per_angle_step

    return motor_speed

def main():

    # In a standard fly scan, the configuration parameters (rotation_start, rotation_step, and num_angles)
    # are used to compute the rotation motor speed. Projection images are acquired continuously as the sample rotates.
    # The rotation speed is set to ensure there is no motion blur during each exposure.
    logging.info('******** ******** ********')
    logging.info('******** ******** ********')
    logging.info('******** FLY SCAN ********')
    logging.info('******** ******** ********')
    logging.info('******** ******** ********')
    rotation_start       = 0
    rotation_step        = 0.12
    num_angles           = 1500
    rotation_velocity = rotary_stage_velocity(rotation_start, rotation_step, num_angles)
    logging.info('Rotary stage velocity: %f °/s', rotation_velocity)

    # In an interlaced scan, the goal is instead to define 
    # N = number of frames per rotation
    # n = numnber of rotation
    # and set the rotation stage to run at the maximum velocity that does not cause image blurring

    N = 6
    n = 3
    delta_theta = n * 180 / (N * n - 1)


    # Let's measure the detector frame rate
    logging.info('******** *************** ********')
    logging.info('******** *************** ********')
    logging.info('******** INTERLACED SCAN ********')
    logging.info('******** *************** ********')
    logging.info('******** *************** ********')
    fps = frame_rate()
    if fps is not None:
        logging.info('Frame rate function returned: %.2f Hz', fps)
    else:
        logging.warning('Frame rate measurement failed.')

if __name__ == '__main__':
    main()
