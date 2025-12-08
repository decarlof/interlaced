# this will be used instead of the original program_PSO (see https://github.com/tomography/tomoscan/blob/08d77831c9a95e90f10cec781dc644159bd42bf0/tomoscan/tomoscan_pso.py#L199C9-L199C20)
# when using Zynq to program the encoder pulses (see https://docs2bm.readthedocs.io/en/latest/source/ops/item_060.html)

def program_PSO_Zynq(self):
    '''Performs programming of PSO output on the Aerotech driver.
    '''
    self.epics_pvs['ScanStatus'].put('Programming PSO')
    overall_sense, user_direction = self._compute_senses()
    log.info(f'Overall sense = {overall_sense}')
    log.info(f'User direction = {user_direction}')
    pso_command = self.epics_pvs['PSOCommand.BOUT']
    pso_model = self.epics_pvs['PSOControllerModel'].get(as_string=True)
    pso_axis = self.epics_pvs['PSOAxisName'].get(as_string=True)
    pso_input = int(self.epics_pvs['PSOEncoderInput'].get(as_string=True))

    # Place the motor at the position where the first PSO pulse should be triggered
    self.epics_pvs['RotationSpeed'].put(self.max_rotation_speed)
    self.epics_pvs['Rotation'].put(self.rotation_start_new, wait=True, timeout=600)
    self.epics_pvs['RotationSpeed'].put(self.motor_speed)

    # Make sure the PSO control is off
    pso_command.put('PSOCONTROL %s RESET' % pso_axis, wait=True, timeout=10.0)
    # Set the output to occur from the I/O terminal on the controller
    if (pso_model == 'Ensemble'):
        pso_command.put('PSOOUTPUT %s CONTROL 1' % pso_axis, wait=True, timeout=10.0)
    elif (pso_model == 'A3200'):
        pso_command.put('PSOOUTPUT %s CONTROL 0 1' % pso_axis, wait=True, timeout=10.0)
    # Set the pulse width.  The total width and active width are the same, since this is a single pulse.
    pulse_width = self.epics_pvs['PSOPulseWidth'].get()
    pso_command.put('PSOPULSE %s TIME %f,%f' % (pso_axis, pulse_width, pulse_width), wait=True, timeout=10.0)
    # Set the pulses to only occur in a specific window
    # pso_command.put('PSOOUTPUT %s PULSE WINDOW MASK' % pso_axis, wait=True, timeout=10.0)
    # Set which encoder we will use.  3 = the MXH (encoder multiplier) input, which is what we generally want
    pso_command.put('PSOTRACK %s INPUT %d' % (pso_axis, pso_input), wait=True, timeout=10.0)
    # Set the distance between pulses. Do this in encoder counts.
    pso_command.put('PSODISTANCE %s FIXED %d' % (pso_axis, 1) , wait=True, timeout=10.0)
    # Which encoder is being used to calculate whether we are in the window.  1 for single axis
    # pso_command.put('PSOWINDOW %s 1 INPUT %d' % (pso_axis, pso_input), wait=True, timeout=10.0)

    # Calculate window function parameters.  Must be in encoder counts, and is 
    # referenced from the stage location where we arm the PSO.  We are at that point now.
    # We want pulses to start at start - delta/2, end at end + delta/2.  
    # range_start = -round(np.abs(self.epics_pvs['PSOEncoderCountsPerStep'].get())/ 2) * overall_sense
    # range_length = np.abs(self.epics_pvs['PSOEncoderCountsPerStep'].get()) * self.num_angles
    # The start of the PSO window must be < end.  Handle this.
    # if overall_sense > 0:
    #     window_start = range_start
    #     window_end = window_start + range_length
    # else:
    #     window_end = range_start
    #     window_start = window_end - range_length
    # pso_command.put('PSOWINDOW %s 1 RANGE %d,%d' % (pso_axis, window_start-5, window_end+5), wait=True, timeout=10.0)
    # Arm the PSO
    pso_command.put('PSOCONTROL %s ARM' % pso_axis, wait=True, timeout=10.0)
