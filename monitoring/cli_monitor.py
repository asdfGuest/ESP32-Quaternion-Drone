from log_reader import LogReader

log_reader = LogReader(port='COM6', baud=921600)


for frame in log_reader.iter():

    stats = log_reader.stats()
    frame_info = (
        '| Frame Info | OK:%d  FPS:%6.1f  CRC:%d  DS:%d  LEN:%d  ERR:%.2f'%
        (
            stats['ok'], stats['fps'], stats['crc_fail'], stats['desync'], stats['len_err'], stats['error_rate']*100
        )
    )

    data = {
        'usage[mean/max]': '%5.3f %5.3f'%(frame.usage, frame.usage_max),
        'dt[mean/max]': '%5.3f %5.3f'%(frame.dt, frame.dt_max),
        'raw_accel[x/y/z]': '%+5.2f %+5.2f %+5.2f'%(frame.raw_accel_x, frame.raw_accel_y, frame.raw_accel_z),
        'raw_gyro[x/y/z]': '%+5.1f %+5.1f %+5.1f'%(frame.raw_gyro_x, frame.raw_gyro_y, frame.raw_gyro_z),
        'accel[x/y/z]': '%+5.2f %+5.2f %+5.2f'%(frame.accel_x, frame.accel_y, frame.accel_z),
        'gyro[x/y/z]': '%+5.1f %+5.1f %+5.1f'%(frame.gyro_x, frame.gyro_y, frame.gyro_z),
        'temp': '%5.2f'%(frame.temp),
        'rot[w/x/y/z]': '%+5.2f %+5.2f %+5.2f %+5.2f'%(frame.rot_w, frame.rot_x, frame.rot_y, frame.rot_z),
        'tilt[w/x/y/z]': '%+5.2f %+5.2f %+5.2f %+5.2f'%(frame.target_tilt_quat_w, frame.target_tilt_quat_x, frame.target_tilt_quat_y, frame.target_tilt_quat_z),
        'raw_rc[t/r/p/y]': '%5.2f %+5.2f %+5.2f %+5.2f'%(frame.raw_rc_throttle, frame.raw_rc_roll, frame.raw_rc_pitch, frame.raw_rc_yaw),
        'rc[t/r/p/y]': '%5.2f %+5.2f %+5.2f %+5.2f'%(frame.rc_throttle, frame.rc_roll, frame.rc_pitch, frame.rc_yaw),
        'arm[rc/drone]': '%5d %5d'%(frame.rc_arm, frame.drone_arm),
        'rc_ctrl': '%5d'%(frame.rc_ctrl_mode,),
        'pid[x/y/z]': '%+5.2f %+5.2f %+5.2f'%(*frame.pid_val,),
        'esc[fl/fr/bl/br]': '%5.3f %5.3f %5.3f %5.3f'%(*frame.esc_throttle,),
        '': '',
    }
    format = [
        ('usage[mean/max]', 'dt[mean/max]'),
        ('raw_accel[x/y/z]', 'accel[x/y/z]'),
        ('raw_gyro[x/y/z]', 'gyro[x/y/z]'),
        ('', 'temp'),
        ('rot[w/x/y/z]', 'tilt[w/x/y/z]'),
        ('raw_rc[t/r/p/y]', 'rc[t/r/p/y]'),
        ('arm[rc/drone]', 'rc_ctrl'),
        ('', 'pid[x/y/z]'),
        ('', 'esc[fl/fr/bl/br]'),
    ]

    total_width = (11 + (16 + 23) * 2)
    output = ''
    output += '\n\n\n\n\n'
    output += frame_info + '\n'
    output += '-' * total_width + '\n'
    for keys in format:
        output += '| %16s: %23s | %16s: %23s |\n'%(keys[0], data[keys[0]], keys[1], data[keys[1]])
    output += '-' * total_width + '\n'
    
    print(output, end='')


log_reader.close()
