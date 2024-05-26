import loguru
import re
import os


def _repalce_file(file_path, _old, _new):
    with open(file_path) as f:
        conf = f.read()
    conf = re.sub(_old, _new, conf)
    with open(file_path, 'w') as f:
        f.write(conf)

def _add_if_not_exists_in_file(file_path, _new, _old_pattern=None):
    with open(file_path) as f:
        conf = f.read()
    if (_old_pattern is None and _new not in conf) or (_old_pattern is not None and re.search(_old_pattern, conf) is None):
        with open(file_path, 'a') as f:
            f.write('\n')
            f.write(_new)
    elif _old_pattern is not None:
        _repalce_file(file_path, _old_pattern, _new)

def create_tmp_map():
    loguru.logger.info('Creating tmp map')
    os.system('cp -r /apollo/modules/map/data/borregas_ave/ /apollo/modules/map/data/tmp/')
    
    

def set_mode_mock():
    loguru.logger.info('Setting clock_mode to MODE_MOCK')
    conf_path = '/apollo/cyber/conf/cyber.pb.conf'
    _repalce_file(conf_path, 'clock_mode: MODE_CYBER', 'clock_mode: MODE_MOCK')


def set_enable_right_turn_on_red():
    loguru.logger.info('Setting enable_right_turn_on_red to True')
    conf_path = '/apollo/modules/planning/conf/scenario/traffic_light_unprotected_right_turn_config.pb.txt'
    _repalce_file(conf_path, 'enable_right_turn_on_red: false', 'enable_right_turn_on_red: true')


def set_default_cruise_speed():
    loguru.logger.info('Setting default_cruise_speed to 33.34 m/s')
    conf_path = '/apollo/modules/planning/conf/planning.conf'
    _repalce_file(conf_path, r'--default_cruise_speed.*', '--default_cruise_speed=33.34')


def set_planning_upper_speed_limit():
    loguru.logger.info('Setting planning upper speed limit to 33.34 m/s')
    conf_path = '/apollo/modules/planning/conf/planning.conf'
    _repalce_file(conf_path, r'--planning_upper_speed_limit=.*', '--planning_upper_speed_limit=33.34')


def disable_prediction_multi_thread():
    loguru.logger.info('Disabling prediction multi_thread')
    conf_path = '/apollo/modules/prediction/conf/prediction.conf'
    _repalce_file(conf_path, '\n--enable_multi_thread', '\n# --enable_multi_thread')
    _repalce_file(conf_path, '\n--max_thread_num=.*', '\n# --max_thread_num=8')
    _repalce_file(conf_path, '\n--max_caution_thread_num=.*', '\n# --max_caution_thread_num=4')


def disable_reference_line_provider_thread():
    loguru.logger.info('Disabling reference_line_provider_thread')
    conf_path = '/apollo/modules/planning/conf/planning.conf'
    _add_if_not_exists_in_file(conf_path, '--enable_reference_line_provider_thread=false')


def disable_open_space_planner_thread():
    loguru.logger.info('Disabling reference_line_provider_thread')
    conf_path = '/apollo/modules/planning/conf/planning.conf'
    _add_if_not_exists_in_file(conf_path, '--enable_open_space_planner_thread=false')


if __name__ == '__main__':
    set_mode_mock()
    set_enable_right_turn_on_red()
    set_default_cruise_speed()
    set_planning_upper_speed_limit()
    disable_prediction_multi_thread()
    disable_reference_line_provider_thread()
    disable_open_space_planner_thread()

