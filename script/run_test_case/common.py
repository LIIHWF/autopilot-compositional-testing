import os

class PathManager:
    def __init__(self, autopilot, vista, ve):
        self.autopilot = autopilot
        self.vista = vista
        self.ve = ve
        
    def verdict_dir(self):
        save_dir = f'test_result/{self.autopilot}/{self.vista}/ve={int(self.ve + 0.1)}'
        return save_dir

    def verdict_path(self, test_case):
        if self.vista == 'crossing_with_traffic_lights':
            file_name = f'xf={round(test_case["xf"], 1)}.json'
        else:
            file_name = f'xf={round(test_case["xf"], 1)};xa={round(test_case["xa"], 1)}.json'
        verdict = os.path.join(self.verdict_dir(), file_name)
        return verdict
    
    def test_case_path(self):
        path = f'test_case/{self.autopilot}_{self.vista}.json'
        return path
